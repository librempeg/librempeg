/*
 * Copyright (c) 2022, Ruben Nunez (Aikku; aik AT aol DOT com DOT au)
 *
 * ULC audio decoder
 *
 * This file is part of Librempeg
 *
 * Librempeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Librempeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with Librempeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/tx.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "bytestream.h"

#define MIN_CHANS     1
#define MAX_CHANS   255
#define MIN_BANDS   256
#define MAX_BANDS 32768
#define BUFFER_ALIGNMENT 64u

typedef struct BitContext {
    GetByteContext b;
    unsigned offset;
} BitContext;

typedef struct ChannelContext {
    float *buffer;
    float *temp;
    float *inv_lap;
} ChannelContext;

typedef struct ULCContext {
    BitContext gb;

    uint8_t *bitstream;
    int64_t max_framesize;
    int bitstream_size;
    int bitstream_index;

    int nb_samples;

    int last_subblock_size;
    ChannelContext cc[MAX_CHANS];

    float *win[15];
    AVTXContext *tx_ctx[7];
    av_tx_fn tx_fn[7];
} ULCContext;

static av_cold int ulc_close(AVCodecContext *avctx)
{
    ULCContext *s = avctx->priv_data;

    for (int ch = 0; ch < MAX_CHANS; ch++) {
        ChannelContext *cc = &s->cc[ch];

        av_freep(&cc->buffer);
        av_freep(&cc->temp);
        av_freep(&cc->inv_lap);
    }

    for (int n = MIN_BANDS; n <= s->nb_samples; n *= 2) {
        const int idx = av_ceil_log2(n) - 8;

        av_tx_uninit(&s->tx_ctx[idx]);
    }

    for (int n = 1; n < 16; n++)
        av_freep(&s->win[n-1]);

    av_freep(&s->bitstream);
    s->bitstream_size = 0;

    return 0;
}

static int init_state(AVCodecContext *avctx)
{
    ULCContext *s = avctx->priv_data;
    int nb_channels = avctx->ch_layout.nb_channels;
    int nb_samples = s->nb_samples;

    for (int ch = 0; ch < nb_channels; ch++) {
        ChannelContext *cc = &s->cc[ch];

        cc->buffer = av_calloc(nb_samples, sizeof(*cc->buffer));
        cc->temp = av_calloc(nb_samples*2, sizeof(*cc->temp));
        cc->inv_lap = av_calloc(nb_samples, sizeof(*cc->inv_lap));
        if (!cc->buffer || !cc->temp || !cc->inv_lap)
            return AVERROR(ENOMEM);
    }

    s->last_subblock_size = 0;

    for (int n = MIN_BANDS; n <= s->nb_samples; n *= 2) {
        const int idx = av_ceil_log2(n) - 8;
        float scale = 1.f;
        int ret;

        ret = av_tx_init(&s->tx_ctx[idx], &s->tx_fn[idx], AV_TX_FLOAT_MDCT, 1, n, &scale, 0);
        if (ret < 0)
            return ret;
    }

    for (int n = 1; n < 16; n++) {
        const int size = 1 << n;
        const int idx = n-1;

        s->win[idx] = av_calloc(size, sizeof(*s->win[0]));
        if (!s->win[idx])
            return AVERROR(ENOMEM);

        for (int i = 0; i < size; i++)
            s->win[idx][i] = sin(M_PI_2 * (i+0.5) / size);
    }

    return 0;
}

static av_cold int ulc_init(AVCodecContext *avctx)
{
    ULCContext *s = avctx->priv_data;

    avctx->sample_fmt = AV_SAMPLE_FMT_FLTP;

    if (avctx->ch_layout.nb_channels < MIN_CHANS ||
        avctx->ch_layout.nb_channels > MAX_CHANS)
        return AVERROR_INVALIDDATA;

    if (avctx->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    if (!avctx->extradata || avctx->extradata_size < 20)
        return AVERROR_INVALIDDATA;

    s->nb_samples = AV_RL16(avctx->extradata);
    if (s->nb_samples < MIN_BANDS || s->nb_samples > MAX_BANDS)
        return AVERROR_INVALIDDATA;
    if ((s->nb_samples & (-s->nb_samples)) != s->nb_samples)
        return AVERROR_INVALIDDATA;

    s->max_framesize = s->nb_samples * avctx->ch_layout.nb_channels * 16LL;
    if (s->max_framesize > (INT32_MAX - AV_INPUT_BUFFER_PADDING_SIZE) / 8)
        return AVERROR_INVALIDDATA;

    s->bitstream = av_calloc(s->max_framesize + AV_INPUT_BUFFER_PADDING_SIZE, sizeof(*s->bitstream));
    if (!s->bitstream)
        return AVERROR(ENOMEM);

    return init_state(avctx);
}

#define ESCAPE_SEQUENCE_STOP           (-1)
#define ESCAPE_SEQUENCE_STOP_NOISEFILL (-2)
static uint32_t update_random_seed(void)
{
    static uint32_t seed = 1234567;

    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed <<  5;

    return seed;
}

static int read_nibble(BitContext *gb)
{
    int x = bytestream2_peek_byte(&gb->b);

    gb->offset += 4;
    if ((gb->offset % 8) == 0) {
        x >>= 4;
        bytestream2_skip(&gb->b, 1);
    }

    return x & 0xF;
}

static int read_quantizer(BitContext *gb)
{
    int qi = read_nibble(gb);

    if (qi == 0xF)
        return ESCAPE_SEQUENCE_STOP_NOISEFILL;
    if (qi == 0xE)
        qi += read_nibble(gb);
    if (qi == 0xE + 0xF)
        return ESCAPE_SEQUENCE_STOP;

    return qi;
}

static float expand_quantizer(int qi)
{
    return 0x1.0p-31f * ((1u<<(31-5)) >> qi);
}

static int decode_coeffs(BitContext *gb, float *coeffs, int N)
{
    int n, v;

    v = read_quantizer(gb);
    if (v == ESCAPE_SEQUENCE_STOP) {
        for (int n = 0; n < N; n++)
            coeffs[n] = 0.0f;

        return 0;
    }

    float quant = expand_quantizer(v);
    for (;;) {
        v = read_nibble(gb);
        if (v != 0x0 && v != 0x1 && v != 0x8 && v != 0xF) {
            v = (v^0x8) - 0x8;
            v = (v < 0) ? (-v*v) : (+v*v);
            *coeffs++ = v * quant;

            N--;
            if (N <= 0)
                break;

            continue;
        }

        if (v == 0x0) {
            n = read_nibble(gb) + 1;
            if (n > N)
                return AVERROR_INVALIDDATA;
            N -= n;

            while (n-- > 0)
                *coeffs++ = 0.0f;

            if (N <= 0)
                break;

            continue;
        }

        if (v == 0x1) {
            n  = read_nibble(gb);
            n  = read_nibble(gb) | (n << 4);
            n += 33;
            if (n > N)
                return AVERROR_INVALIDDATA;
            N -= n;

            while (n-- > 0)
                *coeffs++ = 0.0f;

            if (N <= 0)
                break;

            continue;
        }

        if (v == 0x8) {
            n  = read_nibble(gb);
            n  = read_nibble(gb) | (n<<4);
            v  = read_nibble(gb);
            n  = (v&1) | (n<<1);
            v  = (v>>1) + 1;
            n += 16;
            if (n > N)
                return AVERROR_INVALIDDATA;
            N -= n;

            float p = (v*v) * quant * 0.25f;
            while (n-- > 0) {
                if (update_random_seed() & 0x80000000)
                    p = -p;

                *coeffs++ = p;
            }

            if (N <= 0)
                break;

            continue;
        }

        v = read_quantizer(gb);
        if (v >= 0) {
            quant = expand_quantizer(v);
            continue;
        }

        if (v == ESCAPE_SEQUENCE_STOP_NOISEFILL) {
            v = read_nibble(gb) + 1;
            n = read_nibble(gb);
            n = read_nibble(gb) | (n<<4);
            float p = (v*v) * quant * (1.0f/16);
            float r = 1.0f + (n*n)*-0x1.0p-19f;
            while (N-- > 0) {
                if (update_random_seed() & 0x80000000)
                    p = -p;

                *coeffs++ = p;
                p *= r;
            }

            break;
        }

        if (v == ESCAPE_SEQUENCE_STOP) {
            while (N-- > 0)
                *coeffs++ = 0.0f;

            break;
        }
    }

    return 0;
}

static uint16_t ULCi_SubBlockDecimationPattern(int window_ctrl)
{
    static const uint16_t patterns[] = {
        0x0000 | 0x0000, 0x0000 | 0x0008, 0x0011 | 0x0008, 0x0011 | 0x0080,
        0x0122 | 0x0008, 0x0122 | 0x0080, 0x0221 | 0x0080, 0x0221 | 0x0800,
        0x1233 | 0x0008, 0x1233 | 0x0080, 0x1332 | 0x0080, 0x1332 | 0x0800,
        0x2331 | 0x0080, 0x2331 | 0x0800, 0x3321 | 0x0800, 0x3321 | 0x8000,
    };

    return patterns[window_ctrl >> 4];
}

static void run_imdct(ULCContext *ctx, float *out, const float *in,
                      float *buf_lap, float *buf_tmp,
                      const int N, const int overlap)
{
    const int idx = av_ceil_log2(N)-8;
    const int N_2 = N/2;
    const float *lap = buf_lap + N_2;
    const float *tmp = buf_tmp + N_2;
    float *out_lo = out;
    float *out_hi = out + N;

    ctx->tx_fn[idx](ctx->tx_ctx[idx], buf_tmp, (void *)in, sizeof(float));
    for (int i = 0; i < N_2; i++)
        FFSWAP(float, buf_tmp[i], buf_tmp[N-i-1]);

    for (int i = 0; i < (N-overlap)/2; i++) {
        float a = *--lap;
        float b = *tmp++;
        *out_lo++ = a;
        *--out_hi = b;
    }

    if (overlap > 0) {
        const int widx = av_ceil_log2(overlap)-1;
        const float *win = ctx->win[widx] + overlap/2;

        for (int i = -(overlap/2), j = overlap/2-1; i < 0; i++, j--) {
            float a = *--lap;
            float b = *tmp++;
            float wi = win[i];
            float wj = win[j];
            *out_lo++ = wj*a - wi*b;
            *--out_hi = wi*a + wj*b;
        }
    }

    memcpy(buf_lap, buf_tmp, N_2 * sizeof(float));
}

static int decode_block(AVCodecContext *avctx, AVFrame *out)
{
    ULCContext *s = avctx->priv_data;
    BitContext *gb = &s->gb;
    const int nb_channels = avctx->ch_layout.nb_channels;
    const int nb_samples = s->nb_samples;
    int last_subblock_size = 0;
    int window_ctrl;
    int ret;

    {
        window_ctrl = read_nibble(gb);
        if (window_ctrl & 0x8)
            window_ctrl |= read_nibble(gb) << 4;
        else
            window_ctrl |= 1 << 4;
    }

    for (int ch = 0; ch < nb_channels; ch++) {
        ChannelContext *cc = &s->cc[ch];
        last_subblock_size = s->last_subblock_size;
        float *dst = (float *)out->extended_data[ch];
        float *src = cc->buffer;
        float *lap = cc->inv_lap;
        float *temp = cc->temp;

        uint16_t decimation = ULCi_SubBlockDecimationPattern(window_ctrl);

        do {
            int subblock_size = nb_samples >> (decimation & 0x7);
            ret = decode_coeffs(gb, src, subblock_size);
            if (ret < 0)
                return ret;

            int overlap = subblock_size;
            if (decimation & 0x8)
                overlap >>= window_ctrl & 0x7;
            if (overlap > last_subblock_size)
                overlap = last_subblock_size;
            last_subblock_size = subblock_size;

            if (subblock_size == nb_samples) {
                run_imdct(s, dst, src, lap, temp, subblock_size, overlap);

                break;
            }

            float *dec = temp + subblock_size;
            run_imdct(s, dec, src, lap, temp, subblock_size, overlap);

            int available = (nb_samples - subblock_size) / 2;
            float *lap_dst = lap + nb_samples/2;
            const float *lap_src = lap_dst;

            if (subblock_size <= available) {
                for (int n = 0; n < subblock_size; n++)
                    *dst++ = *--lap_src;
                for (int n = subblock_size; n < available; n++)
                    *--lap_dst = *--lap_src;
                for (int n = 0; n < subblock_size; n++)
                    *--lap_dst = *dec++;
            } else {
                for (int n = 0; n < available; n++)
                    *dst++ = *--lap_src;
                for (int n = available; n < subblock_size; n++)
                    *dst++ = *dec++;
                for (int n = 0; n < available; n++)
                    *--lap_dst = *dec++;
            }
        } while (decimation >>= 4);
    }

    for (int ch = 1; ch < nb_channels; ch += 2) {
        float *prev = (float *)out->extended_data[ch-1];
        float *cur = (float *)out->extended_data[ch];

        for (int n = 0; n < nb_samples; n++) {
            float a = prev[n];
            float b = cur[n];

            prev[n] = a+b;
            cur[n]  = a-b;
        }
    }

    s->last_subblock_size = last_subblock_size;

    return 0;
}

static int ulc_decode(AVCodecContext *avctx, AVFrame *frame,
                      int *got_frame_ptr, AVPacket *pkt)
{
    ULCContext *s = avctx->priv_data;
    BitContext *gb = &s->gb;
    int buf_size, input_buf_size;
    int ret = AVERROR_INVALIDDATA;
    const uint8_t *buf;

    if ((!pkt->size && !s->bitstream_size) || s->nb_samples == 0) {
        *got_frame_ptr = 0;
        return pkt->size;
    }

    buf_size = FFMIN(pkt->size, s->max_framesize - s->bitstream_size);
    input_buf_size = buf_size;
    if (s->bitstream_index + s->bitstream_size + buf_size + AV_INPUT_BUFFER_PADDING_SIZE > s->max_framesize) {
        memmove(s->bitstream, &s->bitstream[s->bitstream_index], s->bitstream_size);
        s->bitstream_index = 0;
    }
    if (pkt->data)
        memcpy(&s->bitstream[s->bitstream_index + s->bitstream_size], pkt->data, buf_size);
    buf                = &s->bitstream[s->bitstream_index];
    buf_size          += s->bitstream_size;
    s->bitstream_size  = buf_size;
    if (buf_size < s->max_framesize && pkt->data) {
        *got_frame_ptr = 0;
        return input_buf_size;
    }

    frame->nb_samples = s->nb_samples;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        goto fail;

    bytestream2_init(&gb->b, buf, buf_size);
    gb->offset = 0;;

    ret = decode_block(avctx, frame);
    if (ret < 0)
        return ret;

    ret = (gb->offset + 7) >> 3;

    if (ret > buf_size) {
fail:
        s->bitstream_size = 0;
        s->bitstream_index = 0;
        return AVERROR_INVALIDDATA;
    }

    *got_frame_ptr = 1;

    if (s->bitstream_size) {
        s->bitstream_index += ret;
        s->bitstream_size  -= ret;
        return input_buf_size;
    }
    return ret;
}

const FFCodec ff_ulc_decoder = {
    .p.name           = "ulc",
    CODEC_LONG_NAME("ULC (Ultra-Low Complexity Codec)"),
    .p.type           = AVMEDIA_TYPE_AUDIO,
    .p.id             = AV_CODEC_ID_ULC,
    .priv_data_size   = sizeof(ULCContext),
    .init             = ulc_init,
    FF_CODEC_DECODE_CB(ulc_decode),
    .close            = ulc_close,
    .p.capabilities   = AV_CODEC_CAP_DELAY |
                        AV_CODEC_CAP_DR1,
    .caps_internal    = FF_CODEC_CAP_INIT_CLEANUP,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP),
};
