/*
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

#include "libavutil/crc.h"
#include "libavutil/float_dsp.h"
#include "libavutil/mem.h"
#include "libavutil/mem_internal.h"
#include "libavutil/tx.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#define BITSTREAM_READER_LE
#include "get_bits.h"

#define RELIC_MAX_SCALES  6
#define RELIC_BASE_SCALE  10.0f
#define RELIC_FREQUENCY_MASKING_FACTOR  1.0f
#define RELIC_CRITICAL_BAND_COUNT  27
#define RELIC_SIZE_LOW  128
#define RELIC_SIZE_MID  256
#define RELIC_SIZE_HIGH 512
#define RELIC_MAX_SIZE  RELIC_SIZE_HIGH
#define RELIC_MAX_FREQ  (RELIC_MAX_SIZE / 2)
#define RELIC_MIN_BITRATE  256
#define RELIC_MAX_BITRATE  2048
#define RELIC_SAMPLES_PER_FRAME 512

typedef struct ChannelContext {
    uint8_t exponents[RELIC_MAX_FREQ];
    DECLARE_ALIGNED(32, float, freq1)[RELIC_MAX_SIZE];
    DECLARE_ALIGNED(32, float, freq2)[RELIC_MAX_SIZE];
    DECLARE_ALIGNED(32, float, wave_prv)[RELIC_MAX_SIZE];
    DECLARE_ALIGNED(32, float, wave_tmp)[RELIC_MAX_SIZE];
} ChannelContext;

typedef struct RelicContext {
    GetBitContext gb;

    int codec_rate;
    int frame_size;
    int bitrate;
    int freq_size;

    ChannelContext ch[2];
    float scales[RELIC_MAX_SCALES];
    float win[RELIC_SIZE_HIGH];

    av_tx_fn      tx_fn;
    AVTXContext  *tx_ctx;
} RelicContext;

static void init_dequantization(float *scales)
{
    scales[0] = RELIC_BASE_SCALE;
    for (int i = 1; i < RELIC_MAX_SCALES; i++)
        scales[i] = scales[i - 1] * scales[0];

    for (int i = 0; i < RELIC_MAX_SCALES; i++)
        scales[i] = RELIC_FREQUENCY_MASKING_FACTOR / (float) ((1 << (i + 1)) - 1) * scales[i];
}

static void init_window(float *window, const int N)
{
    const float scale = 1.f / N;

    for (int i = 0; i < N; i++)
        window[i] = sinf(i * M_PIf * scale);
}

static int init_relic(AVCodecContext *avctx,
                      const int bitrate,
                      const int codec_rate)
{
    RelicContext *c = avctx->priv_data;
    float scale = -M_SQRT1_2f / (1 << 17);
    int ret;

    if (avctx->ch_layout.nb_channels <= 0 || avctx->ch_layout.nb_channels > FF_ARRAY_ELEMS(c->ch))
        return AVERROR(EINVAL);

    if (bitrate < RELIC_MIN_BITRATE || bitrate > RELIC_MAX_BITRATE)
        return AVERROR_INVALIDDATA;

    if (codec_rate < 22050) {
        c->freq_size = RELIC_SIZE_LOW;
    } else if (codec_rate == 22050) {
        c->freq_size = RELIC_SIZE_MID;
    } else if (codec_rate > 22050) {
        c->freq_size = RELIC_SIZE_HIGH;
    }

    c->frame_size = bitrate >> 3;
    init_window(c->win, RELIC_SIZE_HIGH);
    init_dequantization(c->scales);

    ret = av_tx_init(&c->tx_ctx, &c->tx_fn, AV_TX_FLOAT_MDCT, 1, RELIC_SIZE_HIGH>>1, &scale, AV_TX_FULL_IMDCT);
    if (ret < 0)
        return ret;

    return 0;
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    RelicContext *c = avctx->priv_data;

    avctx->sample_fmt = AV_SAMPLE_FMT_FLTP;
    avctx->sample_rate = 44100;

    if (!avctx->extradata || avctx->extradata_size < 6)
        return AVERROR_INVALIDDATA;

    c->bitrate = AV_RB16(avctx->extradata);
    c->codec_rate = AV_RB32(avctx->extradata+2);

    return init_relic(avctx, c->bitrate, c->codec_rate);
}

static const int16_t critical_band_data[RELIC_CRITICAL_BAND_COUNT] =
{
    0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 15, 17, 20, 23, 27,
    31, 37, 43, 51, 62, 74, 89, 110, 139, 180, 256
};

static int unpack_channel(AVCodecContext *avctx, AVPacket *avpkt, const int ch)
{
    uint8_t flags, cb_bits, ev_bits, ei_bits, qv_bits, ev, move;
    RelicContext *c = avctx->priv_data;
    GetBitContext *gb = &c->gb;
    const int freq_size = c->freq_size;
    const int freq_half = freq_size >> 1;
    uint8_t *exponents = c->ch[ch].exponents;
    float *freq1 = c->ch[ch].freq1;
    float *freq2 = c->ch[ch].freq2;
    const float *scales = c->scales;
    int qv, ret, sgn;

    if (avpkt->size < 256 * (ch+1))
        return AVERROR_INVALIDDATA;

    ret = init_get_bits8(&c->gb, avpkt->data + 256 * ch, 256);
    if (ret < 0)
        return ret;

    memset(freq1, 0, sizeof(c->ch[0].freq1));
    memset(freq2, 0, sizeof(c->ch[0].freq2));

    flags   = get_bits(gb, 2);
    cb_bits = get_bits(gb, 3);
    ev_bits = get_bits(gb, 2);
    ei_bits = get_bits(gb, 4);

    if ((flags & 1) == 1)
        memset(exponents, 0, RELIC_MAX_FREQ);

    if (cb_bits > 0 && ev_bits > 0) {
        int pos = 0;
        for (int i = 0; i < RELIC_CRITICAL_BAND_COUNT - 1; i++) {
            move = get_bits(gb, cb_bits);

            if (i > 0 && move == 0)
                break;
            pos += move;

            ev = get_bits(gb, ev_bits);

            if (pos + 1 >= FF_ARRAY_ELEMS(critical_band_data))
                return AVERROR_INVALIDDATA;

            for (int j = critical_band_data[pos]; j < critical_band_data[pos + 1]; j++)
                exponents[j] = ev;
        }
    }

    if (freq_half > 0 && ei_bits > 0) {
        int pos = 0;

        for (int i = 0; i < RELIC_MAX_FREQ; i++) {
            move = get_bits(gb, ei_bits);
            if (i > 0 && move == 0)
                break;
            pos += move;

            if (pos >= RELIC_MAX_FREQ)
                return AVERROR_INVALIDDATA;

            qv_bits = exponents[pos];
            qv = get_bits(gb, qv_bits + 1);
            sgn = get_bits1(gb);
            if (sgn)
                qv = -qv;

            if (qv != 0 && pos < freq_half && qv_bits < 6)
                freq1[pos] = qv * scales[qv_bits];
        }

        if ((flags & 2) == 2) {
            memcpy(freq2, freq1, RELIC_MAX_FREQ * sizeof(float));
        } else {
            pos = 0;
            for (int i = 0; i < RELIC_MAX_FREQ; i++) {
                if (get_bits_left(gb) <= 0)
                    break;

                move = get_bits(gb, ei_bits);

                if (i > 0 && move == 0)
                    break;
                pos += move;

                if (pos >= RELIC_MAX_FREQ)
                    return AVERROR_INVALIDDATA;

                qv_bits = exponents[pos];
                qv = get_bits(gb, qv_bits + 1);
                sgn = get_bits1(gb);
                if (sgn)
                    qv = -qv;

                if (qv != 0 && pos < freq_half && qv_bits < 6)
                    freq2[pos] = qv * scales[qv_bits];
            }
        }
    }

    return 0;
}

static void decode_channel(AVCodecContext *avctx, AVFrame *frame, const int ch)
{
    RelicContext *c = avctx->priv_data;
    const float *window = c->win;
    const float *freq1 = c->ch[ch].freq1;
    const float *freq2 = c->ch[ch].freq2;
    float *wave_prv = c->ch[ch].wave_prv;
    float *wave_cur = (float *)frame->extended_data[ch];
    float *wave_tmp = c->ch[ch].wave_tmp;
    const int dct_half = RELIC_SAMPLES_PER_FRAME >> 1;

    memcpy(wave_cur, wave_prv, RELIC_MAX_SIZE * sizeof(float));

    c->tx_fn(c->tx_ctx, wave_tmp, (void *)freq1, sizeof(float));
    c->tx_fn(c->tx_ctx, wave_prv, (void *)freq2, sizeof(float));

    for (int i = 0; i < dct_half; i++) {
        wave_cur[dct_half + i] = wave_tmp[i] * window[i] + wave_cur[dct_half + i] * window[dct_half + i];
        wave_prv[i]            = wave_prv[i] * window[i] + wave_tmp[dct_half + i] * window[dct_half + i];
    }
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame_ptr, AVPacket *avpkt)
{
    int ret;

    for (int ch = 0; ch < avctx->ch_layout.nb_channels; ch++) {
        ret = unpack_channel(avctx, avpkt, ch);
        if (ret < 0)
            return ret;
    }

    frame->nb_samples = 512;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    for (int ch = 0; ch < avctx->ch_layout.nb_channels; ch++)
        decode_channel(avctx, frame, ch);

    *got_frame_ptr = 1;

    return avpkt->size;
}

static av_cold int decode_close(AVCodecContext *avctx)
{
    RelicContext *c = avctx->priv_data;

    av_tx_uninit(&c->tx_ctx);

    return 0;
}

static av_cold void decode_flush(AVCodecContext *avctx)
{
    RelicContext *c = avctx->priv_data;

    for (int ch = 0; ch < avctx->ch_layout.nb_channels; ch++) {
        memset(c->ch[ch].wave_prv, 0, sizeof(c->ch[0].wave_prv));
        memset(c->ch[ch].wave_tmp, 0, sizeof(c->ch[0].wave_tmp));
    }
}

const FFCodec ff_relic_decoder = {
    .p.name         = "relic",
    CODEC_LONG_NAME("Relic"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_RELIC,
    .priv_data_size = sizeof(RelicContext),
    .init           = decode_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .flush          = decode_flush,
    .close          = decode_close,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP),
};
