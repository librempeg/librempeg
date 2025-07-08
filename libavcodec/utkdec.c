/*
 * Electronic Arts MicroTalk decoder
 * Copyright (c) 2017 Andrew D'Addesio
 * Copyright (c) 2017 Peter Ross
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

/**
 * @file
 * Electronic Arts MicroTalk decoder
 * Documentation: http://wiki.niotso.org/UTK
 */

#include "config_components.h"

#include "codec_internal.h"
#include "decode.h"

#include "avcodec.h"
#include "bytestream.h"
#define BITSTREAM_READER_LE
#include "get_bits.h"
#include "internal.h"
#include "libavutil/thread.h"
#include "utkdata.h"
#include "vlc.h"

const VLCElem * vlc[NB_MODELS];
static VLCElem table_data[512];

static void utk_init_static_data(void)
{
    VLCInitState state = VLC_INIT_STATE(table_data);

    for (int i = 0; i < NB_MODELS; i++)
        vlc[i] = ff_vlc_init_tables_sparse(&state, 8, 15,
                 utk_vlc_bits[i], 1, 1,
                 utk_vlc_codes[i], 1, 1,
                 utk_vlc_cmds[i], 1, 1, VLC_INIT_LE);
}

#define MAX_FRAME_SIZE 8192
#define FRAME_SAMPLES 432

typedef struct UTKContext {
    int reduced_bw;
    int multipulse_thresh;
    float fixed_gains[64];
    float rc[12];
    float synth_history[12];
#define ADAPT_CB_SAMPLES 324
    float buffer[ADAPT_CB_SAMPLES + 432];
    uint8_t bitstream[MAX_FRAME_SIZE + AV_INPUT_BUFFER_PADDING_SIZE];
    int bitstream_index;
    int bitstream_size;
    int skip;
    int parse_header;
} UTKContext;

static av_cold int decode_init(AVCodecContext *avctx)
{
    static AVOnce init_static_once = AV_ONCE_INIT;

    if (avctx->ch_layout.nb_channels != 1) {
        avpriv_request_sample(avctx, "Channel count %d", avctx->ch_layout.nb_channels);
        return AVERROR_PATCHWELCOME;
    }

    avctx->sample_fmt = AV_SAMPLE_FMT_FLTP;

    ff_thread_once(&init_static_once, utk_init_static_data);
    return 0;
}

static void utk_parse_header(UTKContext *ctx, GetBitContext *gb)
{
    float multiplier;

    ctx->reduced_bw = get_bits(gb, 1);
    ctx->multipulse_thresh = 32 - get_bits(gb, 4);
    ctx->fixed_gains[0] = 8.0f * (1 + get_bits(gb, 4));
    multiplier = 1.04f + get_bits(gb, 6)*0.001f;

    for (int i = 1; i < 64; i++)
        ctx->fixed_gains[i] = ctx->fixed_gains[i-1] * multiplier;
}

static void utk_decode_excitation(UTKContext *ctx, GetBitContext *gb, int use_multipulse, float *out, int stride)
{
    int i;

    if (use_multipulse) {
        /* multi-pulse model: n pulses are coded explicitly; the rest are zero */
        int model, cmd;
        model = MDL_NORMAL;
        i = 0;
        while (i < 108) {
            cmd = get_vlc2(gb, vlc[model], 8, 1);
            model = utk_commands[cmd].next_model;
            if (cmd > 3) {
                /* insert a pulse with magnitude <= 6.0f */
                out[i] = utk_commands[cmd].pulse_value;
                i += stride;
            } else if (cmd > 1) {
                /* insert between 7 and 70 zeros */
                int count = 7 + get_bits(gb, 6);
                if (i + count * stride > 108)
                    count = (108 - i)/stride;

                while (count > 0) {
                    out[i] = 0.0f;
                    i += stride;
                    count--;
                }
            } else {
                /* insert a pulse with magnitude >= 7.0f */
                int x = 7;

                while (get_bits(gb, 1))
                    x++;

                if (!get_bits(gb, 1))
                    x *= -1;

                out[i] = x;
                i += stride;
            }
        }
    } else {
        /* RELP model: entire residual (excitation) signal is coded explicitly */
        i = 0;
        while (i < 108) {
            if (!get_bits(gb, 1))
                out[i] = 0.0f;
            else if (!get_bits(gb, 1))
                out[i] = -2.0f;
            else
                out[i] = 2.0f;

            i += stride;
        }
    }
}

static void rc_to_lpc(const float *rc, float *lpc)
{
    float tmp1[12];
    float tmp2[12];

    for (int i = 10; i >= 0; i--)
        tmp2[1+i] = rc[i];

    tmp2[0] = 1.0f;

    for (int i = 0; i < 12; i++) {
        float x = -tmp2[11] * rc[11];

        for (int j = 10; j >= 0; j--) {
            x -= tmp2[j] * rc[j];
            tmp2[j+1] = x * rc[j] + tmp2[j];
        }

        tmp1[i] = tmp2[0] = x;

        for (int j = 0; j < i; j++)
            x -= tmp1[i-1-j] * lpc[j];

        lpc[i] = x;
    }
}

static void utk_lp_synthesis_filter(UTKContext *ctx, int offset, int num_blocks)
{
    float lpc[12];
    float *ptr = &ctx->buffer[ADAPT_CB_SAMPLES + offset];

    rc_to_lpc(ctx->rc, lpc);

    for (int i = 0; i < num_blocks; i++) {
        for (int j = 0; j < 12; j++) {
            int k;
            float x = *ptr;

            for (k = 0; k < j; k++)
                x += lpc[k] * ctx->synth_history[k-j+12];
            for (; k < 12; k++)
                x += lpc[k] * ctx->synth_history[k-j];

            ctx->synth_history[11-j] = x;
            *ptr++ = x;
        }
    }
}

static void utk_decode_frame(UTKContext *ctx, GetBitContext *gb, int *parse_header)
{
    int use_multipulse = 0;
    float excitation[5+108+5];
    float rc_delta[12];

    if (!parse_header[0]) {
        utk_parse_header(ctx, gb);
        parse_header[0] = 1;
    }

    memset(&excitation[0], 0, 5*sizeof(float));
    memset(&excitation[5+108], 0, 5*sizeof(float));

    /* read the reflection coefficients */
    for (int i = 0; i < 12; i++) {
        int idx;
        if (i == 0) {
            idx = get_bits(gb, 6);
            if (idx < ctx->multipulse_thresh)
                use_multipulse = 1;
        } else if (i < 4) {
            idx = get_bits(gb, 6);
        } else {
            idx = 16 + get_bits(gb, 5);
        }

        rc_delta[i] = (utk_rc_table[idx] - ctx->rc[i])*0.25f;
    }

    /* decode four subframes */
    for (int i = 0; i < 4; i++) {
        int pitch_lag = get_bits(gb, 8);
        float pitch_gain = get_bits(gb, 4)/15.0f;
        float fixed_gain = ctx->fixed_gains[get_bits(gb, 6)];

        if (!ctx->reduced_bw) {
            utk_decode_excitation(ctx, gb, use_multipulse, &excitation[5], 1);
        } else {
            /* residual (excitation) signal is encoded at reduced bandwidth */
            int align = get_bits(gb, 1);
            int zero = get_bits(gb, 1);

            utk_decode_excitation(ctx, gb, use_multipulse, &excitation[5+align], 2);

            if (zero) {
                /* fill the remaining samples with zero
                ** (spectrum is duplicated into high frequencies) */
                for (int j = 0; j < 54; j++)
                    excitation[5+(1-align)+2*j] = 0.0f;
            } else {
                /* interpolate the remaining samples
                ** (spectrum is low-pass filtered) */
                float *ptr = &excitation[5+(1-align)];
                for (int j = 0; j < 108; j += 2)
                    ptr[j] =   ptr[j-5] * 0.01803267933428287506103515625f
                             - ptr[j-3] * 0.114591561257839202880859375f
                             + ptr[j-1] * 0.597385942935943603515625f
                             + ptr[j+1] * 0.597385942935943603515625f
                             - ptr[j+3] * 0.114591561257839202880859375f
                             + ptr[j+5] * 0.01803267933428287506103515625f;

                /* scale by 0.5f to give the sinc impulse response unit energy */
                fixed_gain *= 0.5f;
            }
        }

        for (int j = 0; j < 108; j++) {
            int idx = 108*i+216-pitch_lag+j;
            ctx->buffer[ADAPT_CB_SAMPLES+108*i+j] = fixed_gain * excitation[5+j]
                                                  + (idx >= 0 ? pitch_gain * ctx->buffer[idx] : 0);
        }
    }

    memcpy(ctx->buffer, ctx->buffer + ADAPT_CB_SAMPLES+108, ADAPT_CB_SAMPLES * sizeof(float));

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 12; j++)
            ctx->rc[j] += rc_delta[j];
        utk_lp_synthesis_filter(ctx, 12*i, i < 3 ? 1 : 33);
    }
}

static int utk_r3_decode_frame(UTKContext *ctx, GetBitContext *gb, int *parse_header, const uint8_t *start)
{
    int pcm_data_present = (get_bits(gb, 8) == 0xee);
    const uint8_t *ptr;
    GetByteContext gb2;
    int ret;

    utk_decode_frame(ctx, gb, parse_header);

    ptr = align_get_bits(gb);
    bytestream2_init(&gb2, ptr, get_bits_left(gb) / 8);

    if (pcm_data_present) {
        /* Overwrite n samples at a given offset in the decoded frame with
        ** raw PCM data. */
        int offset = sign_extend(bytestream2_get_be16(&gb2), 16);
        int count = sign_extend(bytestream2_get_be16(&gb2), 16);

        if (offset < 0 || offset > 432 || count < 0 || count > 432 - offset)
            return AVERROR_INVALIDDATA;

        for (int i = 0; i < count; i++)
            ctx->buffer[ADAPT_CB_SAMPLES + offset+i] = sign_extend(bytestream2_get_be16(&gb2), 16);

        if ((ret = init_get_bits8(gb, gb2.buffer, bytestream2_get_bytes_left(&gb2))) < 0)
            return ret;
    }

    return 0;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame_ptr, AVPacket *avpkt)
{
    int ret, n, input_buf_size, buf_size;
    UTKContext *s = avctx->priv_data;
    GetBitContext gbc, *gb = &gbc;
    const uint8_t *buf;
    float *dst;

    if (!avpkt->size && !s->bitstream_size) {
        *got_frame_ptr = 0;
        return avpkt->size;
    }

    if (avctx->extradata_size > 0 && avctx->extradata[0] == 0) {
        buf_size = input_buf_size = FFMIN(avpkt->size, MAX_FRAME_SIZE - s->bitstream_size);
        if (s->bitstream_index + s->bitstream_size + buf_size + AV_INPUT_BUFFER_PADDING_SIZE > MAX_FRAME_SIZE) {
            memmove(s->bitstream, &s->bitstream[s->bitstream_index], s->bitstream_size);
            s->bitstream_index = 0;
        }
        if (avpkt->data)
            memcpy(&s->bitstream[s->bitstream_index + s->bitstream_size], avpkt->data, buf_size);
        buf      = &s->bitstream[s->bitstream_index];
        buf_size += s->bitstream_size;
        s->bitstream_size  = buf_size;
        if (buf_size < MAX_FRAME_SIZE && avpkt->data) {
            *got_frame_ptr = 0;
            return input_buf_size;
        }
    } else {
        buf = avpkt->data;
        buf_size = avpkt->size;
    }

    frame->nb_samples = avpkt->duration ? avpkt->duration : FRAME_SAMPLES;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        goto fail;
    dst = (float *)frame->data[0];

    if ((ret = init_get_bits8(gb, buf, buf_size)) < 0)
        goto fail;
    skip_bits(gb, s->skip);

    for (int i = 0; i < frame->nb_samples; i += FRAME_SAMPLES) {
        const int nb_samples = FFMIN(FRAME_SAMPLES, frame->nb_samples - i);

        if (avctx->codec_id == AV_CODEC_ID_UTK)
            utk_decode_frame(s, gb, &s->parse_header);
        else if ((ret = utk_r3_decode_frame(s, gb, &s->parse_header, avpkt->data)) < 0)
            goto fail;

        for (int j = 0; j < nb_samples; j++)
            dst[i+j] = s->buffer[ADAPT_CB_SAMPLES+j] / 32768;
    }

    if (avctx->extradata_size > 0 && avctx->extradata[0] == 0) {
        s->skip = get_bits_count(gb) - 8 * (get_bits_count(gb) / 8);
        n = get_bits_count(gb) / 8;
    } else {
        s->skip = 0;
        n = avpkt->size;
    }

    if (n > buf_size) {
fail:
        s->bitstream_size = 0;
        s->bitstream_index = 0;
        return AVERROR_INVALIDDATA;
    }

    *got_frame_ptr = 1;

    if (s->bitstream_size) {
        s->bitstream_index += n;
        s->bitstream_size  -= n;
        return input_buf_size;
    }

    return n;
}

static av_cold void decode_flush(AVCodecContext *avctx)
{
    UTKContext *s = avctx->priv_data;

    s->parse_header = 0;
    s->bitstream_size = 0;
    s->bitstream_index = 0;
    memset(s->rc, 0, sizeof(s->rc));
    memset(s->bitstream, 0, sizeof(s->bitstream));
    memset(s->synth_history, 0, sizeof(s->synth_history));
    memset(s->buffer, 0, sizeof(s->buffer));
}

const FFCodec ff_utk_decoder = {
    .p.name         = "utk",
    CODEC_LONG_NAME("Electronic Arts MicroTalk"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_UTK,
    .priv_data_size = sizeof(UTKContext),
    .init           = decode_init,
    .flush          = decode_flush,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_DELAY,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP),
};

const FFCodec ff_utk_r3_decoder = {
    .p.name         = "utk_r3",
    CODEC_LONG_NAME("Electronic Arts MicroTalk Revision 3"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_UTK_R3,
    .priv_data_size = sizeof(UTKContext),
    .init           = decode_init,
    .flush          = decode_flush,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP),
};
