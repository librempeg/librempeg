/*
 * DWVW decoder
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
 * DWVW decoder
 */

#include "libavutil/internal.h"
#include "avcodec.h"
#include "codec_id.h"
#include "codec_internal.h"
#include "decode.h"
#include "get_bits.h"
#include "unary.h"

typedef struct DWVWContext {
    int skip_bits;
    int bit_width, dwm_maxsize, max_delta, span;
    int last_delta_width, last_sample;
    uint8_t prev[8];
    int prev_size;
    GetBitContext gbc;
} DWVWContext;

static int dwvw_decode_samples(AVCodecContext *avctx, AVFrame *frame, AVPacket *avpkt)
{
    int32_t *dst = (int32_t *)frame->data[0];
    const int nb_samples = frame->nb_samples;
    int ret, delta, sample, delta_width;
    DWVWContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gbc;
    int delta_width_modifier;

    delta_width = s->last_delta_width;
    sample = s->last_sample;
    frame->nb_samples = 0;

    ret = init_get_bits8(gb, avpkt->data, avpkt->size);
    if (ret < 0)
        return ret;
    skip_bits(gb, s->skip_bits);

    for (int n = 0; n < nb_samples; n++) {
        if (get_bits_left(gb) < (s->dwm_maxsize+4+s->bit_width-1)) {
            frame->nb_samples = n;
            break;
        }

        delta_width_modifier = get_unary(gb, 1, s->dwm_maxsize);
        if (delta_width_modifier) {
            if (get_bits1(gb))
                delta_width_modifier = -delta_width_modifier;
        }

        delta_width += delta_width_modifier;
        delta_width += s->bit_width;
        delta_width = delta_width % s->bit_width;
        delta = 0;

        if (delta_width) {
            int delta_negative;

            delta = get_bitsz(gb, delta_width-1) | (1 << (delta_width - 1));
            delta_negative = get_bits1(gb);
            if (delta == s->max_delta - 1)
                delta += get_bits1(gb);

            if (delta_negative)
                delta = -delta;
        }

        sample += delta;

        if (sample >= s->max_delta)
            sample -= s->span;
        else if (sample < -s->max_delta)
            sample += s->span;

        dst[n] = sample * (1 << (32 - s->bit_width));

        if (get_bits_left(gb) <= 0) {
            frame->nb_samples = n;
            break;
        }
    }

    s->last_delta_width = delta_width;
    s->last_sample = sample;
    s->skip_bits = get_bits_count(gb) & 7;

    return get_bits_count(gb) >> 3;
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    DWVWContext *s = avctx->priv_data;

    s->bit_width   = avctx->bits_per_coded_sample;
    s->dwm_maxsize = s->bit_width / 2;
    s->max_delta   = 1 << (s->bit_width - 1);
    s->span        = 1 << s->bit_width;

    avctx->sample_fmt = AV_SAMPLE_FMT_S32;

    return 0;
}

static av_cold void decode_flush(AVCodecContext *avctx)
{
    DWVWContext *s = avctx->priv_data;

    s->last_sample = s->last_delta_width = 0;
    s->prev_size = s->skip_bits = 0;

    memset(s->prev, 0, sizeof(s->prev));
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame, AVPacket *avpkt)
{
    DWVWContext *s = avctx->priv_data;
    int ret;

    frame->nb_samples = avpkt->size * 8LL;
    if (frame->nb_samples <= 0) {
        *got_frame = 0;
        return avpkt->size;
    }

    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    if (s->prev_size > 0) {
        if ((ret = av_packet_make_writable(avpkt)) < 0)
            return ret;

        if ((ret = av_grow_packet(avpkt, s->prev_size)) < 0)
            return ret;

        memmove(avpkt->data+s->prev_size, avpkt->data, avpkt->size-s->prev_size);
        memcpy(avpkt->data, s->prev, s->prev_size);
    }

    ret = dwvw_decode_samples(avctx, frame, avpkt);

    *got_frame = frame->nb_samples > 0;

    if (ret >= 0) {
        s->prev_size = FFMIN(avpkt->size-ret, sizeof(s->prev));
        memcpy(s->prev, avpkt->data + ret, s->prev_size);
    }

    return avpkt->size;
}

const FFCodec ff_dwvw_decoder = {
    .p.name         = "dwvw",
    CODEC_LONG_NAME("DWVW (Delta Word Variable Width)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_DWVW,
    .priv_data_size = sizeof(DWVWContext),
    .init           = decode_init,
    .flush          = decode_flush,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
