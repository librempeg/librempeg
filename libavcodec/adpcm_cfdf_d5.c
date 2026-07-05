/*
 * CyberFlix DreamFactory v5/D5 (adpcm_cfdf_d5) audio decoder
 * Copyright (c) 2026
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

/*
 * DreamFactory 5 (.move / .trak) SOUN audio. The container stores audio as a
 * sequence of independent blocks; codec state resets at every block, so the
 * demuxer emits one block per packet and this decoder is stateless across
 * packets. Three per-stream variants exist, selected by extradata[0] (set by
 * the demuxer):
 *
 *   0 = v4.0-based ADPCM control stream (Mode I absolute / II nibble deltas /
 *       III RLE), DC offset 0x40, output scaled <<9. Block byte[0] seeds the
 *       running sample (not emitted), control bytes follow.
 *   1 = v4.1-based DPCM: per byte, high bit selects absolute vs 7-bit delta,
 *       accumulator reset to 0 per block, one sample per byte.
 *   2 = standard IMA ADPCM under DreamFactory framing: 3-byte block header
 *       (s16 predictor + u8 step index), nibble data follows (low nibble
 *       first); sample 0 of the block is the predictor emitted verbatim; a
 *       block whose step index exceeds 0x58 produces no samples.
 */

#include <stdint.h>

#include "libavutil/intreadwrite.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "adpcm_data.h"

enum {
    CFDF_D5_V40 = 0,
    CFDF_D5_V41 = 1,
    CFDF_D5_IMA = 2,
};

typedef struct CFDFD5Context {
    int variant;
} CFDFD5Context;

/* (sample - 0x40) << 9, truncated to 16 bits (matches the engine's 16-bit store) */
#define V40_S16(x) ((int16_t)((((int)(int8_t)(x)) - 0x40) * 512))

static av_cold int cfdf_d5_init(AVCodecContext *avctx)
{
    CFDFD5Context *s = avctx->priv_data;

    if (avctx->extradata_size < 1)
        return AVERROR_INVALIDDATA;
    s->variant = avctx->extradata[0];
    if (s->variant > CFDF_D5_IMA)
        return AVERROR_INVALIDDATA;

    avctx->sample_fmt = AV_SAMPLE_FMT_S16;
    av_channel_layout_uninit(&avctx->ch_layout);
    avctx->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;

    return 0;
}

/* Count the output samples of a v4.0 control-stream block (used to size the frame). */
static int cfdf_d5_v40_count(const uint8_t *buf, int size)
{
    int n = 0, p = 1; /* skip the seed byte */

    while (p < size) {
        uint8_t c = buf[p++];

        if (!(c & 0x80)) {                /* Mode I: absolute */
            n += 1;
        } else if (!(c & 0x40)) {         /* Mode II: nibble pairs, consumes count bytes */
            int cnt = (c & 0x3f) + 1;
            n += 2 * cnt;
            p += cnt;
        } else {                          /* Mode III: RLE */
            n += (c & 0x3f) + 1;
        }
    }

    return n;
}

static int cfdf_d5_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                                int *got_frame_ptr, AVPacket *avpkt)
{
    CFDFD5Context *s = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int size = avpkt->size;
    int16_t *dst;
    int nb, ret, n = 0;

    switch (s->variant) {
    case CFDF_D5_IMA:
        if (size < 3 || buf[2] > 0x58) {
            *got_frame_ptr = 0;
            return size;
        }
        nb = 1 + 2 * (size - 3);
        break;
    case CFDF_D5_V41:
        nb = size;
        break;
    default: /* CFDF_D5_V40 */
        if (size < 1) {
            *got_frame_ptr = 0;
            return size;
        }
        nb = cfdf_d5_v40_count(buf, size);
        break;
    }

    if (nb <= 0) {
        *got_frame_ptr = 0;
        return size;
    }

    frame->nb_samples = nb;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;
    dst = (int16_t *)frame->data[0];

    switch (s->variant) {
    case CFDF_D5_IMA: {
        int hist       = (int16_t)AV_RL16(buf);
        int step_index = buf[2];
        int nibbles    = 2 * (size - 3);

        dst[n++] = hist; /* sample 0: predictor verbatim */
        for (int k = 0; k < nibbles; k++) {
            int byte   = buf[3 + (k >> 1)];
            int nibble = (k & 1) ? (byte >> 4) : (byte & 0x0f);
            int step   = ff_adpcm_step_table[step_index];
            int diff   = step >> 3;

            if (nibble & 4) diff += step;
            if (nibble & 2) diff += step >> 1;
            if (nibble & 1) diff += step >> 2;
            if (nibble & 8) diff  = -diff;

            hist        = av_clip_int16(hist + diff);
            step_index  = av_clip(step_index + ff_adpcm_index_table[nibble], 0, 88);
            dst[n++]    = hist;
        }
        break;
    }
    case CFDF_D5_V41: {
        int16_t acc = 0; /* reset per block */

        for (int i = 0; i < size; i++) {
            int b = buf[i];

            if (b & 0x80) {
                acc = (int16_t)(b << 9);
            } else {
                int16_t delta = (int16_t)(b << 9);
                delta >>= 4;
                acc = (int16_t)(acc + delta);
            }
            dst[n++] = acc;
        }
        break;
    }
    default: { /* CFDF_D5_V40 */
        int prev = (int8_t)buf[0]; /* seed, not emitted */
        int p = 1;

        while (p < size && n < nb) {
            uint8_t c = buf[p++];

            if (!(c & 0x80)) {                 /* Mode I: absolute */
                prev = (int8_t)c;
                dst[n++] = V40_S16(prev);
            } else if (!(c & 0x40)) {          /* Mode II: nibble deltas */
                int cnt = (c & 0x3f) + 1;

                for (int j = 0; j < cnt && p < size && n < nb; j++) {
                    uint8_t tv      = buf[p++];
                    int step_delta  = (int8_t)tv >> 4;
                    int index_delta = (int8_t)(tv << 4) >> 4;
                    int s1          = (int8_t)(prev + step_delta);
                    int s2          = (int8_t)(s1 + index_delta);

                    dst[n++] = V40_S16(s1);
                    if (n < nb)
                        dst[n++] = V40_S16(s2);
                    prev = s2;
                }
            } else {                            /* Mode III: RLE */
                int cnt = (c & 0x3f) + 1;

                for (int j = 0; j < cnt && n < nb; j++)
                    dst[n++] = V40_S16(prev);
            }
        }
        break;
    }
    }

    frame->nb_samples = n;
    *got_frame_ptr = 1;

    return size;
}

const FFCodec ff_adpcm_cfdf_d5_decoder = {
    .p.name         = "adpcm_cfdf_d5",
    CODEC_LONG_NAME("ADPCM Cyberflix DreamFactory D5 (v5)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ADPCM_CFDF_D5,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .priv_data_size = sizeof(CFDFD5Context),
    .init           = cfdf_d5_init,
    FF_CODEC_DECODE_CB(cfdf_d5_decode_frame),
};

