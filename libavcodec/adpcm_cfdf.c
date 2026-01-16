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

#include <stdint.h>

#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "mathops.h"

typedef struct ADPCMCFDFContext {
    int left;
    int8_t prev_sample;
    uint8_t control_byte;
} ADPCMCFDFContext;

static av_cold int decode_init(AVCodecContext *avctx)
{
    avctx->sample_fmt = AV_SAMPLE_FMT_S16;
    avctx->ch_layout.nb_channels = 1;

    return 0;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame_ptr, AVPacket *avpkt)
{
    ADPCMCFDFContext *s = avctx->priv_data;
    uint8_t control_byte = s->control_byte;
    int8_t prev_sample = s->prev_sample;
    GetByteContext gbc, *gb = &gbc;
    int left = s->left;
    int16_t *dst;
    int ret, n;

    bytestream2_init(gb, avpkt->data, avpkt->size);

    frame->nb_samples = avpkt->size * 16;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    n = 0;
    dst = (int16_t *)frame->data[0];

    if (avpkt->pts == 0) {
        prev_sample = bytestream2_get_byte(gb);
        dst[n] = prev_sample * 256;
        left = 0;
        n++;
    }

    while (n < frame->nb_samples && bytestream2_get_bytes_left(gb) > 0) {
        if (left <= 0)
            control_byte = bytestream2_get_byte(gb);

        if (!(control_byte & 0x80)) {
            prev_sample = (int8_t)control_byte;
            dst[n++] = (prev_sample - 0x40) * 256;
        } else if (!(control_byte & 0x40)) {
            if (left <= 0)
                left = (control_byte & 0x3f) + 1;

            while (left > 0 && n < frame->nb_samples &&
                   bytestream2_get_bytes_left(gb) > 0) {
                uint8_t table_val = bytestream2_get_byte(gb);
                int8_t step_delta = (int8_t)table_val >> 4;
                int8_t index_delta = (int8_t)(table_val << 4) >> 4;
                int8_t step_sample = prev_sample + step_delta;
                int8_t index_sample = step_sample + index_delta;
                dst[n++] = (step_sample - 0x40) * 256;
                dst[n++] = (index_sample - 0x40) * 256;
                prev_sample = index_sample;
                left--;
            }
        } else {
            if (left <= 0)
                left = (control_byte & 0x3f) + 1;

            while (left > 0 && n < frame->nb_samples) {
                dst[n++] = (prev_sample - 0x40) * 256;
                left--;
            }
        }
    }

    s->control_byte = control_byte;
    s->prev_sample = prev_sample;
    s->left = left;

    frame->nb_samples = n;

    *got_frame_ptr = 1;

    return avpkt->size;
}

const FFCodec ff_adpcm_cfdf_decoder = {
    .p.name         = "adpcm_cfdf",
    CODEC_LONG_NAME("ADPCM Cyberflix DreamFactory CFDF"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ADPCM_CFDF,
    .priv_data_size = sizeof(ADPCMCFDFContext),
    .init           = decode_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
