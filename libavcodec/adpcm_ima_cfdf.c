/*
 * CyberFlix DreamFactory IMA ADPCM decoder
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
 * packets. This is the D5-only IMA variant: a 3-byte block header
 *       (s16 predictor + u8 step index), nibble data follows (low nibble
 *       first); sample 0 is the predictor and the final high nibble is
 *       padding. A step index above 0x58 produces no samples.
 */

#include <stdint.h>

#include "libavutil/intreadwrite.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "adpcm_data.h"

static av_cold int cfdf_ima_init(AVCodecContext *avctx)
{
    if (avctx->extradata_size != 1 || avctx->extradata[0] != 2)
        return AVERROR_INVALIDDATA;

    avctx->sample_fmt = AV_SAMPLE_FMT_S16;
    av_channel_layout_uninit(&avctx->ch_layout);
    avctx->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;

    return 0;
}

static int cfdf_ima_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                                int *got_frame_ptr, AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int size = avpkt->size;
    int16_t *dst;
    int nb, ret, n = 0;

    if (size < 4 || buf[2] > 0x58) {
        *got_frame_ptr = 0;
        return size;
    }
    nb = 2 * (size - 3);

    frame->nb_samples = nb;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;
    dst = (int16_t *)frame->data[0];

    {
        int hist       = (int16_t)AV_RL16(buf);
        int step_index = buf[2];
        int nibbles    = nb - 1;

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
    }

    frame->nb_samples = n;
    *got_frame_ptr = 1;

    return size;
}

const FFCodec ff_adpcm_ima_cfdf_decoder = {
    .p.name         = "adpcm_ima_cfdf",
    CODEC_LONG_NAME("ADPCM IMA Cyberflix DreamFactory"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ADPCM_IMA_CFDF,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .init           = cfdf_ima_init,
    FF_CODEC_DECODE_CB(cfdf_ima_decode_frame),
};
