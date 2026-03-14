/*
 * Copyright (c) 2008 Dave Coffin, dcoffin a cybercom o net
 *
 * RCA VOC decoder
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

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"

typedef struct RCAVOCContext {
    AVClass *class;

    int pred[24];
} RCAVOCContext;

static av_cold int rcavoc_decode_init(AVCodecContext *avctx)
{
    RCAVOCContext *rca = avctx->priv_data;

    rca->pred[1] = 544;
    avctx->sample_rate = 8000;
    avctx->ch_layout.nb_channels = 1;
    avctx->sample_fmt = AV_SAMPLE_FMT_S16;

    return 0;
}

static int hq_decode(AVCodecContext *avctx, const uint8_t *in, int16_t *out)
{
    RCAVOCContext *rca = avctx->priv_data;
    static const int16_t tab[2][16] = {
        { -2048,4,135,213,273,323,373,425,425,373,323,273,213,135,4,-2048 },
        { -12,18,41,64,112,198,355,1122,1122,355,198,112,64,41,18,-12 }
    };
    int *pred = rca->pred;

    for (int s = 0; s < 80; s++) {
        int nib = in[s >> 1] >> ((s & 1) << 2) & 15;
        int diff;

        if (nib == 15) {
            diff = 0;
        } else {
            int map = FFMAX((pred[1] >> 2) + tab[0][nib], 0);
            diff = ((map & 127) + 128) << ((map >> 7 & 15) + 1);
            if (nib & 8)
                diff = -diff;
        }
        out[s] = av_clip((pred[0] * 4) + (diff >> 6), -32768, 32767);
        pred[1] += tab[1][nib] + (-pred[1] >> 5);
        pred[1] = av_clip(pred[1], 544, 5120);

        for (int i = 24; --i > 12; )
            pred[i] = pred[i-1];

        pred[17] = diff;
        pred[15] = av_clip(diff + (pred[2] * 256), -8388608, 8388607);
        pred[12] = av_clip(diff + (pred[3] * 256), -8388608, 8388607);
        pred[5] = ((pred[5] * 32512 -
                    av_clip(pred[4] * (1 << 10), -8388608, 8388607) *
                    ((pred[12] ^ pred[13]) >> 31 | 1)) >> 15) +
                   (((pred[12] ^ pred[14]) >> 31 | 1) * (1 << 7));
        pred[5] = av_clip(pred[5], -12288, 12288);
        pred[4] = (pred[4] * 255 >> 8) + 192 *
                  ((pred[12] ^ pred[13]) >> 31 | 1);
        pred[4] = av_clip(pred[4], -(15360-pred[5]), 15360-pred[5]);

        for (int i = 6; i < 12; i++) {
            pred[i] = pred[i] * 255 >> 8;
            if (diff)
                pred[i] += ((diff ^ pred[i+12]) >> 31 | 1) * (1 << 7);
            pred[i] = av_clip(pred[i], -32768, 32767);
        }

        pred[2] = pred[3] = 0;
        for (int i = 0; i < 8; i++)
            pred[2 + (i > 1)] += (int64_t)pred[4+i] * pred[15+i] >> 23 * 2;
        pred[0] = pred[2] += pred[3];
    }

    return 0;
}

static int rcavoc_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                               int *got_frame_ptr, AVPacket *avpkt)
{
    int ret;

    frame->nb_samples = 80;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    switch (avctx->profile) {
    case 0:
        if (avpkt->size < 7)
            return AVERROR_INVALIDDATA;

        break;
    case 3:
        if (avpkt->size < 40)
            return AVERROR_INVALIDDATA;

        hq_decode(avctx, avpkt->data, (int16_t *)frame->data[0]);
        break;
    default:
        return AVERROR_PATCHWELCOME;
    }

    *got_frame_ptr = 1;

    return avpkt->size;
}

const FFCodec ff_rcavoc_decoder = {
    .p.name         = "rcavoc",
    CODEC_LONG_NAME("RCA VOC"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_RCAVOC,
    .priv_data_size = sizeof(RCAVOCContext),
    .init           = rcavoc_decode_init,
    FF_CODEC_DECODE_CB(rcavoc_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
