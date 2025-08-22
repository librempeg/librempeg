/*
 * CD-I DYUV video decoder
 * Copyright (c) 2025 Paul B Mahol
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

#include "libavutil/mem.h"
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"

static const uint8_t quant[] = { 0, 1, 4, 9, 16, 27, 44, 79, 128, 177, 212, 229, 240, 247, 252, 255 };

static av_cold int dyuv_decode_init(AVCodecContext *avctx)
{
    if (!avctx->width || !avctx->height) {
        avctx->width = 384;
        avctx->height = 240;
    }
    avctx->pix_fmt = AV_PIX_FMT_YUV422P;

    return 0;
}

static int dyuv_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                             int *got_frame, AVPacket *avpkt)
{
    GetByteContext gbc, *gb = &gbc;
    const int h = avctx->height;
    const int w2 = avctx->width/2;
    int ret;

    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    bytestream2_init(gb, avpkt->data, avpkt->size);

    for (int y = 0; y < h; y++) {
        uint8_t *y_dst = frame->data[0] + y * frame->linesize[0];
        uint8_t *u_dst = frame->data[1] + y * frame->linesize[1];
        uint8_t *v_dst = frame->data[2] + y * frame->linesize[2];
        int y_pred = 16, u_pred = 128, v_pred = 128;

        for (int x = 0; x < w2; x++) {
            int s0, s1;

            s0 = bytestream2_get_byte(gb);
            s1 = bytestream2_get_byte(gb);

            u_dst[  x  ] = u_pred = (u_pred + quant[s0 >> 4]) & 255;
            y_dst[2*x+0] = y_pred = (y_pred + quant[s0 & 15]) & 255;
            v_dst[  x  ] = v_pred = (v_pred + quant[s1 >> 4]) & 255;
            y_dst[2*x+1] = y_pred = (y_pred + quant[s1 & 15]) & 255;
        }
    }

    *got_frame = 1;

    return avpkt->size;
}

const FFCodec ff_dyuv_decoder = {
    .p.name         = "dyuv",
    CODEC_LONG_NAME("CD-I Delta YUV"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_DYUV,
    .init           = dyuv_decode_init,
    FF_CODEC_DECODE_CB(dyuv_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
