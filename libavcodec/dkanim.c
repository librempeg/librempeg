/*
 * DKAnim video decoder
 * Copyright (c) 2011-2012 Paul B Mahol
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

#include "libavutil/display.h"
#include "libavutil/intreadwrite.h"
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "mathops.h"

typedef struct DKAnimVideoContext {
    int unused;
} DKAnimVideoContext;

static av_cold int dkanim_decode_init(AVCodecContext *avctx)
{
    avctx->pix_fmt = AV_PIX_FMT_PAL8;
    avctx->width = 640;
    avctx->height = 480;

    return 0;
}

static int dkanim_decode_frame(AVCodecContext *avctx, AVFrame *p,
                              int *got_frame, AVPacket *avpkt)
{
    GetByteContext gbc, *gb = &gbc;
    int run = 0, fill = 0;
    ptrdiff_t linesize;
    int bx[2], by[2];
    int mode = -1;
    uint8_t *dst;
    int ret;

    if (avpkt->size == 1)
        return avpkt->size;

    bytestream2_init(gb, avpkt->data, avpkt->size);

    if ((ret = ff_reget_buffer(avctx, p, 0)) < 0)
        return ret;

    for (int n = 0; n < 256; n++) {
        AV_WB32(p->data[1] + n * 4, (n << 24) | (n << 16) | (n << 8) | 0xFF);
    }

    bx[0] = bytestream2_get_le16(gb);
    by[0] = bytestream2_get_le16(gb);
    bx[1] = bytestream2_get_le16(gb);
    by[1] = bytestream2_get_le16(gb);

    bytestream2_skip(gb, 4);

    linesize = p->linesize[0];
    dst = p->data[0] + by[0] * linesize;;

    for (int y = by[0]; y < by[1]; y++) {
        for (int x = bx[0]; x < bx[1]; x++) {
            if (run > 0) {
                if (mode == 0)
                    dst[x] = fill;
                else
                    dst[x] = bytestream2_get_byte(gb);
                run--;
                if (run == 0)
                    mode = -1;
            }
            if (mode == -1) {
                mode = bytestream2_get_byte(gb);
                run = mode & 0x7f;
                mode >>= 7;
                if (mode == 0)
                    fill = bytestream2_get_byte(gb);
            }
        }

        dst += linesize;
    }

    *got_frame = 1;

    return avpkt->size;
}

const FFCodec ff_dkanim_decoder = {
    .p.name         = "dkanim",
    CODEC_LONG_NAME("DK Animation Video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_DKANIM,
    .priv_data_size = sizeof(DKAnimVideoContext),
    .init           = dkanim_decode_init,
    FF_CODEC_DECODE_CB(dkanim_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
