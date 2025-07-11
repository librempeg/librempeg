/*
 * Renderware TeXture Dictionary (.txd) image decoder
 * Copyright (c) 2007 Ivo van Poorten
 *
 * See also: http://wiki.multimedia.cx/index.php?title=TXD
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

#include "bytestream.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "texturedsp.h"

#define TXD_DXT1 0x31545844
#define TXD_DXT3 0x33545844

static int txd_decode_frame(AVCodecContext *avctx, AVFrame *p,
                            int *got_frame, AVPacket *avpkt)
{
    GetByteContext gb;
    TextureDSPContext dxtc;
    unsigned int version, w, h, d3d_format, depth, stride, flags;
    unsigned int y, v;
    uint8_t *ptr;
    uint32_t *pal;
    int i, j;
    int ret;

    if (avpkt->size < 88)
        return AVERROR_INVALIDDATA;

    ff_texturedsp_init(&dxtc);

    bytestream2_init(&gb, avpkt->data, avpkt->size);
    version         = bytestream2_get_le32(&gb);
    bytestream2_skip(&gb, 72);
    d3d_format      = bytestream2_get_le32(&gb);
    w               = bytestream2_get_le16(&gb);
    h               = bytestream2_get_le16(&gb);
    depth           = bytestream2_get_byte(&gb);
    bytestream2_skip(&gb, 2);
    flags           = bytestream2_get_byte(&gb);

    if (version < 8 || version > 9) {
        avpriv_report_missing_feature(avctx, "Texture data version %u", version);
        return AVERROR_PATCHWELCOME;
    }

    if (depth == 8) {
        avctx->pix_fmt = AV_PIX_FMT_PAL8;
        if (bytestream2_get_bytes_left(&gb) < w * h + 4 * 256)
            return AVERROR_INVALIDDATA;
    } else if (depth == 16) {
        avctx->pix_fmt = AV_PIX_FMT_RGBA;
        switch (d3d_format) {
        case 0:
            if (!(flags & 1))
                goto unsupported;
        case TXD_DXT1:
            if (bytestream2_get_bytes_left(&gb) < AV_CEIL_RSHIFT(w, 2) * AV_CEIL_RSHIFT(h, 2) * 8 + 4)
                return AVERROR_INVALIDDATA;
            break;
        case TXD_DXT3:
            if (bytestream2_get_bytes_left(&gb) < AV_CEIL_RSHIFT(w, 2) * AV_CEIL_RSHIFT(h, 2) * 16 + 4)
                return AVERROR_INVALIDDATA;
        }
    } else if (depth == 32) {
        avctx->pix_fmt = AV_PIX_FMT_RGBA;
        if (bytestream2_get_bytes_left(&gb) < h * w * 4)
            return AVERROR_INVALIDDATA;
    } else {
        avpriv_report_missing_feature(avctx, "Color depth of %u", depth);
        return AVERROR_PATCHWELCOME;
    }

    if ((ret = ff_set_dimensions(avctx, w, h)) < 0)
        return ret;

    avctx->coded_width  = FFALIGN(w, 4);
    avctx->coded_height = FFALIGN(h, 4);

    if ((ret = ff_get_buffer(avctx, p, 0)) < 0)
        return ret;

    p->pict_type = AV_PICTURE_TYPE_I;

    ptr    = p->data[0];
    stride = p->linesize[0];

    if (depth == 8) {
        pal = (uint32_t *) p->data[1];
        for (y = 0; y < 256; y++) {
            v = bytestream2_get_be32(&gb);
            pal[y] = (v >> 8) + (v << 24);
        }
        bytestream2_skip(&gb, 4);
        for (y=0; y<h; y++) {
            bytestream2_get_buffer(&gb, ptr, w);
            ptr += stride;
        }
    } else if (depth == 16) {
        bytestream2_skip(&gb, 4);
        switch (d3d_format) {
        case 0:
        case TXD_DXT1:
            for (j = 0; j < avctx->height; j += 4) {
                for (i = 0; i < avctx->width; i += 4) {
                    uint8_t *p = ptr + i * 4 + j * stride;
                    int step = dxtc.dxt1_block(p, stride, gb.buffer);
                    bytestream2_skip(&gb, step);
                }
            }
            break;
        case TXD_DXT3:
            for (j = 0; j < avctx->height; j += 4) {
                for (i = 0; i < avctx->width; i += 4) {
                    uint8_t *p = ptr + i * 4 + j * stride;
                    int step = dxtc.dxt3_block(p, stride, gb.buffer);
                    bytestream2_skip(&gb, step);
                }
            }
            break;
        default:
            goto unsupported;
        }
    } else if (depth == 32) {
        switch (d3d_format) {
        case 0x15:
        case 0x16:
            for (y=0; y<h; y++) {
                bytestream2_get_buffer(&gb, ptr, w * 4);
                ptr += stride;
            }
            break;
        default:
            goto unsupported;
        }
    }

    *got_frame = 1;

    return avpkt->size;

unsupported:
    avpriv_report_missing_feature(avctx, "d3d format (%08x)", d3d_format);
    return AVERROR_PATCHWELCOME;
}

const FFCodec ff_txd_decoder = {
    .p.name         = "txd",
    CODEC_LONG_NAME("Renderware TXD (TeXture Dictionary) image"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_TXD,
    .p.capabilities = AV_CODEC_CAP_DR1,
    FF_CODEC_DECODE_CB(txd_decode_frame),
};
