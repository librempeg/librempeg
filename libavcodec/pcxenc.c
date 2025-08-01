/*
 * PC Paintbrush PCX (.pcx) image encoder
 * Copyright (c) 2009 Daniel Verkamp <daniel at drv.nu>
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
 * PCX image encoder
 * @author Daniel Verkamp
 * @see http://bespin.org/~qz/pc-gpe/pcx.txt
 */

#include "libavutil/imgutils_internal.h"
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "encode.h"

static const uint32_t monoblack_pal[16] = { 0x000000, 0xFFFFFF };

/**
 * PCX run-length encoder
 * @param dst output buffer
 * @param dst_size size of output buffer
 * @param src input buffer
 * @param src_plane_size size of one plane of input buffer in bytes
 * @param nplanes number of planes in input buffer
 * @return number of bytes written to dst or -1 on error
 * @bug will not work for nplanes != 1 && bpp != 8
 */
static int pcx_rle_encode(      uint8_t *dst, int dst_size,
                          const uint8_t *src, int src_plane_size, int nplanes)
{
    int p;
    const uint8_t *dst_start = dst;

    // check worst-case upper bound on dst_size
    if (dst_size < 2LL * src_plane_size * nplanes || src_plane_size <= 0)
        return AVERROR(EINVAL);

    for (p = 0; p < nplanes; p++) {
        int count = 1;
        const uint8_t *src_plane = src + p;
        const uint8_t *src_plane_end = src_plane + src_plane_size * nplanes;
        uint8_t prev = *src_plane;
        src_plane += nplanes;

        for (; ; src_plane += nplanes) {
            if (src_plane < src_plane_end && *src_plane == prev && count < 0x3F) {
                // current byte is same as prev
                ++count;
            } else {
                // output prev * count
                if (count != 1 || prev >= 0xC0)
                    *dst++ = 0xC0 | count;
                *dst++ = prev;

                if (src_plane == src_plane_end)
                    break;

                // start new run
                count = 1;
                prev = *src_plane;
            }
        }
    }

    return dst - dst_start;
}

static int pcx_encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                            const AVFrame *frame, int *got_packet)
{
    const uint8_t *buf_end;
    uint8_t *buf;

    int bpp, nplanes, i, y, line_bytes, written, ret, max_pkt_size, sw, sh;
    const uint32_t *pal = NULL;
    uint32_t palette256[256];
    const uint8_t *src;

    if (avctx->width > 65535 || avctx->height > 65535) {
        av_log(avctx, AV_LOG_ERROR, "image dimensions do not fit in 16 bits\n");
        return AVERROR(EINVAL);
    }

    switch (avctx->pix_fmt) {
    case AV_PIX_FMT_RGB24:
        bpp = 8;
        nplanes = 3;
        break;
    case AV_PIX_FMT_RGB8:
    case AV_PIX_FMT_BGR8:
    case AV_PIX_FMT_RGB4_BYTE:
    case AV_PIX_FMT_BGR4_BYTE:
    case AV_PIX_FMT_GRAY8:
        bpp = 8;
        nplanes = 1;
        avpriv_set_systematic_pal2(palette256, avctx->pix_fmt);
        pal = palette256;
        break;
    case AV_PIX_FMT_PAL8:
        bpp = 8;
        nplanes = 1;
        pal = (uint32_t *)frame->data[1];
        break;
    case AV_PIX_FMT_MONOBLACK:
        bpp = 1;
        nplanes = 1;
        pal = monoblack_pal;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "unsupported pixfmt\n");
        return AVERROR(EINVAL);
    }

    line_bytes = (avctx->width * bpp + 7) >> 3;
    line_bytes = (line_bytes + 1) & ~1;

    max_pkt_size = 128 + avctx->height * 2 * line_bytes * nplanes + (pal ? 256*3 + 1 : 0);
    if ((ret = ff_alloc_packet(avctx, pkt, max_pkt_size)) < 0)
        return ret;
    buf     = pkt->data;
    buf_end = pkt->data + pkt->size;

    sw = avctx->sample_aspect_ratio.num;
    sh = avctx->sample_aspect_ratio.den;
    if (sw > 0xFFFFu || sh > 0xFFFFu)
        av_reduce(&sw, &sh, sw, sh, 0xFFFFu);

    bytestream_put_byte(&buf, 10);                  // manufacturer
    bytestream_put_byte(&buf, 5);                   // version
    bytestream_put_byte(&buf, 1);                   // encoding
    bytestream_put_byte(&buf, bpp);                 // bits per pixel per plane
    bytestream_put_le16(&buf, 0);                   // x min
    bytestream_put_le16(&buf, 0);                   // y min
    bytestream_put_le16(&buf, avctx->width - 1);    // x max
    bytestream_put_le16(&buf, avctx->height - 1);   // y max
    bytestream_put_le16(&buf, sw);                  // horizontal DPI
    bytestream_put_le16(&buf, sh);                  // vertical DPI
    for (i = 0; i < 16; i++)
        bytestream_put_be24(&buf, pal ? pal[i] : 0);// palette (<= 16 color only)
    bytestream_put_byte(&buf, 0);                   // reserved
    bytestream_put_byte(&buf, nplanes);             // number of planes
    bytestream_put_le16(&buf, line_bytes);          // scanline plane size in bytes

    while (buf - pkt->data < 128)
        *buf++= 0;

    src = frame->data[0];

    for (y = 0; y < avctx->height; y++) {
        if ((written = pcx_rle_encode(buf, buf_end - buf,
                                      src, line_bytes, nplanes)) < 0) {
            av_log(avctx, AV_LOG_ERROR, "buffer too small\n");
            return AVERROR_BUG;
        }
        buf += written;
        src += frame->linesize[0];
    }

    if (nplanes == 1 && bpp == 8) {
        if (buf_end - buf < 257) {
            av_log(avctx, AV_LOG_ERROR, "buffer too small\n");
            return AVERROR_BUG;
        }
        bytestream_put_byte(&buf, 12);
        for (i = 0; i < 256; i++) {
            bytestream_put_be24(&buf, pal[i]);
        }
    }

    pkt->size   = buf - pkt->data;
    *got_packet = 1;

    return 0;
}

const FFCodec ff_pcx_encoder = {
    .p.name         = "pcx",
    CODEC_LONG_NAME("PC Paintbrush PCX image"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_PCX,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_ENCODER_REORDERED_OPAQUE,
    FF_CODEC_ENCODE_CB(pcx_encode_frame),
    CODEC_PIXFMTS(AV_PIX_FMT_RGB24,
                  AV_PIX_FMT_RGB8, AV_PIX_FMT_BGR8,
                  AV_PIX_FMT_RGB4_BYTE, AV_PIX_FMT_BGR4_BYTE,
                  AV_PIX_FMT_GRAY8, AV_PIX_FMT_PAL8,
                  AV_PIX_FMT_MONOBLACK),
};
