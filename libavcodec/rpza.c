/*
 * Quicktime Video (RPZA) Video Decoder
 * Copyright (C) 2003 The FFmpeg project
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
 * QT RPZA Video Decoder by Roberto Togni
 * For more information about the RPZA format, visit:
 *   http://www.pcisys.net/~melanson/codecs/
 *
 * The RPZA decoder outputs RGB555 colorspace data.
 *
 * Note that this decoder reads big endian RGB555 pixel values from the
 * bytestream, arranges them in the host's endian order, and outputs
 * them to the final rendered map in the same host endian order. This is
 * intended behavior as the libavcodec documentation states that RGB555
 * pixels shall be stored in native CPU endianness.
 */

#include <stdint.h>

#include "libavutil/internal.h"
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"

typedef struct RpzaContext {

    AVCodecContext *avctx;
    AVFrame *frame;

    GetByteContext gb;
} RpzaContext;

#define CHECK_BLOCK()                                                         \
    if (total_blocks < 1) {                                                    \
        av_log(s->avctx, AV_LOG_ERROR,                                         \
               "Block counter just went negative (this should not happen)\n"); \
        return AVERROR_INVALIDDATA;                                            \
    }                                                                          \

#define ADVANCE_BLOCK()             \
    {                               \
        pixel_ptr += 4;             \
        if (pixel_ptr >= width)     \
        {                           \
            pixel_ptr = 0;          \
            row_ptr  += stride * 4; \
        }                           \
        total_blocks--;             \
    }

static int rpza_decode_stream(RpzaContext *s)
{
    int width = s->avctx->width;
    int stride, row_inc, ret;
    int chunk_size;
    uint16_t colorA = 0, colorB;
    uint16_t color4[4];
    uint16_t ta, tb;
    uint16_t *pixels;

    int row_ptr = 0;
    int pixel_ptr = 0;
    int block_ptr;
    int pixel_x, pixel_y;
    int total_blocks;

    /* First byte is always 0xe1. Warn if it's different */
    if (bytestream2_peek_byte(&s->gb) != 0xe1)
        av_log(s->avctx, AV_LOG_ERROR, "First chunk byte is 0x%02x instead of 0xe1\n",
               bytestream2_peek_byte(&s->gb));

    /* Get chunk size, ignoring first byte */
    chunk_size = bytestream2_get_be32(&s->gb) & 0x00FFFFFF;

    /* If length mismatch use size from MOV file and try to decode anyway */
    if (chunk_size != bytestream2_get_bytes_left(&s->gb) + 4)
        av_log(s->avctx, AV_LOG_WARNING,
               "MOV chunk size %d != encoded chunk size %d\n",
               chunk_size,
               bytestream2_get_bytes_left(&s->gb) + 4
              );

    /* Number of 4x4 blocks in frame. */
    total_blocks = ((s->avctx->width + 3) / 4) * ((s->avctx->height + 3) / 4);

    if (total_blocks / 32 > bytestream2_get_bytes_left(&s->gb))
        return AVERROR_INVALIDDATA;

    if ((ret = ff_reget_buffer(s->avctx, s->frame, 0)) < 0)
        return ret;
    pixels = (uint16_t *)s->frame->data[0];
    stride = s->frame->linesize[0] / 2;
    row_inc = stride - 4;

    /* Process chunk data */
    while (bytestream2_get_bytes_left(&s->gb)) {
        uint8_t opcode = bytestream2_get_byte(&s->gb); /* Get opcode */

        int n_blocks = (opcode & 0x1f) + 1; /* Extract block counter from opcode */

        /* If opcode MSbit is 0, we need more data to decide what to do */
        if ((opcode & 0x80) == 0) {
            colorA = (opcode << 8) | bytestream2_get_byte(&s->gb);
            opcode = 0;
            if ((bytestream2_peek_byte(&s->gb) & 0x80) != 0) {
                /* Must behave as opcode 110xxxxx, using colorA computed
                 * above. Use fake opcode 0x20 to enter switch block at
                 * the right place */
                opcode = 0x20;
                n_blocks = 1;
            }
        }

        n_blocks = FFMIN(n_blocks, total_blocks);

        switch (opcode & 0xe0) {

        /* Skip blocks */
        case 0x80:
            while (n_blocks--) {
                CHECK_BLOCK();
                ADVANCE_BLOCK();
            }
            break;

        /* Fill blocks with one color */
        case 0xa0:
            colorA = bytestream2_get_be16(&s->gb);
            while (n_blocks--) {
                CHECK_BLOCK();
                block_ptr = row_ptr + pixel_ptr;
                for (pixel_y = 0; pixel_y < 4; pixel_y++) {
                    for (pixel_x = 0; pixel_x < 4; pixel_x++){
                        pixels[block_ptr] = colorA;
                        block_ptr++;
                    }
                    block_ptr += row_inc;
                }
                ADVANCE_BLOCK();
            }
            break;

        /* Fill blocks with 4 colors */
        case 0xc0:
            colorA = bytestream2_get_be16(&s->gb);
        case 0x20:
            colorB = bytestream2_get_be16(&s->gb);

            /* sort out the colors */
            color4[0] = colorB;
            color4[1] = 0;
            color4[2] = 0;
            color4[3] = colorA;

            /* red components */
            ta = (colorA >> 10) & 0x1F;
            tb = (colorB >> 10) & 0x1F;
            color4[1] |= ((11 * ta + 21 * tb) >> 5) << 10;
            color4[2] |= ((21 * ta + 11 * tb) >> 5) << 10;

            /* green components */
            ta = (colorA >> 5) & 0x1F;
            tb = (colorB >> 5) & 0x1F;
            color4[1] |= ((11 * ta + 21 * tb) >> 5) << 5;
            color4[2] |= ((21 * ta + 11 * tb) >> 5) << 5;

            /* blue components */
            ta = colorA & 0x1F;
            tb = colorB & 0x1F;
            color4[1] |= ((11 * ta + 21 * tb) >> 5);
            color4[2] |= ((21 * ta + 11 * tb) >> 5);

            if (bytestream2_get_bytes_left(&s->gb) < n_blocks * 4)
                return AVERROR_INVALIDDATA;
            while (n_blocks--) {
                CHECK_BLOCK();
                block_ptr = row_ptr + pixel_ptr;
                for (pixel_y = 0; pixel_y < 4; pixel_y++) {
                    uint8_t index = bytestream2_get_byteu(&s->gb);
                    for (pixel_x = 0; pixel_x < 4; pixel_x++){
                        uint8_t idx = (index >> (2 * (3 - pixel_x))) & 0x03;
                        pixels[block_ptr] = color4[idx];
                        block_ptr++;
                    }
                    block_ptr += row_inc;
                }
                ADVANCE_BLOCK();
            }
            break;

        /* Fill block with 16 colors */
        case 0x00:
            if (bytestream2_get_bytes_left(&s->gb) < 30)
                return AVERROR_INVALIDDATA;
            CHECK_BLOCK();
            block_ptr = row_ptr + pixel_ptr;
            for (pixel_y = 0; pixel_y < 4; pixel_y++) {
                for (pixel_x = 0; pixel_x < 4; pixel_x++){
                    /* We already have color of upper left pixel */
                    if ((pixel_y != 0) || (pixel_x != 0))
                        colorA = bytestream2_get_be16u(&s->gb);
                    pixels[block_ptr] = colorA;
                    block_ptr++;
                }
                block_ptr += row_inc;
            }
            ADVANCE_BLOCK();
            break;

        /* Unknown opcode */
        default:
            av_log(s->avctx, AV_LOG_ERROR, "Unknown opcode %d in rpza chunk."
                 " Skip remaining %d bytes of chunk data.\n", opcode,
                 bytestream2_get_bytes_left(&s->gb));
            return AVERROR_INVALIDDATA;
        } /* Opcode switch */
    }

    return 0;
}

static av_cold int rpza_decode_init(AVCodecContext *avctx)
{
    RpzaContext *s = avctx->priv_data;

    s->avctx = avctx;
    avctx->pix_fmt = AV_PIX_FMT_RGB555;

    s->frame = av_frame_alloc();
    if (!s->frame)
        return AVERROR(ENOMEM);

    return 0;
}

static int rpza_decode_frame(AVCodecContext *avctx, AVFrame *rframe,
                             int *got_frame, AVPacket *avpkt)
{
    RpzaContext *s = avctx->priv_data;
    int ret;

    bytestream2_init(&s->gb, avpkt->data, avpkt->size);

    ret = rpza_decode_stream(s);
    if (ret < 0)
        return ret;

    if ((ret = av_frame_ref(rframe, s->frame)) < 0)
        return ret;

    *got_frame      = 1;

    /* always report that the buffer was completely consumed */
    return avpkt->size;
}

static av_cold int rpza_decode_end(AVCodecContext *avctx)
{
    RpzaContext *s = avctx->priv_data;

    av_frame_free(&s->frame);

    return 0;
}

const FFCodec ff_rpza_decoder = {
    .p.name         = "rpza",
    CODEC_LONG_NAME("QuickTime video (RPZA)"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_RPZA,
    .priv_data_size = sizeof(RpzaContext),
    .init           = rpza_decode_init,
    .close          = rpza_decode_end,
    FF_CODEC_DECODE_CB(rpza_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
