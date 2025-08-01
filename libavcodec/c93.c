/*
 * Interplay C93 video decoder
 * Copyright (c) 2007 Anssi Hannula <anssi.hannula@gmail.com>
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
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"

typedef struct C93DecoderContext {
    AVFrame *pictures[2];
    int currentpic;
} C93DecoderContext;

typedef enum {
    C93_8X8_FROM_PREV  = 0x02,
    C93_4X4_FROM_PREV  = 0x06,
    C93_4X4_FROM_CURR  = 0x07,
    C93_8X8_2COLOR     = 0x08,
    C93_4X4_2COLOR     = 0x0A,
    C93_4X4_4COLOR_GRP = 0x0B,
    C93_4X4_4COLOR     = 0x0D,
    C93_NOOP           = 0x0E,
    C93_8X8_INTRA      = 0x0F,
} C93BlockType;

#define WIDTH   320
#define HEIGHT  192

#define C93_HAS_PALETTE 0x01
#define C93_FIRST_FRAME 0x02

static av_cold int decode_end(AVCodecContext *avctx)
{
    C93DecoderContext * const c93 = avctx->priv_data;

    av_frame_free(&c93->pictures[0]);
    av_frame_free(&c93->pictures[1]);

    return 0;
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    C93DecoderContext *s = avctx->priv_data;
    avctx->pix_fmt = AV_PIX_FMT_PAL8;

    s->pictures[0] = av_frame_alloc();
    s->pictures[1] = av_frame_alloc();
    if (!s->pictures[0] || !s->pictures[1])
        return AVERROR(ENOMEM);

    return 0;
}

static inline int copy_block(AVCodecContext *avctx, uint8_t *to,
        uint8_t *from, int offset, int height, int stride)
{
    int i;
    int width = height;
    int from_x = offset % WIDTH;
    int from_y = offset / WIDTH;
    int overflow = from_x + width - WIDTH;

    if (!from) {
        /* silently ignoring predictive blocks in first frame */
        return 0;
    }

    if (from_y + height > HEIGHT) {
        av_log(avctx, AV_LOG_ERROR, "invalid offset %d during C93 decoding\n",
               offset);
        return AVERROR_INVALIDDATA;
    }

    if (overflow > 0) {
        width -= overflow;
        for (i = 0; i < height; i++) {
            memcpy(&to[i*stride+width], &from[(from_y+i)*stride], overflow);
        }
    }

    for (i = 0; i < height; i++) {
        memcpy(&to[i*stride], &from[(from_y+i)*stride+from_x], width);
    }

    return 0;
}

static inline void draw_n_color(uint8_t *out, int stride, int width,
         int height, int bpp, uint8_t cols[4], uint8_t grps[4], uint32_t col)
{
    int x, y;
    for (y = 0; y < height; y++) {
        if (grps)
            cols[0] = grps[3 * (y >> 1)];
        for (x = 0; x < width; x++) {
            if (grps)
                cols[1]= grps[(x >> 1) + 1];
            out[x + y*stride] = cols[col & ((1 << bpp) - 1)];
            col >>= bpp;
        }
    }
}

static int decode_frame(AVCodecContext *avctx, AVFrame *rframe,
                        int *got_frame, AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    C93DecoderContext * const c93 = avctx->priv_data;
    AVFrame * const newpic = c93->pictures[c93->currentpic];
    AVFrame * const oldpic = c93->pictures[c93->currentpic^1];
    GetByteContext gb;
    uint8_t *out;
    int ret, i, x, y, b, bt = 0;
    ptrdiff_t stride;

    if ((ret = ff_set_dimensions(avctx, WIDTH, HEIGHT)) < 0)
        return ret;

    c93->currentpic ^= 1;

    if ((ret = ff_reget_buffer(avctx, newpic, 0)) < 0)
        return ret;

    stride = newpic->linesize[0];

    bytestream2_init(&gb, buf, buf_size);
    b = bytestream2_get_byte(&gb);
    if (b & C93_FIRST_FRAME) {
        newpic->pict_type = AV_PICTURE_TYPE_I;
        newpic->flags |= AV_FRAME_FLAG_KEY;
    } else {
        newpic->pict_type = AV_PICTURE_TYPE_P;
        newpic->flags &= ~AV_FRAME_FLAG_KEY;
    }

    for (y = 0; y < HEIGHT; y += 8) {
        out = newpic->data[0] + y * stride;
        for (x = 0; x < WIDTH; x += 8) {
            uint8_t *copy_from = oldpic->data[0];
            uint8_t cols[4], grps[4];
            C93BlockType block_type;

            if (!bt)
                bt = bytestream2_get_byte(&gb);

            block_type= bt & 0x0F;
            switch (block_type) {
            case C93_8X8_FROM_PREV: {
                int offset = bytestream2_get_le16(&gb);
                if ((ret = copy_block(avctx, out, copy_from, offset, 8, stride)) < 0)
                    return ret;
                break;
            }

            case C93_4X4_FROM_CURR:
                copy_from = newpic->data[0];
            case C93_4X4_FROM_PREV:
                for (int j = 0; j < 8; j += 4) {
                    for (i = 0; i < 8; i += 4) {
                        int offset = bytestream2_get_le16(&gb);
                        int from_x = offset % WIDTH;
                        int from_y = offset / WIDTH;
                        if (block_type == C93_4X4_FROM_CURR && from_y == y+j &&
                            (FFABS(from_x - x-i) < 4 || FFABS(from_x - x-i) > WIDTH-4)) {
                            avpriv_request_sample(avctx, "block overlap %d %d %d %d", from_x, x+i, from_y, y+j);
                            return AVERROR_INVALIDDATA;
                        }
                        if ((ret = copy_block(avctx, &out[j*stride+i],
                                              copy_from, offset, 4, stride)) < 0)
                            return ret;
                    }
                }
                break;

            case C93_8X8_2COLOR:
                bytestream2_get_buffer(&gb, cols, 2);
                for (i = 0; i < 8; i++) {
                    draw_n_color(out + i*stride, stride, 8, 1, 1, cols,
                                     NULL, bytestream2_get_byte(&gb));
                }

                break;

            case C93_4X4_2COLOR:
            case C93_4X4_4COLOR:
            case C93_4X4_4COLOR_GRP:
                for (int j = 0; j < 8; j += 4) {
                    for (i = 0; i < 8; i += 4) {
                        if (block_type == C93_4X4_2COLOR) {
                            bytestream2_get_buffer(&gb, cols, 2);
                            draw_n_color(out + i + j*stride, stride, 4, 4,
                                    1, cols, NULL, bytestream2_get_le16(&gb));
                        } else if (block_type == C93_4X4_4COLOR) {
                            bytestream2_get_buffer(&gb, cols, 4);
                            draw_n_color(out + i + j*stride, stride, 4, 4,
                                    2, cols, NULL, bytestream2_get_le32(&gb));
                        } else {
                            bytestream2_get_buffer(&gb, grps, 4);
                            draw_n_color(out + i + j*stride, stride, 4, 4,
                                    1, cols, grps, bytestream2_get_le16(&gb));
                        }
                    }
                }
                break;

            case C93_NOOP:
                break;

            case C93_8X8_INTRA:
                for (int j = 0; j < 8; j++)
                    bytestream2_get_buffer(&gb, out + j*stride, 8);
                break;

            default:
                av_log(avctx, AV_LOG_ERROR, "unexpected type %x at %dx%d\n",
                       block_type, x, y);
                return AVERROR_INVALIDDATA;
            }
            bt >>= 4;
            out += 8;
        }
    }

    if (b & C93_HAS_PALETTE) {
        uint32_t *palette = (uint32_t *) newpic->data[1];
        for (i = 0; i < 256; i++) {
            palette[i] = 0xFFU << 24 | bytestream2_get_be24(&gb);
        }
    } else {
        if (oldpic->data[1])
            memcpy(newpic->data[1], oldpic->data[1], 256 * 4);
    }

    if ((ret = av_frame_ref(rframe, newpic)) < 0)
        return ret;
    *got_frame = 1;

    return buf_size;
}

const FFCodec ff_c93_decoder = {
    .p.name         = "c93",
    CODEC_LONG_NAME("Interplay C93"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_C93,
    .priv_data_size = sizeof(C93DecoderContext),
    .init           = decode_init,
    .close          = decode_end,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
