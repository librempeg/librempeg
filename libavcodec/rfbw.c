/*
 * RFBW decoder
 * Copyright (c) 2023 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <stdio.h>
#include <string.h>

#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "thread.h"
#include "inflate.h"

typedef struct RFBWSprite {
    uint16_t index;
    uint8_t w, h;
    unsigned offset;
} RFBWSprite;

typedef struct RFBWRect {
    unsigned offset;
    unsigned w, h;
    int left, right, bottom, top;
    uint8_t color;
    struct RFBWRect *prev, *next;
} RFBWRect;

typedef struct RFBWContext {
    int key;
    AVFrame *last_frame;

    uint8_t  *data;
    unsigned  data_size;

    unsigned  nb_rects;
    RFBWRect *rects;

    unsigned    nb_sprites;
    RFBWSprite *sprites;

    uint8_t  *sprite_data;
    unsigned  sprite_data_size;

    uint32_t palette[256];

    InflateContext ic;
} RFBWContext;

static av_cold int decode_init(AVCodecContext *avctx)
{
    RFBWContext *s = avctx->priv_data;
    avctx->pix_fmt = AV_PIX_FMT_PAL8;

    s->last_frame = av_frame_alloc();
    if (!s->last_frame)
        return AVERROR(ENOMEM);

    if (avctx->extradata && avctx->extradata_size >= 1024) {
        for (int n = 0; n < 256; n++)
            s->palette[n] = AV_RL32(avctx->extradata + n * 4) | (0xffu << 24);
    }

    return 0;
}

static void xset(uint8_t *dst, ptrdiff_t linesize, int x, int y, int w, int h,
                 uint8_t color)
{
    if (!w || !h)
        return;

    dst += linesize * y + x;

    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++)
            dst[i] = color;
        dst += linesize;
    }
}

static void xcopy(uint8_t *dst, ptrdiff_t linesize, int x, int y,
                  const uint8_t *src, int size)
{
    if (size <= 0)
        return;

    dst += linesize * y + x;

    for (int i = 0; i < size; i++)
        dst[-i] = src[i];
}

static int xread(GetByteContext *gb)
{
    int x = bytestream2_get_byte(gb);

    if (x == 255) {
        return -xread(gb);
    } else if (x < 128) {
        return x;
    } else if (x < 192) {
        int y = bytestream2_get_byte(gb);
        return ((x & 63) << 8) | y;
    } else {
        int y = bytestream2_get_byte(gb);
        int z = bytestream2_get_byte(gb);
        return ((x & 63) << 16) | (y << 8) | z;
    }
}

static int decode_rfbw(AVCodecContext *avctx, GetByteContext *gb,
                       AVFrame *frame, AVPacket *avpkt)
{
    RFBWContext *s = avctx->priv_data;
    InflateContext *const ic = &s->ic;
    unsigned raw_pixels, width, height, raw_index, raw_offset = 0;
    unsigned rect_index = 0, compression, decompressed_size;
    unsigned version, payload_size, command, bpp, size, size1;
    unsigned first_new_sprite, new_sprite_data_size;
    unsigned sprite_offset;
    unsigned x0, y0, compressed_size;
    RFBWRect *arect = NULL, *rect = NULL;
    RFBWRect head = { 0 }, tail = { 0 };
    int rect_offset, last_offset, offset;
    GetByteContext dgb, sgb1, sgb2;
    ptrdiff_t linesize;
    uint8_t *dst, *src;
    int src_len, ret;

    version = bytestream2_get_le32(gb);
    payload_size = bytestream2_get_le32(gb);
    command = bytestream2_get_le32(gb);
    bytestream2_skip(gb, 4);
    if (command == 38 || command == 39)
        return 0;

    x0 = bytestream2_get_le32(gb);
    y0 = bytestream2_get_le32(gb);
    bytestream2_get_le32(gb); // w
    bytestream2_get_le32(gb); // h
    bytestream2_get_byte(gb); // cursor
    bpp = bytestream2_get_byte(gb);
    if (bpp != 8)
        return AVERROR_INVALIDDATA;

    if (bytestream2_get_bytes_left(gb) < payload_size)
        return AVERROR_INVALIDDATA;

    if (version != 0 && version != 3)
        bytestream2_skip(gb, 4);
    if (version == 3)
        bytestream2_skip(gb, 12);

    switch (command) {
    case 4:
    case 36:
        s->key = 1;
        s->nb_sprites = 0;
        s->sprite_data_size = 0;
        bytestream2_skip(gb, 40);
        if (bpp == 8) {
            for (int n = 0; n < 256; n++)
                s->palette[n] = bytestream2_get_le32(gb) | (0xffu << 24);
        }
        break;
    case 5:
    case 37:
        s->key = 0;
        break;
    default:
        return 0;
    }

    decompressed_size = avctx->width * avctx->height;
    compression = bytestream2_get_le32(gb);
    compressed_size = bytestream2_get_le32(gb);
    if (compression == 13)
        bytestream2_skip(gb, 11);

    av_fast_padded_malloc(&s->data, &s->data_size, decompressed_size);
    if (!s->data)
        return AVERROR(ENOMEM);

    src  = avpkt->data + bytestream2_tell(gb);
    src_len = FFMIN(compressed_size, avpkt->size - bytestream2_tell(gb));

    ret = ff_inflate(ic, src, src_len, s->data, 1, s->data_size, s->data_size);
    if (ret < 0)
        return ret;

    bytestream2_skip(gb, avpkt->size);

    bytestream2_init(&dgb, s->data, s->data_size);
    bytestream2_skip(&dgb, 4);

    width = xread(&dgb);
    height = xread(&dgb);
    raw_pixels = xread(&dgb);
    raw_index = bytestream2_tell(&dgb);

    bytestream2_skip(&dgb, raw_pixels);

    s->nb_rects = xread(&dgb);
    s->rects = av_realloc_f(s->rects, s->nb_rects, sizeof(*s->rects));
    if (!s->rects)
        return AVERROR(ENOMEM);

    for (int n = 0; n < s->nb_rects; n++) {
        RFBWRect *rect = &s->rects[n];
        int offset, w, h, color;

        if (bytestream2_get_bytes_left(&dgb) <= 0)
            return AVERROR_INVALIDDATA;

        offset = xread(&dgb);
        w = xread(&dgb);
        h = xread(&dgb);
        color = bytestream2_get_byte(&dgb);

        rect->offset = offset;
        rect->w = w;
        rect->h = h;
        rect->color = color;
        rect->left =
        rect->right =
        rect->top =
        rect->bottom = 0;
        rect->prev = NULL;
        rect->next = NULL;
    }

    first_new_sprite = s->nb_sprites;
    new_sprite_data_size = s->sprite_data_size;
    sprite_offset = new_sprite_data_size;
    size = bytestream2_get_be32(&dgb);
    while (size > 0) {
        RFBWSprite *sprite;

        if (s->nb_sprites >= UINT16_MAX)
            return AVERROR_INVALIDDATA;

        s->sprites = av_realloc_f(s->sprites, s->nb_sprites + 1, sizeof(*s->sprites));
        if (!s->sprites)
            return AVERROR(ENOMEM);

        sprite = &s->sprites[s->nb_sprites];
        sprite->index = bytestream2_get_le16(&dgb);
        sprite->w = bytestream2_get_byte(&dgb);
        sprite->h = bytestream2_get_byte(&dgb);
        if (!sprite->w || !sprite->h) {
            av_log(avctx, AV_LOG_ERROR, "sprite w/h invalid\n");
            return AVERROR_INVALIDDATA;
        }

        sprite->offset = bytestream2_tell(&dgb);
        new_sprite_data_size += sprite->w * sprite->h;
        bytestream2_skip(&dgb, sprite->w * sprite->h);

        if (size < sprite->w * sprite->h + 4)
            return AVERROR_INVALIDDATA;

        size -= sprite->w * sprite->h + 4;

        s->nb_sprites++;
    }

    s->sprite_data = av_realloc_f(s->sprite_data, new_sprite_data_size, sizeof(*s->sprite_data));
    if (!s->sprite_data)
        return AVERROR(ENOMEM);
    s->sprite_data_size = new_sprite_data_size;

    for (int n = first_new_sprite; n < s->nb_sprites; n++) {
        RFBWSprite *sprite = &s->sprites[n];

        memcpy(s->sprite_data + sprite_offset, s->data + sprite->offset,
               sprite->w * sprite->h);
        sprite->offset = sprite_offset;
        sprite_offset += sprite->w * sprite->h;
    }

    size1 = bytestream2_get_be32(&dgb);
    sgb1 = dgb;
    bytestream2_skip(&dgb, size1);
    bytestream2_get_be32(&dgb);
    sgb2 = dgb;

    if ((ret = ff_reget_buffer(avctx, s->last_frame, 0)) < 0)
        return ret;

    ret = av_frame_ref(frame, s->last_frame);
    if (ret < 0)
        return ret;

    head.next = &tail;
    head.left = 0;
    head.top = -1;
    head.right = -1;
    tail.prev = &head;

    if (s->nb_rects > 0) {
        rect = &s->rects[rect_index];
        rect_offset = rect->offset;
    } else {
        rect = NULL;
        rect_offset = raw_pixels;
    }

    last_offset = width * height;

    linesize = frame->linesize[0];
    dst = frame->data[0] + y0 * linesize + x0;
    for (int y = height - 1; y >= 0; y--) {
        int x = width - 1;

        arect = tail.prev;
        while (x >= 0) {
            while (arect->top > y) {
                arect->prev->next = arect->next;
                arect->next->prev = arect->prev;
                arect = arect->prev;
            }

            if (rect_offset > 0) {
                int skip = FFMIN(x - arect->right, rect_offset);

                xcopy(dst, linesize, x, y,
                      s->data + raw_index + raw_offset, skip);
                x -= skip;
                raw_offset  += skip;
                rect_offset -= skip;
                last_offset -= skip;
            }

            if (rect_offset == 0 && rect != NULL) {
                while (x >= 0 && arect->right == x) {
                    xset(dst, linesize, arect->left, y,
                         arect->w, 1, arect->color);

                    x -= arect->right - arect->left + 1;
                    arect = arect->prev;

                    while (arect->top > y) {
                        arect->prev->next = arect->next;
                        arect->next->prev = arect->prev;
                        arect = arect->prev;
                    }
                }

                if (x == -1)
                    continue;

                rect->right  = x;
                rect->bottom = y;
                rect->left   = x - rect->w + 1;
                rect->top    = y - rect->h + 1;

                rect->next = arect->next;
                arect->next->prev = rect;
                rect->prev = arect;
                arect->next = rect;
                arect = rect;

                last_offset -= rect->w * rect->h;

                rect_index++;
                if (rect_index < s->nb_rects) {
                    rect = &s->rects[rect_index];
                    rect_offset = rect->offset;
                } else {
                    rect = NULL;
                    rect_offset = last_offset;
                }
            }

            xset(dst, linesize, arect->left, y,
                 arect->w, 1, arect->color);

            x -= arect->right - arect->left + 1;

            arect = arect->prev;
        }
    }

    memcpy(frame->data[1], s->palette, sizeof(s->palette));

    if (s->nb_sprites == 0)
        return 1;

    offset = 0;
    while (size1 > 0) {
        uint8_t *dst = frame->data[0] + y0 * linesize + x0;
        unsigned sprite_offset, sprite_w, sprite_h;
        int dw, dh, dx, dy, src_offset = 0, index;
        const uint8_t *src;

        size1 -= 2;
        index = bytestream2_get_be16(&sgb1);
        if (index >= s->nb_sprites) {
            av_log(avctx, AV_LOG_DEBUG, "sprite index: %d out of range: %d\n",
                   index, s->nb_sprites);
            continue;
        }

        sprite_offset = s->sprites[index].offset;
        sprite_w = s->sprites[index].w;
        sprite_h = s->sprites[index].h;

        offset += xread(&sgb2);

        dx = width  - (offset % width) - 1;
        dy = height - (offset / width) - 1;

        src = s->sprite_data + sprite_offset;
        dw = FFMIN(dx + sprite_w, width);
        dh = FFMIN(dy + sprite_h, height);
        for (int y = dy; y < dh; y++) {
            uint8_t *dsty = dst + y * linesize;
            for (int i = 0, x = dx; x < dw; i++, x++)
                dsty[x] = src[src_offset + i];
            src_offset += sprite_w;
        }
    }

    return 1;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame, AVPacket *avpkt)
{
    RFBWContext *s = avctx->priv_data;
    GetByteContext gb;
    int ret;

    if (avpkt->size < 22)
        return AVERROR_INVALIDDATA;

    bytestream2_init(&gb, avpkt->data, avpkt->size);

    if ((ret = decode_rfbw(avctx, &gb, frame, avpkt)) <= 0)
        return ret;

    if (s->key)
        frame->flags |= AV_FRAME_FLAG_KEY;
    frame->pict_type = (frame->flags & AV_FRAME_FLAG_KEY) ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

    *got_frame = 1;

    return avpkt->size;
}

static av_cold int decode_close(AVCodecContext *avctx)
{
    RFBWContext *s = avctx->priv_data;

    av_freep(&s->sprite_data);
    av_freep(&s->sprites);
    av_freep(&s->rects);
    av_freep(&s->data);
    av_frame_free(&s->last_frame);
    ff_inflate(&s->ic, NULL, 0, NULL, 0, 0, 0);

    return 0;
}

const FFCodec ff_rfbw_decoder = {
    .p.name           = "rfbw",
    CODEC_LONG_NAME("RFBW Video"),
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_RFBW,
    .priv_data_size   = sizeof(RFBWContext),
    .init             = decode_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .close            = decode_close,
    .caps_internal    = FF_CODEC_CAP_INIT_CLEANUP,
    .p.capabilities   = AV_CODEC_CAP_DR1,
};
