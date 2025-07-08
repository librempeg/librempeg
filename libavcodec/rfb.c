/*
 * RFB decoder
 * Copyright (c) 2024 Paul B Mahol
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

#include <stdio.h>
#include <string.h>

#include "libavutil/fifo.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"

#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "thread.h"

#define HextileRaw                   (1 << 0)
#define HextileBackgroundSpecified   (1 << 1)
#define HextileForegroundSpecified   (1 << 2)
#define HextileAnySubrects           (1 << 3)
#define HextileSubrectsColoured      (1 << 4)

typedef struct Packet {
    int64_t size;
    int64_t pts;
} Packet;

typedef struct RFBContext {
    GetByteContext gb;

    int bpp;
    int key;
    AVFrame *last_frame;

    uint8_t *bytestream;
    int64_t bytestream_start;
    int64_t bytestream_stop;
    int64_t max_framesize;

    AVFifo *fifo;
    int length;
} RFBContext;

static av_cold int decode_init(AVCodecContext *avctx)
{
    RFBContext *s = avctx->priv_data;

    if (!avctx->extradata || avctx->extradata_size < 20)
        return AVERROR_INVALIDDATA;

    avctx->width  = AV_RB16(avctx->extradata);
    avctx->height = AV_RB16(avctx->extradata+2);
    s->bpp = avctx->extradata[5];

    avctx->coded_width  = avctx->width  + 16;
    avctx->coded_height = avctx->height + 16;

    s->last_frame = av_frame_alloc();
    if (!s->last_frame)
        return AVERROR(ENOMEM);

    s->max_framesize = 8LL * ((s->bpp+7)/8) * avctx->width * avctx->height + 1024LL;
    if (s->max_framesize > (INT32_MAX - AV_INPUT_BUFFER_PADDING_SIZE))
        return AVERROR_INVALIDDATA;

    s->bytestream = av_calloc(s->max_framesize + AV_INPUT_BUFFER_PADDING_SIZE, sizeof(*s->bytestream));
    if (!s->bytestream)
        return AVERROR(ENOMEM);

    s->fifo = av_fifo_alloc2(1024, sizeof(Packet), AV_FIFO_FLAG_AUTO_GROW);
    if (!s->fifo)
        return AVERROR(ENOMEM);

    switch (s->bpp) {
    case 32:
        avctx->pix_fmt = AV_PIX_FMT_ARGB;
        break;
    case 24:
    case 8:
        avctx->pix_fmt = AV_PIX_FMT_0RGB;
        break;
    case 16:
        avctx->pix_fmt = AV_PIX_FMT_RGB565;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    return 0;
}

static void fill_rect(AVCodecContext *avctx, uint32_t fill,
                      int x, int y, int w, int h)
{
    RFBContext *s = avctx->priv_data;
    const ptrdiff_t linesize = s->last_frame->linesize[0];
    uint8_t *dst = s->last_frame->data[0];

    for (int j = y; j < y + h; j++) {
        uint8_t *line = dst + j * linesize + x*4;

        for (int i = 0; i < w; i++)
            AV_WL32(line + i*4, fill);
    }
}

static void fill_rect16(AVCodecContext *avctx, uint32_t fill,
                        int x, int y, int w, int h)
{
    RFBContext *s = avctx->priv_data;
    const ptrdiff_t linesize = s->last_frame->linesize[0];
    uint8_t *dst = s->last_frame->data[0];

    for (int j = y; j < y + h; j++) {
        uint8_t *line = dst + j * linesize + x*2;

        for (int i = 0; i < w; i++)
            AV_WL16(line + i*2, fill);
    }
}

static void copy_rect8(AVCodecContext *avctx,
                       int x, int y, int w, int h)
{
    RFBContext *s = avctx->priv_data;
    const ptrdiff_t linesize = s->last_frame->linesize[0];
    uint8_t *dst = s->last_frame->data[0];
    GetByteContext *gb = &s->gb;

    for (int j = y; j < y + h; j++) {
        uint8_t *line = dst + j * linesize + x*4;

        for (int i = 0; i < w; i++) {
            unsigned item = bytestream2_get_byte(gb);
            unsigned fill = (item << 8) | (item << 16) | (item << 24);

            AV_WL32(line + i*4, fill);
        }
    }
}

static void hextile_subrect8(AVCodecContext *avctx, int tx, int ty, int tw, int th,
                             unsigned *fg, unsigned *bg)
{
    RFBContext *s = avctx->priv_data;
    uint8_t *dst = s->last_frame->data[0];
    int b1, b2, sx, sy, sw, sh, nb_subrects;
    GetByteContext *gb = &s->gb;
    unsigned subencoding;

    subencoding = bytestream2_get_byte(gb);

    // Is it a raw-encoded sub-rectangle?
    if (subencoding & HextileRaw) {
        const ptrdiff_t linesize = s->last_frame->linesize[0];

        for (int j = ty; j < ty + th; j++) {
            uint8_t *line = dst + j * linesize + tx*4;

            for (int i = 0; i < tw; i++) {
                unsigned item = bytestream2_get_byte(gb);
                unsigned fill = (item << 8) | (item << 16) | (item << 24);

                AV_WL32(line + i*4, fill);
            }
        }
        return;
    }

    // Read and draw the background if specified.
    if (subencoding & HextileBackgroundSpecified) {
        unsigned item = bytestream2_get_byte(gb);

        *bg = (item << 8) | (item << 16) | (item << 24);
    }

    fill_rect(avctx, *bg, tx, ty, tw, th);

    // Read the foreground color if specified.
    if (subencoding & HextileForegroundSpecified) {
        unsigned item = bytestream2_get_byte(gb);

        *fg = (item << 8) | (item << 16) | (item << 24);
    }

    // Done with this tile if there is no sub-rectangles.
    if (!(subencoding & HextileAnySubrects))
        return;

    nb_subrects = bytestream2_get_byte(gb);
    if (subencoding & HextileSubrectsColoured) {
        for (int j = 0; j < nb_subrects; j++) {
            unsigned item = bytestream2_get_byte(gb);
            unsigned cf = (item << 8) | (item << 16) | (item << 24);

            b1 = bytestream2_get_byte(gb);
            b2 = bytestream2_get_byte(gb);
            sx = tx + (b1 >> 4);
            sy = ty + (b1 & 0xf);
            sw = (b2 >> 4) + 1;
            sh = (b2 & 0xf) + 1;

            fill_rect(avctx, cf, sx, sy, sw, sh);
        }
    } else {
        for (int j = 0; j < nb_subrects; j++) {
            b1 = bytestream2_get_byte(gb);
            b2 = bytestream2_get_byte(gb);
            sx = tx + (b1 >> 4);
            sy = ty + (b1 & 0xf);
            sw = (b2 >> 4) + 1;
            sh = (b2 & 0xf) + 1;

            fill_rect(avctx, *fg, sx, sy, sw, sh);
        }
    }
}

static void hextile_rect8(AVCodecContext *avctx, int x, int y, int w, int h)
{
    uint32_t fg = 0, bg = 0;

    for (int ty = y; ty < y + h; ty += 16) {
        int th = 16;
        if (y + h - ty < 16)
            th = y + h - ty;

        for (int tx = x; tx < x + w; tx += 16) {
            int tw = 16;
            if (x + w - tx < 16)
                tw = x + w - tx;

            hextile_subrect8(avctx, tx, ty, tw, th, &fg, &bg);
        }
    }
}

static void copy_rect16(AVCodecContext *avctx,
                        int x, int y, int w, int h)
{
    RFBContext *s = avctx->priv_data;
    const ptrdiff_t linesize = s->last_frame->linesize[0];
    uint8_t *dst = s->last_frame->data[0];
    GetByteContext *gb = &s->gb;

    for (int j = y; j < y + h; j++) {
        uint8_t *line = dst + j * linesize + x*2;

        for (int i = 0; i < w; i++) {
            unsigned fill = bytestream2_get_be16(gb);

            AV_WL16(line + i*2, fill);
        }
    }
}

static void hextile_subrect16(AVCodecContext *avctx, int tx, int ty, int tw, int th,
                              unsigned *fg, unsigned *bg)
{
    RFBContext *s = avctx->priv_data;
    uint8_t *dst = s->last_frame->data[0];
    int b1, b2, sx, sy, sw, sh, nb_subrects;
    GetByteContext *gb = &s->gb;
    unsigned subencoding;

    subencoding = bytestream2_get_byte(gb);

    // Is it a raw-encoded sub-rectangle?
    if (subencoding & HextileRaw) {
        const ptrdiff_t linesize = s->last_frame->linesize[0];

        for (int j = ty; j < ty + th; j++) {
            uint8_t *line = dst + j * linesize + tx*2;

            for (int i = 0; i < tw; i++) {
                unsigned fill = bytestream2_get_be16(gb);

                AV_WL16(line + i*2, fill);
            }
        }
        return;
    }

    // Read and draw the background if specified.
    if (subencoding & HextileBackgroundSpecified)
        *bg = bytestream2_get_be16(gb);

    fill_rect16(avctx, *bg, tx, ty, tw, th);

    // Read the foreground color if specified.
    if (subencoding & HextileForegroundSpecified)
        *fg = bytestream2_get_be16(gb);

    // Done with this tile if there is no sub-rectangles.
    if (!(subencoding & HextileAnySubrects))
        return;

    nb_subrects = bytestream2_get_byte(gb);
    if (subencoding & HextileSubrectsColoured) {
        for (int j = 0; j < nb_subrects; j++) {
            unsigned cf = bytestream2_get_be16(gb);

            b1 = bytestream2_get_byte(gb);
            b2 = bytestream2_get_byte(gb);
            sx = tx + (b1 >> 4);
            sy = ty + (b1 & 0xf);
            sw = (b2 >> 4) + 1;
            sh = (b2 & 0xf) + 1;

            fill_rect16(avctx, cf, sx, sy, sw, sh);
        }
    } else {
        for (int j = 0; j < nb_subrects; j++) {
            b1 = bytestream2_get_byte(gb);
            b2 = bytestream2_get_byte(gb);
            sx = tx + (b1 >> 4);
            sy = ty + (b1 & 0xf);
            sw = (b2 >> 4) + 1;
            sh = (b2 & 0xf) + 1;

            fill_rect16(avctx, *fg, sx, sy, sw, sh);
        }
    }
}

static void hextile_rect16(AVCodecContext *avctx, int x, int y, int w, int h)
{
    uint32_t fg = 0, bg = 0;

    for (int ty = y; ty < y + h; ty += 16) {
        int th = 16;
        if (y + h - ty < 16)
            th = y + h - ty;

        for (int tx = x; tx < x + w; tx += 16) {
            int tw = 16;
            if (x + w - tx < 16)
                tw = x + w - tx;

            hextile_subrect16(avctx, tx, ty, tw, th, &fg, &bg);
        }
    }
}

static void copy_rect(AVCodecContext *avctx,
                      int x, int y, int w, int h)
{
    RFBContext *s = avctx->priv_data;
    const ptrdiff_t linesize = s->last_frame->linesize[0];
    uint8_t *dst = s->last_frame->data[0];
    GetByteContext *gb = &s->gb;

    for (int j = y; j < y + h; j++) {
        uint8_t *line = dst + j * linesize + x*4;

        for (int i = 0; i < w; i++) {
            uint32_t fill = bytestream2_get_be32(gb);

            AV_WL32(line + i*4, fill);
        }
    }
}

static void hextile_subrect(AVCodecContext *avctx, int tx, int ty, int tw, int th,
                            unsigned *fg, unsigned *bg)
{
    RFBContext *s = avctx->priv_data;
    uint8_t *dst = s->last_frame->data[0];
    int b1, b2, sx, sy, sw, sh, nb_subrects;
    GetByteContext *gb = &s->gb;
    unsigned subencoding;

    subencoding = bytestream2_get_byte(gb);

    // Is it a raw-encoded sub-rectangle?
    if (subencoding & HextileRaw) {
        const ptrdiff_t linesize = s->last_frame->linesize[0];

        for (int j = ty; j < ty + th; j++) {
            uint8_t *line = dst + j * linesize + tx*4;

            for (int i = 0; i < tw; i++) {
                unsigned color = bytestream2_get_be32(gb);

                AV_WL32(line + i*4, color);
            }
        }
        return;
    }

    // Read and draw the background if specified.
    if (subencoding & HextileBackgroundSpecified)
        *bg = bytestream2_get_be32(gb);

    fill_rect(avctx, *bg, tx, ty, tw, th);

    // Read the foreground color if specified.
    if (subencoding & HextileForegroundSpecified)
        *fg = bytestream2_get_be32(gb);

    // Done with this tile if there is no sub-rectangles.
    if (!(subencoding & HextileAnySubrects))
        return;

    nb_subrects = bytestream2_get_byte(gb);
    if (subencoding & HextileSubrectsColoured) {
        for (int j = 0; j < nb_subrects; j++) {
            unsigned cf = bytestream2_get_be32(gb);
            b1 = bytestream2_get_byte(gb);
            b2 = bytestream2_get_byte(gb);
            sx = tx + (b1 >> 4);
            sy = ty + (b1 & 0xf);
            sw = (b2 >> 4) + 1;
            sh = (b2 & 0xf) + 1;

            fill_rect(avctx, cf, sx, sy, sw, sh);
        }
    } else {
        for (int j = 0; j < nb_subrects; j++) {
            b1 = bytestream2_get_byte(gb);
            b2 = bytestream2_get_byte(gb);
            sx = tx + (b1 >> 4);
            sy = ty + (b1 & 0xf);
            sw = (b2 >> 4) + 1;
            sh = (b2 & 0xf) + 1;

            fill_rect(avctx, *fg, sx, sy, sw, sh);
        }
    }
}

static void hextile_rect(AVCodecContext *avctx, int x, int y, int w, int h)
{
    uint32_t fg = 0, bg = 0;

    for (int ty = y; ty < y + h; ty += 16) {
        int th = 16;
        if (y + h - ty < 16)
            th = y + h - ty;

        for (int tx = x; tx < x + w; tx += 16) {
            int tw = 16;
            if (x + w - tx < 16)
                tw = x + w - tx;

            hextile_subrect(avctx, tx, ty, tw, th, &fg, &bg);
        }
    }
}

static void move_rect(AVCodecContext *avctx, int srcx, int srcy,
                      int dstx, int dsty, int w, int h)
{
    RFBContext *s = avctx->priv_data;
    const ptrdiff_t linesize = s->last_frame->linesize[0];
    uint8_t *buffer = s->last_frame->data[0];

    if (dsty <= srcy) {
        for (int y = 0; y < h; y++)
            memmove(&buffer[((dsty + y) * linesize) + dstx*4],
                    &buffer[((srcy + y) * linesize) + srcx*4], w * 4);
    } else {
        for (int y = h-1; y >= 0; y--)
            memmove(&buffer[((dsty + y) * linesize) + dstx*4],
                    &buffer[((srcy + y) * linesize) + srcx*4], w * 4);
    }
}

static void move_rect16(AVCodecContext *avctx, int srcx, int srcy,
                        int dstx, int dsty, int w, int h)
{
    RFBContext *s = avctx->priv_data;
    const ptrdiff_t linesize = s->last_frame->linesize[0];
    uint8_t *buffer = s->last_frame->data[0];

    if (dsty <= srcy) {
        for (int y = 0; y < h; y++)
            memmove(&buffer[((dsty + y) * linesize) + dstx*2],
                    &buffer[((srcy + y) * linesize) + srcx*2], w*2);
    } else {
        for (int y = h-1; y >= 0; y--)
            memmove(&buffer[((dsty + y) * linesize) + dstx*2],
                    &buffer[((srcy + y) * linesize) + srcx*2], w*2);
    }
}

static int decode_rects8(AVCodecContext *avctx)
{
    RFBContext *s = avctx->priv_data;
    GetByteContext *gb = &s->gb;
    unsigned nrects;

    nrects = bytestream2_get_be16(gb);
    for (int i = 0; i < nrects; i++) {
        unsigned sx, sy, x, y, w, h, coding;

        x = bytestream2_get_be16(gb);
        y = bytestream2_get_be16(gb);
        w = bytestream2_get_be16(gb);
        h = bytestream2_get_be16(gb);

        if (x + w > avctx->width ||
            y + h > avctx->height) {
            return AVERROR_INVALIDDATA;
        }

        coding = bytestream2_get_be32(gb);
        s->key = (nrects == 1) && (x == 0) && (y == 0) && (w == avctx->width) && (h == avctx->height);

        switch (coding) {
        case 0:
            copy_rect8(avctx, x, y, w, h);
            break;
        case 1:
            sx = bytestream2_get_be16(gb);
            sy = bytestream2_get_be16(gb);
            move_rect(avctx, sx, sy, x, y, w, h);
            break;
        case 5:
            hextile_rect8(avctx, x, y, w, h);
            break;
        default:
            avpriv_request_sample(avctx, "Unsupported coding: %X", coding);
            break;
        }
    }

    return 0;
}

static int decode_rects16(AVCodecContext *avctx)
{
    RFBContext *s = avctx->priv_data;
    GetByteContext *gb = &s->gb;
    unsigned nrects;

    nrects = bytestream2_get_be16(gb);
    for (int i = 0; i < nrects; i++) {
        unsigned sx, sy, x, y, w, h, coding;

        x = bytestream2_get_be16(gb);
        y = bytestream2_get_be16(gb);
        w = bytestream2_get_be16(gb);
        h = bytestream2_get_be16(gb);

        if (x + w > avctx->width ||
            y + h > avctx->height) {
            return AVERROR_INVALIDDATA;
        }

        coding = bytestream2_get_be32(gb);
        s->key = (nrects == 1) && (x == 0) && (y == 0) && (w == avctx->width) && (h == avctx->height);

        switch (coding) {
        case 0:
            copy_rect16(avctx, x, y, w, h);
            break;
        case 1:
            sx = bytestream2_get_be16(gb);
            sy = bytestream2_get_be16(gb);
            move_rect16(avctx, sx, sy, x, y, w, h);
            break;
        case 5:
            hextile_rect16(avctx, x, y, w, h);
            break;
        default:
            avpriv_request_sample(avctx, "Unsupported coding: %X", coding);
            break;
        }
    }

    return 0;
}

static int decode_rects32(AVCodecContext *avctx)
{
    RFBContext *s = avctx->priv_data;
    GetByteContext *gb = &s->gb;
    unsigned nrects;

    nrects = bytestream2_get_be16(gb);
    for (int i = 0; i < nrects; i++) {
        unsigned sx, sy, x, y, w, h, coding;

        x = bytestream2_get_be16(gb);
        y = bytestream2_get_be16(gb);
        w = bytestream2_get_be16(gb);
        h = bytestream2_get_be16(gb);

        if (x + w > avctx->width ||
            y + h > avctx->height)
            return AVERROR_INVALIDDATA;

        coding = bytestream2_get_be32(gb);
        s->key = (nrects == 1) && (x == 0) && (y == 0) && (w == avctx->width) && (h == avctx->height);

        switch (coding) {
        case 0:
            copy_rect(avctx, x, y, w, h);
            break;
        case 1:
            sx = bytestream2_get_be16(gb);
            sy = bytestream2_get_be16(gb);
            move_rect(avctx, sx, sy, x, y, w, h);
            break;
        case 5:
            hextile_rect(avctx, x, y, w, h);
            break;
        default:
            avpriv_request_sample(avctx, "Unsupported coding: %X", coding);
            return AVERROR_PATCHWELCOME;
        }
    }

    return 0;
}

static int decode_cmap(AVCodecContext *avctx)
{
    RFBContext *s = avctx->priv_data;
    GetByteContext *gb = &s->gb;
    unsigned nb_colors;

    bytestream2_skip(gb, 2);
    bytestream2_get_be16(gb);
    nb_colors = bytestream2_get_be16(gb);
    for (int i = 0; i < nb_colors; i++)
        bytestream2_skip(gb, 6);

    return 0;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame_ptr, AVPacket *avpkt)
{
    RFBContext *s = avctx->priv_data;
    int buf_size, input_buf_size;
    GetByteContext *gb = &s->gb;
    const uint8_t *buf;
    int n = 0, ret, type;

    if (!avpkt->size && s->bytestream_start >= s->bytestream_stop) {
        *got_frame_ptr = 0;
        return 0;
    }

    buf_size = FFMIN(avpkt->size, s->max_framesize - s->bytestream_stop);
    input_buf_size = buf_size;
    if (s->bytestream_start > 0) {
        memmove(s->bytestream, &s->bytestream[s->bytestream_start], s->bytestream_stop - s->bytestream_start);
        s->bytestream_stop -= s->bytestream_start;
        s->bytestream_start = 0;
    }
    if (avpkt->data) {
        memcpy(&s->bytestream[s->bytestream_stop], avpkt->data, buf_size);
        s->bytestream_stop += buf_size;
    }
    if (s->bytestream_stop < s->max_framesize && avpkt->data) {
        Packet packet;

        packet.size = avpkt->size;
        packet.pts = avpkt->pts;
        av_fifo_write(s->fifo, &packet, 1);

        *got_frame_ptr = 0;
        return input_buf_size;
    }

    buf = &s->bytestream[s->bytestream_start];
    buf_size = s->bytestream_stop - s->bytestream_start;

    bytestream2_init(gb, buf, buf_size);

    if ((ret = ff_reget_buffer(avctx, s->last_frame, 0)) < 0)
        goto fail;

    s->key = 0;
    type = bytestream2_peek_byte(gb);
    switch (type) {
    case 0:
        bytestream2_skip(gb, 2);

        switch (s->bpp) {
        case 32:
        case 24:
            ret = decode_rects32(avctx);
            break;
        case 16:
            ret = decode_rects16(avctx);
            break;
        case 8:
            ret = decode_rects8(avctx);
            break;
        }
        break;
    case 1:
        ret = decode_cmap(avctx);
        break;
    case 2:
        bytestream2_skip(gb, 1);
        break;
    case 3:
        bytestream2_skip(gb, 4);
        bytestream2_skip(gb, bytestream2_get_be32(gb));
        break;
    default:
        bytestream2_skip(gb, 1);
        avpriv_request_sample(avctx, "Unsupported type: %X", type);
        break;
    }

    if (ret < 0)
        goto fail;

    n = bytestream2_tell(gb);

    ret = av_frame_ref(frame, s->last_frame);
    if (ret < 0)
        goto fail;

    if (n > buf_size) {
fail:
        s->bytestream_stop = 0;
        s->bytestream_start = 0;
        return ret;
    }

    if (s->key)
        frame->flags |= AV_FRAME_FLAG_KEY;
    frame->pict_type = (frame->flags & AV_FRAME_FLAG_KEY) ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

    *got_frame_ptr = 1;

    {
        int64_t pts = AV_NOPTS_VALUE;
        int skip = n;

        while (skip > 0) {
            Packet packet;

            if (av_fifo_can_read(s->fifo) <= 0)
                break;

            av_fifo_peek(s->fifo, &packet, 1, 0);

            pts = packet.pts;
            av_fifo_drain2(s->fifo, 1);
            skip -= packet.size;
        }

        frame->pts = pts;
    }

    s->bytestream_start += n;
    return input_buf_size;
}

static av_cold int decode_close(AVCodecContext *avctx)
{
    RFBContext *s = avctx->priv_data;

    av_frame_free(&s->last_frame);
    av_fifo_freep2(&s->fifo);
    av_freep(&s->bytestream);

    return 0;
}

const FFCodec ff_rfb_decoder = {
    .p.name           = "rfb",
    CODEC_LONG_NAME("RFB (Remote FrameBuffer)"),
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_RFB,
    .priv_data_size   = sizeof(RFBContext),
    .init             = decode_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .close            = decode_close,
    .caps_internal    = FF_CODEC_CAP_INIT_CLEANUP,
    .p.capabilities   = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_DELAY,
};
