/*
 * WinCAM Motion Video decoder
 *
 * Copyright (c) 2018 Paul B Mahol
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

#include "libavutil/imgutils.h"

#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "inflate.h"

typedef struct WCMVContext {
    int         bpp;
    InflateContext ic;
    GetByteContext gb;
    int         width, height;
    int         x, y, w, h, cy;
    int         block;
    int         blocks;
    int         intra;
    AVFrame    *prev_frame;
    uint8_t     block_data[65536*8];
    uint8_t     tmp[32768];
} WCMVContext;

static void handle_block_row(void *arg, uint8_t *dst, ptrdiff_t dst_stride,
                             uint8_t *tmp, int *cy, int *ow, const int uh)
{
    WCMVContext *s = arg;
    GetByteContext *gb = &s->gb;
    AVFrame *frame = s->prev_frame;

    dst = frame->data[0] + (s->height - s->y - s->cy - 1) * frame->linesize[0] + s->x * s->bpp;
    memcpy(dst, tmp, s->w * s->bpp);

    s->cy++;
    *cy = s->cy;
    if (s->cy >= s->h) {
        if (s->block >= s->blocks)
            return;

        s->x = bytestream2_get_le16(gb);
        s->y = bytestream2_get_le16(gb);
        s->w = bytestream2_get_le16(gb);
        s->h = bytestream2_get_le16(gb);

        if (s->blocks == 1 && s->x == 0 && s->y == 0 &&
            s->w == s->width && s->h == s->height)
            s->intra = 1;

        if (s->x + s->w > s->width || s->y + s->h > s->height)
            return;

        if (s->w > s->width || s->h > s->height)
            return;

        *ow = s->w * s->bpp;
        s->cy = *cy = 0;
        s->block++;
    }
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame, AVPacket *avpkt)
{
    WCMVContext *s = avctx->priv_data;
    InflateContext *ic = &s->ic;
    int skip, blocks, ret, flags = 0, bpp = s->bpp;
    GetByteContext gb;

    bytestream2_init(&gb, avpkt->data, avpkt->size);
    blocks = bytestream2_get_le16(&gb);
    if (!blocks)
        flags |= FF_REGET_BUFFER_FLAG_READONLY;

    if ((ret = ff_reget_buffer(avctx, s->prev_frame, flags)) < 0)
        return ret;

    s->width = avctx->width;
    s->height = avctx->height;

    if (blocks > 5) {
        GetByteContext bgb;
        int x = 0, size;

        if (blocks * 8 >= 0xFFFF)
            size = bytestream2_get_le24(&gb);
        else if (blocks * 8 >= 0xFF)
            size = bytestream2_get_le16(&gb);
        else
            size = bytestream2_get_byte(&gb);

        skip = bytestream2_tell(&gb);
        if (size > avpkt->size - skip)
            return AVERROR_INVALIDDATA;

        ret = ff_inflate(ic, avpkt->data + skip, size, s->block_data, 1, sizeof(s->block_data), sizeof(s->block_data));
        if (ret < 0)
            return ret;

        bytestream2_skip(&gb, size);
        bytestream2_init(&bgb, s->block_data, blocks * 8);

        for (int i = 0; i < blocks; i++) {
            int w, h;

            bytestream2_skip(&bgb, 4);
            w = bytestream2_get_le16(&bgb);
            h = bytestream2_get_le16(&bgb);
            if (x + bpp * (int64_t)w * h > INT_MAX)
                return AVERROR_INVALIDDATA;
            x += bpp * w * h;
        }

        if (x >= 0xFFFF)
            bytestream2_skip(&gb, 3);
        else if (x >= 0xFF)
            bytestream2_skip(&gb, 2);
        else
            bytestream2_skip(&gb, 1);

        skip = bytestream2_tell(&gb);

        bytestream2_init(&gb, s->block_data, blocks * 8);
    } else if (blocks) {
        int x = 0;

        bytestream2_seek(&gb, 2, SEEK_SET);

        for (int i = 0; i < blocks; i++) {
            int w, h;

            bytestream2_skip(&gb, 4);
            w = bytestream2_get_le16(&gb);
            h = bytestream2_get_le16(&gb);
            if (x + bpp * (int64_t)w * h > INT_MAX)
                return AVERROR_INVALIDDATA;
            x += bpp * w * h;
        }

        if (x >= 0xFFFF)
            bytestream2_skip(&gb, 3);
        else if (x >= 0xFF)
            bytestream2_skip(&gb, 2);
        else
            bytestream2_skip(&gb, 1);

        skip = bytestream2_tell(&gb);

        bytestream2_seek(&gb, 2, SEEK_SET);
    }

    if (bytestream2_get_bytes_left(&gb) < 8LL * blocks)
        return AVERROR_INVALIDDATA;

    if (!avctx->frame_num) {
        ptrdiff_t linesize[4] = { s->prev_frame->linesize[0], 0, 0, 0 };
        av_image_fill_black(s->prev_frame->data, linesize, avctx->pix_fmt, 0,
                            avctx->width, avctx->height);
    }

    if (blocks > 0) {
        s->x = bytestream2_get_le16(&gb);
        s->y = bytestream2_get_le16(&gb);
        s->w = bytestream2_get_le16(&gb);
        s->h = bytestream2_get_le16(&gb);

        s->gb = gb;
        s->cy = 0;
        s->block = 0;
        s->intra = 0;
        s->blocks = blocks;

        if (s->blocks == 1 && s->x == 0 && s->y == 0 &&
            s->w == frame->width && s->h == frame->height)
            s->intra = 1;

        if (s->x + s->w > avctx->width || s->y + s->h > avctx->height)
            return AVERROR_INVALIDDATA;

        if (s->w > avctx->width || s->h > avctx->height)
            return AVERROR_INVALIDDATA;

        ret = ff_inflatex(ic, avpkt->data + skip, avpkt->size - skip, s->prev_frame->data[0],
                          avctx->height, s->w * bpp, -s->prev_frame->linesize[0], s, s->tmp, handle_block_row);
        if (ret < 0)
            return ret;
    }

    if (s->intra)
        s->prev_frame->flags |= AV_FRAME_FLAG_KEY;
    else
        s->prev_frame->flags &= ~AV_FRAME_FLAG_KEY;
    s->prev_frame->pict_type = s->intra ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

    if ((ret = av_frame_ref(frame, s->prev_frame)) < 0)
        return ret;

    *got_frame = 1;

    return avpkt->size;
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    WCMVContext *s = avctx->priv_data;

    switch (avctx->bits_per_coded_sample) {
    case 16: avctx->pix_fmt = AV_PIX_FMT_RGB565LE; break;
    case 24: avctx->pix_fmt = AV_PIX_FMT_BGR24;  break;
    case 32: avctx->pix_fmt = AV_PIX_FMT_BGRA;   break;
    default: av_log(avctx, AV_LOG_ERROR, "Unsupported bits_per_coded_sample: %d\n",
                    avctx->bits_per_coded_sample);
             return AVERROR_PATCHWELCOME;
    }

    s->bpp = avctx->bits_per_coded_sample >> 3;

    s->prev_frame = av_frame_alloc();
    if (!s->prev_frame)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold int decode_close(AVCodecContext *avctx)
{
    WCMVContext *s = avctx->priv_data;

    av_frame_free(&s->prev_frame);
    ff_inflate(&s->ic, NULL, 0, NULL, 0, 0, 0);

    return 0;
}

const FFCodec ff_wcmv_decoder = {
    .p.name           = "wcmv",
    CODEC_LONG_NAME("WinCAM Motion Video"),
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_WCMV,
    .priv_data_size   = sizeof(WCMVContext),
    .init             = decode_init,
    .close            = decode_close,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities   = AV_CODEC_CAP_DR1,
    .caps_internal    = FF_CODEC_CAP_INIT_CLEANUP,
};
