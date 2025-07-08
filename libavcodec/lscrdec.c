/*
 * Copyright (c) 2019 Paul B Mahol
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

#include <stdint.h>

#include "libavutil/frame.h"
#include "libavutil/error.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"

#include "avcodec.h"
#include "bytestream.h"
#include "codec.h"
#include "codec_internal.h"
#include "decode.h"
#include "packet.h"
#include "png.h"
#include "pngdsp.h"
#include "inflate.h"

typedef struct LSCRContext {
    PNGDSPContext   dsp;
    AVCodecContext *avctx;

    AVFrame        *last_picture;
    AVFrame        *filter_frame;
    uint8_t        *last_row;
    unsigned int    last_row_size;
    uint8_t        *comp_buf;
    unsigned int    comp_buf_size;

    GetByteContext  gb;
    uint8_t        *image_buf;
    int             image_linesize;
    int             row_size;
    int             cur_h;
    int             y;

    InflateContext  ic;
} LSCRContext;

static void handle_row(LSCRContext *s)
{
    uint8_t *ptr, *last_row, *filter_row;

    filter_row = s->filter_frame->data[0] + s->filter_frame->linesize[0] * s->y;
    ptr = s->image_buf + s->image_linesize * s->y;
    if (s->y == 0)
        last_row = s->last_row;
    else
        last_row = ptr - s->image_linesize;

    ff_png_filter_row(&s->dsp, ptr, filter_row[15], filter_row + 16,
                      last_row, s->row_size, 3);

    s->y++;
}

static int decode_idat(LSCRContext *s, int length)
{
    InflateContext *ic = &s->ic;
    uint8_t *filter = s->filter_frame->data[0] + 15;
    int ret;

    ret = ff_inflate(ic, s->comp_buf, length, filter, s->cur_h, s->row_size + 1,
                     s->filter_frame->linesize[0]);
    if (ret < 0)
        return ret;

    for (int y = 0; y < s->cur_h; y++)
        handle_row(s);

    return 0;
}

static int decode_frame_lscr(AVCodecContext *avctx, AVFrame *rframe,
                             int *got_frame, AVPacket *avpkt)
{
    LSCRContext *const s = avctx->priv_data;
    GetByteContext *gb = &s->gb;
    AVFrame *frame = s->last_picture;
    int ret, nb_blocks, offset = 0;

    if (avpkt->size < 2)
        return AVERROR_INVALIDDATA;
    if (avpkt->size == 2)
        return 0;

    bytestream2_init(gb, avpkt->data, avpkt->size);

    nb_blocks = bytestream2_get_le16(gb);
    if (bytestream2_get_bytes_left(gb) < 2 + nb_blocks * (12 + 8))
        return AVERROR_INVALIDDATA;

    ret = ff_reget_buffer(avctx, frame,
                          nb_blocks ? 0 : FF_REGET_BUFFER_FLAG_READONLY);
    if (ret < 0)
        return ret;

    for (int b = 0; b < nb_blocks; b++) {
        int x, y, x2, y2, w, h, left;
        uint32_t csize, size, comp_size = 0;

        bytestream2_seek(gb, 2 + b * 12, SEEK_SET);

        x = bytestream2_get_le16(gb);
        y = bytestream2_get_le16(gb);
        x2 = bytestream2_get_le16(gb);
        y2 = bytestream2_get_le16(gb);
        w = x2-x;
        s->cur_h = h = y2-y;

        if (w <= 0 || x < 0 || x >= avctx->width || w + x > avctx->width ||
            h <= 0 || y < 0 || y >= avctx->height || h + y > avctx->height)
            return AVERROR_INVALIDDATA;

        size = bytestream2_get_le32(gb);

        if ((nb_blocks == 1) &&
            (w == avctx->width) &&
            (h == avctx->height) &&
            (x == 0) && (y == 0))
            frame->flags |= AV_FRAME_FLAG_KEY;
        else
            frame->flags &= ~AV_FRAME_FLAG_KEY;

        bytestream2_seek(gb, 2 + nb_blocks * 12 + offset, SEEK_SET);
        csize = bytestream2_get_be32(gb);
        if (bytestream2_get_le32(gb) != MKTAG('I', 'D', 'A', 'T'))
            return AVERROR_INVALIDDATA;

        offset += size;
        left = size;

        s->y                 = 0;
        s->row_size          = w * 3;

        av_fast_padded_malloc(&s->last_row, &s->last_row_size, s->row_size);
        if (!s->last_row)
            return AVERROR(ENOMEM);

        av_fast_padded_malloc(&s->comp_buf, &s->comp_buf_size, size);
        if (!s->last_row)
            return AVERROR(ENOMEM);

        s->image_buf         = frame->data[0] + (avctx->height - y - 1) * frame->linesize[0] + x * 3;
        s->image_linesize    =-frame->linesize[0];

        while (left > 12) {
            memcpy(s->comp_buf + comp_size, gb->buffer, csize);
            bytestream2_skip(gb, csize);
            comp_size += csize;
            left -= csize + 12;
            if (left > 12) {
                bytestream2_skip(gb, 4);
                csize = bytestream2_get_be32(gb);
                if (bytestream2_get_le32(gb) != MKTAG('I', 'D', 'A', 'T'))
                    return AVERROR_INVALIDDATA;
            }
        }

        ret = decode_idat(s, comp_size);
        if (ret < 0)
            return ret;
    }

    frame->pict_type = (frame->flags & AV_FRAME_FLAG_KEY) ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

    if ((ret = av_frame_ref(rframe, frame)) < 0)
        return ret;

    *got_frame = 1;

    return avpkt->size;
}

static av_cold int lscr_decode_close(AVCodecContext *avctx)
{
    LSCRContext *s = avctx->priv_data;

    av_frame_free(&s->last_picture);
    av_frame_free(&s->filter_frame);
    av_freep(&s->last_row);
    av_freep(&s->comp_buf);

    return 0;
}

static av_cold int lscr_decode_init(AVCodecContext *avctx)
{
    LSCRContext *s = avctx->priv_data;
    int ret;

    avctx->color_range = AVCOL_RANGE_JPEG;
    avctx->pix_fmt     = AV_PIX_FMT_BGR24;

    s->avctx = avctx;
    s->last_picture = av_frame_alloc();
    if (!s->last_picture)
        return AVERROR(ENOMEM);

    s->filter_frame = av_frame_alloc();
    if (!s->filter_frame)
        return AVERROR(ENOMEM);

    s->filter_frame->height = avctx->height;
    s->filter_frame->width = avctx->width*3 + 16;
    s->filter_frame->format = AV_PIX_FMT_GRAY8;
    if ((ret = ff_get_buffer(avctx, s->filter_frame, 0)) < 0)
        return ret;

    ff_pngdsp_init(&s->dsp);

    return 0;
}

static av_cold void lscr_decode_flush(AVCodecContext *avctx)
{
    LSCRContext *s = avctx->priv_data;

    av_frame_unref(s->last_picture);
    ff_inflate(&s->ic, NULL, 0, NULL, 0, 0, 0);
}

const FFCodec ff_lscr_decoder = {
    .p.name         = "lscr",
    CODEC_LONG_NAME("LEAD Screen Capture"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_LSCR,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .priv_data_size = sizeof(LSCRContext),
    .init           = lscr_decode_init,
    .close          = lscr_decode_close,
    FF_CODEC_DECODE_CB(decode_frame_lscr),
    .flush          = lscr_decode_flush,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
