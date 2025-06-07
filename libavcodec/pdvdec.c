/*
 * PDV video format
 *
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

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "inflate.h"

typedef struct PDVContext {
    AVFrame  *previous_frame;
    InflateContext ic;
} PDVContext;

static av_cold int decode_init(AVCodecContext *avctx)
{
    PDVContext *s = avctx->priv_data;

    avctx->pix_fmt = AV_PIX_FMT_MONOBLACK;

    s->previous_frame = av_frame_alloc();
    if (!s->previous_frame)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold int decode_end(AVCodecContext *avctx)
{
    PDVContext *s = avctx->priv_data;

    av_frame_free(&s->previous_frame);

    return 0;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame, AVPacket *avpkt)
{
    PDVContext *s = avctx->priv_data;
    AVFrame *prev_frame = s->previous_frame;
    InflateContext *ic = &s->ic;
    uint8_t *dst, *prev = prev_frame->data[0];
    int ret;

    if (avctx->skip_frame >= AVDISCARD_ALL ||
        (avctx->skip_frame >= AVDISCARD_NONINTRA &&
         !(avpkt->flags & AV_PKT_FLAG_KEY)))
        return avpkt->size;

    if ((ret = ff_get_buffer(avctx, frame, AV_GET_BUFFER_FLAG_REF)) < 0)
        return ret;

    dst = frame->data[0];
    ret = ff_inflate(ic, avpkt->data, avpkt->size,
                     dst, avctx->height,
                     (avctx->width + 7) >> 3,
                     frame->linesize[0]);
    if (ret < 0)
        return ret;

    if (prev && !(avpkt->flags & AV_PKT_FLAG_KEY)) {
        for (int i = 0; i < avctx->height; i++) {
            for (int j = 0; j < (avctx->width + 7) >> 3; j++)
                dst[j] ^= prev[j];

            dst += frame->linesize[0];
            prev += prev_frame->linesize[0];
        }
    }

    if ((ret = av_frame_replace(s->previous_frame, frame)) < 0)
        return ret;

    if (avpkt->flags & AV_PKT_FLAG_KEY) {
        frame->flags |= AV_FRAME_FLAG_KEY;
        frame->pict_type = AV_PICTURE_TYPE_I;
    } else {
        frame->pict_type = AV_PICTURE_TYPE_P;
    }

    *got_frame = 1;

    return avpkt->size;
}

static void decode_flush(AVCodecContext *avctx)
{
    PDVContext *s = avctx->priv_data;

    ff_inflate(&s->ic, NULL, 0, NULL, 0, 0, 0);

    av_frame_unref(s->previous_frame);
}

const FFCodec ff_pdv_decoder = {
    .p.name         = "pdv",
    CODEC_LONG_NAME("PDV (PlayDate Video)"),
    .priv_data_size = sizeof(PDVContext),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_PDV,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_SKIP_FRAME_FILL_PARAM |
                      FF_CODEC_CAP_INIT_CLEANUP,
    .init           = decode_init,
    .close          = decode_end,
    .flush          = decode_flush,
    FF_CODEC_DECODE_CB(decode_frame),
};
