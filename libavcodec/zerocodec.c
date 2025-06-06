/*
 * ZeroCodec Decoder
 *
 * Copyright (c) 2012, Derek Buitenhuis
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "inflate.h"
#include "libavutil/common.h"

typedef struct ZeroCodecContext {
    AVFrame  *previous_frame;

    InflateContext ic;
} ZeroCodecContext;

static int zerocodec_decode_frame(AVCodecContext *avctx, AVFrame *pic,
                                  int *got_frame, AVPacket *avpkt)
{
    ZeroCodecContext *zc = avctx->priv_data;
    AVFrame *prev_pic    = zc->previous_frame;
    uint8_t *prev        = prev_pic->data[0];
    uint8_t *dst;
    int ret;

    if (avpkt->flags & AV_PKT_FLAG_KEY) {
        pic->flags |= AV_FRAME_FLAG_KEY;
        pic->pict_type = AV_PICTURE_TYPE_I;
    } else {
        if (!prev) {
            av_log(avctx, AV_LOG_ERROR, "Missing reference frame.\n");
            return AVERROR_INVALIDDATA;
        }

        prev += (avctx->height - 1) * prev_pic->linesize[0];

        pic->flags &= ~AV_FRAME_FLAG_KEY;
        pic->pict_type = AV_PICTURE_TYPE_P;
    }

    if ((ret = ff_get_buffer(avctx, pic, AV_GET_BUFFER_FLAG_REF)) < 0)
        return ret;

    dst = pic->data[0] + (avctx->height - 1) * pic->linesize[0];

    /**
     * ZeroCodec has very simple interframe compression. If a value
     * is the same as the previous frame, set it to 0.
     */
    ret = ff_inflate(&zc->ic, avpkt->data, avpkt->size,
                     dst, avctx->height, avctx->width*2,
                     -pic->linesize[0]);
    if (ret < 0)
        return ret;

    if (!(avpkt->flags & AV_PKT_FLAG_KEY)) {
        for (int i = 0; i < avctx->height; i++) {
            for (int j = 0; j < avctx->width << 1; j++)
                dst[j] += prev[j] & -!dst[j];
            prev -= prev_pic->linesize[0];
            dst  -= pic->linesize[0];
        }
    }

    if ((ret = av_frame_replace(zc->previous_frame, pic)) < 0)
        return ret;

    *got_frame = 1;

    return avpkt->size;
}

static av_cold int zerocodec_decode_close(AVCodecContext *avctx)
{
    ZeroCodecContext *zc = avctx->priv_data;

    av_frame_free(&zc->previous_frame);

    return 0;
}

static av_cold int zerocodec_decode_init(AVCodecContext *avctx)
{
    ZeroCodecContext *zc = avctx->priv_data;

    avctx->pix_fmt             = AV_PIX_FMT_UYVY422;
    avctx->bits_per_raw_sample = 8;

    zc->previous_frame = av_frame_alloc();
    if (!zc->previous_frame)
        return AVERROR(ENOMEM);

    return 0;
}

static void zerocodec_decode_flush(AVCodecContext *avctx)
{
    ZeroCodecContext *zc = avctx->priv_data;

    ff_inflate(&zc->ic, NULL, 0, NULL, 0, 0, 0);

    av_frame_unref(zc->previous_frame);
}

const FFCodec ff_zerocodec_decoder = {
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.name         = "zerocodec",
    CODEC_LONG_NAME("ZeroCodec Lossless Video"),
    .p.id           = AV_CODEC_ID_ZEROCODEC,
    .priv_data_size = sizeof(ZeroCodecContext),
    .init           = zerocodec_decode_init,
    FF_CODEC_DECODE_CB(zerocodec_decode_frame),
    .flush          = zerocodec_decode_flush,
    .close          = zerocodec_decode_close,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
