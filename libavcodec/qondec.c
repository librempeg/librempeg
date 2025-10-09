/*
 * QON video format
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

typedef struct QONContext {
    AVFrame  *previous_frame;

    int channels;
} QONContext;

static av_cold int decode_init(AVCodecContext *avctx)
{
    QONContext *q = avctx->priv_data;
    int space;

    if (!avctx->extradata || avctx->extradata_size < 4)
        return AVERROR_INVALIDDATA;

    q->channels = avctx->extradata[0];
    space = avctx->extradata[1];
    if (q->channels < 3 || q->channels > 4)
        return AVERROR_INVALIDDATA;

    switch (q->channels) {
    case 3: avctx->pix_fmt = AV_PIX_FMT_RGB24; break;
    case 4: avctx->pix_fmt = AV_PIX_FMT_RGBA;  break;
    default: return AVERROR_INVALIDDATA;
    }

    switch (space) {
    case 0: break;
    case 1: avctx->color_trc = AVCOL_TRC_LINEAR; break;
    default: return AVERROR_INVALIDDATA;
    }

    q->previous_frame = av_frame_alloc();
    if (!q->previous_frame)
        return AVERROR(ENOMEM);

    return 0;
}

#define QOI2_OP_INDEX  0x00 /* 00xxxxxx */
#define QOI2_OP_DIFF   0x40 /* 01xxxxxx */
#define QOI2_OP_RGBA   0x6a /* 01101010 */
#define QOI2_OP_LUMA   0x80 /* 10xxxxxx */
#define QOI2_OP_RUN    0xc0 /* 110xxxxx */
#define QOI2_OP_RUN_EX 0xe0 /* 111xxxxx */
#define QOI2_OP_RGB_EX 0xfc /* 111111xx */

#define QOI2_MASK_2    0xc0 /* 11000000 */
#define QOI2_MASK_3    0xe0 /* 11100000 */

#define QOI2_COLOR_HASH(px) (px[0]*3 + px[1]*5 + px[2]*7 + px[3]*11)

static int decode_frame(AVCodecContext *avctx, AVFrame *p,
                        int *got_frame, AVPacket *avpkt)
{
    QONContext *q = avctx->priv_data;
    AVFrame *prev_pic = q->previous_frame;
    uint8_t *prev = prev_pic->data[0];
    const int channels = q->channels;
    const int width = avctx->width;
    const int height = avctx->height;
    uint8_t index[64][4] = { 0 };
    uint8_t px[4] = { 0, 0, 0, 255 };
    int ret, run = 0, rgb_run = 0;
    GetByteContext gb;
    uint8_t *dst;
    uint64_t len;

    bytestream2_init(&gb, avpkt->data, avpkt->size);

    if (avctx->skip_frame >= AVDISCARD_ALL)
        return avpkt->size;

    if (avpkt->flags & AV_PKT_FLAG_KEY) {
        p->flags |= AV_FRAME_FLAG_KEY;
        p->pict_type = AV_PICTURE_TYPE_I;
    } else {
        if (!prev) {
            av_log(avctx, AV_LOG_ERROR, "Missing reference frame.\n");
            return AVERROR_INVALIDDATA;
        }

        p->flags &= ~AV_FRAME_FLAG_KEY;
        p->pict_type = AV_PICTURE_TYPE_P;
    }

    if ((ret = ff_get_buffer(avctx, p, AV_GET_BUFFER_FLAG_REF)) < 0)
        return ret;

    dst = p->data[0];
    len = width * height * channels;
    for (int n = 0, off_x = 0; n < len; n += channels, off_x++) {
        if (off_x >= width) {
            off_x = 0;
            dst += p->linesize[0];
        }
        if (run > 0) {
            run--;
            if (rgb_run) {
                bytestream2_get_buffer(&gb, px, 3);
                memcpy(index[QOI2_COLOR_HASH(px) & 63], px, 4);
            }
        } else if (bytestream2_get_bytes_left(&gb) >= 1) {
            int chunk = bytestream2_get_byteu(&gb);

            rgb_run = 0;
            if (chunk == QOI2_OP_RGBA) {
                bytestream2_get_buffer(&gb, px, 4);
            } else if ((chunk & QOI2_OP_RGB_EX) == QOI2_OP_RGB_EX) {
                bytestream2_get_buffer(&gb, px, 3);
                run = chunk & 0x3;
                rgb_run = 1;
            } else if ((chunk & QOI2_MASK_2) == QOI2_OP_INDEX) {
                memcpy(px, index[chunk], 4);
            } else if ((chunk & QOI2_MASK_2) == QOI2_OP_DIFF) {
                px[0] += ((chunk >> 4) & 0x03) - 2;
                px[1] += ((chunk >> 2) & 0x03) - 2;
                px[2] += ( chunk       & 0x03) - 2;
            } else if ((chunk & QOI2_MASK_2) == QOI2_OP_LUMA) {
                int b2 = bytestream2_get_byte(&gb);
                int vg = (chunk & 0x3f) - 32;
                px[0] += vg - 8 + ((b2 >> 4) & 0x0f);
                px[1] += vg;
                px[2] += vg - 8 +  (b2       & 0x0f);
            } else if ((chunk & QOI2_MASK_3) == QOI2_OP_RUN) {
                run = chunk & 0x1f;
            } else if ((chunk & QOI2_MASK_3) == QOI2_OP_RUN_EX) {
                run = (((chunk & 0x1f) + 2) * 32) - 1;
            }

            memcpy(index[QOI2_COLOR_HASH(px) & 63], px, 4);
        } else {
            break;
        }

        memcpy(&dst[off_x * channels], px, channels);
    }

    if (!(avpkt->flags & AV_PKT_FLAG_KEY)) {
        dst = p->data[0];
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width * channels; j++)
                dst[j] += prev[j];
            prev += prev_pic->linesize[0];
            dst  += p->linesize[0];
        }
    }

    if ((ret = av_frame_replace(q->previous_frame, p)) < 0)
        return ret;

    *got_frame = 1;

    return avpkt->size;
}

static av_cold int decode_close(AVCodecContext *avctx)
{
    QONContext *q = avctx->priv_data;

    av_frame_free(&q->previous_frame);

    return 0;
}

static av_cold void decode_flush(AVCodecContext *avctx)
{
    QONContext *q = avctx->priv_data;

    av_frame_unref(q->previous_frame);
}

const FFCodec ff_qon_decoder = {
    .p.name         = "qon",
    CODEC_LONG_NAME("QON (Quite OK aNimation)"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_QON,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .priv_data_size = sizeof(QONContext),
    .caps_internal  = FF_CODEC_CAP_SKIP_FRAME_FILL_PARAM,
    .init           = decode_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .flush          = decode_flush,
    .close          = decode_close,
};
