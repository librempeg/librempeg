/*
 * CamStudio decoder
 * Copyright (c) 2006 Reimar Doeffinger
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

#include "libavutil/mem.h"
#include "libavutil/lzo.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "inflate.h"

typedef struct CamStudioContext {
    AVFrame *pic;
    int linelen, height, bpp;
    unsigned int decomp_size;
    unsigned char* decomp_buf;

    InflateContext ic;
} CamStudioContext;

static void copy_frame_default(AVFrame *f, const uint8_t *src,
                               int linelen, int height)
{
    int i, src_stride = FFALIGN(linelen, 4);
    uint8_t *dst = f->data[0];
    dst += (height - 1) * f->linesize[0];
    for (i = height; i; i--) {
        memcpy(dst, src, linelen);
        src += src_stride;
        dst -= f->linesize[0];
    }
}

static void add_frame_default(AVFrame *f, const uint8_t *src,
                              int linelen, int height)
{
    int i, j, src_stride = FFALIGN(linelen, 4);
    uint8_t *dst = f->data[0];
    dst += (height - 1) * f->linesize[0];
    for (i = height; i; i--) {
        for (j = linelen; j; j--)
            *dst++ += *src++;
        src += src_stride - linelen;
        dst -= f->linesize[0] + linelen;
    }
}

static int decode_frame(AVCodecContext *avctx, AVFrame *rframe,
                        int *got_frame, AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    CamStudioContext *c = avctx->priv_data;
    int ret;
    int bpp = avctx->bits_per_coded_sample / 8;
    int bugdelta = FFALIGN(avctx->width * bpp, 4)       * avctx->height
                 -        (avctx->width     & ~3) * bpp * avctx->height;

    if (buf_size < 2) {
        av_log(avctx, AV_LOG_ERROR, "coded frame too small\n");
        return AVERROR_INVALIDDATA;
    }

    if ((ret = ff_reget_buffer(avctx, c->pic, 0)) < 0)
        return ret;

    // decompress data
    switch ((buf[0] >> 1) & 7) {
    case 0: { // lzo compression
        int outlen = c->decomp_size, inlen = buf_size - 2;
        if (av_lzo1x_decode(c->decomp_buf, &outlen, &buf[2], &inlen) || (outlen && outlen != bugdelta)) {
            av_log(avctx, AV_LOG_ERROR, "error during lzo decompression\n");
            return AVERROR_INVALIDDATA;
        }
        break;
    }
    case 1: // zlib compression
        if ((ret = ff_inflate(&c->ic, &buf[2], buf_size - 2, c->decomp_buf, c->height, c->linelen, c->linelen)) < 0) {
            av_log(avctx, AV_LOG_ERROR, "error during zlib decompression\n");
            return ret;
        }
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "unknown compression\n");
        return AVERROR_INVALIDDATA;
    }

    // flip upside down, add difference frame
    if (buf[0] & 1) { // keyframe
        c->pic->pict_type = AV_PICTURE_TYPE_I;
        c->pic->flags |= AV_FRAME_FLAG_KEY;
              copy_frame_default(c->pic, c->decomp_buf,
                                 c->linelen, c->height);
    } else {
        c->pic->pict_type = AV_PICTURE_TYPE_P;
        c->pic->flags &= ~AV_FRAME_FLAG_KEY;
              add_frame_default(c->pic, c->decomp_buf,
                                c->linelen, c->height);
    }

    *got_frame = 1;
    if ((ret = av_frame_ref(rframe, c->pic)) < 0)
        return ret;

    return buf_size;
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    CamStudioContext *c = avctx->priv_data;
    int stride;
    switch (avctx->bits_per_coded_sample) {
    case 16: avctx->pix_fmt = AV_PIX_FMT_RGB555LE; break;
    case 24: avctx->pix_fmt = AV_PIX_FMT_BGR24; break;
    case 32: avctx->pix_fmt = AV_PIX_FMT_BGR0; break;
    default:
        av_log(avctx, AV_LOG_ERROR,
               "CamStudio codec error: invalid depth %i bpp\n",
               avctx->bits_per_coded_sample);
        return AVERROR_INVALIDDATA;
    }
    c->bpp = avctx->bits_per_coded_sample;
    c->linelen = avctx->width * avctx->bits_per_coded_sample / 8;
    c->height = avctx->height;
    stride = FFALIGN(c->linelen, 4);
    c->decomp_size = c->height * stride;
    c->decomp_buf = av_malloc(c->decomp_size + AV_LZO_OUTPUT_PADDING);
    if (!c->decomp_buf) {
        av_log(avctx, AV_LOG_ERROR, "Can't allocate decompression buffer.\n");
        return AVERROR(ENOMEM);
    }
    c->pic = av_frame_alloc();
    if (!c->pic)
        return AVERROR(ENOMEM);
    return 0;
}

static av_cold int decode_end(AVCodecContext *avctx)
{
    CamStudioContext *c = avctx->priv_data;

    ff_inflate(&c->ic, NULL, 0, NULL, 0, 0, 0);
    av_freep(&c->decomp_buf);
    av_frame_free(&c->pic);

    return 0;
}

const FFCodec ff_cscd_decoder = {
    .p.name         = "camstudio",
    CODEC_LONG_NAME("CamStudio"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_CSCD,
    .priv_data_size = sizeof(CamStudioContext),
    .init           = decode_init,
    .close          = decode_end,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
