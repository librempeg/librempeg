/*
 * TechSmith Camtasia decoder
 * Copyright (c) 2004 Konstantin Shishkov
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
 * TechSmith Camtasia decoder
 *
 * Fourcc: TSCC
 *
 * Codec is very simple:
 *  it codes picture (picture difference, really)
 *  with algorithm almost identical to Windows RLE8,
 *  only without padding and with greater pixel sizes,
 *  then this coded picture is packed with ZLib
 *
 * Supports: BGR8,BGR555,BGR24 - only BGR8 and BGR555 tested
 */

#include "libavutil/mem.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "msrledec.h"
#include "inflate.h"

typedef struct TsccContext {

    AVCodecContext *avctx;
    AVFrame *frame;

    // Bits per pixel
    int bpp;
    // Decompressed data size
    unsigned int decomp_size;
    // Decompression buffer
    uint8_t *decomp_buf;
    GetByteContext gb;
    int height;
    InflateContext ic;

    uint32_t pal[256];
} CamtasiaContext;

static int decode_frame(AVCodecContext *avctx, AVFrame *rframe,
                        int *got_frame, AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    CamtasiaContext * const c = avctx->priv_data;
    InflateContext *ic = &c->ic;
    AVFrame *frame = c->frame;
    int palette_has_changed = 0;
    int ret;

    if (c->avctx->pix_fmt == AV_PIX_FMT_PAL8)
        palette_has_changed = ff_copy_palette(c->pal, avpkt, avctx);

    if (buf_size > 2) {
        ret = ff_inflate(ic, buf, buf_size, c->decomp_buf, 1, c->decomp_size, c->decomp_size);
        if (ret < 0 && !palette_has_changed)
            return buf_size;
        if (ret < 0)
            return ret;
    } else {
        return buf_size;
    }

    if ((ret = ff_reget_buffer(avctx, frame, 0)) < 0)
        return ret;

    if (ic->x > 0) {
        bytestream2_init(&c->gb, c->decomp_buf, ic->x);
        ff_msrle_decode(avctx, frame, c->bpp, &c->gb);
    }

    /* make the palette available on the way out */
    if (c->avctx->pix_fmt == AV_PIX_FMT_PAL8) {
        memcpy(frame->data[1], c->pal, AVPALETTE_SIZE);
    }

    if ((ret = av_frame_ref(rframe, frame)) < 0)
        return ret;
    *got_frame      = 1;

    /* always report that the buffer was completely consumed */
    return buf_size;
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    CamtasiaContext * const c = avctx->priv_data;

    c->avctx = avctx;

    c->height = avctx->height;

    switch(avctx->bits_per_coded_sample){
    case  8: avctx->pix_fmt = AV_PIX_FMT_PAL8; break;
    case 16: avctx->pix_fmt = AV_PIX_FMT_RGB555; break;
    case 24:
             avctx->pix_fmt = AV_PIX_FMT_BGR24;
             break;
    case 32: avctx->pix_fmt = AV_PIX_FMT_0RGB32; break;
    default: av_log(avctx, AV_LOG_ERROR, "Camtasia error: unknown depth %i bpp\n", avctx->bits_per_coded_sample);
             return AVERROR_PATCHWELCOME;
    }
    c->bpp = avctx->bits_per_coded_sample;
    // buffer size for RLE 'best' case when 2-byte code precedes each pixel and there may be padding after it too
    c->decomp_size = (((avctx->width * c->bpp + 7) >> 3) + 3 * avctx->width + 2) * avctx->height + 2;

    /* Allocate decompression buffer */
    if (c->decomp_size) {
        if (!(c->decomp_buf = av_malloc(c->decomp_size))) {
            av_log(avctx, AV_LOG_ERROR, "Can't allocate decompression buffer.\n");
            return AVERROR(ENOMEM);
        }
    }

    c->frame = av_frame_alloc();
    if (!c->frame)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold int decode_end(AVCodecContext *avctx)
{
    CamtasiaContext * const c = avctx->priv_data;

    av_freep(&c->decomp_buf);
    av_frame_free(&c->frame);
    ff_inflate(&c->ic, NULL, 0, NULL, 0, 0, 0);

    return 0;
}

const FFCodec ff_tscc_decoder = {
    .p.name         = "camtasia",
    CODEC_LONG_NAME("TechSmith Screen Capture Codec"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_TSCC,
    .priv_data_size = sizeof(CamtasiaContext),
    .init           = decode_init,
    .close          = decode_end,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
