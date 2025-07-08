/*
 * LCL (LossLess Codec Library) Codec
 * Copyright (c) 2002-2004 Roberto Togni
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
 * LCL (LossLess Codec Library) Video Codec
 * Decoder for MSZH and ZLIB codecs
 * Experimental encoder for ZLIB RGB24
 *
 * Fourcc: MSZH, ZLIB
 *
 * Original Win32 dll:
 * Ver2.23 By Kenji Oshima 2000.09.20
 * avimszh.dll, avizlib.dll
 *
 * A description of the decoding algorithm can be found here:
 *   http://www.pcisys.net/~melanson/codecs
 *
 * Supports: BGR24 (RGB 24bpp)
 */

#include <stdio.h>
#include <stdlib.h>

#include "libavutil/avassert.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "encode.h"
#include "lcl.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "deflate.h"

typedef struct LclEncContext {
    // Image type
    int imgtype;
    // Compression type
    int compression;
    // Flags
    int flags;

    DeflateContext dc;
} LclEncContext;

static int encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                        const AVFrame *p, int *got_packet)
{
    LclEncContext *c = avctx->priv_data;
    DeflateContext *dc = &c->dc;
    int max_size = ff_deflate_bound(avctx->width * avctx->height * 3);
    int ret;

    if ((ret = ff_alloc_packet(avctx, pkt, max_size)) < 0)
        return ret;

    if (avctx->pix_fmt != AV_PIX_FMT_BGR24){
        av_log(avctx, AV_LOG_ERROR, "Format not supported!\n");
        return -1;
    }

    ret = ff_deflate(dc, pkt->data, pkt->size, p->data[0] + p->linesize[0] * (avctx->height-1),
                     avctx->height, avctx->width * 3, -p->linesize[0]);
    if (ret < 0)
        return ret;

    pkt->size   = ret;
    *got_packet = 1;

    return 0;
}

static av_cold int encode_init(AVCodecContext *avctx)
{
    LclEncContext *c = avctx->priv_data;

    av_assert0(avctx->width && avctx->height);

    avctx->extradata = av_mallocz(8 + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!avctx->extradata)
        return AVERROR(ENOMEM);

    c->compression = avctx->compression_level == FF_COMPRESSION_DEFAULT ?
                            COMP_ZLIB_NORMAL :
                            av_clip(avctx->compression_level, 0, 9);
    c->flags = 0;
    c->imgtype = IMGTYPE_RGB24;
    avctx->bits_per_coded_sample= 24;

    avctx->extradata[0]= 4;
    avctx->extradata[1]= 0;
    avctx->extradata[2]= 0;
    avctx->extradata[3]= 0;
    avctx->extradata[4]= c->imgtype;
    avctx->extradata[5]= c->compression;
    avctx->extradata[6]= c->flags;
    avctx->extradata[7]= CODEC_ZLIB;
    avctx->extradata_size= 8;

    return 0;
}

const FFCodec ff_zlib_encoder = {
    .p.name         = "zlib",
    CODEC_LONG_NAME("LCL (LossLess Codec Library) ZLIB"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_ZLIB,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_FRAME_THREADS |
                      AV_CODEC_CAP_ENCODER_REORDERED_OPAQUE,
    .priv_data_size = sizeof(LclEncContext),
    .init           = encode_init,
    FF_CODEC_ENCODE_CB(encode_frame),
    CODEC_PIXFMTS(AV_PIX_FMT_BGR24),
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
