/*
 * WavPack (v1-v3) audio decoder
 * Copyright (c) 2026 Librempeg
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

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"

#define BITSTREAM_READER_LE
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "get_bits.h"
#include "libavutil/refstruct.h"
#include "unary.h"
#include "wavpack_old.h"

typedef struct WavpackOldContext {
    AVCodecContext *avctx;
    WavPackV2Header hdr;
} WavpackOldContext;

static av_cold int wavpack_old_decode_init(AVCodecContext *avctx)
{
    WavpackOldContext *s = avctx->priv_data;
    WavPackV2Header *hdr = &s->hdr;

    s->avctx = avctx;

    if ((avctx->extradata) && (avctx->extradata_size >= 2)) {
        hdr->version = AV_RL16(avctx->extradata);
        if ((hdr->version < 1) && (hdr->version > 3))
            return AVERROR_INVALIDDATA;
    } else
        return AVERROR_INVALIDDATA;

    return 0;
}

static av_cold int wavpack_old_decode_close(AVCodecContext *avctx)
{
    return 0;
}

static av_cold int wavpack_old_decode_frame(AVCodecContext *avctx, AVFrame *frame,
              int *got_frame_ptr, AVPacket *avpkt)
{
    return 0;
}

const FFCodec ff_wavpack_old_decoder = {
    .p.name         = "wavpack_old",
    CODEC_LONG_NAME("WavPack (monolithic stream)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_WAVPACK_OLD,
    .priv_data_size = sizeof(WavpackOldContext),
    .init           = wavpack_old_decode_init,
    .close          = wavpack_old_decode_close,
    FF_CODEC_DECODE_CB(wavpack_old_decode_frame),
    //.flush          = wavpack_decode_flush,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
