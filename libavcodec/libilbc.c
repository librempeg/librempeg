/*
 * iLBC encoder stub
 * Copyright (c) 2012 Martin Storsjo
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

#include <ilbc.h>

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/opt.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "encode.h"

#ifndef LIBILBC_VERSION_MAJOR
#define LIBILBC_VERSION_MAJOR 2
#endif

static int get_mode(AVCodecContext *avctx)
{
    if (avctx->block_align == 38)
        return 20;
    else if (avctx->block_align == 50)
        return 30;
    else if (avctx->bit_rate > 0)
        return avctx->bit_rate <= 14000 ? 30 : 20;
    else
        return -1;
}

typedef struct ILBCEncContext {
    const AVClass *class;
#if LIBILBC_VERSION_MAJOR < 3
    iLBC_Enc_Inst_t encoder;
#else
    IlbcEncoder encoder;
#endif
    int mode;
} ILBCEncContext;

static const AVOption ilbc_enc_options[] = {
    { "mode", "iLBC mode (20 or 30 ms frames)", offsetof(ILBCEncContext, mode), AV_OPT_TYPE_INT, { .i64 = 20 }, 20, 30, AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_ENCODING_PARAM },
    { NULL }
};

static const AVClass ilbc_enc_class = {
    .class_name = "libilbc",
    .option     = ilbc_enc_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

static av_cold int ilbc_encode_init(AVCodecContext *avctx)
{
    ILBCEncContext *s = avctx->priv_data;
    int mode;

    if (avctx->sample_rate != 8000) {
        av_log(avctx, AV_LOG_ERROR, "Only 8000Hz sample rate supported\n");
        return AVERROR(EINVAL);
    }

    if (avctx->ch_layout.nb_channels != 1) {
        av_log(avctx, AV_LOG_ERROR, "Only mono supported\n");
        return AVERROR(EINVAL);
    }

    if ((mode = get_mode(avctx)) > 0)
        s->mode = mode;
    else
        s->mode = s->mode != 30 ? 20 : 30;
    WebRtcIlbcfix_InitEncode(&s->encoder, s->mode);

    avctx->block_align = s->encoder.no_of_bytes;
    avctx->frame_size  = s->encoder.blockl;

    return 0;
}

static int ilbc_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                             const AVFrame *frame, int *got_packet_ptr)
{
    ILBCEncContext *s = avctx->priv_data;
    int ret;

    if ((ret = ff_alloc_packet(avctx, avpkt, 50)) < 0)
        return ret;

    WebRtcIlbcfix_EncodeImpl((uint16_t *) avpkt->data, (const int16_t *) frame->data[0], &s->encoder);

    avpkt->size     = s->encoder.no_of_bytes;
    *got_packet_ptr = 1;
    return 0;
}

static const FFCodecDefault ilbc_encode_defaults[] = {
    { "b", "0" },
    { NULL }
};

const FFCodec ff_libilbc_encoder = {
    .p.name         = "libilbc",
    CODEC_LONG_NAME("iLBC (Internet Low Bitrate Codec)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ILBC,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_ENCODER_REORDERED_OPAQUE,
    .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE,
    .priv_data_size = sizeof(ILBCEncContext),
    .init           = ilbc_encode_init,
    FF_CODEC_ENCODE_CB(ilbc_encode_frame),
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16),
    .defaults       = ilbc_encode_defaults,
    .p.priv_class   = &ilbc_enc_class,
    .p.wrapper_name = "libbilbc",
};
