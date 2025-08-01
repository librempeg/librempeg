/*
 * Interface to libgsm for GSM decoding
 * Copyright (c) 2005 Alban Bedel <albeu@free.fr>
 * Copyright (c) 2006, 2007 Michel Bardiaux <mbardiaux@mediaxim.be>
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
 * Interface to libgsm for GSM decoding
 */

// The idiosyncrasies of GSM-in-WAV are explained at http://kbs.cs.tu-berlin.de/~jutta/toast.html

#include "config.h"
#include "config_components.h"
#if HAVE_GSM_H
#include <gsm.h>
#else
#include <gsm/gsm.h>
#endif

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "gsm.h"

typedef struct LibGSMDecodeContext {
    struct gsm_state *state;
} LibGSMDecodeContext;

static av_cold int libgsm_decode_init(AVCodecContext *avctx) {
    LibGSMDecodeContext *s = avctx->priv_data;

    av_channel_layout_uninit(&avctx->ch_layout);
    avctx->ch_layout      = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
    if (!avctx->sample_rate)
        avctx->sample_rate = 8000;
    avctx->sample_fmt     = AV_SAMPLE_FMT_S16;

    s->state = gsm_create();

    switch(avctx->codec_id) {
    case AV_CODEC_ID_GSM:
        avctx->frame_size  = GSM_FRAME_SIZE;
        avctx->block_align = GSM_BLOCK_SIZE;
        break;
    case AV_CODEC_ID_GSM_MS: {
        int one = 1;
        gsm_option(s->state, GSM_OPT_WAV49, &one);
        avctx->frame_size  = 2 * GSM_FRAME_SIZE;
        avctx->block_align = GSM_MS_BLOCK_SIZE;
        }
    }

    return 0;
}

static av_cold int libgsm_decode_close(AVCodecContext *avctx) {
    LibGSMDecodeContext *s = avctx->priv_data;

    gsm_destroy(s->state);
    s->state = NULL;
    return 0;
}

static int libgsm_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                               int *got_frame_ptr, AVPacket *avpkt)
{
    int i, ret;
    LibGSMDecodeContext *s = avctx->priv_data;
    uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    int16_t *samples;

    if (buf_size < avctx->block_align) {
        av_log(avctx, AV_LOG_ERROR, "Packet is too small\n");
        return AVERROR_INVALIDDATA;
    }

    /* get output buffer */
    frame->nb_samples = avctx->frame_size;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;
    samples = (int16_t *)frame->data[0];

    for (i = 0; i < avctx->frame_size / GSM_FRAME_SIZE; i++) {
        if ((ret = gsm_decode(s->state, buf, samples)) < 0)
            return -1;
        buf     += GSM_BLOCK_SIZE;
        samples += GSM_FRAME_SIZE;
    }

    *got_frame_ptr = 1;

    return avctx->block_align;
}

static void libgsm_flush(AVCodecContext *avctx) {
    LibGSMDecodeContext *s = avctx->priv_data;
    int one = 1;

    gsm_destroy(s->state);
    s->state = gsm_create();
    if (avctx->codec_id == AV_CODEC_ID_GSM_MS)
        gsm_option(s->state, GSM_OPT_WAV49, &one);
}

#if CONFIG_LIBGSM_DECODER
const FFCodec ff_libgsm_decoder = {
    .p.name         = "libgsm",
    CODEC_LONG_NAME("libgsm GSM"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_GSM,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_CHANNEL_CONF,
    .p.wrapper_name = "libgsm",
    .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE,
    .priv_data_size = sizeof(LibGSMDecodeContext),
    .init           = libgsm_decode_init,
    .close          = libgsm_decode_close,
    FF_CODEC_DECODE_CB(libgsm_decode_frame),
    .flush          = libgsm_flush,
};
#endif
#if CONFIG_LIBGSM_MS_DECODER
const FFCodec ff_libgsm_ms_decoder = {
    .p.name         = "libgsm_ms",
    CODEC_LONG_NAME("libgsm GSM Microsoft variant"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_GSM_MS,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_CHANNEL_CONF,
    .p.wrapper_name = "libgsm",
    .caps_internal  = FF_CODEC_CAP_NOT_INIT_THREADSAFE,
    .priv_data_size = sizeof(LibGSMDecodeContext),
    .init           = libgsm_decode_init,
    .close          = libgsm_decode_close,
    FF_CODEC_DECODE_CB(libgsm_decode_frame),
    .flush          = libgsm_flush,
};
#endif
