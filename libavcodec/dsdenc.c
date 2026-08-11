/*
 * Direct Stream Digital (DSD) encoder
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

/**
 * @file
 * DSD (Direct Stream Digital) "encoder": AV_SAMPLE_FMT_DSD is
 * already the bitstream, so packets are a plain copy of the samples.
 */

#include <string.h>

#include "avcodec.h"
#include "codec_internal.h"
#include "encode.h"

static av_cold int dsd_encode_init(AVCodecContext *avctx)
{
    avctx->bits_per_coded_sample = 8;
    avctx->block_align           = avctx->ch_layout.nb_channels;
    avctx->bit_rate              = 8LL * avctx->block_align * avctx->sample_rate;
    return 0;
}

static int dsd_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                            const AVFrame *frame, int *got_packet_ptr)
{
    int64_t size = frame->nb_samples * (int64_t)avctx->ch_layout.nb_channels;
    int ret;

    if ((ret = ff_get_encode_buffer(avctx, avpkt, size, 0)) < 0)
        return ret;

    memcpy(avpkt->data, frame->data[0], size);

    *got_packet_ptr = 1;
    return 0;
}

const FFCodec ff_dsd_msbf_encoder = {
    .p.name         = "dsd_msbf",
    CODEC_LONG_NAME("DSD (Direct Stream Digital), most significant bit first"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_DSD_MSBF,
    .p.capabilities = AV_CODEC_CAP_VARIABLE_FRAME_SIZE,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_DSD),
    .init           = dsd_encode_init,
    FF_CODEC_ENCODE_CB(dsd_encode_frame),
};
