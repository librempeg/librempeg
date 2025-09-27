/*
 * AHX to MP2 bitstream filter
 * Copyright (c) 2024 Paul B Mahol
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
 * AHX to MP2 bitstream filter.
 */

#include "libavutil/intreadwrite.h"
#include "bsf.h"
#include "bsf_internal.h"

static av_cold int init(AVBSFContext *ctx)
{
    ctx->par_out->codec_id = AV_CODEC_ID_MP2;

    return 0;
}

static int filter(AVBSFContext *ctx, AVPacket *pkt)
{
    int ret;

    ret = ff_bsf_get_packet_ref(ctx, pkt);
    if (ret < 0)
        return ret;

    if (pkt->size < 1044) {
        int original_size = pkt->size;
        ret = av_grow_packet(pkt, 1044-pkt->size);
        if (ret < 0) {
            av_packet_unref(pkt);
            return ret;
        }
        memset(pkt->data + original_size, 0, 1044 - original_size);
    }

    return 0;
}

const FFBitStreamFilter ff_ahx_to_mp2_bsf = {
    .p.name         = "ahx_to_mp2",
    .p.codec_ids    = (const enum AVCodecID []){ AV_CODEC_ID_AHX, AV_CODEC_ID_NONE },
    .init           = init,
    .filter         = filter,
};
