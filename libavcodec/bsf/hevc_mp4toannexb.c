/*
 * HEVC MP4 to Annex B byte stream format filter
 * copyright (c) 2015 Anton Khirnov
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

#include <string.h>

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"

#include "bsf.h"
#include "bsf_internal.h"
#include "bytestream.h"
#include "defs.h"

#include "hevc/hevc.h"

#define MIN_HEVCC_LENGTH 23

typedef struct HEVCBSFContext {
    uint8_t  length_size;
    int      extradata_parsed;
} HEVCBSFContext;

static int hevc_extradata_to_annexb(AVBSFContext *ctx)
{
    GetByteContext gb;
    int length_size, num_arrays, i, j;
    int ret = 0;

    uint8_t *new_extradata = NULL;
    size_t   new_extradata_size = 0;

    bytestream2_init(&gb, ctx->par_in->extradata, ctx->par_in->extradata_size);

    bytestream2_skip(&gb, 21);
    length_size = (bytestream2_get_byte(&gb) & 3) + 1;
    num_arrays  = bytestream2_get_byte(&gb);

    for (i = 0; i < num_arrays; i++) {
        int type = bytestream2_get_byte(&gb) & 0x3f;
        int cnt  = bytestream2_get_be16(&gb);

        if (!(type == HEVC_NAL_VPS || type == HEVC_NAL_SPS || type == HEVC_NAL_PPS ||
              type == HEVC_NAL_SEI_PREFIX || type == HEVC_NAL_SEI_SUFFIX)) {
            av_log(ctx, AV_LOG_ERROR, "Invalid NAL unit type in extradata: %d\n",
                   type);
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }

        for (j = 0; j < cnt; j++) {
            const int nalu_len = bytestream2_get_be16(&gb);

            if (!nalu_len ||
                nalu_len > bytestream2_get_bytes_left(&gb) ||
                4 + AV_INPUT_BUFFER_PADDING_SIZE + nalu_len > SIZE_MAX - new_extradata_size) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }
            ret = av_reallocp(&new_extradata, new_extradata_size + nalu_len + 4 + AV_INPUT_BUFFER_PADDING_SIZE);
            if (ret < 0)
                goto fail;

            AV_WB32(new_extradata + new_extradata_size, 1); // add the startcode
            bytestream2_get_buffer(&gb, new_extradata + new_extradata_size + 4, nalu_len);
            new_extradata_size += 4 + nalu_len;
            memset(new_extradata + new_extradata_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);
        }
    }

    av_freep(&ctx->par_out->extradata);
    ctx->par_out->extradata      = new_extradata;
    ctx->par_out->extradata_size = new_extradata_size;

    if (!new_extradata_size)
        av_log(ctx, AV_LOG_WARNING, "No parameter sets in the extradata\n");

    return length_size;
fail:
    av_freep(&new_extradata);
    return ret;
}

static int hevc_mp4toannexb_init(AVBSFContext *ctx)
{
    HEVCBSFContext *s = ctx->priv_data;
    int ret;

    if (ctx->par_in->extradata_size < MIN_HEVCC_LENGTH ||
        AV_RB24(ctx->par_in->extradata) == 1           ||
        AV_RB32(ctx->par_in->extradata) == 1) {
        av_log(ctx, AV_LOG_VERBOSE,
               "The input looks like it is Annex B already\n");
    } else {
        ret = hevc_extradata_to_annexb(ctx);
        if (ret < 0)
            return ret;
        s->length_size      = ret;
        s->extradata_parsed = 1;
    }

    return 0;
}

static int hevc_mp4toannexb_filter(AVBSFContext *ctx, AVPacket *out)
{
    HEVCBSFContext *s = ctx->priv_data;
    AVPacket *in;
    GetByteContext gb;

    int got_irap = 0;
    int got_ps = 0, seen_irap_ps = 0;
    int i, ret = 0;

    ret = ff_bsf_get_packet(ctx, &in);
    if (ret < 0)
        return ret;

    if (!s->extradata_parsed) {
        av_packet_move_ref(out, in);
        av_packet_free(&in);
        return 0;
    }

    bytestream2_init(&gb, in->data, in->size);

    while (!got_irap && bytestream2_get_bytes_left(&gb)) {
        uint32_t nalu_size = 0;
        int      nalu_type;

        if (bytestream2_get_bytes_left(&gb) < s->length_size) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }
        for (i = 0; i < s->length_size; i++)
            nalu_size = (nalu_size << 8) | bytestream2_get_byte(&gb);

        if (nalu_size < 2 || nalu_size > bytestream2_get_bytes_left(&gb)) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }

        nalu_type = (bytestream2_peek_byte(&gb) >> 1) & 0x3f;
        bytestream2_skip(&gb, nalu_size);
        got_irap |= nalu_type >= HEVC_NAL_BLA_W_LP &&
                    nalu_type <= HEVC_NAL_RSV_IRAP_VCL23;
        got_ps   |= nalu_type >= HEVC_NAL_VPS && nalu_type <= HEVC_NAL_PPS;
    }
    seen_irap_ps = got_irap && got_ps;
    got_irap = got_ps = 0;

    bytestream2_init(&gb, in->data, in->size);

    while (bytestream2_get_bytes_left(&gb)) {
        uint32_t nalu_size = 0;
        int      nalu_type;
        int is_irap, is_ps, add_extradata, extra_size, prev_size;

        if (bytestream2_get_bytes_left(&gb) < s->length_size) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }
        for (i = 0; i < s->length_size; i++)
            nalu_size = (nalu_size << 8) | bytestream2_get_byte(&gb);

        if (nalu_size < 2 || nalu_size > bytestream2_get_bytes_left(&gb)) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }

        nalu_type = (bytestream2_peek_byte(&gb) >> 1) & 0x3f;

        /* prepend extradata to IRAP frames */
        is_irap = nalu_type >= HEVC_NAL_BLA_W_LP &&
                  nalu_type <= HEVC_NAL_RSV_IRAP_VCL23;
        is_ps   = nalu_type >= HEVC_NAL_VPS && nalu_type <= HEVC_NAL_PPS && seen_irap_ps;
        add_extradata = (is_ps || is_irap) && !got_ps && !got_irap;
        extra_size    = add_extradata * ctx->par_out->extradata_size;
        got_irap     |= is_irap;
        got_ps       |= is_ps;

        if (FFMIN(INT_MAX, SIZE_MAX) < 4ULL + nalu_size + extra_size) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }

        prev_size = out->size;

        ret = av_grow_packet(out, 4 + nalu_size + extra_size);
        if (ret < 0)
            goto fail;

        if (extra_size)
            memcpy(out->data + prev_size, ctx->par_out->extradata, extra_size);
        AV_WB32(out->data + prev_size + extra_size, 1);
        bytestream2_get_buffer(&gb, out->data + prev_size + 4 + extra_size, nalu_size);
    }

    ret = av_packet_copy_props(out, in);
    if (ret < 0)
        goto fail;

fail:
    if (ret < 0)
        av_packet_unref(out);
    av_packet_free(&in);

    return ret;
}

static const enum AVCodecID codec_ids[] = {
    AV_CODEC_ID_HEVC, AV_CODEC_ID_NONE,
};

const FFBitStreamFilter ff_hevc_mp4toannexb_bsf = {
    .p.name         = "hevc_mp4toannexb",
    .p.codec_ids    = codec_ids,
    .priv_data_size = sizeof(HEVCBSFContext),
    .init           = hevc_mp4toannexb_init,
    .filter         = hevc_mp4toannexb_filter,
};
