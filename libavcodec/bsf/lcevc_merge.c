/*
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

#include "libavutil/opt.h"

#include "libavcodec/bsf.h"
#include "libavcodec/bsf_internal.h"
#include "libavcodec/bsf/packetsync.h"
#include "libavcodec/bytestream.h"
#include "libavcodec/h2645_parse.h"
#include "libavcodec/packet.h"
#include "libavcodec/packet_internal.h"

typedef struct LCEVCMergeContext {
    AVClass *class;
    FFPacketSync ps;
    int nal_length_size;
} LCEVCMergeContext;

static AVPacket *lcevc_merge_packets(AVBitStreamFilterContext *ctx,
                                     AVPacket *base_pkt, const AVPacket *lcevc_pkt)
{
    LCEVCMergeContext *lcevc = ctx->priv_data;

    if (lcevc->nal_length_size && lcevc_pkt->size > lcevc->nal_length_size) {
        GetByteContext bc;
        size_t size = 0;

        bytestream2_init(&bc, lcevc_pkt->data, lcevc_pkt->size);
        do {
            int i = 0;
            int nal_size = get_nalsize(lcevc->nal_length_size,
                                       bc.buffer, bytestream2_get_bytes_left(&bc), &i, ctx);
            if (nal_size < 0)
                return base_pkt;
            if (nal_size > INT_MAX - 4)
                return base_pkt;
            size += nal_size + 4;
            nal_size += i;
            if (nal_size > bytestream2_get_bytes_left(&bc))
                return base_pkt;
            bytestream2_skip(&bc, nal_size);
        } while (bytestream2_get_bytes_left(&bc) > 0);

        uint8_t *buf = av_packet_new_side_data(base_pkt, AV_PKT_DATA_LCEVC, size);
        if (!buf)
            return base_pkt;

        PutByteContext pc;
        bytestream2_init_writer(&pc, buf, size);
        bytestream2_init(&bc, lcevc_pkt->data, lcevc_pkt->size);
        do {
            int i = 0;
            int nal_size = get_nalsize(lcevc->nal_length_size,
                                       bc.buffer, bytestream2_get_bytes_left(&bc), &i, ctx);
            bytestream2_skipu(&bc, i);
            bytestream2_put_be32u(&pc, 1); // start code
            bytestream2_put_bufferu(&pc, bc.buffer, nal_size);
            bytestream2_skipu(&bc, nal_size);
        } while (bytestream2_get_bytes_left(&bc) > 0);
    } else {
        uint8_t *buf = av_packet_new_side_data(base_pkt, AV_PKT_DATA_LCEVC, lcevc_pkt->size);
        if (buf)
            memcpy(buf, lcevc_pkt->data, lcevc_pkt->size);
    }

    return base_pkt;
}

static int lcevc_merge(FFPacketSync *fs)
{
    AVBitStreamFilterContext *ctx = fs->parent;
    AVPacket *base, *enhancement, *out = NULL;
    int ret;

    ret = ff_packetsync_dualinput_get(fs, &base, &enhancement);
    if (ret < 0)
        return ret;
    if (!enhancement)
        return ff_bsf_filter_packet(ctx->outputs[0], base);
    out = lcevc_merge_packets(ctx, base, enhancement);
    return ff_bsf_filter_packet(ctx->outputs[0], out);
}

static av_cold int config_enhancement(AVBitStreamFilterLink *inlink)
{
    AVBitStreamFilterContext *ctx = inlink->dst;
    LCEVCMergeContext *lcevc = ctx->priv_data;

    if (inlink->par->extradata_size > 4 && inlink->par->extradata[0])
        lcevc->nal_length_size = (inlink->par->extradata[4] >> 6) + 1;

    return 0;
}

static int config_output(AVBitStreamFilterLink *outlink)
{
    AVBitStreamFilterContext *ctx = outlink->src;
    LCEVCMergeContext *lcevc = ctx->priv_data;
    int ret;

    ret = ff_packetsync_init_dualinput(&lcevc->ps, ctx);
    if (ret < 0)
        return ret;

    ret = ff_packetsync_configure(&lcevc->ps);
    outlink->time_base = lcevc->ps.time_base;

    return ret;
}

static av_cold int init(AVBitStreamFilterContext *ctx)
{
    LCEVCMergeContext *lcevc = ctx->priv_data;

    lcevc->ps.on_event = lcevc_merge;
    return 0;
}

static av_cold void uninit(AVBitStreamFilterContext *ctx)
{
    LCEVCMergeContext *lcevc = ctx->priv_data;

    ff_packetsync_uninit(&lcevc->ps);
}

PACKETSYNC_DEFINE_CLASS_EXT(lcevc_merge, LCEVCMergeContext, ps, NULL);

static int activate(AVBitStreamFilterContext *ctx)
{
    LCEVCMergeContext *lcevc = ctx->priv_data;
    return ff_packetsync_activate(&lcevc->ps);
}

static const enum AVCodecID enhancement_codec_ids[] = {
    AV_CODEC_ID_LCEVC, AV_CODEC_ID_NONE,
};

static const AVBitStreamFilterPad lcevc_merge_inputs[] = {
    {
        .name          = "base",
    },
    {
        .name          = "enhancement",
        .codec_ids     = enhancement_codec_ids,
        .config_props  = config_enhancement,
    },
};

static const AVBitStreamFilterPad lcevc_merge_outputs[] = {
    {
        .name          = "default",
        .config_props  = config_output,
    },
};

const FFBitStreamFilter ff_lcevc_merge_bsf = {
    .p.name         = "lcevc_merge",
    .p.priv_class   = &lcevc_merge_class,
    .priv_data_size = sizeof(LCEVCMergeContext),
    .preinit        = lcevc_merge_packetsync_preinit,
    .init2          = init,
    .uninit         = uninit,
    .activate       = activate,
    BSFILTER_INPUTS(lcevc_merge_inputs),
    BSFILTER_OUTPUTS(lcevc_merge_outputs),
};
