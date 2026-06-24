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

/**
 * @file
 * Packet sink. Heavily based on libavfilter/buffersink.c
 */

#include "libavutil/avassert.h"
#include "libavutil/opt.h"

#include "libavcodec/bsf.h"
#include "libavcodec/bsf_internal.h"
#include "libavcodec/packet.h"
#include "libavcodec/packet_internal.h"

typedef struct BufferSinkContext {
    const AVClass *class;
    unsigned warning_limit;

    AVPacket *peeked_pkt;
} BufferSinkContext;

static int return_or_keep_packet(BufferSinkContext *buf, AVPacket *out, AVPacket *in, int flags)
{
    if ((flags & AV_BSF_SINK_FLAG_PEEK)) {
        buf->peeked_pkt = in;
        return av_packet_ref(out, in);
    } else {
        buf->peeked_pkt = NULL;
        av_packet_move_ref(out, in);
        av_packet_free(&in);
        return 0;
    }
}

int attribute_align_arg av_bsf_sink_get_packet(AVBitStreamFilterContext *ctx, AVPacket *pkt, int flags)
{
    BufferSinkContext *buf = ctx->priv_data;
    AVBitStreamFilterLink *inlink = ctx->inputs[0];
    BitStreamFilterLinkInternal *li = ff_link_internal(inlink);
    int status, ret;
    AVPacket *cur_pkt;
    int64_t pts;
    int buffersrc_empty = 0;

    if (buf->peeked_pkt)
        return return_or_keep_packet(buf, pkt, buf->peeked_pkt, flags);

    while (1) {
        ret = ff_bsf_inlink_consume_packet(inlink, &cur_pkt);
        if (ret < 0) {
            return ret;
        } else if (ret) {
            return return_or_keep_packet(buf, pkt, cur_pkt, flags);
        } else if (ff_bsf_inlink_acknowledge_status(inlink, &status, &pts)) {
            return status;
        } else if ((flags & AV_BSF_SINK_FLAG_NO_REQUEST)) {
            return AVERROR(EAGAIN);
        } else if (li->packet_wanted_out) {
            ret = ff_bsf_graph_run_once(ctx->graph);
            if (ret == FFERROR_SOURCE_EMPTY) {
                buffersrc_empty = 1;
            } else if (ret == AVERROR(EAGAIN)) {
                if (buffersrc_empty)
                    return ret;
                ff_bsf_inlink_request_packet(inlink);
            } else if (ret < 0) {
                return ret;
            }
        } else {
            ff_bsf_inlink_request_packet(inlink);
        }
    }
}

static int init(AVBitStreamFilterContext *ctx)
{
    BufferSinkContext *s = ctx->priv_data;

    s->warning_limit = 100;

    return 0;
}

static void uninit(AVBitStreamFilterContext *ctx)
{
    BufferSinkContext *buf = ctx->priv_data;

    av_packet_free(&buf->peeked_pkt);
}

static int activate(AVBitStreamFilterContext *ctx)
{
    BufferSinkContext *buf = ctx->priv_data;
    BitStreamFilterLinkInternal * const li = ff_link_internal(ctx->inputs[0]);

    if (buf->warning_limit &&
        av_container_fifo_can_read(li->fifo) >= buf->warning_limit) {
        av_log(ctx, AV_LOG_WARNING,
               "%d buffers queued in %s, something may be wrong.\n",
               buf->warning_limit,
               (char *)av_x_if_null(ctx->name, ctx->filter->name));
        buf->warning_limit *= 10;
    }

    /* The packet is queued, the rest is up to av_bsf_sink_get_packet */
    return 0;
}

AVRational av_bsf_sink_get_time_base(const AVBitStreamFilterContext *ctx)
{
    av_assert0(ff_bsf(ctx->filter)->activate == activate);
    return ctx->inputs[0]->time_base;
}

const AVCodecParameters *av_bsf_sink_get_parameters(const AVBitStreamFilterContext *ctx)
{
    av_assert0(ff_bsf(ctx->filter)->activate == activate);
    return ctx->inputs[0]->par;
}

BSF_DEFINE_CLASS_EXT(sink, "sink", NULL);

const FFBitStreamFilter ff_sink_bsf = {
    .p.name        = "sink",
    .p.priv_class  = &sink_class,
    .priv_data_size = sizeof(BufferSinkContext),
    .init2         = init,
    .uninit        = uninit,
    .activate      = activate,
    BSFILTER_INPUTS(ff_default_bsf_pad),
};
