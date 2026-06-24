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
 * Packet source. Heavily based on libavfilter/buffersrc.c
 */

#include <float.h>

#include "libavutil/container_fifo.h"
#include "libavutil/opt.h"

#include "libavcodec/bsf.h"
#include "libavcodec/bsf_internal.h"
#include "libavcodec/packet.h"
#include "libavcodec/packet_internal.h"

typedef struct SourceContext {
    const AVClass    *class;
    AVCodecParameters *par;
    AVRational        time_base;     ///< time_base to set in the output link

    unsigned          nb_failed_requests;
    unsigned          warning_limit;

    int eof;
    int64_t last_pts;
} SourceContext;

int av_bsf_source_parameters_set(AVBitStreamFilterContext *ctx, const AVCodecParameters *par)
{
    SourceContext *s = ctx->priv_data;

    return avcodec_parameters_copy(s->par, par);
}

static int push_packet(AVBitStreamFilterGraph *graph)
{
    int ret;

    while (1) {
        ret = ff_bsf_graph_run_once(graph);
        if (ret == AVERROR(EAGAIN))
            break;
        if (ret < 0 && ret != FFERROR_SOURCE_EMPTY)
            return ret;
    }
    return 0;
}

int attribute_align_arg av_bsf_source_add_packet(AVBitStreamFilterContext *ctx, AVPacket *pkt, int flags)
{
    SourceContext *s = ctx->priv_data;
    AVPacket *copy;
    int ret;

    s->nb_failed_requests = 0;

    if (!pkt || AVPACKET_IS_EMPTY(pkt))
        return av_bsf_source_close(ctx, s->last_pts, flags);
    if (s->eof)
        return AVERROR_EOF;

    s->last_pts = pkt->pts + pkt->duration;

    copy = av_packet_alloc();
    if (!copy)
        return AVERROR(ENOMEM);

    if ((flags & AV_BSF_SOURCE_FLAG_KEEP_REF)) {
        ret = av_packet_ref(copy, pkt);
        if (ret < 0)
            return ret;
    } else
        av_packet_move_ref(copy, pkt);

    ret = ff_bsf_filter_packet(ctx->outputs[0], copy);
    if (ret < 0)
        return ret;

    if ((flags & AV_BSF_SOURCE_FLAG_PUSH)) {
        ret = push_packet(ctx->graph);
        if (ret < 0)
            return ret;
    }

    BitStreamFilterLinkInternal *const li = ff_link_internal(ctx->outputs[0]);
    if (s->warning_limit &&
        av_container_fifo_can_read(li->fifo) >= s->warning_limit) {
        av_log(s, AV_LOG_WARNING,
               "%d buffers queued in %s, something may be wrong.\n",
               s->warning_limit,
               (char *)av_x_if_null(ctx->name, ctx->filter->name));
        s->warning_limit *= 10;
    }

    return 0;
}

int av_bsf_source_close(AVBitStreamFilterContext *ctx, int64_t pts, unsigned flags)
{
    SourceContext *s = ctx->priv_data;

    s->eof = 1;
    ff_bsf_link_set_in_status(ctx->outputs[0], AVERROR_EOF, pts);
    return 0;
}

int av_bsf_source_get_status(AVBitStreamFilterContext *ctx)
{
    SourceContext *s = ctx->priv_data;

    if (!s->eof && ff_bsf_outlink_get_status(ctx->outputs[0]))
        s->eof = 1;

    return s->eof ? AVERROR(EOF) : 0;
}

static av_cold int preinit(AVBitStreamFilterContext *ctx)
{
    SourceContext *c = ctx->priv_data;

    c->par = avcodec_parameters_alloc();
    if (!c->par)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold int init(AVBitStreamFilterContext *ctx)
{
    SourceContext *c = ctx->priv_data;

    if (av_q2d(c->time_base) <= 0) {
        av_log(ctx, AV_LOG_ERROR, "Invalid time base %d/%d\n", c->time_base.num, c->time_base.den);
        return AVERROR(EINVAL);
    }

    c->warning_limit = 100;
    return 0;
}

unsigned ff_bsf_source_get_nb_failed_requests(const AVBitStreamFilterContext *buffer_src)
{
    return ((SourceContext *)buffer_src->priv_data)->nb_failed_requests;
}

#define OFFSET(x) offsetof(SourceContext, x)
#define FLAGS (AV_OPT_FLAG_BSF_PARAM|AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption buffer_options[] = {
    { "time_base", NULL, OFFSET(time_base), AV_OPT_TYPE_RATIONAL, { .dbl = 0 }, 0, DBL_MAX, FLAGS },
    { NULL },
};

BSF_DEFINE_CLASS(buffer);

static av_cold void uninit(AVBitStreamFilterContext *ctx)
{
    SourceContext *s = ctx->priv_data;
    avcodec_parameters_free(&s->par);
}

static int config_props(AVBitStreamFilterLink *link)
{
    SourceContext *c = link->src->priv_data;

    link->time_base = c->time_base;
    return avcodec_parameters_copy(link->par, c->par);
}

static int activate(AVBitStreamFilterContext *ctx)
{
    AVBitStreamFilterLink *outlink = ctx->outputs[0];
    SourceContext *c = ctx->priv_data;

    if (!c->eof && ff_bsf_outlink_get_status(outlink)) {
        c->eof = 1;
        return 0;
    }

    if (c->eof) {
        ff_bsf_link_set_in_status(outlink, AVERROR_EOF, c->last_pts);
        return 0;
    }
    c->nb_failed_requests++;
    return FFERROR_SOURCE_EMPTY;
}

static const AVBitStreamFilterPad source_outputs[] = {
    {
        .name          = "default",
        .config_props  = config_props,
    },
};

const FFBitStreamFilter ff_source_bsf = {
    .p.name        = "source",
    .p.priv_class  = &buffer_class,
    .priv_data_size = sizeof(SourceContext),
    .activate  = activate,
    .preinit   = preinit,
    .init2     = init,
    .uninit    = uninit,

    BSFILTER_OUTPUTS(source_outputs),
};
