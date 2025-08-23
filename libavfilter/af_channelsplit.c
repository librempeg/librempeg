/*
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
 * Channel split filter
 *
 * Split an audio stream into per-channel streams.
 */
#include "libavutil/avassert.h"
#include "libavutil/attributes.h"
#include "libavutil/channel_layout.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct ChannelSplitContext {
    const AVClass *class;

    AVChannelLayout channel_layout;
    AVChannelLayout channels;

    int     *map;

    AVFrame *in;
} ChannelSplitContext;

#define OFFSET(x) offsetof(ChannelSplitContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM
static const AVOption channelsplit_options[] = {
    { "channel_layout", "Input channel layout", OFFSET(channel_layout),   AV_OPT_TYPE_CHLAYOUT, { .str = "stereo" }, .flags = A|F },
    { "channels",        "Channels to extract", OFFSET(channels),         AV_OPT_TYPE_CHLAYOUT, { .str = "24c" },    .flags = A|F },
    { NULL }
};

AVFILTER_DEFINE_CLASS(channelsplit);

static av_cold int init(AVFilterContext *ctx)
{
    ChannelSplitContext *s = ctx->priv;
    int ret;

    s->map = av_calloc(s->channel_layout.nb_channels, sizeof(*s->map));
    if (!s->map)
        return AVERROR(ENOMEM);

    for (int i = 0; i < s->channel_layout.nb_channels; i++) {
        enum AVChannel channel = av_channel_layout_channel_from_index(&s->channel_layout, i);
        AVFilterPad pad = { .flags = AVFILTERPAD_FLAG_FREE_NAME };
        char buf[128];

        if (s->channel_layout.order == AV_CHANNEL_ORDER_UNSPEC &&
            s->channels.order == AV_CHANNEL_ORDER_CUSTOM) {
            int have_ch = 0;

            for (int j = 0; j < s->channels.nb_channels; j++) {
                if (i == s->channels.u.map[j].id) {
                    snprintf(buf, sizeof(buf), "USR%d", i);
                    have_ch = 1;
                    break;
                }
            }

            if (!have_ch)
                continue;
        } else if (s->channel_layout.order == AV_CHANNEL_ORDER_UNSPEC &&
                   s->channels.order == AV_CHANNEL_ORDER_NATIVE) {
            int have_ch = 0;

            for (int j = 0; j < s->channels.nb_channels; j++) {
                if (i == j) {
                    have_ch = 1;
                    snprintf(buf, sizeof(buf), "USR%d", i);
                    break;
                }
            }

            if (!have_ch)
                continue;
        } else {
            if (channel > AV_CHAN_NONE) {
                if (av_channel_layout_index_from_channel(&s->channels, channel) < 0)
                    continue;
            }
            av_channel_name(buf, sizeof(buf), channel);
        }

        pad.type = AVMEDIA_TYPE_AUDIO;
        pad.name = av_strdup(buf);
        if (!pad.name) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        s->map[ctx->nb_outputs] = i;

        if ((ret = ff_append_outpad(ctx, &pad)) < 0)
            goto fail;
    }

fail:
    return ret;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ChannelSplitContext *s = ctx->priv;

    av_frame_free(&s->in);
    av_freep(&s->map);
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const ChannelSplitContext *s = ctx->priv;
    AVFilterChannelLayouts *in_layouts = NULL;
    int ret;

    ret = ff_set_common_formats2(ctx, cfg_in, cfg_out, ff_planar_sample_fmts());
    if (ret < 0)
        return ret;

    if ((ret = ff_add_channel_layout(&in_layouts, &s->channel_layout)) < 0 ||
        (ret = ff_channel_layouts_ref(in_layouts, &cfg_in[0]->channel_layouts)) < 0)
        return ret;

    for (int i = 0; i < ctx->nb_outputs; i++) {
        AVChannelLayout channel_layout = { 0 };
        AVFilterChannelLayouts *out_layouts = NULL;
        enum AVChannel channel = av_channel_layout_channel_from_index(&s->channel_layout, s->map[i]);

        channel_layout.u.map = av_mallocz(sizeof(*channel_layout.u.map));
        if (!channel_layout.u.map)
            return AVERROR(ENOMEM);

        channel_layout.u.map[0].id = channel;
        channel_layout.nb_channels = 1;
        channel_layout.order       = (channel == AV_CHAN_NONE) ? AV_CHANNEL_ORDER_UNSPEC : AV_CHANNEL_ORDER_CUSTOM;

        ret = av_channel_layout_retype(&channel_layout, AV_CHANNEL_ORDER_UNSPEC, AV_CHANNEL_LAYOUT_RETYPE_FLAG_CANONICAL);
        if (ret < 0) {
            av_channel_layout_uninit(&channel_layout);
            return ret;
        }

        ret = ff_add_channel_layout(&out_layouts, &channel_layout);
        av_channel_layout_uninit(&channel_layout);
        if (ret < 0)
            return ret;

        ret = ff_channel_layouts_ref(out_layouts, &cfg_out[i]->channel_layouts);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int filter_frame(AVFilterLink *outlink, AVFrame *in)
{
    const int i = FF_OUTLINK_IDX(outlink);
    AVFilterContext *ctx = outlink->src;
    ChannelSplitContext *s = ctx->priv;
    AVFrame *out;
    int ret;

    out = ff_graph_frame_alloc(ctx);
    if (!out)
        return AVERROR(ENOMEM);

    out->nb_samples = in->nb_samples;
    ret = ff_filter_get_buffer_ext(ctx, out, i, 0, 0);
    if (ret < 0) {
        av_frame_free(&out);
        return ret;
    }

    out->pts = in->pts;
    out->duration = in->duration;
    memcpy(out->extended_data[0], in->extended_data[s->map[i]],
           in->nb_samples * av_get_bytes_per_sample(in->format));

    return ff_filter_frame(outlink, out);
}

static int filter_prepare(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    ChannelSplitContext *s = ctx->priv;
    int status, ret;
    int64_t pts;

    ff_graph_frame_free(ctx, &s->in);

    for (int i = 0; i < ctx->nb_outputs; i++) {
        ret = ff_outlink_get_status(ctx->outputs[i]);
        if (ret) {
            for (int j = 0; j < ctx->nb_inputs; j++)
                ff_inlink_set_status(ctx->inputs[j], ret);
            return AVERROR_EOF;
        }
    }

    ret = ff_inlink_consume_frame(inlink, &s->in);
    if (ret < 0)
        return ret;
    if (ret > 0)
        return 0;

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        for (int i = 0; i < ctx->nb_outputs; i++) {
            if (ff_outlink_get_status(ctx->outputs[i]))
                continue;
            ff_outlink_set_status(ctx->outputs[i], status, pts);
        }
        return AVERROR_EOF;
    }

    for (int i = 0; i < ctx->nb_outputs; i++) {
        if (ff_outlink_get_status(ctx->outputs[i]))
            continue;

        if (ff_outlink_frame_wanted(ctx->outputs[i])) {
            ff_inlink_request_frame(inlink);
            return AVERROR(EAGAIN);
        }
    }

    return AVERROR(EAGAIN);
}

static int activate(AVFilterContext *ctx)
{
    ChannelSplitContext *s = ctx->priv;
    AVFrame *in = s->in;
    int ret;

    for (int i = 0; i < ctx->nb_outputs; i++) {
        if (ff_outlink_get_status(ctx->outputs[i]))
            continue;

        ret = filter_frame(ctx->outputs[i], in);
        if (ret < 0)
            break;
    }

    if (ret < 0)
        return ret;

    return 0;
}

#if CONFIG_AVFILTER_THREAD_FRAME
static int transfer_state(AVFilterContext *dst, const AVFilterContext *src)
{
    const ChannelSplitContext *s_src = src->priv;
    ChannelSplitContext       *s_dst = dst->priv;

    if (!ff_filter_is_frame_thread(dst) || ff_filter_is_frame_thread(src))
        return 0;

    av_frame_free(&s_dst->in);
    if (s_src->in) {
        s_dst->in = ff_graph_frame_clone(dst, s_src->in);
        if (!s_dst->in)
            return AVERROR(ENOMEM);
    }

    return 0;
}
#endif

const FFFilter ff_af_channelsplit = {
    .p.name         = "channelsplit",
    .p.description  = NULL_IF_CONFIG_SMALL("Split audio into per-channel streams."),
    .p.priv_class   = &channelsplit_class,
    .p.flags        = AVFILTER_FLAG_DYNAMIC_OUTPUTS |
                      AVFILTER_FLAG_FRAME_THREADS,
    .priv_size      = sizeof(ChannelSplitContext),
    .init           = init,
    .filter_prepare = filter_prepare,
#if CONFIG_AVFILTER_THREAD_FRAME
    .transfer_state = transfer_state,
#endif
    .activate       = activate,
    .uninit         = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
};
