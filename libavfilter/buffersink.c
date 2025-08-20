/*
 * Copyright (c) 2011 Stefano Sabatini
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
 * buffer sink
 */

#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "avfilter_internal.h"
#include "buffersink.h"
#include "filters.h"
#include "formats.h"
#include "framequeue.h"
#include "video.h"

typedef struct BufferSinkContext {
    const AVClass *class;
    unsigned warning_limit;
    unsigned frame_size;

    /* only used for video */
    enum AVPixelFormat *pixel_formats;
    unsigned         nb_pixel_formats;

    int                *color_spaces;
    unsigned         nb_color_spaces;

    int                *color_ranges;
    unsigned         nb_color_ranges;

    /* only used for audio */
    enum AVSampleFormat *sample_formats;
    unsigned          nb_sample_formats;

    int                 *sample_rates;
    unsigned          nb_sample_rates;

    AVChannelLayout     *channel_layouts;
    unsigned          nb_channel_layouts;

    AVFrame *peeked_frame;
} BufferSinkContext;

int attribute_align_arg av_buffersink_get_frame(AVFilterContext *ctx, AVFrame *frame)
{
    return av_buffersink_get_frame_flags(ctx, frame, 0);
}

static int return_or_keep_frame(AVFilterContext *ctx, AVFrame *out, AVFrame *in, int flags)
{
    BufferSinkContext *buf = ctx->priv;

    if ((flags & AV_BUFFERSINK_FLAG_PEEK)) {
        buf->peeked_frame = in;
        return out ? av_frame_ref(out, in) : 0;
    } else {
        av_assert1(out);
        buf->peeked_frame = NULL;
        av_frame_move_ref(out, in);
        ff_graph_frame_free(ctx, &in);
        return 0;
    }
}

static int get_frame_internal(AVFilterContext *ctx, AVFrame *frame, int flags, int samples)
{
    BufferSinkContext *buf = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    FilterLinkInternal *li = ff_link_internal(inlink);
    int status, ret;
    AVFrame *cur_frame;
    int64_t pts;

    if (buf->peeked_frame)
        return return_or_keep_frame(ctx, frame, buf->peeked_frame, flags);

    while (1) {
        ret = samples ? ff_inlink_consume_samples(inlink, samples, samples, &cur_frame) :
                        ff_inlink_consume_frame(inlink, &cur_frame);
        if (ret < 0) {
            return ret;
        } else if (ret) {
            /* TODO return the frame instead of copying it */
            return return_or_keep_frame(ctx, frame, cur_frame, flags);
        } else if (li->frame_wanted_out) {
            ret = ff_filter_graph_run_once(ctx->graph);
            if (ret < 0)
                return ret;
        } else if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
            return status;
        } else if (flags & AV_BUFFERSINK_FLAG_NO_REQUEST) {
            return AVERROR(EAGAIN);
        } else {
            ff_inlink_request_frame(inlink);
        }
    }
}

int attribute_align_arg av_buffersink_get_frame_flags(AVFilterContext *ctx, AVFrame *frame, int flags)
{
    return get_frame_internal(ctx, frame, flags,
                              ff_filter_link(ctx->inputs[0])->min_samples);
}

int attribute_align_arg av_buffersink_get_samples(AVFilterContext *ctx,
                                                  AVFrame *frame, int nb_samples)
{
    return get_frame_internal(ctx, frame, 0, nb_samples);
}

static av_cold int common_init(AVFilterContext *ctx)
{
    BufferSinkContext *buf = ctx->priv;
    buf->warning_limit = 100;
    return 0;
}

#define TERMINATE_ARRAY(arr, val)                                                   \
    if (s->arr) {                                                                   \
        void *tmp = av_realloc_array(s->arr, s->nb_ ## arr + 1, sizeof(*s->arr));   \
        if (!tmp)                                                                   \
            return AVERROR(ENOMEM);                                                 \
        s->arr = tmp;                                                               \
        s->arr[s->nb_ ## arr] = val;                                                \
    }

static int init_video(AVFilterContext *ctx)
{
    BufferSinkContext *s = ctx->priv;

    TERMINATE_ARRAY(pixel_formats, AV_PIX_FMT_NONE);
    TERMINATE_ARRAY(color_ranges, -1);
    TERMINATE_ARRAY(color_spaces, -1);

    return common_init(ctx);
}

static int init_audio(AVFilterContext *ctx)
{
    BufferSinkContext *s = ctx->priv;

    TERMINATE_ARRAY(sample_formats, AV_SAMPLE_FMT_NONE);
    TERMINATE_ARRAY(sample_rates, -1);
    TERMINATE_ARRAY(channel_layouts, (AVChannelLayout){ .nb_channels = 0 });

    return common_init(ctx);
}

#undef TERMINATE_ARRAY

static void uninit(AVFilterContext *ctx)
{
    BufferSinkContext *buf = ctx->priv;

    av_frame_free(&buf->peeked_frame);
}

static int activate(AVFilterContext *ctx)
{
    BufferSinkContext *buf = ctx->priv;
    FilterLinkInternal * const li = ff_link_internal(ctx->inputs[0]);

    if (buf->warning_limit &&
        ff_framequeue_queued_frames(&li->fifo) >= buf->warning_limit) {
        av_log(ctx, AV_LOG_WARNING,
               "%d buffers queued in %s, something may be wrong.\n",
               buf->warning_limit,
               (char *)av_x_if_null(ctx->name, ctx->filter->name));
        buf->warning_limit *= 10;
    }

    /* The frame is queued, the rest is up to get_frame_internal */
    return 0;
}

static int config_input_audio(AVFilterLink *inlink)
{
    BufferSinkContext *buf = inlink->dst->priv;
    FilterLink *l = ff_filter_link(inlink);

    l->min_samples = l->max_samples = buf->frame_size;

    return 0;
}

void av_buffersink_set_frame_size(AVFilterContext *ctx, unsigned frame_size)
{
    BufferSinkContext *buf = ctx->priv;
    buf->frame_size = frame_size;

    if (ctx->inputs && ctx->inputs[0]) {
        FilterLink *l = ff_filter_link(ctx->inputs[0]);
        l->min_samples = l->max_samples = buf->frame_size;
    }
}

#define MAKE_AVFILTERLINK_ACCESSOR(type, field) \
type av_buffersink_get_##field(const AVFilterContext *ctx) { \
    av_assert0(fffilter(ctx->filter)->activate == activate); \
    return ctx->inputs[0]->field; \
}

MAKE_AVFILTERLINK_ACCESSOR(enum AVMediaType , type               )
MAKE_AVFILTERLINK_ACCESSOR(AVRational       , time_base          )
MAKE_AVFILTERLINK_ACCESSOR(int              , format             )

MAKE_AVFILTERLINK_ACCESSOR(int              , w                  )
MAKE_AVFILTERLINK_ACCESSOR(int              , h                  )
MAKE_AVFILTERLINK_ACCESSOR(AVRational       , sample_aspect_ratio)
MAKE_AVFILTERLINK_ACCESSOR(enum AVColorSpace, colorspace)
MAKE_AVFILTERLINK_ACCESSOR(enum AVColorRange, color_range)

MAKE_AVFILTERLINK_ACCESSOR(int              , sample_rate        )

AVRational av_buffersink_get_frame_rate(const AVFilterContext *ctx)
{
    FilterLink *l = ff_filter_link(ctx->inputs[0]);
    av_assert0(fffilter(ctx->filter)->activate == activate);
    return l->frame_rate;
}

AVBufferRef* av_buffersink_get_hw_frames_ctx(const AVFilterContext *ctx)
{
    FilterLink *l = ff_filter_link(ctx->inputs[0]);
    av_assert0(fffilter(ctx->filter)->activate == activate);
    return l->hw_frames_ctx;
}

int av_buffersink_get_channels(const AVFilterContext *ctx)
{
    av_assert0(fffilter(ctx->filter)->activate == activate);
    return ctx->inputs[0]->ch_layout.nb_channels;
}

int av_buffersink_get_ch_layout(const AVFilterContext *ctx, AVChannelLayout *out)
{
    AVChannelLayout ch_layout = { 0 };
    int ret;

    av_assert0(fffilter(ctx->filter)->activate == activate);
    ret = av_channel_layout_copy(&ch_layout, &ctx->inputs[0]->ch_layout);
    if (ret < 0)
        return ret;
    *out = ch_layout;
    return 0;
}

const AVFrameSideData *const *av_buffersink_get_side_data(const AVFilterContext *ctx,
                                                          int *nb_side_data)
{
    av_assert0(fffilter(ctx->filter)->activate == activate);
    *nb_side_data = ctx->inputs[0]->nb_side_data;
    return (const AVFrameSideData *const *)ctx->inputs[0]->side_data;
}

static int vsink_query_formats(const AVFilterContext *ctx,
                               AVFilterFormatsConfig **cfg_in,
                               AVFilterFormatsConfig **cfg_out)
{
    const BufferSinkContext *buf = ctx->priv;

    if (buf->nb_pixel_formats) {
        int ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, buf->pixel_formats);
        if (ret < 0)
            return ret;
    }
    if (buf->nb_color_spaces) {
        int ret = ff_set_common_color_spaces_from_list2(ctx, cfg_in, cfg_out, buf->color_spaces);
        if (ret < 0)
            return ret;
    }
    if (buf->nb_color_ranges) {
        int ret = ff_set_common_color_ranges_from_list2(ctx, cfg_in, cfg_out, buf->color_ranges);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int asink_query_formats(const AVFilterContext *ctx,
                               AVFilterFormatsConfig **cfg_in,
                               AVFilterFormatsConfig **cfg_out)
{
    const BufferSinkContext *buf = ctx->priv;

    if (buf->nb_sample_formats) {
        int ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, buf->sample_formats);
        if (ret < 0)
            return ret;
    }
    if (buf->nb_sample_rates) {
        int ret = ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, buf->sample_rates);
        if (ret < 0)
            return ret;
    }
    if (buf->nb_channel_layouts) {
        int ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, buf->channel_layouts);
        if (ret < 0)
            return ret;
    }

    return 0;
}

void av_buffersink_close(AVFilterContext *ctx)
{
    ff_inlink_set_status(ctx->inputs[0], AVERROR_EOF);
}

#define OFFSET(x) offsetof(BufferSinkContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
static const AVOptionArrayDef def_array = {.def=NULL,.size_min=0,.sep='|'};
static const AVOption buffersink_options[] = {
    { "pixel_formats", "array of supported pixel formats", OFFSET(pixel_formats),
        AV_OPT_TYPE_PIXEL_FMT | AV_OPT_TYPE_FLAG_ARRAY, {.arr=&def_array}, .max = INT_MAX, .flags = FLAGS },
    { "color_spaces", "array of supported color spaces",  OFFSET(color_spaces),
        AV_OPT_TYPE_INT | AV_OPT_TYPE_FLAG_ARRAY, {.arr=&def_array}, .max = INT_MAX, .flags = FLAGS },
    { "color_ranges", "array of supported color ranges",  OFFSET(color_ranges),
        AV_OPT_TYPE_INT | AV_OPT_TYPE_FLAG_ARRAY, {.arr=&def_array}, .max = INT_MAX, .flags = FLAGS },

    { NULL },
};
#undef FLAGS
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_AUDIO_PARAM
static const AVOption abuffersink_options[] = {
    { "sample_formats", "array of supported sample formats", OFFSET(sample_formats),
        AV_OPT_TYPE_SAMPLE_FMT | AV_OPT_TYPE_FLAG_ARRAY, {.arr=&def_array}, .max = INT_MAX, .flags = FLAGS },
    { "sample_rates", "array of supported sample rates", OFFSET(sample_rates),
        AV_OPT_TYPE_INT | AV_OPT_TYPE_FLAG_ARRAY, {.arr=&def_array}, .max = INT_MAX, .flags = FLAGS },
    { "channel_layouts", "array of supported channel layouts", OFFSET(channel_layouts),
        AV_OPT_TYPE_CHLAYOUT | AV_OPT_TYPE_FLAG_ARRAY, {.arr=&def_array}, .flags = FLAGS },
    { NULL },
};
#undef FLAGS

AVFILTER_DEFINE_CLASS(buffersink);
AVFILTER_DEFINE_CLASS(abuffersink);

const FFFilter ff_vsink_buffer = {
    .p.name        = "buffersink",
    .p.description = NULL_IF_CONFIG_SMALL("Buffer video frames, and make them available to the end of the filter graph."),
    .p.priv_class  = &buffersink_class,
    .p.outputs     = NULL,
    .priv_size     = sizeof(BufferSinkContext),
    .init          = init_video,
    .uninit        = uninit,
    .activate      = activate,
    FILTER_INPUTS(ff_video_default_filterpad),
    FILTER_QUERY_FUNC2(vsink_query_formats),
};

static const AVFilterPad inputs_audio[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input_audio,
    },
};

const FFFilter ff_asink_abuffer = {
    .p.name        = "abuffersink",
    .p.description = NULL_IF_CONFIG_SMALL("Buffer audio frames, and make them available to the end of the filter graph."),
    .p.priv_class  = &abuffersink_class,
    .p.outputs     = NULL,
    .priv_size     = sizeof(BufferSinkContext),
    .init          = init_audio,
    .uninit        = uninit,
    .activate      = activate,
    FILTER_INPUTS(inputs_audio),
    FILTER_QUERY_FUNC2(asink_query_formats),
};
