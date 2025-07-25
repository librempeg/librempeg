/*
 * Copyright (c) 2018 Paul B Mahol
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

#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"
#include "drawutils.h"
#include "video.h"

enum PadMode {
    MODE_ADD = 0,
    MODE_CLONE,
    NB_MODE
};

typedef struct TPadContext {
    const AVClass *class;
    int pad_start;
    int pad_stop;
    int start_mode;
    int stop_mode;
    int64_t start_duration;
    int64_t stop_duration;
    uint8_t rgba_color[4];  ///< color for the padding area

    FFDrawContext draw;
    FFDrawColor color;
    int64_t pts;
    int eof;
    AVFrame *cache_start;
    AVFrame *cache_stop;
} TPadContext;

#define OFFSET(x) offsetof(TPadContext, x)
#define VF AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption tpad_options[] = {
    { "start", "set the number of frames to delay input",              OFFSET(pad_start),  AV_OPT_TYPE_INT,   {.i64=0},        0,   INT_MAX, VF },
    { "stop",  "set the number of frames to add after input finished", OFFSET(pad_stop),   AV_OPT_TYPE_INT,   {.i64=0},       -1,   INT_MAX, VF },
    { "start_mode", "set the mode of added frames to start",           OFFSET(start_mode), AV_OPT_TYPE_INT,   {.i64=MODE_ADD}, 0, NB_MODE-1, VF, .unit = "mode" },
    { "add",   "add solid-color frames",                               0,                  AV_OPT_TYPE_CONST, {.i64=MODE_ADD},   0,         0, VF, .unit = "mode" },
    { "clone", "clone first/last frame",                               0,                  AV_OPT_TYPE_CONST, {.i64=MODE_CLONE}, 0,         0, VF, .unit = "mode" },
    { "stop_mode",  "set the mode of added frames to end",             OFFSET(stop_mode),  AV_OPT_TYPE_INT,   {.i64=MODE_ADD}, 0, NB_MODE-1, VF, .unit = "mode" },
    { "start_duration", "set the duration to delay input",             OFFSET(start_duration), AV_OPT_TYPE_DURATION, {.i64=0}, 0, INT64_MAX, VF },
    { "stop_duration",  "set the duration to pad input",               OFFSET(stop_duration),  AV_OPT_TYPE_DURATION, {.i64=0}, 0, INT64_MAX, VF },
    { "color", "set the color of the added frames",                    OFFSET(rgba_color), AV_OPT_TYPE_COLOR, {.str="black"},  0,         0, VF },
    { NULL }
};

AVFILTER_DEFINE_CLASS(tpad);

static int needs_drawing(const TPadContext *s) {
    return (
        (s->stop_mode  == MODE_ADD && (s->pad_stop  != 0 || s->stop_duration  != 0)) ||
        (s->start_mode == MODE_ADD && (s->pad_start != 0 || s->start_duration != 0))
    );
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const TPadContext *s = ctx->priv;
    if (needs_drawing(s))
        return ff_set_common_formats2(ctx, cfg_in, cfg_out,
                                      ff_draw_supported_pixel_formats(0));

    return ff_set_common_formats2(ctx, cfg_in, cfg_out,
                                  ff_all_formats(AVMEDIA_TYPE_VIDEO));
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    FilterLink *l = ff_filter_link(outlink);
    TPadContext *s = ctx->priv;
    AVFrame *frame = NULL;
    int ret, status;
    int64_t duration, pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (!s->eof && ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (status == AVERROR_EOF) {
            pts = av_rescale_q(pts, inlink->time_base, outlink->time_base);
            if (!s->pad_stop && !s->pad_start) {
                ff_outlink_set_status(outlink, status, pts);
                return 0;
            }
            s->eof = 1;
            s->pts += pts;
        }
    }

    if (s->start_mode == MODE_ADD && s->pad_start > 0 && ff_outlink_frame_wanted(outlink)) {
        frame = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!frame)
            return AVERROR(ENOMEM);
        ff_fill_rectangle(&s->draw, &s->color,
                          frame->data, frame->linesize,
                          0, 0, frame->width, frame->height);
        duration = av_rescale_q(1, av_inv_q(l->frame_rate), outlink->time_base);
        frame->pts = s->pts;
        frame->duration = duration;
        s->pts += duration;
        s->pad_start--;
        return ff_filter_frame(outlink, frame);
    }

    if (s->start_mode == MODE_CLONE && s->pad_start > 0) {
        if (s->eof) {
            ff_outlink_set_status(outlink, AVERROR_EOF, 0);
            return 0;
        } else if (!s->cache_start && ff_inlink_queued_frames(inlink)) {
            s->cache_start = ff_inlink_peek_frame(inlink, 0);
        } else if (!s->cache_start) {
            FF_FILTER_FORWARD_WANTED(outlink, inlink);
        }
        frame = av_frame_clone(s->cache_start);
        if (!frame)
            return AVERROR(ENOMEM);
        duration = av_rescale_q(1, av_inv_q(l->frame_rate), outlink->time_base);
        frame->pts = s->pts;
        frame->duration = duration;
        s->pts += duration;
        s->pad_start--;
        if (s->pad_start == 0)
            s->cache_start = NULL;
        return ff_filter_frame(outlink, frame);
    }

    if (!s->eof && !s->pad_start) {
        ret = ff_inlink_consume_frame(inlink, &frame);
        if (ret < 0)
            return ret;
        if (ret > 0) {
            if (s->stop_mode == MODE_CLONE && s->pad_stop != 0) {
                av_frame_free(&s->cache_stop);
                s->cache_stop = av_frame_clone(frame);
            }
            frame->pts += s->pts;
            return ff_filter_frame(outlink, frame);
        }
    }

    if (s->eof) {
        if (!s->pad_stop) {
            ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
            return 0;
        }
        if (s->stop_mode == MODE_ADD) {
            frame = ff_get_video_buffer(outlink, outlink->w, outlink->h);
            if (!frame)
                return AVERROR(ENOMEM);
            ff_fill_rectangle(&s->draw, &s->color,
                              frame->data, frame->linesize,
                              0, 0, frame->width, frame->height);
        } else if (s->stop_mode == MODE_CLONE) {
            if (!s->cache_stop) {
                s->pad_stop = 0;
                ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
                return 0;
            }
            frame = av_frame_clone(s->cache_stop);
            if (!frame)
                return AVERROR(ENOMEM);
        }
        duration = av_rescale_q(1, av_inv_q(l->frame_rate), outlink->time_base);
        frame->pts = s->pts;
        frame->duration = duration;
        s->pts += duration;
        if (s->pad_stop > 0)
            s->pad_stop--;
        return ff_filter_frame(outlink, frame);
    }

    if (!s->pad_start)
        FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    FilterLink *l = ff_filter_link(inlink);
    TPadContext *s = ctx->priv;
    int ret;

    if (needs_drawing(s)) {
        ret = ff_draw_init2(&s->draw, inlink->format, inlink->colorspace, inlink->color_range, 0);
        if (ret < 0)
            return ret;
        ff_draw_color(&s->draw, &s->color, s->rgba_color);
    }

    if (s->start_duration)
        s->pad_start = av_rescale_q(s->start_duration, l->frame_rate, av_inv_q(AV_TIME_BASE_Q));
    if (s->stop_duration)
        s->pad_stop = av_rescale_q(s->stop_duration, l->frame_rate, av_inv_q(AV_TIME_BASE_Q));

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    TPadContext *s = ctx->priv;

    av_frame_free(&s->cache_stop);
}

static const AVFilterPad tpad_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
    },
};

const FFFilter ff_vf_tpad = {
    .p.name        = "tpad",
    .p.description = NULL_IF_CONFIG_SMALL("Temporarily pad video frames."),
    .p.priv_class  = &tpad_class,
    .priv_size     = sizeof(TPadContext),
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(tpad_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
};
