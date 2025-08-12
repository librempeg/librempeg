/*
 * Copyright (c) 2012 Stefano Sabatini
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
 * Video black detector, loosely based on blackframe with extended
 * syntax and features
 */

#include <float.h>
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/timestamp.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "video.h"
#include "vf_blackdetect.h"

typedef struct BlackDetectContext {
    const AVClass *class;
    double  black_min_duration_time; ///< minimum duration of detected black, in seconds
    int64_t black_min_duration;      ///< minimum duration of detected black, expressed in timebase units
    int64_t black_start;             ///< pts start time of the first black picture
    int64_t black_end;               ///< pts end time of the last black picture
    int64_t last_picref_pts;         ///< pts of the last input picture
    int black_started;

    double       picture_black_ratio_th;
    double       pixel_black_th;
    unsigned int pixel_black_th_i;
    int          alpha;

    unsigned int nb_black_pixels;   ///< number of black pixels counted so far
    AVRational   time_base;
    int          depth;
    int          nb_threads;
    unsigned int *counter;

    ff_blackdetect_fn func;
} BlackDetectContext;

#define OFFSET(x) offsetof(BlackDetectContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption blackdetect_options[] = {
    { "d",                  "set minimum detected black duration in seconds", OFFSET(black_min_duration_time), AV_OPT_TYPE_DOUBLE, {.dbl=2}, 0, DBL_MAX, FLAGS },
    { "black_min_duration", "set minimum detected black duration in seconds", OFFSET(black_min_duration_time), AV_OPT_TYPE_DOUBLE, {.dbl=2}, 0, DBL_MAX, FLAGS },
    { "picture_black_ratio_th", "set the picture black ratio threshold", OFFSET(picture_black_ratio_th), AV_OPT_TYPE_DOUBLE, {.dbl=.98}, 0, 1, FLAGS },
    { "pic_th",                 "set the picture black ratio threshold", OFFSET(picture_black_ratio_th), AV_OPT_TYPE_DOUBLE, {.dbl=.98}, 0, 1, FLAGS },
    { "pixel_black_th", "set the pixel black threshold", OFFSET(pixel_black_th), AV_OPT_TYPE_DOUBLE, {.dbl=.10}, 0, 1, FLAGS },
    { "pix_th",         "set the pixel black threshold", OFFSET(pixel_black_th), AV_OPT_TYPE_DOUBLE, {.dbl=.10}, 0, 1, FLAGS },
    { "alpha",          "check alpha instead of luma", OFFSET(alpha), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(blackdetect);

#define YUVJ_FORMATS \
    AV_PIX_FMT_YUVJ411P, AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ440P

#define YUVA_FORMATS \
    AV_PIX_FMT_YUVA420P,  AV_PIX_FMT_YUVA422P,   AV_PIX_FMT_YUVA444P, \
    AV_PIX_FMT_YUVA444P9, AV_PIX_FMT_YUVA444P10, AV_PIX_FMT_YUVA444P12, AV_PIX_FMT_YUVA444P16, \
    AV_PIX_FMT_YUVA422P9, AV_PIX_FMT_YUVA422P10, AV_PIX_FMT_YUVA422P12, AV_PIX_FMT_YUVA422P16, \
    AV_PIX_FMT_YUVA420P9, AV_PIX_FMT_YUVA420P10, AV_PIX_FMT_YUVA420P16

static const enum AVPixelFormat yuvj_formats[] = {
    YUVJ_FORMATS, AV_PIX_FMT_NONE
};

static const enum AVPixelFormat yuva_formats[] = {
    YUVA_FORMATS, AV_PIX_FMT_NONE
};

static const enum AVPixelFormat yuv_formats[] = {
    AV_PIX_FMT_GRAY8,
    AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV411P,
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_NV12, AV_PIX_FMT_NV21,
    YUVJ_FORMATS,
    AV_PIX_FMT_GRAY10, AV_PIX_FMT_GRAY12, AV_PIX_FMT_GRAY14,
    AV_PIX_FMT_GRAY16,
    AV_PIX_FMT_YUV420P9, AV_PIX_FMT_YUV422P9, AV_PIX_FMT_YUV444P9,
    AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV444P10,
    AV_PIX_FMT_YUV440P10,
    AV_PIX_FMT_YUV444P12, AV_PIX_FMT_YUV422P12, AV_PIX_FMT_YUV420P12,
    AV_PIX_FMT_YUV440P12,
    AV_PIX_FMT_YUV444P14, AV_PIX_FMT_YUV422P14, AV_PIX_FMT_YUV420P14,
    AV_PIX_FMT_YUV420P16, AV_PIX_FMT_YUV422P16, AV_PIX_FMT_YUV444P16,
    YUVA_FORMATS, AV_PIX_FMT_NONE
};

static int query_format(const AVFilterContext *ctx,
                        AVFilterFormatsConfig **cfg_in,
                        AVFilterFormatsConfig **cfg_out)
{
    const BlackDetectContext *s = ctx->priv;
    AVFilterFormats *formats;
    if (s->alpha)
        formats = ff_make_format_list(yuva_formats);
    else
        formats = ff_make_format_list(yuv_formats);

    return ff_set_common_formats2(ctx, cfg_in, cfg_out, formats);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    BlackDetectContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    const int depth = desc->comp[0].depth;

    s->depth = depth;
    s->nb_threads = ff_filter_get_nb_threads(ctx);
    s->time_base = inlink->time_base;
    s->black_min_duration = s->black_min_duration_time / av_q2d(s->time_base);
    s->counter = av_calloc(s->nb_threads, sizeof(*s->counter));
    s->func = ff_blackdetect_get_fn(depth);
    if (!s->counter)
        return AVERROR(ENOMEM);

    av_log(ctx, AV_LOG_VERBOSE,
           "black_min_duration:%s pixel_black_th:%f picture_black_ratio_th:%f alpha:%d\n",
           av_ts2timestr(s->black_min_duration, &s->time_base),
           s->pixel_black_th, s->picture_black_ratio_th, s->alpha);
    return 0;
}

static void check_black_end(AVFilterContext *ctx)
{
    BlackDetectContext *s = ctx->priv;

    if ((s->black_end - s->black_start) >= s->black_min_duration) {
        av_log(ctx, AV_LOG_INFO,
               "black_start:%s black_end:%s black_duration:%s\n",
               av_ts2timestr(s->black_start, &s->time_base),
               av_ts2timestr(s->black_end,   &s->time_base),
               av_ts2timestr(s->black_end - s->black_start, &s->time_base));
    }
}

static int black_counter(AVFilterContext *ctx, void *arg,
                         int jobnr, int nb_jobs)
{
    BlackDetectContext *s = ctx->priv;
    const AVFrame *in = arg;
    const int plane = s->alpha ? 3 : 0;
    const int linesize = in->linesize[plane];
    const int h = in->height;
    const int start = (h * jobnr) / nb_jobs;
    const int end = (h * (jobnr+1)) / nb_jobs;

    s->counter[jobnr] = s->func(in->data[plane] + start * linesize,
                                linesize, in->width, end - start,
                                s->pixel_black_th_i);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *picref)
{
    FilterLink *inl = ff_filter_link(inlink);
    AVFilterContext *ctx = inlink->dst;
    BlackDetectContext *s = ctx->priv;
    double picture_black_ratio = 0;
    const int max = (1 << s->depth) - 1;
    const int factor = (1 << (s->depth - 8));
    const int full = picref->color_range == AVCOL_RANGE_JPEG ||
                     ff_fmt_is_in(picref->format, yuvj_formats) ||
                     s->alpha;

    s->pixel_black_th_i = full ? s->pixel_black_th * max :
        // luminance_minimum_value + pixel_black_th * luminance_range_size
        16 * factor + s->pixel_black_th * (235 - 16) * factor;

    ff_filter_execute(ctx, black_counter, picref, NULL,
                      FFMIN(inlink->h, s->nb_threads));

    for (int i = 0; i < s->nb_threads; i++)
        s->nb_black_pixels += s->counter[i];

    picture_black_ratio = (double)s->nb_black_pixels / (inlink->w * inlink->h);

    av_log(ctx, AV_LOG_DEBUG,
           "frame:%"PRId64" picture_black_ratio:%f pts:%s t:%s type:%c\n",
           inl->frame_count_out, picture_black_ratio,
           av_ts2str(picref->pts), av_ts2timestr(picref->pts, &s->time_base),
           av_get_picture_type_char(picref->pict_type));

    if (picture_black_ratio >= s->picture_black_ratio_th) {
        if (!s->black_started) {
            /* black starts here */
            s->black_started = 1;
            s->black_start = picref->pts;
            av_dict_set(&picref->metadata, "lavfi.black_start",
                av_ts2timestr(s->black_start, &s->time_base), 0);
        }
    } else if (s->black_started) {
        /* black ends here */
        s->black_started = 0;
        s->black_end = picref->pts;
        check_black_end(ctx);
        av_dict_set(&picref->metadata, "lavfi.black_end",
            av_ts2timestr(s->black_end, &s->time_base), 0);
    }

    s->last_picref_pts = picref->pts;
    s->nb_black_pixels = 0;
    return ff_filter_frame(inlink->dst->outputs[0], picref);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    BlackDetectContext *s = ctx->priv;

    av_freep(&s->counter);

    if (s->black_started) {
        // FIXME: black_end should be set to last_picref_pts + last_picref_duration
        s->black_end = s->last_picref_pts;
        check_black_end(ctx);
    }
}

static const AVFilterPad blackdetect_inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_input,
        .filter_frame  = filter_frame,
    },
};

const FFFilter ff_vf_blackdetect = {
    .p.name        = "blackdetect",
    .p.description = NULL_IF_CONFIG_SMALL("Detect video intervals that are (almost) black."),
    .p.priv_class  = &blackdetect_class,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS | AVFILTER_FLAG_METADATA_ONLY,
    .priv_size     = sizeof(BlackDetectContext),
    FILTER_INPUTS(blackdetect_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_QUERY_FUNC2(query_format),
    .uninit        = uninit,
};
