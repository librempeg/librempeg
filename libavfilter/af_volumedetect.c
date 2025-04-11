/*
 * Copyright (c) 2012 Nicolas George
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/channel_layout.h"
#include "libavutil/avassert.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "audio.h"
#include "avfilter.h"

#define HISTOGRAM_SIZE 32769

typedef struct VolDetectContext {
    uint64_t histogram[HISTOGRAM_SIZE];
    float max;
    double sum2;

    void (*update_histogram)(AVFilterContext *ctx, const AVFrame *in);
    void (*print_stats)(AVFilterContext *ctx);
} VolDetectContext;

#define DEPTH 16
#define PLANAR 0
#include "volumedetect_template.c"

#undef PLANAR
#define PLANAR 1
#include "volumedetect_template.c"

#undef DEPTH
#define DEPTH 32
#include "volumedetect_template.c"

#undef PLANAR
#define PLANAR 0
#include "volumedetect_template.c"

#undef DEPTH
#define DEPTH 64
#include "volumedetect_template.c"

#undef PLANAR
#define PLANAR 1
#include "volumedetect_template.c"

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    VolDetectContext *s = ctx->priv;

    s->update_histogram(ctx, in);

    return ff_filter_frame(ctx->outputs[0], in);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    VolDetectContext *s = ctx->priv;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_DBL:
        s->update_histogram = update_histogram_dbl;
        s->print_stats = print_stats_dbl;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->update_histogram = update_histogram_dblp;
        s->print_stats = print_stats_dblp;
        break;
    case AV_SAMPLE_FMT_FLT:
        s->update_histogram = update_histogram_flt;
        s->print_stats = print_stats_flt;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->update_histogram = update_histogram_fltp;
        s->print_stats = print_stats_fltp;
        break;
    case AV_SAMPLE_FMT_S16:
        s->update_histogram = update_histogram_s16;
        s->print_stats = print_stats_s16;
        break;
    case AV_SAMPLE_FMT_S16P:
        s->update_histogram = update_histogram_s16p;
        s->print_stats = print_stats_s16p;
        break;
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    VolDetectContext *s = ctx->priv;

    if (s->print_stats)
        s->print_stats(ctx);
}

static const AVFilterPad volumedetect_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad volumedetect_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_volumedetect = {
    .p.name        = "volumedetect",
    .p.description = NULL_IF_CONFIG_SMALL("Detect audio volume."),
    .priv_size     = sizeof(VolDetectContext),
    .uninit        = uninit,
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY |
                     AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
    FILTER_INPUTS(volumedetect_inputs),
    FILTER_OUTPUTS(volumedetect_outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_S16,
                      AV_SAMPLE_FMT_S16P,
                      AV_SAMPLE_FMT_FLT,
                      AV_SAMPLE_FMT_FLTP,
                      AV_SAMPLE_FMT_DBL,
                      AV_SAMPLE_FMT_DBLP),
};
