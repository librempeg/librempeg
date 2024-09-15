/*
 * Copyright (c) 1999 Chris Bagwell
 * Copyright (c) 1999 Nick Bailey
 * Copyright (c) 2007 Rob Sykes <robs@users.sourceforge.net>
 * Copyright (c) 2013 Paul B Mahol
 * Copyright (c) 2014 Andrew Kelley
 *
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
 * audio compand filter
 */

#include <float.h>

#include "libavutil/avassert.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct CompandContext {
    const AVClass *class;

    int nb_segments;
    float *attacks;
    unsigned nb_attacks;
    float *decays;
    unsigned nb_decays;
    float *in_points;
    unsigned nb_in_points;
    float *out_points;
    unsigned nb_out_points;

    void *segments;
    void *channels;

    double in_min_log;
    double out_min_lin;
    double curve_dB;
    double gain_dB;
    double initial_volume;
    double delay;
    AVFrame *delay_frame;
    int delay_samples;
    int delay_count;
    int delay_index;
    int64_t pts;

    int (*prepare)(AVFilterContext *ctx, AVFilterLink *outlink);
    int (*compand)(AVFilterContext *ctx, AVFrame *frame);
    void (*drain)(AVFilterContext *ctx, AVFrame *frame);
} CompandContext;

#define OFFSET(x) offsetof(CompandContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_attacks = {.def="0",.size_min=1,.sep='|'};
static const AVOptionArrayDef def_decays  = {.def="0.8",.size_min=1,.sep='|'};
static const AVOptionArrayDef def_in_points  = {.def="-70|-60|1",.size_min=1,.sep='|'};
static const AVOptionArrayDef def_out_points = {.def="-70|-20|0",.size_min=1,.sep='|'};

static const AVOption compand_options[] = {
    { "attacks", "set time over which increase of volume is determined", OFFSET(attacks), AV_OPT_TYPE_FLOAT|AR, { .arr = &def_attacks }, 0, 10, A },
    { "decays", "set time over which decrease of volume is determined", OFFSET(decays), AV_OPT_TYPE_FLOAT|AR, { .arr = &def_decays }, 0, 10, A },
    { "ipoints", "set input points of transfer function", OFFSET(in_points), AV_OPT_TYPE_FLOAT|AR, { .arr = &def_in_points }, -900, 900, A },
    { "opoints", "set output points of transfer function", OFFSET(out_points), AV_OPT_TYPE_FLOAT|AR, { .arr = &def_out_points }, -900, 900, A },
    { "soft-knee", "set soft-knee", OFFSET(curve_dB), AV_OPT_TYPE_DOUBLE, { .dbl = 0.01 }, 0.01, 900, A },
    { "gain", "set output gain", OFFSET(gain_dB), AV_OPT_TYPE_DOUBLE, { .dbl = 0 }, -900, 900, A },
    { "volume", "set initial volume", OFFSET(initial_volume), AV_OPT_TYPE_DOUBLE, { .dbl = 0 }, -900, 0, A },
    { "delay", "set delay for samples before sending them to volume adjuster", OFFSET(delay), AV_OPT_TYPE_DOUBLE, { .dbl = 0 }, 0, 20, A },
    { NULL }
};

AVFILTER_DEFINE_CLASS(compand);

static av_cold void uninit(AVFilterContext *ctx)
{
    CompandContext *s = ctx->priv;

    av_freep(&s->channels);
    av_freep(&s->segments);
    av_frame_free(&s->delay_frame);
}

#define DEPTH 32
#include "compand_template.c"

#undef DEPTH
#define DEPTH 64
#include "compand_template.c"

static int compand_drain(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    CompandContext *s    = ctx->priv;
    AVFrame *frame;

    /* 2048 is to limit output frame size during drain */
    frame = ff_get_audio_buffer(outlink, FFMIN(2048, s->delay_count));
    if (!frame)
        return AVERROR(ENOMEM);
    frame->pts = s->pts;
    s->pts += frame->nb_samples;

    s->drain(ctx, frame);

    return ff_filter_frame(outlink, frame);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx  = outlink->src;
    CompandContext *s     = ctx->priv;
    const int sample_rate = outlink->sample_rate;
    const int channels    = outlink->ch_layout.nb_channels;
    int ret;

    if (s->nb_attacks > channels || s->nb_decays > channels)
        av_log(ctx, AV_LOG_WARNING,
                "Number of attacks/decays bigger than number of channels. Ignoring rest of entries.\n");

    uninit(ctx);

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->drain = drain_fltp;
        s->prepare = prepare_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->drain = drain_dblp;
        s->prepare = prepare_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    s->nb_segments = (FFMAX(s->nb_in_points, s->nb_out_points) + 4) * 2;

    ret = s->prepare(ctx, outlink);
    if (ret < 0)
        return ret;

    s->delay_samples = lrint(s->delay * sample_rate);
    if (s->delay_samples <= 0) {
        s->compand = (outlink->format == AV_SAMPLE_FMT_FLTP) ? compand_nodelay_fltp : compand_nodelay_dblp;
    } else {
        s->delay_frame = ff_get_audio_buffer(outlink, s->delay_samples);
        if (!s->delay_frame)
            return AVERROR(ENOMEM);

        s->compand = (outlink->format == AV_SAMPLE_FMT_FLTP) ? compand_delay_fltp : compand_delay_dblp;
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    CompandContext *s    = ctx->priv;

    return s->compand(ctx, frame);
}

static int request_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    CompandContext *s    = ctx->priv;
    int ret = 0;

    ret = ff_request_frame(ctx->inputs[0]);

    if (ret == AVERROR_EOF && !ctx->is_disabled && s->delay_count)
        ret = compand_drain(outlink);

    return ret;
}

static const AVFilterPad compand_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad compand_outputs[] = {
    {
        .name          = "default",
        .request_frame = request_frame,
        .config_props  = config_output,
        .type          = AVMEDIA_TYPE_AUDIO,
    },
};


const AVFilter ff_af_compand = {
    .name           = "compand",
    .description    = NULL_IF_CONFIG_SMALL(
            "Compress or expand audio dynamic range."),
    .priv_size      = sizeof(CompandContext),
    .priv_class     = &compand_class,
    .uninit         = uninit,
    FILTER_INPUTS(compand_inputs),
    FILTER_OUTPUTS(compand_outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
};
