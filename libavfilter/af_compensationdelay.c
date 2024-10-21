/*
 * Copyright (c) 2001-2010 Krzysztof Foltman, Markus Schmidt, Thor Harald Johansen, Vladimir Sadovnikov and others
 * Copyright (c) 2015 Paul B Mahol
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

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"

typedef struct CompensationDelayContext {
    const AVClass *class;
    int *distance_mm;
    unsigned distance_mm_size;
    int *distance_cm;
    unsigned distance_cm_size;
    int *distance_m;
    unsigned distance_m_size;
    double dry, wet;
    int *temp;
    unsigned temp_size;

    unsigned *w_ptr;
    unsigned buf_size;
    AVFrame *delay_frame;

    int (*delay_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} CompensationDelayContext;

#define OFFSET(x) offsetof(CompensationDelayContext, x)
#define A (AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM)
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_mm = {.def="0",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_cm = {.def="0",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_m  = {.def="0",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_t  = {.def="20",.size_min=1,.sep=' '};

static const AVOption compensationdelay_options[] = {
    { "mm",   "set mm distance",    OFFSET(distance_mm), AV_OPT_TYPE_INT|AR, {.arr=&def_mm}, 0,  10, A },
    { "cm",   "set cm distance",    OFFSET(distance_cm), AV_OPT_TYPE_INT|AR, {.arr=&def_cm}, 0, 100, A },
    { "m",    "set meter distance", OFFSET(distance_m),  AV_OPT_TYPE_INT|AR, {.arr=&def_m},  0, 100, A },
    { "dry",  "set dry amount",     OFFSET(dry),         AV_OPT_TYPE_DOUBLE, {.dbl=0},       0,   1, A },
    { "wet",  "set wet amount",     OFFSET(wet),         AV_OPT_TYPE_DOUBLE, {.dbl=1},       0,   1, A },
    { "temp", "set temperature °C", OFFSET(temp),        AV_OPT_TYPE_INT|AR, {.arr=&def_t},-50,  50, A },
    { NULL }
};

AVFILTER_DEFINE_CLASS(compensationdelay);

typedef struct ThreadData {
    AVFrame *out, *in;
} ThreadData;

#define DEPTH 32
#include "compensationdelay_template.c"

#undef DEPTH
#define DEPTH 64
#include "compensationdelay_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    CompensationDelayContext *s = ctx->priv;

    s->buf_size = 1 << av_ceil_log2(lrint(inlink->sample_rate * COMP_DELAY_MAX_DELAY));
    s->delay_frame = ff_get_audio_buffer(inlink, s->buf_size);
    s->w_ptr = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->w_ptr));
    if (!s->delay_frame || !s->w_ptr)
        return AVERROR(ENOMEM);

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->delay_channels = delay_channels_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->delay_channels = delay_channels_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    CompensationDelayContext *s = ctx->priv;
    ThreadData td;
    AVFrame *out;

    if (ff_filter_disabled(ctx)) {
        out = in;
    } else {
        out = ff_get_audio_buffer(outlink, in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    td.out = out;
    td.in = in;
    ff_filter_execute(ctx, s->delay_channels, &td, NULL,
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    if (out != in)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    CompensationDelayContext *s = ctx->priv;

    av_frame_free(&s->delay_frame);
    av_freep(&s->w_ptr);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_af_compensationdelay = {
    .p.name        = "compensationdelay",
    .p.description = NULL_IF_CONFIG_SMALL("Audio Compensation Delay Line."),
    .p.priv_class  = &compensationdelay_class,
    .priv_size     = sizeof(CompensationDelayContext),
    .uninit        = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .process_command = ff_filter_process_command,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
};
