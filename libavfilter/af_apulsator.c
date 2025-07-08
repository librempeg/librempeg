/*
 * Copyright (c) 2001-2010 Krzysztof Foltman, Markus Schmidt, Thor Harald Johansen and others
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

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "audio.h"

enum PulsatorModes { SINE, TRIANGLE, SQUARE, SAWUP, SAWDOWN, NB_MODES };
enum PulsatorTimings { UNIT_BPM, UNIT_MS, UNIT_HZ, NB_TIMINGS };

typedef struct AudioPulsatorContext {
    const AVClass *class;

    int *mode;
    unsigned nb_mode;

    double *level_in;
    unsigned nb_level_in;

    double *level_out;
    unsigned nb_level_out;

    double *amount;
    unsigned nb_amount;

    double *offset;
    unsigned nb_offset;

    double *width;
    unsigned nb_width;

    double *bpm;
    unsigned nb_bpm;

    double *hertz;
    unsigned nb_hertz;

    double *ms;
    unsigned nb_ms;

    int *timing;
    unsigned nb_timing;

    double *phase;

    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} AudioPulsatorContext;

#define OFFSET(x) offsetof(AudioPulsatorContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_level = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_mode  = {.def="sine",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_amount= {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_offset= {.def="0",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_width = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_timing= {.def="hz",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_bpm   = {.def="120",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_ms    = {.def="500",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_hz    = {.def="2",.size_min=1,.sep=' '};

static const AVOption apulsator_options[] = {
    { "level_in",   "set input gain", OFFSET(level_in),  AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_level}, 0.015625, 64, FLAGS, },
    { "level_out", "set output gain", OFFSET(level_out), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_level}, 0.015625, 64, FLAGS, },
    { "mode",             "set mode", OFFSET(mode),      AV_OPT_TYPE_INT|AR,    {.arr=&def_mode},  SINE, NB_MODES-1, FLAGS, .unit = "mode" },
    {   "sine",                 NULL, 0,                 AV_OPT_TYPE_CONST,     {.i64=SINE},    0,            0, FLAGS, .unit = "mode" },
    {   "triangle",             NULL, 0,                 AV_OPT_TYPE_CONST,     {.i64=TRIANGLE},0,            0, FLAGS, .unit = "mode" },
    {   "square",               NULL, 0,                 AV_OPT_TYPE_CONST,     {.i64=SQUARE},  0,            0, FLAGS, .unit = "mode" },
    {   "sawup",                NULL, 0,                 AV_OPT_TYPE_CONST,     {.i64=SAWUP},   0,            0, FLAGS, .unit = "mode" },
    {   "sawdown",              NULL, 0,                 AV_OPT_TYPE_CONST,     {.i64=SAWDOWN}, 0,            0, FLAGS, .unit = "mode" },
    { "amount",     "set modulation", OFFSET(amount),    AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_amount},        0, 1, FLAGS },
    { "offset",         "set offset", OFFSET(offset),    AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_offset},        0, 1, FLAGS },
    { "width",     "set pulse width", OFFSET(width),     AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_width},   0.01, 1.99, FLAGS },
    { "timing",         "set timing", OFFSET(timing),    AV_OPT_TYPE_INT|AR,    {.arr=&def_timing},0,NB_TIMINGS-1,FLAGS,.unit = "timing" },
    {   "bpm",                  NULL, 0,                 AV_OPT_TYPE_CONST,     {.i64=UNIT_BPM},0,            0, FLAGS, .unit = "timing" },
    {   "ms",                   NULL, 0,                 AV_OPT_TYPE_CONST,     {.i64=UNIT_MS}, 0,            0, FLAGS, .unit = "timing" },
    {   "hz",                   NULL, 0,                 AV_OPT_TYPE_CONST,     {.i64=UNIT_HZ}, 0,            0, FLAGS, .unit = "timing" },
    { "bpm",               "set BPM", OFFSET(bpm),       AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_bpm},30,         300, FLAGS },
    { "ms",                 "set ms", OFFSET(ms),        AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_ms}, 10,        2000, FLAGS },
    { "hz",          "set frequency", OFFSET(hertz),     AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_hz}, 0.01,       100, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(apulsator);

typedef struct ThreadData {
    AVFrame *out, *in;
} ThreadData;

#define DEPTH 32
#include "apulsator_template.c"

#undef DEPTH
#define DEPTH 64
#include "apulsator_template.c"

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioPulsatorContext *s = ctx->priv;
    ThreadData td;
    AVFrame *out;

    if (av_frame_is_writable(in)) {
        out = in;
    } else {
        out = ff_get_audio_buffer(inlink, in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    td.in = in;
    td.out = out;
    ff_filter_execute(ctx, s->filter_channels, &td, NULL,
                      FFMIN(inlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    if (in != out)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioPulsatorContext *s = ctx->priv;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channels = filter_channels_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channels = filter_channels_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    s->phase = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->phase));
    if (!s->phase)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioPulsatorContext *s = ctx->priv;

    av_freep(&s->phase);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_apulsator = {
    .p.name        = "apulsator",
    .p.description = NULL_IF_CONFIG_SMALL("Audio pulsator."),
    .p.priv_class  = &apulsator_class,
    .priv_size     = sizeof(AudioPulsatorContext),
    .uninit        = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                     AVFILTER_FLAG_SLICE_THREADS,
    .process_command = ff_filter_process_command,
};
