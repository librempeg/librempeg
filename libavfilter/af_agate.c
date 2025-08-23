/*
 * Copyright (C) 2001-2010 Krzysztof Foltman, Markus Schmidt, Thor Harald Johansen, Damien Zammit
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
 * Audio (Sidechain) Gate filter
 */

#include <float.h>

#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"

enum LinkMode {
    LINKMODE_NONE,
    LINKMODE_AVG,
    LINKMODE_MAX,
    NB_LINKMODE
};

typedef struct AudioGateContext {
    const AVClass *class;

    double level_in;
    double level_sc;
    double attack;
    double release;
    double threshold;
    double ratio;
    double knee;
    double makeup;
    double range;
    int link;
    int detection;
    int direction;
    int sidechain;

    double thres;
    double knee_start;
    double knee_stop;
    double lin_knee_start;
    double lin_knee_stop;
    double attack_coeff;
    double release_coeff;

    void *lin_slope;

    AVFrame *in, *sc;

    void (*gate)(AVFilterContext *ctx, AVFrame *out, const int nb_samples,
                 AVFilterLink *inlink, AVFilterLink *sclink);
} AudioGateContext;

#define OFFSET(x) offsetof(AudioGateContext, x)
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption agate_options[] = {
    { "level_in",  "set input level",        OFFSET(level_in),  AV_OPT_TYPE_DOUBLE, {.dbl=1},           0,   64, A },
    { "direction", "set filtering direction",OFFSET(direction), AV_OPT_TYPE_INT,    {.i64=0},           0, 1, A, .unit = "direction" },
    {   "downward",0,                        0,                 AV_OPT_TYPE_CONST,  {.i64=0},           0, 0, A, .unit = "direction" },
    {   "upward",  0,                        0,                 AV_OPT_TYPE_CONST,  {.i64=1},           0, 0, A, .unit = "direction" },
    { "range",     "set max gain reduction", OFFSET(range),     AV_OPT_TYPE_DOUBLE, {.dbl=0.06125},     0, 1, A },
    { "threshold", "set threshold",          OFFSET(threshold), AV_OPT_TYPE_DOUBLE, {.dbl=0.125},       0, 1, A },
    { "ratio",     "set ratio",              OFFSET(ratio),     AV_OPT_TYPE_DOUBLE, {.dbl=2},           1, 1000, A },
    { "attack",    "set attack",             OFFSET(attack),    AV_OPT_TYPE_DOUBLE, {.dbl=20},          0.01, 1000, A },
    { "release",   "set release",            OFFSET(release),   AV_OPT_TYPE_DOUBLE, {.dbl=250},         0.01, 9000, A },
    { "makeup",    "set makeup gain",        OFFSET(makeup),    AV_OPT_TYPE_DOUBLE, {.dbl=1},           1,   64, A },
    { "knee",      "set knee",               OFFSET(knee),      AV_OPT_TYPE_DOUBLE, {.dbl=2.828427125}, 1,    8, A },
    { "detection", "set detection",          OFFSET(detection), AV_OPT_TYPE_INT,    {.i64=1},           0,    1, A, .unit = "detection" },
    {   "peak",    0,                        0,                 AV_OPT_TYPE_CONST,  {.i64=0},           0,    0, A, .unit = "detection" },
    {   "rms",     0,                        0,                 AV_OPT_TYPE_CONST,  {.i64=1},           0,    0, A, .unit = "detection" },
    { "link",   "set channels linking type", OFFSET(link),      AV_OPT_TYPE_INT,    {.i64=LINKMODE_NONE},0,NB_LINKMODE-1, A, .unit = "link" },
    {   "none",    0,                        0,                 AV_OPT_TYPE_CONST,  {.i64=LINKMODE_NONE},0,   0, A, .unit = "link" },
    {   "average", 0,                        0,                 AV_OPT_TYPE_CONST,  {.i64=LINKMODE_AVG},0,    0, A, .unit = "link" },
    {   "maximum", 0,                        0,                 AV_OPT_TYPE_CONST,  {.i64=LINKMODE_MAX},0,    0, A, .unit = "link" },
    { "level_sc",  "set sidechain gain",     OFFSET(level_sc),  AV_OPT_TYPE_DOUBLE, {.dbl=1},           0,   64, A },
    { "sidechain", "enable sidechain input", OFFSET(sidechain),AV_OPT_TYPE_BOOL,    {.i64=0},           0,    1, AF },
    { NULL }
};

AVFILTER_DEFINE_CLASS(agate);

#define DEPTH 32
#include "agate_template.c"

#undef DEPTH
#define DEPTH 64
#include "agate_template.c"

static int filter_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioGateContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *sclink = s->sidechain ? ctx->inputs[1] : inlink;
    AVFrame *out;

    if (av_frame_is_writable(s->in)) {
        out = s->in;
    } else {
        out = ff_get_audio_buffer(outlink, s->in->nb_samples);
        if (!out) {
            av_frame_free(&s->in);
            av_frame_free(&s->sc);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, s->in);
    }

    s->gate(ctx, out, s->in->nb_samples, inlink, sclink);

    if (out != s->in)
        ff_graph_frame_free(ctx, &s->in);
    s->in = NULL;
    ff_graph_frame_free(ctx, &s->sc);
    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    AudioGateContext *s = ctx->priv;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    if (!s->in) {
        int ret = ff_inlink_consume_frame(inlink, &s->in);
        if (ret < 0)
            return ret;
    }

    if (s->in) {
        if (s->sidechain && !s->sc) {
            AVFilterLink *sclink = ctx->inputs[1];
            int ret = ff_inlink_consume_samples(sclink, s->in->nb_samples,
                                                s->in->nb_samples, &s->sc);
            if (ret < 0)
                return ret;

            if (!ret) {
                FF_FILTER_FORWARD_STATUS(sclink, outlink);
                FF_FILTER_FORWARD_WANTED(outlink, sclink);
                return 0;
            }
        }

        return filter_frame(outlink);
    }

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    AudioGateContext *s = ctx->priv;
    double lin_threshold = s->threshold;
    double lin_knee_sqrt = sqrt(s->knee);
    size_t sample_size;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLT:
        s->gate = gate_flt;
        sample_size = sizeof(float);
        break;
    case AV_SAMPLE_FMT_DBL:
        s->gate = gate_dbl;
        sample_size = sizeof(double);
        break;
    default:
        return AVERROR_BUG;
    }

    if (!s->lin_slope)
        s->lin_slope = av_calloc(inlink->ch_layout.nb_channels, sample_size);
    if (!s->lin_slope)
        return AVERROR(ENOMEM);

    outlink->time_base = inlink->time_base;

    if (s->detection)
        lin_threshold *= lin_threshold;

    s->attack_coeff  = FFMIN(1., 1. / (s->attack * inlink->sample_rate / 4000.));
    s->release_coeff = FFMIN(1., 1. / (s->release * inlink->sample_rate / 4000.));
    s->lin_knee_stop = lin_threshold * lin_knee_sqrt;
    s->lin_knee_start = lin_threshold / lin_knee_sqrt;
    s->thres = log(lin_threshold + FLT_EPSILON);
    s->knee_start = log(s->lin_knee_start);
    s->knee_stop = log(s->lin_knee_stop);

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    AudioGateContext *s = ctx->priv;

    if (s->sidechain) {
        AVFilterPad pad = { NULL };

        pad.type = AVMEDIA_TYPE_AUDIO;
        pad.name = "sidechain";
        return ff_append_inpad(ctx, &pad);
    }

    return 0;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    AVFilterLink *outlink = ctx->outputs[0];
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return config_output(outlink);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioGateContext *s = ctx->priv;

    av_frame_free(&s->in);
    av_frame_free(&s->sc);

    av_freep(&s->lin_slope);
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_agate = {
    .p.name         = "agate",
    .p.description  = NULL_IF_CONFIG_SMALL("Audio gate."),
    .p.priv_class   = &agate_class,
    .priv_size      = sizeof(AudioGateContext),
    .activate       = activate,
    .init           = init,
    .uninit         = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_DBL),
    .process_command = process_command,
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                      AVFILTER_FLAG_DYNAMIC_INPUTS,
};
