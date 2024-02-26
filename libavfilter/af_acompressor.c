/*
 * Copyright (C) 2001-2010 Krzysztof Foltman, Markus Schmidt, Thor Harald Johansen and others
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

/**
 * @file
 * Audio (Sidechain) Compressor filter
 */

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "hermite.h"
#include "internal.h"

typedef struct AudioCompressorContext {
    const AVClass *class;

    double level_in;
    double level_sc;
    double attack, attack_coeff;
    double release, release_coeff;
    double lin_slope;
    double ratio;
    double threshold;
    double makeup;
    double mix;
    double thres;
    double knee;
    double knee_start;
    double knee_stop;
    double lin_knee_start;
    double lin_knee_stop;
    double adj_knee_start;
    double adj_knee_stop;
    double compressed_knee_start;
    double compressed_knee_stop;
    int link;
    int detection;
    int mode;
    int sidechain;

    AVFrame *in, *sc;
} AudioCompressorContext;

#define OFFSET(x) offsetof(AudioCompressorContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM
#define R AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption acompressor_options[] = {
    { "level_in",  "set input gain",     OFFSET(level_in),  AV_OPT_TYPE_DOUBLE, {.dbl=1},        0.015625,   64, A|F|R },
    { "mode",      "set mode",           OFFSET(mode),      AV_OPT_TYPE_INT,    {.i64=0},               0,    1, A|F|R, .unit = "mode" },
    {   "downward",0,                    0,                 AV_OPT_TYPE_CONST,  {.i64=0},               0,    0, A|F|R, .unit = "mode" },
    {   "upward",  0,                    0,                 AV_OPT_TYPE_CONST,  {.i64=1},               0,    0, A|F|R, .unit = "mode" },
    { "threshold", "set threshold",      OFFSET(threshold), AV_OPT_TYPE_DOUBLE, {.dbl=0.125}, 0.000976563,    1, A|F|R },
    { "ratio",     "set ratio",          OFFSET(ratio),     AV_OPT_TYPE_DOUBLE, {.dbl=2},               1,   20, A|F|R },
    { "attack",    "set attack",         OFFSET(attack),    AV_OPT_TYPE_DOUBLE, {.dbl=20},           0.01, 2000, A|F|R },
    { "release",   "set release",        OFFSET(release),   AV_OPT_TYPE_DOUBLE, {.dbl=250},          0.01, 9000, A|F|R },
    { "makeup",    "set make up gain",   OFFSET(makeup),    AV_OPT_TYPE_DOUBLE, {.dbl=1},               1,   64, A|F|R },
    { "knee",      "set knee",           OFFSET(knee),      AV_OPT_TYPE_DOUBLE, {.dbl=2.82843},         1,    8, A|F|R },
    { "link",      "set link type",      OFFSET(link),      AV_OPT_TYPE_INT,    {.i64=0},               0,    1, A|F|R, .unit = "link" },
    {   "average", 0,                    0,                 AV_OPT_TYPE_CONST,  {.i64=0},               0,    0, A|F|R, .unit = "link" },
    {   "maximum", 0,                    0,                 AV_OPT_TYPE_CONST,  {.i64=1},               0,    0, A|F|R, .unit = "link" },
    { "detection", "set detection",      OFFSET(detection), AV_OPT_TYPE_INT,    {.i64=1},               0,    1, A|F|R, .unit = "detection" },
    {   "peak",    0,                    0,                 AV_OPT_TYPE_CONST,  {.i64=0},               0,    0, A|F|R, .unit = "detection" },
    {   "rms",     0,                    0,                 AV_OPT_TYPE_CONST,  {.i64=1},               0,    0, A|F|R, .unit = "detection" },
    { "level_sc",  "set sidechain gain", OFFSET(level_sc),  AV_OPT_TYPE_DOUBLE, {.dbl=1},        0.015625,   64, A|F|R },
    { "mix",       "set mix",            OFFSET(mix),       AV_OPT_TYPE_DOUBLE, {.dbl=1},               0,    1, A|F|R },
    { "sidechain", "enable sidechain input",OFFSET(sidechain),AV_OPT_TYPE_BOOL, {.i64=0},               0,    1, A|F },
    { NULL }
};

AVFILTER_DEFINE_CLASS(acompressor);

// A fake infinity value (because real infinity may break some hosts)
#define FAKE_INFINITY (65536.0 * 65536.0)

// Check for infinity (with appropriate-ish tolerance)
#define IS_FAKE_INFINITY(value) (fabs(value-FAKE_INFINITY) < 1.0)

static double output_gain(double lin_slope, double ratio, double thres,
                          double knee, double knee_start, double knee_stop,
                          double compressed_knee_start,
                          double compressed_knee_stop,
                          int detection, int mode)
{
    double slope = log(lin_slope);
    double gain = 0.0;
    double delta = 0.0;

    if (detection)
        slope *= 0.5;

    if (IS_FAKE_INFINITY(ratio)) {
        gain = thres;
        delta = 0.0;
    } else {
        gain = (slope - thres) / ratio + thres;
        delta = 1.0 / ratio;
    }

    if (mode) {
        if (knee > 1.0 && slope > knee_start)
            gain = hermite_interpolation(slope, knee_stop, knee_start,
                                         knee_stop, compressed_knee_start,
                                         1.0, delta);
    } else {
        if (knee > 1.0 && slope < knee_stop)
            gain = hermite_interpolation(slope, knee_start, knee_stop,
                                         knee_start, compressed_knee_stop,
                                         1.0, delta);
    }

    return exp(gain - slope);
}

static void compressor(AudioCompressorContext *s,
                       const double *src, double *dst, const double *scsrc, int nb_samples,
                       double level_in, double level_sc,
                       AVFilterLink *inlink, AVFilterLink *sclink)
{
    AVFilterContext *ctx = inlink->dst;
    const double makeup = s->makeup;
    const double mix = s->mix;
    int i, c;

    for (i = 0; i < nb_samples; i++) {
        double abs_sample, gain = 1.0;
        double detector, factor;
        int detected;

        abs_sample = fabs(scsrc[0] * level_sc);

        if (s->link == 1) {
            for (c = 1; c < sclink->ch_layout.nb_channels; c++)
                abs_sample = FFMAX(fabs(scsrc[c] * level_sc), abs_sample);
        } else {
            for (c = 1; c < sclink->ch_layout.nb_channels; c++)
                abs_sample += fabs(scsrc[c] * level_sc);

            abs_sample /= sclink->ch_layout.nb_channels;
        }

        if (s->detection)
            abs_sample *= abs_sample;

        s->lin_slope += (abs_sample - s->lin_slope) * (abs_sample > s->lin_slope ? s->attack_coeff : s->release_coeff);

        if (s->mode) {
            detector = (s->detection ? s->adj_knee_stop : s->lin_knee_stop);
            detected = s->lin_slope < detector;
        } else {
            detector = (s->detection ? s->adj_knee_start : s->lin_knee_start);
            detected = s->lin_slope > detector;
        }

        if (s->lin_slope > 0.0 && detected)
            gain = output_gain(s->lin_slope, s->ratio, s->thres, s->knee,
                               s->knee_start, s->knee_stop,
                               s->compressed_knee_start,
                               s->compressed_knee_stop,
                               s->detection, s->mode);

        factor = ctx->is_disabled ? 1.0 : level_in * (gain * makeup * mix + (1. - mix));
        for (c = 0; c < inlink->ch_layout.nb_channels; c++)
            dst[c] = src[c] * factor;

        src += inlink->ch_layout.nb_channels;
        dst += inlink->ch_layout.nb_channels;
        scsrc += sclink->ch_layout.nb_channels;
    }
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    AudioCompressorContext *s = ctx->priv;

    outlink->time_base = inlink->time_base;

    s->thres = log(s->threshold);
    s->lin_knee_start = s->threshold / sqrt(s->knee);
    s->lin_knee_stop = s->threshold * sqrt(s->knee);
    s->adj_knee_start = s->lin_knee_start * s->lin_knee_start;
    s->adj_knee_stop = s->lin_knee_stop * s->lin_knee_stop;
    s->knee_start = log(s->lin_knee_start);
    s->knee_stop = log(s->lin_knee_stop);
    s->compressed_knee_start = (s->knee_start - s->thres) / s->ratio + s->thres;
    s->compressed_knee_stop = (s->knee_stop - s->thres) / s->ratio + s->thres;

    s->attack_coeff = FFMIN(1., 1. / (s->attack * outlink->sample_rate / 4000.));
    s->release_coeff = FFMIN(1., 1. / (s->release * outlink->sample_rate / 4000.));

    return 0;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *args,
                           char *res, int res_len, int flags)
{
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret < 0)
        return ret;

    config_output(ctx->outputs[0]);

    return 0;
}

static int filter_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioCompressorContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *sclink = s->sidechain ? ctx->inputs[1] : inlink;
    AVFrame *sc = s->sc ? s->sc : s->in;
    const double *scsrc = (const double *)sc->data[0];
    const double *src = (const double *)s->in->data[0];
    AVFrame *out;
    double *dst;

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
    dst = (double *)out->data[0];

    compressor(s, src, dst, scsrc, s->in->nb_samples,
               s->level_in, s->level_in,
               inlink, sclink);

    if (out != s->in)
        av_frame_free(&s->in);
    s->in = NULL;
    av_frame_free(&s->sc);
    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    AudioCompressorContext *s = ctx->priv;

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
        }

        return filter_frame(outlink);
    }

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    if (s->sidechain)
        FF_FILTER_FORWARD_STATUS(ctx->inputs[1], outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold int init(AVFilterContext *ctx)
{
    AudioCompressorContext *s = ctx->priv;
    AVFilterPad pad = { NULL };
    int ret;

    pad.type = AVMEDIA_TYPE_AUDIO;
    pad.name = av_asprintf("main");
    if (!pad.name)
        return AVERROR(ENOMEM);
    if ((ret = ff_append_inpad_free_name(ctx, &pad)) < 0)
        return ret;

    if (s->sidechain) {
        AVFilterPad pad = { NULL };

        pad.type = AVMEDIA_TYPE_AUDIO;
        pad.name = av_asprintf("sidechain");
        if (!pad.name)
            return AVERROR(ENOMEM);
        if ((ret = ff_append_inpad_free_name(ctx, &pad)) < 0)
            return ret;
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioCompressorContext *s = ctx->priv;

    av_frame_free(&s->in);
    av_frame_free(&s->sc);
}

static const AVFilterPad outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
    },
};

const AVFilter ff_af_acompressor = {
    .name           = "acompressor",
    .description    = NULL_IF_CONFIG_SMALL("Audio compressor."),
    .priv_class     = &acompressor_class,
    .priv_size      = sizeof(AudioCompressorContext),
    .activate       = activate,
    .init           = init,
    .uninit         = uninit,
    .inputs         = NULL,
    FILTER_OUTPUTS(outputs),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_DBL),
    .process_command = process_command,
    .flags          = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                      AVFILTER_FLAG_DYNAMIC_INPUTS,
};
