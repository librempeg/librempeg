/*
 * Copyright (c) 2001-2010 Krzysztof Foltman, Markus Schmidt, Thor Harald Johansen and others
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
} AudioPulsatorContext;

#define OFFSET(x) offsetof(AudioPulsatorContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
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

static double lfo_get_value(const int mode, double amount,
                            double phase, double width, double offset)
{
    double phs = phase / width + offset;
    double val;

    if (phs > 1)
        phs = fmod(phs, 1.);

    switch (mode) {
    case SINE:
        val = sin(phs * 2.0 * M_PI);
        break;
    case TRIANGLE:
        if (phs > 0.75)
            val = (phs - 0.75) * 4.0 - 1.0;
        else if (phs > 0.25)
            val = -4.0 * phs + 2.0;
        else
            val = phs * 4.0;
        break;
    case SQUARE:
        val = phs < 0.5 ? -1.0 : 1.0;
        break;
    case SAWUP:
        val = phs * 2.0 - 1.0;
        break;
    case SAWDOWN:
        val = 1.0 - phs * 2.0;
        break;
    default:
        av_assert0(0);
    }

    return val * amount;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioPulsatorContext *s = ctx->priv;
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

    for (int ch = 0; ch < in->ch_layout.nb_channels; ch++) {
        const double *src = (const double *)in->extended_data[ch];
        const double level_out = s->level_out[FFMIN(ch, s->nb_level_out-1)];
        const double level_in = s->level_in[FFMIN(ch, s->nb_level_in-1)];
        const double amount = s->amount[FFMIN(ch, s->nb_amount-1)];
        const int timing = s->timing[FFMIN(ch, s->nb_timing-1)];
        const double offset = s->offset[FFMIN(ch, s->nb_offset-1)];
        const double width = s->width[FFMIN(ch, s->nb_width-1)];
        const double hertz = s->hertz[FFMIN(ch, s->nb_hertz-1)];
        const double bpm = s->bpm[FFMIN(ch, s->nb_bpm-1)];
        const int mode = s->mode[FFMIN(ch, s->nb_mode-1)];
        const double ms = s->ms[FFMIN(ch, s->nb_ms-1)];
        double *dst = (double *)out->extended_data[ch];
        const double fs = 1.0 / inlink->sample_rate;
        const int nb_samples = in->nb_samples;
        double phase = s->phase[ch];
        double freq;

        switch (timing) {
        case UNIT_BPM:
            freq = bpm / 60.0;
            break;
        case UNIT_MS:
            freq = 1.0 / (ms / 1000.0);
            break;
        case UNIT_HZ:
            freq = hertz;
            break;
        default:
            av_assert0(0);
        }

        for (int n = 0; n < nb_samples; n++) {
            double in = src[n] * level_in;
            double proc = in;
            double out;

            proc *= lfo_get_value(mode, amount, phase, width,
                                  offset) * 0.5 + amount / 2.0;

            out = proc + in * (1.0 - amount);
            out *= level_out;

            dst[n] = out;

            phase = fabs(phase + freq * fs);
            if (phase >= 1.0)
                phase = fmod(phase, 1.0);
        }

        s->phase[ch] = phase;
    }

    if (in != out)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioPulsatorContext *s = ctx->priv;

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

const AVFilter ff_af_apulsator = {
    .name          = "apulsator",
    .description   = NULL_IF_CONFIG_SMALL("Audio pulsator."),
    .priv_size     = sizeof(AudioPulsatorContext),
    .priv_class    = &apulsator_class,
    .uninit        = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_DBLP),
};
