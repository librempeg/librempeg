/*
 * Copyright (c) 2019 The FFmpeg Project
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
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"

#define MAX_OVERSAMPLE 64

enum ASoftClipTypes {
    ASC_HARD = -1,
    ASC_TANH,
    ASC_ATAN,
    ASC_CUBIC,
    ASC_EXP,
    ASC_ALG,
    ASC_QUINTIC,
    ASC_SIN,
    ASC_ERF,
    NB_TYPES,
};

typedef struct Lowpass {
    float  fb0, fb1, fb2;
    float  fa0, fa1, fa2;

    double db0, db1, db2;
    double da0, da1, da2;
} Lowpass;

typedef struct ASoftClipContext {
    const AVClass *class;

    int type;
    int oversample;
    int64_t delay;
    double threshold;
    double output;
    double param;

    Lowpass lowpass[MAX_OVERSAMPLE];
    AVFrame *frame[2];

    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} ASoftClipContext;

#define OFFSET(x) offsetof(ASoftClipContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption asoftclip_options[] = {
    { "type", "set softclip type", OFFSET(type), AV_OPT_TYPE_INT,    {.i64=0},         -1, NB_TYPES-1, A, .unit = "types" },
    { "hard",                NULL,            0, AV_OPT_TYPE_CONST,  {.i64=ASC_HARD},   0,          0, A, .unit = "types" },
    { "tanh",                NULL,            0, AV_OPT_TYPE_CONST,  {.i64=ASC_TANH},   0,          0, A, .unit = "types" },
    { "atan",                NULL,            0, AV_OPT_TYPE_CONST,  {.i64=ASC_ATAN},   0,          0, A, .unit = "types" },
    { "cubic",               NULL,            0, AV_OPT_TYPE_CONST,  {.i64=ASC_CUBIC},  0,          0, A, .unit = "types" },
    { "exp",                 NULL,            0, AV_OPT_TYPE_CONST,  {.i64=ASC_EXP},    0,          0, A, .unit = "types" },
    { "alg",                 NULL,            0, AV_OPT_TYPE_CONST,  {.i64=ASC_ALG},    0,          0, A, .unit = "types" },
    { "quintic",             NULL,            0, AV_OPT_TYPE_CONST,  {.i64=ASC_QUINTIC},0,          0, A, .unit = "types" },
    { "sin",                 NULL,            0, AV_OPT_TYPE_CONST,  {.i64=ASC_SIN},    0,          0, A, .unit = "types" },
    { "erf",                 NULL,            0, AV_OPT_TYPE_CONST,  {.i64=ASC_ERF},    0,          0, A, .unit = "types" },
    { "threshold", "set softclip threshold", OFFSET(threshold), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0.000001, 1, A },
    { "output", "set softclip output gain", OFFSET(output), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0.000001, 16, A },
    { "param", "set softclip parameter", OFFSET(param), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0.01,        3, A },
    { "oversample", "set oversample factor", OFFSET(oversample), AV_OPT_TYPE_INT, {.i64=1}, 1, MAX_OVERSAMPLE, A },
    { NULL }
};

AVFILTER_DEFINE_CLASS(asoftclip);

static void get_lowpass(Lowpass *s,
                        double frequency,
                        double sample_rate)
{
    double w0 = 2 * M_PI * frequency / sample_rate;
    double alpha = sin(w0) / (2 * 0.8);
    double factor;

    s->da0 =  1 + alpha;
    s->da1 = -2 * cos(w0);
    s->da2 =  1 - alpha;
    s->db0 = (1 - cos(w0)) / 2;
    s->db1 =  1 - cos(w0);
    s->db2 = (1 - cos(w0)) / 2;

    s->da1 /= s->da0;
    s->da2 /= s->da0;
    s->db0 /= s->da0;
    s->db1 /= s->da0;
    s->db2 /= s->da0;
    s->da0 /= s->da0;

    factor = (s->da0 + s->da1 + s->da2) / (s->db0 + s->db1 + s->db2);
    s->db0 *= factor;
    s->db1 *= factor;
    s->db2 *= factor;

    s->fa0 = s->da0;
    s->fa1 = s->da1;
    s->fa2 = s->da2;
    s->fb0 = s->db0;
    s->fb1 = s->db1;
    s->fb2 = s->db2;
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#define DEPTH 32
#include "asoftclip_template.c"

#undef DEPTH
#define DEPTH 64
#include "asoftclip_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    ASoftClipContext *s = ctx->priv;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP: s->filter_channels = filter_channels_fltp; break;
    case AV_SAMPLE_FMT_DBLP: s->filter_channels = filter_channels_dblp; break;
    default: av_assert0(0);
    }

    s->frame[0] = ff_get_audio_buffer(inlink, 2 * MAX_OVERSAMPLE);
    s->frame[1] = ff_get_audio_buffer(inlink, 2 * MAX_OVERSAMPLE);
    if (!s->frame[0] || !s->frame[1])
        return AVERROR(ENOMEM);

    for (int i = 0; i < MAX_OVERSAMPLE; i++) {
        get_lowpass(&s->lowpass[i], inlink->sample_rate / 2, inlink->sample_rate * (i + 1));
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ASoftClipContext *s = ctx->priv;
    ThreadData td;
    AVFrame *out;

    if (av_frame_is_writable(in) && s->oversample == 1) {
        out = in;
    } else {
        out = ff_get_audio_buffer(outlink, in->nb_samples * s->oversample);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    td.in = in;
    td.out = out;

    ff_filter_execute(ctx, s->filter_channels, &td, NULL,
                      FFMIN(outlink->ch_layout.nb_channels,
                            ff_filter_get_nb_threads(ctx)));

    if (out != in)
        ff_graph_frame_free(ctx, &in);

    out->nb_samples /= s->oversample;
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ASoftClipContext *s = ctx->priv;

    av_frame_free(&s->frame[0]);
    av_frame_free(&s->frame[1]);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_asoftclip = {
    .p.name         = "asoftclip",
    .p.description  = NULL_IF_CONFIG_SMALL("Audio Soft Clipper."),
    .p.priv_class   = &asoftclip_class,
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                      AVFILTER_FLAG_SLICE_THREADS,
    .priv_size      = sizeof(ASoftClipContext),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .uninit         = uninit,
    .process_command = ff_filter_process_command,
};
