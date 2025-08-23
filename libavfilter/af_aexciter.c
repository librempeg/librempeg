/*
 * Copyright (c) Markus Schmidt and Christian Holschuh
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

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "filters.h"
#include "audio.h"

typedef struct AExciterContext {
    const AVClass *class;

    double level_in;
    double level_out;
    double amount;
    double drive;
    double blend;
    double freq;
    double ceil;
    int listen;

    void *cp;

    int (*init_filter)(AVFilterContext *ctx);
    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} AExciterContext;

#define OFFSET(x) offsetof(AExciterContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption aexciter_options[] = {
    { "level_in",  "set level in",    OFFSET(level_in),  AV_OPT_TYPE_DOUBLE, {.dbl=1},           0, 64, A },
    { "level_out", "set level out",   OFFSET(level_out), AV_OPT_TYPE_DOUBLE, {.dbl=1},           0, 64, A },
    { "amount", "set amount",         OFFSET(amount),    AV_OPT_TYPE_DOUBLE, {.dbl=1},           0, 64, A },
    { "drive", "set harmonics",       OFFSET(drive),     AV_OPT_TYPE_DOUBLE, {.dbl=8.5},       0.1, 10, A },
    { "blend", "set blend harmonics", OFFSET(blend),     AV_OPT_TYPE_DOUBLE, {.dbl=0},         -10, 10, A },
    { "freq", "set scope",            OFFSET(freq),      AV_OPT_TYPE_DOUBLE, {.dbl=7500},  2000, 12000, A },
    { "ceil", "set ceiling",          OFFSET(ceil),      AV_OPT_TYPE_DOUBLE, {.dbl=9999},  9999, 20000, A },
    { "listen", "enable listen mode", OFFSET(listen),    AV_OPT_TYPE_BOOL,   {.i64=0},        0,     1, A },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aexciter);

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#undef DEPTH
#define DEPTH 32
#include "aexciter_template.c"

#undef DEPTH
#define DEPTH 64
#include "aexciter_template.c"

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AExciterContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
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
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    if (in != out)
        ff_graph_frame_free(ctx, &in);

    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AExciterContext *s = ctx->priv;

    av_freep(&s->cp);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AExciterContext *s = ctx->priv;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->init_filter = init_filter_fltp;
        s->filter_channels = filter_channels_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->init_filter = init_filter_dblp;
        s->filter_channels = filter_channels_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->init_filter(ctx);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    AVFilterLink *inlink = ctx->inputs[0];
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return config_input(inlink);
}

static const AVFilterPad aexciter_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_af_aexciter = {
    .p.name        = "aexciter",
    .p.description = NULL_IF_CONFIG_SMALL("Enhance high frequency part of audio."),
    .p.priv_class  = &aexciter_class,
    .priv_size     = sizeof(AExciterContext),
    .uninit        = uninit,
    FILTER_INPUTS(aexciter_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .process_command = process_command,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS |
                     AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
};
