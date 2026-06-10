/*
 * Copyright (c) 2026 Paul B Mahol
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

#include <float.h>

#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct ToneDetectContext {
    const AVClass *class;

    double frequency;
    int channels;
    int sample_rate;

    void *state;

    AVFrame *in;

    int (*init)(AVFilterContext *ctx);
    int (*tone_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, int ch);
    void (*uninit)(AVFilterContext *ctx);
} ToneDetectContext;

#define OFFSET(x) offsetof(ToneDetectContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption tonedetect_options[] = {
    { "frequency", "set the tone frequency", OFFSET(frequency), AV_OPT_TYPE_DOUBLE, {.dbl=440}, 0, INT_MAX, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(tonedetect);

#define DEPTH 32
#include "tonedetect_template.c"

#undef DEPTH
#define DEPTH 64
#include "tonedetect_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    ToneDetectContext *s = ctx->priv;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->tone_channel = tone_channel_fltp;
        s->init = init_fltp;
        s->uninit = uninit_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->tone_channel = tone_channel_dblp;
        s->init = init_dblp;
        s->uninit = uninit_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    s->sample_rate = outlink->sample_rate;
    s->channels = outlink->ch_layout.nb_channels;

    return s->init(ctx);
}

static int tone_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ToneDetectContext *s = ctx->priv;
    AVFrame *in = s->in;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->tone_channel(ctx, in, out, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ToneDetectContext *s = ctx->priv;
    AVFrame *out;

    if (av_frame_is_writable(in)) {
        out = in;
    } else {
        out = ff_get_audio_buffer(outlink, in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    s->in = in;
    ff_filter_execute(ctx, tone_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    s->in = NULL;
    if (in != out)
        ff_graph_frame_free(ctx, &in);

    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ToneDetectContext *s = ctx->priv;

    if (s->uninit)
        s->uninit(ctx);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    ToneDetectContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return s->init(ctx);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_tonedetect = {
    .p.name          = "tonedetect",
    .p.description   = NULL_IF_CONFIG_SMALL("Detect audio tone."),
    .p.priv_class    = &tonedetect_class,
    .priv_size       = sizeof(ToneDetectContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_SLICE_THREADS,
    .process_command = process_command,
};
