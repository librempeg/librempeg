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
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct THDNContext {
    const AVClass *class;

    int channels;
    int sample_rate;
    int rdft_size;

    void *state;
    void *window;

    int (*init)(AVFilterContext *ctx);
    int (*thdn_channel)(AVFilterContext *ctx, AVFrame *in, const int ch);
    void (*uninit)(AVFilterContext *ctx);
} THDNContext;

#define OFFSET(x) offsetof(THDNContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption thdn_options[] = {
    { "rdft_size", "set the RDFT size", OFFSET(rdft_size), AV_OPT_TYPE_INT, {.i64=32768}, 256, 1<<24, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(thdn);

#undef DEPTH
#define DEPTH 64
#include "thdn_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    THDNContext *s = ctx->priv;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->thdn_channel = thdn_channel_dblp;
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

static int thdn_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    THDNContext *s = ctx->priv;
    AVFrame *in = arg;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->thdn_channel(ctx, in, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];

    ff_filter_execute(ctx, thdn_channels, in, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    return ff_filter_frame(outlink, in);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    THDNContext *s = ctx->priv;

    if (s->uninit)
        s->uninit(ctx);
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

const FFFilter ff_af_thdn = {
    .p.name          = "thdn",
    .p.description   = NULL_IF_CONFIG_SMALL("Measure THD+N stats."),
    .p.priv_class    = &thdn_class,
    .priv_size       = sizeof(THDNContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_DBLP),
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_SLICE_THREADS,
};
