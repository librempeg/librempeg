/*
 * Copyright (c) 2025 Paul B Mahol
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

#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "libavutil/mem.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct AudioFIRPhaseContext {
    const AVClass *class;

    double phase;
    int nb_taps;

    AVFrame *in;

    int (*rephase)(AVFilterContext *ctx, AVFrame *out, const int ch);
} AudioFIRPhaseContext;

#define OFFSET(x) offsetof(AudioFIRPhaseContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption afirphase_options[] = {
    { "phase", "set the filter phase", OFFSET(phase), AV_OPT_TYPE_DOUBLE, {.dbl=0}, -1, 1, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(afirphase);

#define DEPTH 32
#include "afirphase_template.c"

#undef DEPTH
#define DEPTH 64
#include "afirphase_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioFIRPhaseContext *s = ctx->priv;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->rephase = rephase_float;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->rephase = rephase_double;
        break;
    default:
        return AVERROR_BUG;
    }

    return 0;
}

static int phase_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioFIRPhaseContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->rephase(ctx, out, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioFIRPhaseContext *s = ctx->priv;
    AVFrame *out;
    int ret;

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        ret = AVERROR(ENOMEM);
        av_frame_free(&in);
        return ret;
    }

    s->in = in;
    ff_filter_execute(ctx, phase_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    av_frame_copy_props(out, in);

    s->in = NULL;
    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    int ret, status, available;
    AVFrame *in;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    available = ff_inlink_queued_samples(inlink);
    if (available > 0) {
        if ((4ULL << av_ceil_log2(available)) > INT_MAX)
            return AVERROR_INVALIDDATA;
    }

    if (ff_inlink_check_available_samples(inlink, available + 1) == 1) {
        ret = ff_inlink_consume_samples(inlink, available, available, &in);
        if (ret < 0)
            return ret;

        if (ret > 0)
            return filter_frame(inlink, in);
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        ff_outlink_set_status(outlink, status, pts);
        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_afirphase = {
    .p.name          = "afirphase",
    .p.description   = NULL_IF_CONFIG_SMALL("Adjust FIR filter phase."),
    .p.priv_class    = &afirphase_class,
    .p.flags         = AVFILTER_FLAG_SLICE_THREADS,
    .priv_size       = sizeof(AudioFIRPhaseContext),
    .activate        = activate,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
};
