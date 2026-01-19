/*
 * Copyright (c) 2018 Paul B Mahol
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

#define BINS 32768

typedef struct ChannelStats {
    uint64_t nb_samples;
    uint64_t blknum;
    float blockpeak;
    float secondpeak;
    float peak;
    float sum;
    uint32_t rms[BINS+1];
} ChannelStats;

typedef struct DRMeterContext {
    const AVClass *class;
    ChannelStats *chstats;
    int nb_channels;
    int64_t tc_samples;
    double time_constant;
} DRMeterContext;

#define OFFSET(x) offsetof(DRMeterContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption drmeter_options[] = {
    { "length", "set the window length", OFFSET(time_constant), AV_OPT_TYPE_DOUBLE, {.dbl=3}, .01, 10, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(drmeter);

static int config_output(AVFilterLink *outlink)
{
    DRMeterContext *s = outlink->src->priv;

    s->chstats = av_calloc(outlink->ch_layout.nb_channels, sizeof(*s->chstats));
    if (!s->chstats)
        return AVERROR(ENOMEM);
    s->nb_channels = outlink->ch_layout.nb_channels;
    s->tc_samples = lrint(s->time_constant * outlink->sample_rate);

    return 0;
}

static void finish_block(ChannelStats *p)
{
    float rms;
    int rms_bin;

    if (p->blockpeak > p->peak) {
        p->secondpeak = p->peak;
        p->peak = p->blockpeak;
    }
    rms = sqrtf(2.f * p->sum / p->nb_samples);
    rms_bin = av_clip(lrintf(rms * BINS), 0, BINS);
    p->rms[rms_bin]++;

    p->sum = 0;
    p->blockpeak = 0;
    p->nb_samples = 0;
    p->blknum++;
}

static void update_stat(DRMeterContext *s, ChannelStats *p, float sample)
{
    p->blockpeak = fmaxf(fabsf(sample), p->blockpeak);
    p->sum += sample * sample;
    p->nb_samples++;
    if (p->nb_samples >= s->tc_samples)
        finish_block(p);
}

static int filter_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AVFilterLink *inlink = ctx->inputs[0];
    DRMeterContext *s = ctx->priv;
    AVFrame *in = arg;
    const int nb_channels = s->nb_channels;
    const int nb_samples = in->nb_samples;
    const int start = (nb_channels * jobnr) / nb_jobs;
    const int end = (nb_channels * (jobnr+1)) / nb_jobs;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        for (int ch = start; ch < end; ch++) {
            const float *src = (const float *)in->extended_data[ch];
            ChannelStats *p = &s->chstats[ch];

            for (int n = 0; n < nb_samples; n++)
                update_stat(s, p, src[n]);
        }
        break;
    case AV_SAMPLE_FMT_FLT:
        for (int ch = start; ch < end; ch++) {
            const float *src = ((const float *)in->extended_data[0]) + ch;
            ChannelStats *p = &s->chstats[ch];

            for (int n = 0; n < nb_samples; n++, src += nb_channels)
                update_stat(s, p, *src);
        }
        break;
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    DRMeterContext *s = ctx->priv;
    const int nb_channels = s->nb_channels;

    ff_filter_execute(ctx, filter_channels, in, NULL,
                      FFMIN(nb_channels, ff_filter_get_nb_threads(ctx)));

    return ff_filter_frame(ctx->outputs[0], in);
}

#define SQR(a) ((a)*(a))

static void print_stats(AVFilterContext *ctx)
{
    DRMeterContext *s = ctx->priv;
    float dr = 0.f;

    for (int ch = 0; ch < s->nb_channels; ch++) {
        ChannelStats *p = &s->chstats[ch];
        float chdr, rmssum = 0.f;
        int last = lrintf(0.2f * p->blknum);

        if (!p->nb_samples) {
            av_log(ctx, AV_LOG_INFO, "No data, dynamic range not measurable\n");
            return;
        }

        if (p->nb_samples)
            finish_block(p);

        for (int64_t i = BINS, j = 0; i >= 0 && j < last; i--) {
            if (p->rms[i]) {
                rmssum += SQR(i / (float)BINS) * p->rms[i];
                j += p->rms[i];
            }
        }

        chdr = 20.f * log10f(p->secondpeak / sqrtf(rmssum / (float)last));
        dr += chdr;
        av_log(ctx, AV_LOG_INFO, "Channel %d: DR: %g\n", ch + 1, chdr);
    }

    av_log(ctx, AV_LOG_INFO, "Overall DR: %g\n", dr / s->nb_channels);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    DRMeterContext *s = ctx->priv;

    if (s->nb_channels)
        print_stats(ctx);
    av_freep(&s->chstats);
}

static const AVFilterPad drmeter_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad drmeter_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_drmeter = {
    .p.name        = "drmeter",
    .p.description = NULL_IF_CONFIG_SMALL("Measure audio dynamic range."),
    .p.priv_class  = &drmeter_class,
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY |
                     AVFILTER_FLAG_SLICE_THREADS,
    .priv_size     = sizeof(DRMeterContext),
    .uninit        = uninit,
    FILTER_INPUTS(drmeter_inputs),
    FILTER_OUTPUTS(drmeter_outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_FLT),
};
