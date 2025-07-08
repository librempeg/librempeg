/*
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
 * Crossover filter
 *
 * Split an audio stream into several bands.
 */

#include "libavutil/attributes.h"
#include "libavutil/channel_layout.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

#define MAX_SPLITS 16
#define MAX_BANDS MAX_SPLITS + 1

typedef struct AudioCrossoverContext {
    const AVClass *class;

    double *splits_opt;
    unsigned nb_splits_opt;
    double *gains_opt;
    unsigned nb_gains_opt;
    double *resonance_opt;
    unsigned nb_resonance_opt;
    int *active_opt;
    unsigned nb_active_opt;
    int order_opt;
    double level_in;
    int precision;

    int nb_splits;
    int nb_channels;
    double splits[MAX_SPLITS];
    double resonance[MAX_BANDS];
    double gains[MAX_BANDS];
    int active[MAX_BANDS];

    void *svf_cf;
    void *svf;

    AVFrame *frames[MAX_BANDS];

    void (*filter_channel)(AVFilterContext *ctx, void *st, void *stc, const int ch,
                           const int nb_outputs, const int nb_samples,
                           const double level_in, const uint8_t *srcp, uint8_t **dstp,
                           const double *gains, const int *active);
    int (*init_filter)(AVFilterContext *ctx, void **st, void **stc,
                       const double *splits, const double *resonance,
                       const int nb_splits, const int nb_channels,
                       const int sample_rate);
} AudioCrossoverContext;

#define OFFSET(x) offsetof(AudioCrossoverContext, x)
#define AF AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM
#define AFR AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_splits = {.def="500",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_gains = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_active = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_res = {.def="0.25",.size_min=1,.sep=' '};

static const AVOption acrossover_options[] = {
    { "split", "set the split frequencies", OFFSET(splits_opt), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_splits}, .min=0, .max=INT_MAX, .flags=AF },
    { "resonance", "set the bands resonance", OFFSET(resonance_opt),  AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_res}, .min=0, .max=1.0, .flags=AFR },
    { "level", "set the input gain",        OFFSET(level_in),   AV_OPT_TYPE_DOUBLE,  {.dbl=1}, 0, 1, AFR },
    { "gain",  "set the output bands gain", OFFSET(gains_opt),  AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_gains}, .min=-9.0, .max=9.0, .flags=AFR },
    { "active", "set the active bands", OFFSET(active_opt), AV_OPT_TYPE_BOOL|AR, {.arr=&def_active}, .min=0, .max=1, .flags=AFR },
    { "precision",  "set the processing precision", OFFSET(precision), AV_OPT_TYPE_INT, {.i64=0}, 0, 2, AF, .unit = "precision" },
    {  "auto",  "set auto processing precision",                  0, AV_OPT_TYPE_CONST, {.i64=0}, 0, 0, AF, .unit = "precision" },
    {  "float", "set single-floating point processing precision", 0, AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, AF, .unit = "precision" },
    {  "double","set double-floating point processing precision", 0, AV_OPT_TYPE_CONST, {.i64=2}, 0, 0, AF, .unit = "precision" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(acrossover);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AudioCrossoverContext *s = ctx->priv;
    static const enum AVSampleFormat auto_sample_fmts[] = {
        AV_SAMPLE_FMT_FLTP,
        AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE
    };
    enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_FLTP,
        AV_SAMPLE_FMT_NONE
    };
    const enum AVSampleFormat *sample_fmts_list = sample_fmts;

    switch (s->precision) {
    case 0:
        sample_fmts_list = auto_sample_fmts;
        break;
    case 1:
        sample_fmts[0] = AV_SAMPLE_FMT_FLTP;
        break;
    case 2:
        sample_fmts[0] = AV_SAMPLE_FMT_DBLP;
        break;
    default:
        break;
    }

    return ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts_list);
}

static int parse_splits(AVFilterContext *ctx)
{
    AudioCrossoverContext *s = ctx->priv;
    int i;

    for (i = 0; i < s->nb_splits_opt; i++) {
        double freq = s->splits_opt[i];

        if (i >= MAX_SPLITS)
            break;

        if (freq <= 0) {
            av_log(ctx, AV_LOG_ERROR, "Frequency %f must be positive number.\n", freq);
            return AVERROR(EINVAL);
        }

        if (i > 0 && freq <= s->splits[i-1]) {
            av_log(ctx, AV_LOG_ERROR, "Frequency %f must be in increasing order.\n", freq);
            return AVERROR(EINVAL);
        }

        s->splits[i] = freq;
    }

    return i;
}

static void parse_active(AVFilterContext *ctx)
{
    AudioCrossoverContext *s = ctx->priv;
    int i;

    for (i = 0; i < s->nb_active_opt; i++) {
        if (i >= MAX_BANDS)
            break;
        s->active[i] = s->active_opt[i];
    }

    for (; i < MAX_BANDS; i++)
        s->active[i] = s->active[i-1];
}

static void parse_gains(AVFilterContext *ctx)
{
    AudioCrossoverContext *s = ctx->priv;
    int i;

    for (i = 0; i < s->nb_gains_opt; i++) {
        if (i >= MAX_BANDS)
            break;
        s->gains[i] = s->gains_opt[i];
    }

    for (; i < MAX_BANDS && i > 0; i++)
        s->gains[i] = s->gains[i-1];
}

static void parse_resonance(AVFilterContext *ctx)
{
    AudioCrossoverContext *s = ctx->priv;
    int i;

    for (i = 0; i < s->nb_resonance_opt; i++) {
        if (i >= MAX_BANDS)
            break;
        s->resonance[i] = s->resonance_opt[i];
    }

    for (; i < MAX_BANDS; i++)
        s->resonance[i] = s->resonance[i-1];
}

static int next_active_band(const int *active,
                            const int current_band, const int nb_bands)
{
    for (int i = current_band+1; i < nb_bands; i++) {
        if (active[i])
            return i;
    }

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    AudioCrossoverContext *s = ctx->priv;
    int ret;

    ret = parse_splits(ctx);
    if (ret < 0)
        return ret;
    s->nb_splits = ret;

    parse_active(ctx);
    parse_gains(ctx);
    parse_resonance(ctx);

    for (int i = 0; i <= s->nb_splits; i++) {
        AVFilterPad pad  = { 0 };
        char *name;

        pad.type = AVMEDIA_TYPE_AUDIO;
        name = av_asprintf("out%d", ctx->nb_outputs);
        if (!name)
            return AVERROR(ENOMEM);
        pad.name = name;

        if ((ret = ff_append_outpad_free_name(ctx, &pad)) < 0)
            return ret;
    }

    return ret;
}

#define DEPTH 32
#include "acrossover_template.c"

#undef DEPTH
#define DEPTH 64
#include "acrossover_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioCrossoverContext *s = ctx->priv;

    s->nb_channels = inlink->ch_layout.nb_channels;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->init_filter = init_xover_fltp;
        s->filter_channel = xover_channel_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->init_filter = init_xover_dblp;
        s->filter_channel = xover_channel_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->init_filter(ctx, &s->svf, &s->svf_cf, s->splits, s->resonance,
                          s->nb_splits, s->nb_channels, inlink->sample_rate);
}

static int filter_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioCrossoverContext *s = ctx->priv;
    const double level_in = s->level_in;
    AVFrame *in = arg;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_outs = ctx->nb_outputs;
    uint8_t *dstp[MAX_BANDS];

    for (int ch = start; ch < end; ch++) {
        const uint8_t *src = in->extended_data[ch];

        for (int i = 0; i < nb_outs; i++)
            dstp[i] = s->frames[i]->extended_data[ch];

        s->filter_channel(ctx, s->svf, s->svf_cf, ch, nb_outs, in->nb_samples,
                          level_in, src, dstp, s->gains, s->active);
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AudioCrossoverContext *s = ctx->priv;
    AVFrame **frames = s->frames;
    int ret = 1;

    for (int i = 0; i < ctx->nb_outputs; i++) {
        frames[i] = ff_get_audio_buffer(ctx->outputs[i], in->nb_samples);
        if (!frames[i]) {
            ret = AVERROR(ENOMEM);
            break;
        }

        frames[i]->pts = in->pts;
    }

    if (ret < 0)
        goto fail;

    ff_filter_execute(ctx, filter_channels, in, NULL,
                      FFMIN(inlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    for (int i = 0; i < ctx->nb_outputs; i++) {
        if (ff_outlink_get_status(ctx->outputs[i])) {
            av_frame_free(&frames[i]);
            continue;
        }

        ret = ff_filter_frame(ctx->outputs[i], frames[i]);
        frames[i] = NULL;
        if (ret < 0)
            break;
    }

fail:
    for (int i = 0; i < ctx->nb_outputs; i++)
        av_frame_free(&frames[i]);

    return ret;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    int status, ret;
    AVFrame *in;
    int64_t pts;

    for (int i = 0; i < ctx->nb_outputs; i++) {
        FF_FILTER_FORWARD_STATUS_BACK_ALL(ctx->outputs[i], ctx);
    }

    ret = ff_inlink_consume_frame(inlink, &in);
    if (ret < 0)
        return ret;
    if (ret > 0) {
        ret = filter_frame(inlink, in);
        av_frame_free(&in);
        if (ret <= 0)
            return ret;
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        for (int i = 0; i < ctx->nb_outputs; i++) {
            if (ff_outlink_get_status(ctx->outputs[i]))
                continue;
            ff_outlink_set_status(ctx->outputs[i], status, pts);
        }
        return 0;
    }

    for (int i = 0; i < ctx->nb_outputs; i++) {
        if (ff_outlink_get_status(ctx->outputs[i]))
            continue;

        if (ff_outlink_frame_wanted(ctx->outputs[i])) {
            ff_inlink_request_frame(inlink);
            return 0;
        }
    }

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioCrossoverContext *s = ctx->priv;

    av_freep(&s->svf_cf);
    av_freep(&s->svf);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    AudioCrossoverContext *s = ctx->priv;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    parse_active(ctx);
    parse_gains(ctx);
    parse_resonance(ctx);

    return s->init_filter(ctx, &s->svf, &s->svf_cf, s->splits, s->resonance,
                          s->nb_splits, s->nb_channels, ctx->inputs[0]->sample_rate);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_acrossover = {
    .p.name         = "acrossover",
    .p.description  = NULL_IF_CONFIG_SMALL("Split audio into per-bands streams."),
    .p.priv_class   = &acrossover_class,
    .p.outputs      = NULL,
    .p.flags        = AVFILTER_FLAG_DYNAMIC_OUTPUTS |
                      AVFILTER_FLAG_SLICE_THREADS,
    .priv_size      = sizeof(AudioCrossoverContext),
    .init           = init,
    .activate       = activate,
    .uninit         = uninit,
    FILTER_INPUTS(inputs),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = process_command,
};
