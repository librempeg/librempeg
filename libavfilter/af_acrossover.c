/*
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
 * Crossover filter
 *
 * Split an audio stream into several bands.
 */

#include "libavutil/attributes.h"
#include "libavutil/channel_layout.h"
#include "libavutil/internal.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "internal.h"

#define MAX_SPLITS 16
#define MAX_BANDS MAX_SPLITS + 1

typedef struct SVFCache {
    double scd[MAX_BANDS][2][2];
    float scf[MAX_BANDS][2][2];
} SVFCache;

typedef struct SVFCoeffs {
    double gd;
    double rd;
    double kd;
    double gkd;
    double g2d;
    double g2kd;
    float gf;
    float rf;
    float kf;
    float gkf;
    float g2f;
    float g2kf;
} SVFCoeffs;

typedef struct AudioCrossoverContext {
    const AVClass *class;

    float *splits_opt;
    unsigned nb_splits_opt;
    float *gains_opt;
    unsigned nb_gains_opt;
    float *resonance_opt;
    unsigned nb_resonance_opt;
    int *active_opt;
    unsigned nb_active_opt;
    int order_opt;
    float level_in;
    int precision;

    int nb_splits;
    float splits[MAX_SPLITS];
    float resonance[MAX_BANDS];
    float gains[MAX_BANDS];
    int active[MAX_BANDS];

    SVFCoeffs svf_cf[MAX_BANDS];

    SVFCache *svf;

    AVFrame *frames[MAX_BANDS];

    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
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
    { "split", "set the split frequencies", OFFSET(splits_opt), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_splits}, .min=0, .max=INT_MAX, .flags=AF },
    { "resonance", "set the bands resonance", OFFSET(resonance_opt),  AV_OPT_TYPE_FLOAT|AR, {.arr=&def_res}, .min=0, .max=1.0, .flags=AFR },
    { "level", "set the input gain",        OFFSET(level_in),   AV_OPT_TYPE_FLOAT,  {.dbl=1}, 0, 1, AFR },
    { "gain",  "set the output bands gain", OFFSET(gains_opt),  AV_OPT_TYPE_FLOAT|AR, {.arr=&def_gains}, .min=-9.0, .max=9.0, .flags=AFR },
    { "active", "set the active bands", OFFSET(active_opt), AV_OPT_TYPE_BOOL|AR, {.arr=&def_active}, .min=0, .max=1, .flags=AFR },
    { "precision",  "set the processing precision", OFFSET(precision),   AV_OPT_TYPE_INT,   {.i64=0}, 0, 2, AF, .unit = "precision" },
    {  "auto",  "set auto processing precision",                  0, AV_OPT_TYPE_CONST, {.i64=0}, 0, 0, AF, .unit = "precision" },
    {  "float", "set single-floating point processing precision", 0, AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, AF, .unit = "precision" },
    {  "double","set double-floating point processing precision", 0, AV_OPT_TYPE_CONST, {.i64=2}, 0, 0, AF, .unit = "precision" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(acrossover);

static int query_formats(AVFilterContext *ctx)
{
    AudioCrossoverContext *s = ctx->priv;
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
    int ret = ff_set_common_all_channel_counts(ctx);
    if (ret < 0)
        return ret;

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
    ret = ff_set_common_formats_from_list(ctx, sample_fmts_list);
    if (ret < 0)
        return ret;

    return ff_set_common_all_samplerates(ctx);
}

static int parse_splits(AVFilterContext *ctx)
{
    AudioCrossoverContext *s = ctx->priv;
    int i;

    for (i = 0; i < s->nb_splits_opt; i++) {
        float freq = s->splits_opt[i];

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

    for (; i < MAX_BANDS; i++)
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

static void set_params(AVFilterContext *ctx)
{
    AudioCrossoverContext *s = ctx->priv;
    const double sample_rate = ctx->inputs[0]->sample_rate;

    for (int band = 0; band <= s->nb_splits; band++) {
        SVFCoeffs *sf = &s->svf_cf[band];

        sf->gd = tan(M_PI*s->splits[band]/sample_rate);
        sf->kd = 2.0 - 2.0 * s->resonance[band];
        sf->g2d = sf->gd*sf->gd;
        sf->g2kd = sf->g2d*sf->kd;
        sf->gkd = sf->gd*sf->kd;
        sf->rd = 1.0 - s->resonance[band];

        sf->gf = sf->gd;
        sf->kf = sf->kd;
        sf->g2f = sf->g2d;
        sf->g2kf = sf->g2kd;
        sf->gkf = sf->gkd;
        sf->rf = sf->rd;
    }
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioCrossoverContext *s = ctx->priv;

    s->svf = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->svf));
    if (!s->svf)
        return AVERROR(ENOMEM);

    set_params(ctx);

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP: s->filter_channels = filter_channels_fltp; break;
    case AV_SAMPLE_FMT_DBLP: s->filter_channels = filter_channels_dblp; break;
    default: return AVERROR_BUG;
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AudioCrossoverContext *s = ctx->priv;
    AVFrame **frames = s->frames;
    int ret = 0;

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

    ff_filter_execute(ctx, s->filter_channels, in, NULL,
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
        if (ret < 0)
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

    av_freep(&s->svf);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *args,
                           char *res, int res_len, int flags)
{
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret < 0)
        return ret;

    parse_active(ctx);
    parse_gains(ctx);
    parse_resonance(ctx);
    set_params(ctx);

    return 0;
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const AVFilter ff_af_acrossover = {
    .name           = "acrossover",
    .description    = NULL_IF_CONFIG_SMALL("Split audio into per-bands streams."),
    .priv_size      = sizeof(AudioCrossoverContext),
    .priv_class     = &acrossover_class,
    .init           = init,
    .activate       = activate,
    .uninit         = uninit,
    FILTER_INPUTS(inputs),
    .outputs        = NULL,
    FILTER_QUERY_FUNC(query_formats),
    .process_command = process_command,
    .flags          = AVFILTER_FLAG_DYNAMIC_OUTPUTS |
                      AVFILTER_FLAG_SLICE_THREADS,
};
