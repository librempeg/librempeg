/*
 * Copyright (c) 2011 Stefano Sabatini
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

/**
 * @file
 * eval audio source
 */

#include "config_components.h"

#include "libavutil/channel_layout.h"
#include "libavutil/eval.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"

static const char * const var_names[] = {
    "ch",           ///< the value of the current channel
    "n",            ///< number of frame
    "nb_in_channels",
    "nb_out_channels",
    "t",            ///< timestamp expressed in seconds
    "s",            ///< sample rate
    NULL
};

enum var_name {
    VAR_CH,
    VAR_N,
    VAR_NB_IN_CHANNELS,
    VAR_NB_OUT_CHANNELS,
    VAR_T,
    VAR_S,
    VAR_VARS_NB
};

typedef struct EvalContext {
    const AVClass *class;
    int sample_rate;
    AVChannelLayout chlayout;
    char *chlayout_str;
    int nb_channels;            ///< number of output channels
    int nb_in_channels;         ///< number of input channels
    int same_chlayout;          ///< set output as input channel layout
    int64_t pts;
    AVExpr **expr;
    char **exprs;
    unsigned nb_exprs;
    int nb_samples;             ///< number of samples per requested frame
    int64_t duration;
    uint64_t n;
    double var_values[VAR_VARS_NB];
    double *channel_values;
} EvalContext;

static double val(void *priv, double ch)
{
    EvalContext *eval = priv;
    return eval->channel_values[FFMIN((int)ch, eval->nb_in_channels-1)];
}

static double (* const aeval_func1[])(void *, double) = { val, NULL };
static const char * const aeval_func1_names[] = { "val", NULL };

#define OFFSET(x) offsetof(EvalContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_exprs = {.def="0",.size_min=1,.sep='|'};

static const AVOption aevalsrc_options[]= {
    { "exprs",       "set the list of channels expressions",          OFFSET(exprs),        AV_OPT_TYPE_STRING|AR, {.arr=&def_exprs}, .flags = FLAGS },
    { "nb_samples",  "set the number of samples per requested frame", OFFSET(nb_samples),      AV_OPT_TYPE_INT,    {.i64 = 1024},    1,        INT_MAX, FLAGS },
    { "n",           "set the number of samples per requested frame", OFFSET(nb_samples),      AV_OPT_TYPE_INT,    {.i64 = 1024},    1,        INT_MAX, FLAGS },
    { "sample_rate", "set the sample rate",                           OFFSET(sample_rate),     AV_OPT_TYPE_INT,    {.i64 = 44100},   1,        INT_MAX, FLAGS },
    { "s",           "set the sample rate",                           OFFSET(sample_rate),     AV_OPT_TYPE_INT,    {.i64 = 44100},   1,        INT_MAX, FLAGS },
    { "duration",    "set audio duration", OFFSET(duration), AV_OPT_TYPE_DURATION, {.i64 = -1}, -1, INT64_MAX, FLAGS },
    { "d",           "set audio duration", OFFSET(duration), AV_OPT_TYPE_DURATION, {.i64 = -1}, -1, INT64_MAX, FLAGS },
    { "channel_layout", "set channel layout", OFFSET(chlayout_str), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, FLAGS },
    { "c",              "set channel layout", OFFSET(chlayout_str), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aevalsrc);

static int parse_channel_expressions(AVFilterContext *ctx,
                                     int expected_nb_channels)
{
    EvalContext *eval = ctx->priv;
    double (* const *func1)(void *, double) = NULL;
    const char * const *func1_names = NULL;
    int i, ret = 0;

    if (!strcmp(ctx->filter->name, "aeval")) {
        func1 = aeval_func1;
        func1_names = aeval_func1_names;
    }

#define ADD_EXPRESSION(expr_) do {                                      \
        ret = av_dynarray_add_nofree(&eval->expr,                       \
                                     &eval->nb_channels, NULL);         \
        if (ret < 0)                                                    \
            goto end;                                                   \
        eval->expr[eval->nb_channels-1] = NULL;                         \
        ret = av_expr_parse(&eval->expr[eval->nb_channels - 1], expr_,  \
                            var_names, func1_names, func1,              \
                            NULL, NULL, 0, ctx);                        \
        if (ret < 0)                                                    \
            goto end;                                                   \
    } while (0)

    /* reset expressions */
    for (i = 0; i < eval->nb_channels; i++) {
        av_expr_free(eval->expr[i]);
        eval->expr[i] = NULL;
    }
    av_freep(&eval->expr);
    eval->nb_channels = 0;

    for (unsigned n = 0;
         n < ((expected_nb_channels > 0) ? FFMAX(expected_nb_channels, eval->nb_exprs) :
         eval->nb_exprs); n++) {
        const int idx = FFMIN(n, eval->nb_exprs-1);
        const char *expr = eval->exprs[idx];
        ADD_EXPRESSION(expr);
    }

    if (expected_nb_channels > 0 && eval->nb_channels != expected_nb_channels) {
        av_log(ctx, AV_LOG_ERROR,
               "Mismatch between the specified number of channel expressions '%d' "
               "and the number of expected output channels '%d' for the specified channel layout\n",
               eval->nb_channels, expected_nb_channels);
        ret = AVERROR(EINVAL);
        goto end;
    }

end:
    return ret;
}

static av_cold int init(AVFilterContext *ctx)
{
    EvalContext *eval = ctx->priv;
    int ret = 0;

    if (eval->chlayout_str) {
        if (!strcmp(eval->chlayout_str, "same") && !strcmp(ctx->filter->name, "aeval")) {
            eval->same_chlayout = 1;
        } else {
            int nb_output_channels = 0;

            ret = ff_parse_channel_layout(&eval->chlayout, &nb_output_channels, eval->chlayout_str, ctx);
            if (ret < 0)
                return ret;

            ret = parse_channel_expressions(ctx, nb_output_channels);
            if (ret < 0)
                return ret;
        }
    } else {
        /* guess channel layout from nb expressions/channels */
        if ((ret = parse_channel_expressions(ctx, -1)) < 0)
            return ret;

        av_channel_layout_default(&eval->chlayout, eval->nb_channels);
        if (eval->nb_channels <= 0) {
            av_log(ctx, AV_LOG_ERROR, "Invalid number of channels '%d' provided\n",
                   eval->nb_channels);
            return AVERROR(EINVAL);
        }
    }

    eval->n = 0;

    return ret;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    EvalContext *eval = ctx->priv;
    int i;

    for (i = 0; i < eval->nb_channels; i++) {
        av_expr_free(eval->expr[i]);
        eval->expr[i] = NULL;
    }
    av_freep(&eval->expr);
    av_freep(&eval->channel_values);
    av_channel_layout_uninit(&eval->chlayout);
}

static int config_props(AVFilterLink *outlink)
{
    EvalContext *eval = outlink->src->priv;
    char buf[128];

    outlink->time_base = (AVRational){1, eval->sample_rate};
    outlink->sample_rate = eval->sample_rate;

    eval->var_values[VAR_S] = eval->sample_rate;
    eval->var_values[VAR_NB_IN_CHANNELS] = NAN;
    eval->var_values[VAR_NB_OUT_CHANNELS] = outlink->ch_layout.nb_channels;

    av_channel_layout_describe(&eval->chlayout, buf, sizeof(buf));

    av_log(outlink->src, AV_LOG_VERBOSE,
           "sample_rate:%d chlayout:%s duration:%"PRId64"\n",
           eval->sample_rate, buf, eval->duration);

    return 0;
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const EvalContext *eval = ctx->priv;
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_NONE };
    AVChannelLayout chlayouts[] = { eval->chlayout.nb_channels ? eval->chlayout : FF_COUNT2LAYOUT(eval->nb_channels), { 0 } };
    int sample_rates[] = { eval->sample_rate, -1 };
    int ret;

    ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts);
    if (ret < 0)
        return ret;

    ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, chlayouts);
    if (ret < 0)
        return ret;

    return ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, sample_rates);
}

static int aevalsrc_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    EvalContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (s->nb_channels * jobnr) / nb_jobs;
    const int end = (s->nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = out->nb_samples;
    const double scale = 1.0 / s->sample_rate;
    double var_values[VAR_VARS_NB];
    const uint64_t n = s->n;

    memcpy(var_values, s->var_values, sizeof(var_values));

    for (int ch = start; ch < end; ch++) {
        double *dst = (double *)out->extended_data[ch];
        AVExpr *expr = s->expr[ch];

        for (int i = 0; i < nb_samples; i++) {
            var_values[VAR_N] = n + i;
            var_values[VAR_T] = var_values[VAR_N] * scale;

            dst[i] = av_expr_eval(expr, var_values, NULL);
        }
    }

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    EvalContext *eval = ctx->priv;
    AVFrame *samplesref;
    int64_t t = av_rescale(eval->n, AV_TIME_BASE, eval->sample_rate);
    int nb_samples;

    if (!ff_outlink_frame_wanted(outlink))
        return FFERROR_NOT_READY;

    if (eval->duration >= 0 && t >= eval->duration) {
        ff_outlink_set_status(outlink, AVERROR_EOF, eval->pts);
        return 0;
    }

    if (eval->duration >= 0) {
        nb_samples = FFMIN(eval->nb_samples, av_rescale(eval->duration, eval->sample_rate, AV_TIME_BASE) - eval->pts);
        if (!nb_samples) {
            ff_outlink_set_status(outlink, AVERROR_EOF, eval->pts);
            return 0;
        }
    } else {
        nb_samples = eval->nb_samples;
    }
    samplesref = ff_get_audio_buffer(outlink, nb_samples);
    if (!samplesref)
        return AVERROR(ENOMEM);

    ff_filter_execute(ctx, aevalsrc_channels, samplesref, NULL,
                      FFMIN(eval->nb_channels, ff_filter_get_nb_threads(ctx)));

    samplesref->pts = eval->pts;
    samplesref->sample_rate = eval->sample_rate;
    eval->n += nb_samples;
    eval->pts += nb_samples;

    return ff_filter_frame(outlink, samplesref);
}

#if CONFIG_AEVALSRC_FILTER
static const AVFilterPad aevalsrc_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_props,
    },
};

const FFFilter ff_asrc_aevalsrc = {
    .p.name        = "aevalsrc",
    .p.description = NULL_IF_CONFIG_SMALL("Generate an audio signal generated by an expression."),
    .p.priv_class  = &aevalsrc_class,
    .init          = init,
    .uninit        = uninit,
    .activate      = activate,
    .priv_size     = sizeof(EvalContext),
    FILTER_OUTPUTS(aevalsrc_outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS,
};

#endif /* CONFIG_AEVALSRC_FILTER */

#define OFFSET(x) offsetof(EvalContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOption aeval_options[]= {
    { "exprs", "set the list of channels expressions", OFFSET(exprs), AV_OPT_TYPE_STRING|AR, {.arr = &def_exprs}, .flags = FLAGS },
    { "channel_layout", "set channel layout", OFFSET(chlayout_str), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, FLAGS },
    { "c",              "set channel layout", OFFSET(chlayout_str), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aeval);

static int aeval_query_formats(const AVFilterContext *ctx,
                               AVFilterFormatsConfig **cfg_in,
                               AVFilterFormatsConfig **cfg_out)
{
    AVFilterChannelLayouts *layouts;
    const EvalContext *eval = ctx->priv;
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_NONE
    };
    int ret;

    // inlink supports any channel layout
    layouts = ff_all_channel_counts();
    if ((ret = ff_channel_layouts_ref(layouts, &cfg_in[0]->channel_layouts)) < 0)
        return ret;

    if (!eval->same_chlayout) {
        // outlink supports only requested output channel layout
        layouts = NULL;
        if ((ret = ff_add_channel_layout(&layouts, &FF_COUNT2LAYOUT(eval->nb_channels))) < 0)
            return ret;
        if ((ret = ff_channel_layouts_ref(layouts, &cfg_out[0]->channel_layouts)) < 0)
            return ret;
    }

    if ((ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts)) < 0)
        return ret;

    return 0;
}

static int aeval_config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    EvalContext *eval = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    int ret;

    if (eval->same_chlayout) {
        if ((ret = av_channel_layout_copy(&eval->chlayout, &inlink->ch_layout)) < 0)
            return ret;

        if ((ret = parse_channel_expressions(ctx, inlink->ch_layout.nb_channels)) < 0)
            return ret;
    }

    eval->n = 0;
    eval->nb_in_channels = eval->var_values[VAR_NB_IN_CHANNELS] = inlink->ch_layout.nb_channels;
    eval->var_values[VAR_NB_OUT_CHANNELS] = outlink->ch_layout.nb_channels;
    eval->var_values[VAR_S] = inlink->sample_rate;
    eval->var_values[VAR_T] = NAN;

    eval->channel_values = av_realloc_f(eval->channel_values,
                                        inlink->ch_layout.nb_channels, sizeof(*eval->channel_values));
    if (!eval->channel_values)
        return AVERROR(ENOMEM);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    EvalContext *eval     = inlink->dst->priv;
    AVFilterLink *outlink = inlink->dst->outputs[0];
    int nb_samples        = in->nb_samples;
    AVFrame *out;
    double t0;
    int i, j;

    out = ff_get_audio_buffer(outlink, nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    t0 = TS2T(in->pts, inlink->time_base);

    /* evaluate expression for each single sample and for each channel */
    for (i = 0; i < nb_samples; i++, eval->n++) {
        eval->var_values[VAR_N] = eval->n;
        eval->var_values[VAR_T] = t0 + i * (double)1/inlink->sample_rate;

        for (j = 0; j < inlink->ch_layout.nb_channels; j++)
            eval->channel_values[j] = *((double *) in->extended_data[j] + i);

        for (j = 0; j < outlink->ch_layout.nb_channels; j++) {
            eval->var_values[VAR_CH] = j;
            *((double *) out->extended_data[j] + i) =
                av_expr_eval(eval->expr[j], eval->var_values, eval);
        }
    }

    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

#if CONFIG_AEVAL_FILTER

static const AVFilterPad aeval_inputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_AUDIO,
        .filter_frame   = filter_frame,
    },
};

static const AVFilterPad aeval_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = aeval_config_output,
    },
};

const FFFilter ff_af_aeval = {
    .p.name        = "aeval",
    .p.description = NULL_IF_CONFIG_SMALL("Filter audio signal according to a specified expression."),
    .p.priv_class  = &aeval_class,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
    .init          = init,
    .uninit        = uninit,
    .priv_size     = sizeof(EvalContext),
    FILTER_INPUTS(aeval_inputs),
    FILTER_OUTPUTS(aeval_outputs),
    FILTER_QUERY_FUNC2(aeval_query_formats),
};

#endif /* CONFIG_AEVAL_FILTER */
