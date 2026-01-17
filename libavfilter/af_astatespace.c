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

#include <float.h>

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

enum FilterType {
    lowpass,
    highpass,
    bandpass,
    allpass,
    bandreject,
    lowshelf,
    highshelf,
    equalizer,
};

typedef struct StateSpaceContext {
    const AVClass *class;

    int filter_type;
    int order;
    int warp;

    double fs;
    double fc;
    double zeta;
    double gain;
    double S;
    double sigma;

    int bypass;

    void *st;

    int nb_channels;

    int (*init_state)(AVFilterContext *ctx, void **st,
                      const int nb_channels);
    void (*set_fc)(AVFilterContext *ctx, const double new_fc);
    void (*set_gain)(AVFilterContext *ctx, const double new_gain);
    void (*set_sigma)(AVFilterContext *ctx, const double S);
    void (*set_damping)(AVFilterContext *ctx, const double zeta);
    void (*filter)(void *st, const void *ibuf, void *obuf, const int len,
                   const int ch, const int disabled);
} StateSpaceContext;

#undef DEPTH
#define DEPTH 32
#include "astatespace_template.c"

#undef DEPTH
#define DEPTH 64
#include "astatespace_template.c"

static const enum AVSampleFormat sample_fmts[] = {
    AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP,
    AV_SAMPLE_FMT_NONE
};

static int config_filter(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    StateSpaceContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];

    s->fs = inlink->sample_rate;

    return s->init_state(ctx, &s->st, outlink->ch_layout.nb_channels);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    StateSpaceContext *s = ctx->priv;

    s->nb_channels = outlink->ch_layout.nb_channels;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->init_state = init_statespace_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->init_state = init_statespace_dblp;
        break;
    default:
        av_assert0(0);
    }

    return config_filter(outlink);
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

static int filter_channel(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ThreadData *td = arg;
    AVFrame *in = td->in;
    AVFrame *out = td->out;
    StateSpaceContext *s = ctx->priv;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int disabled = ff_filter_disabled(ctx);

    for (int ch = start; ch < end; ch++) {
        s->filter(s->st, in->extended_data[ch], out->extended_data[ch], in->nb_samples,
                  ch, disabled);
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    StateSpaceContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out;
    ThreadData td;

    if (s->bypass)
        return ff_filter_frame(outlink, in);

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

    td.in = in;
    td.out = out;
    ff_filter_execute(ctx, filter_channel, &td, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    if (in != out)
        ff_graph_frame_free(ctx, &in);

    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *in = NULL;
    int64_t pts;
    int status;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_frame(inlink, &in);
    if (ret < 0)
        return ret;
    if (ret > 0)
        return filter_frame(inlink, in);

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        ff_outlink_set_status(outlink, status, pts);
        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    AVFilterLink *outlink = ctx->outputs[0];
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return config_filter(outlink);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    StateSpaceContext *s = ctx->priv;

    av_freep(&s->st);
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

#define OFFSET(x) offsetof(StateSpaceContext, x)
#define AFR AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption astatespace_options[] = {
    { "t", "set the filter type", OFFSET(filter_type), AV_OPT_TYPE_INT, {.i64=0}, 0, 7, AF, .unit = "type" },
    { "l", "lowpass", 0, AV_OPT_TYPE_CONST, {.i64 = lowpass }, 0, 0, AF, .unit = "type" },
    { "h", "highpass", 0, AV_OPT_TYPE_CONST, {.i64 = highpass }, 0, 0, AF, .unit = "type" },
    { "b", "bandpass", 0, AV_OPT_TYPE_CONST, {.i64 = bandpass }, 0, 0, AF, .unit = "type" },
    { "a", "allpass", 0, AV_OPT_TYPE_CONST, {.i64 = allpass }, 0, 0, AF, .unit = "type" },
    { "n", "notch", 0, AV_OPT_TYPE_CONST, {.i64 = bandreject }, 0, 0, AF, .unit = "type" },
    { "L", "lowshelf", 0, AV_OPT_TYPE_CONST, {.i64 = lowshelf }, 0, 0, AF, .unit = "type" },
    { "H", "highshelf", 0, AV_OPT_TYPE_CONST, {.i64 = highshelf }, 0, 0, AF, .unit = "type" },
    { "e", "equalizer", 0, AV_OPT_TYPE_CONST, {.i64 = equalizer }, 0, 0, AF, .unit = "type" },
    { "o", "set the filter order", OFFSET(order), AV_OPT_TYPE_INT, {.i64=2}, 1, 2, AF },
    { "f", "set the filter central frequency", OFFSET(fc), AV_OPT_TYPE_DOUBLE, {.dbl=1000}, 1, INT_MAX, AFR },
    { "g", "set the filter gain", OFFSET(gain), AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0, INT16_MAX, AFR },
    { "s", "set the filter sigma", OFFSET(S), AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0, 2, AFR },
    { "z", "set the filter damping", OFFSET(zeta), AV_OPT_TYPE_DOUBLE, {.dbl=0.71}, 0, INT_MAX, AFR },
    {NULL}
};

AVFILTER_DEFINE_CLASS(astatespace);

const FFFilter ff_af_astatespace = {
    .p.name          = "astatespace",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply 1st or 2nd order state-space audio filters."),
    .p.priv_class    = &astatespace_class,
    .priv_size       = sizeof(StateSpaceContext),
    .uninit          = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS_ARRAY(sample_fmts),
    .activate        = activate,
    .process_command = process_command,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                       AVFILTER_FLAG_SLICE_THREADS,
};
