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

#include "libavutil/channel_layout.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"

typedef struct ASubBoostContext {
    const AVClass *class;

    double dry_gain;
    double wet_gain;
    double feedback;
    double max_boost;
    double decay;
    double delay;
    double cutoff;
    double slope;
    double attack;
    double release;

    double a0, a1, a2;
    double b0, b1, b2;

    AVChannelLayout ch_layout;

    int *write_pos;
    int buffer_samples;

    AVFrame *w;
    AVFrame *buffer;

    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} ASubBoostContext;

static int get_coeffs(AVFilterContext *ctx)
{
    ASubBoostContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    double w0 = 2 * M_PI * s->cutoff / inlink->sample_rate;
    double alpha = sin(w0) / 2 * sqrt(2. * (1. / s->slope - 1.) + 2.);

    s->a0 =  1 + alpha;
    s->a1 = -2 * cos(w0);
    s->a2 =  1 - alpha;
    s->b0 = (1 - cos(w0)) / 2;
    s->b1 =  1 - cos(w0);
    s->b2 = (1 - cos(w0)) / 2;

    s->a1 /= s->a0;
    s->a2 /= s->a0;
    s->b0 /= s->a0;
    s->b1 /= s->a0;
    s->b2 /= s->a0;

    s->buffer_samples = inlink->sample_rate * s->delay / 1000;

    return 0;
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

#define DEPTH 32
#include "asubboost_template.c"

#undef DEPTH
#define DEPTH 64
#include "asubboost_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    ASubBoostContext *s = ctx->priv;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->filter_channels = filter_channels_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->filter_channels = filter_channels_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    s->attack = exp(-1.0 / (2.0 * inlink->sample_rate));
    s->release = exp(-1.0 / (0.0001 * inlink->sample_rate));
    s->buffer = ff_get_audio_buffer(inlink, inlink->sample_rate / 10);
    s->w = ff_get_audio_buffer(inlink, 4);
    s->write_pos = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->write_pos));
    if (!s->buffer || !s->w || !s->write_pos)
        return AVERROR(ENOMEM);

    return get_coeffs(ctx);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ASubBoostContext *s = ctx->priv;
    ThreadData td;
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

    td.in = in; td.out = out;
    ff_filter_execute(ctx, s->filter_channels, &td, NULL,
                      FFMIN(inlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    if (out != in)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ASubBoostContext *s = ctx->priv;

    av_frame_free(&s->buffer);
    av_frame_free(&s->w);
    av_freep(&s->write_pos);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return get_coeffs(ctx);
}

#define OFFSET(x) offsetof(ASubBoostContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption asubboost_options[] = {
    { "dry",      "set dry gain", OFFSET(dry_gain), AV_OPT_TYPE_DOUBLE, {.dbl=1.0},      0,   1, FLAGS },
    { "wet",      "set wet gain", OFFSET(wet_gain), AV_OPT_TYPE_DOUBLE, {.dbl=1.0},      0,   1, FLAGS },
    { "boost",    "set max boost",OFFSET(max_boost),AV_OPT_TYPE_DOUBLE, {.dbl=2.0},      1,  12, FLAGS },
    { "decay",    "set decay",    OFFSET(decay),    AV_OPT_TYPE_DOUBLE, {.dbl=0.0},      0,   1, FLAGS },
    { "feedback", "set feedback", OFFSET(feedback), AV_OPT_TYPE_DOUBLE, {.dbl=0.9},      0,   1, FLAGS },
    { "cutoff",   "set cutoff",   OFFSET(cutoff),   AV_OPT_TYPE_DOUBLE, {.dbl=100},     50, 900, FLAGS },
    { "slope",    "set slope",    OFFSET(slope),    AV_OPT_TYPE_DOUBLE, {.dbl=0.5}, 0.0001,   1, FLAGS },
    { "delay",    "set delay",    OFFSET(delay),    AV_OPT_TYPE_DOUBLE, {.dbl=20},       1, 100, FLAGS },
    { "channels", "set channels to filter", OFFSET(ch_layout), AV_OPT_TYPE_CHLAYOUT, {.str="24c"}, 0, 0, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(asubboost);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_asubboost = {
    .p.name         = "asubboost",
    .p.description  = NULL_IF_CONFIG_SMALL("Boost subwoofer frequencies."),
    .p.priv_class   = &asubboost_class,
    .priv_size      = sizeof(ASubBoostContext),
    .uninit         = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .process_command = process_command,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_SLICE_THREADS,
};
