/*
 * Copyright (c) 2020 Paul B Mahol
 *
 * Speech Normalizer
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
 * Speech Normalizer
 */

#include <float.h>

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/fifo.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"

#define MAX_ITEMS  882000

typedef struct PeriodItem {
    int size;
    int type;
    double max_peak;
    double rms_sum;
} PeriodItem;

typedef struct ChannelContext {
    int state;
    int bypass;
    PeriodItem pi[MAX_ITEMS];
    double gain_state;
    double pi_max_peak;
    double pi_rms_sum;
    int pi_start;
    int pi_end;
    int pi_size;
    int acc;
} ChannelContext;

typedef struct SpeechNormalizerContext {
    const AVClass *class;

    double rms_value;
    double peak_value;
    double max_expansion;
    double max_compression;
    double threshold_value;
    double raise_amount;
    double fall_amount;
    AVChannelLayout ch_layout;
    int invert;
    int link;

    ChannelContext *cc;
    double prev_gain;

    int eof;
    int64_t pts;

    AVFifo *fifo;

    void (*analyze_channel)(AVFilterContext *ctx, ChannelContext *cc,
                            const uint8_t *srcp, int nb_samples);
    void (*filter_channels[2])(AVFilterContext *ctx,
                               AVFrame *in, AVFrame *out, int nb_samples);
} SpeechNormalizerContext;

#define OFFSET(x) offsetof(SpeechNormalizerContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption speechnorm_options[] = {
    { "peak", "set the peak value", OFFSET(peak_value), AV_OPT_TYPE_DOUBLE, {.dbl=0.95}, 0.0, 1.0, FLAGS },
    { "p",    "set the peak value", OFFSET(peak_value), AV_OPT_TYPE_DOUBLE, {.dbl=0.95}, 0.0, 1.0, FLAGS },
    { "expansion", "set the max expansion factor", OFFSET(max_expansion), AV_OPT_TYPE_DOUBLE, {.dbl=2.0}, 1.0, 50.0, FLAGS },
    { "e",         "set the max expansion factor", OFFSET(max_expansion), AV_OPT_TYPE_DOUBLE, {.dbl=2.0}, 1.0, 50.0, FLAGS },
    { "compression", "set the max compression factor", OFFSET(max_compression), AV_OPT_TYPE_DOUBLE, {.dbl=2.0}, 1.0, 50.0, FLAGS },
    { "c",           "set the max compression factor", OFFSET(max_compression), AV_OPT_TYPE_DOUBLE, {.dbl=2.0}, 1.0, 50.0, FLAGS },
    { "threshold", "set the threshold value", OFFSET(threshold_value), AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0.0, 1.0, FLAGS },
    { "t",         "set the threshold value", OFFSET(threshold_value), AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0.0, 1.0, FLAGS },
    { "raise", "set the expansion raising amount", OFFSET(raise_amount), AV_OPT_TYPE_DOUBLE, {.dbl=0.001}, 0.0, 1.0, FLAGS },
    { "r",     "set the expansion raising amount", OFFSET(raise_amount), AV_OPT_TYPE_DOUBLE, {.dbl=0.001}, 0.0, 1.0, FLAGS },
    { "fall", "set the compression raising amount", OFFSET(fall_amount), AV_OPT_TYPE_DOUBLE, {.dbl=0.001}, 0.0, 1.0, FLAGS },
    { "f",    "set the compression raising amount", OFFSET(fall_amount), AV_OPT_TYPE_DOUBLE, {.dbl=0.001}, 0.0, 1.0, FLAGS },
    { "channels", "set channels to filter", OFFSET(ch_layout), AV_OPT_TYPE_CHLAYOUT, {.str="24c"}, 0, 0, FLAGS },
    { "h",        "set channels to filter", OFFSET(ch_layout), AV_OPT_TYPE_CHLAYOUT, {.str="24c"}, 0, 0, FLAGS },
    { "invert", "set inverted filtering", OFFSET(invert), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
    { "i",      "set inverted filtering", OFFSET(invert), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
    { "link", "set linked channels filtering", OFFSET(link), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
    { "l",    "set linked channels filtering", OFFSET(link), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
    { "rms", "set the RMS value", OFFSET(rms_value), AV_OPT_TYPE_DOUBLE, {.dbl=0.0}, 0.0, 1.0, FLAGS },
    { "m",   "set the RMS value", OFFSET(rms_value), AV_OPT_TYPE_DOUBLE, {.dbl=0.0}, 0.0, 1.0, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(speechnorm);

static int get_pi_samples(ChannelContext *cc, int eof)
{
    if (eof) {
        PeriodItem *pi = cc->pi;

        return cc->acc + pi[cc->pi_end].size;
    } else {
        PeriodItem *pi = cc->pi;

        if (pi[cc->pi_start].type == 0)
            return cc->pi_size;
    }

    return cc->acc;
}

static int available_samples(AVFilterContext *ctx)
{
    SpeechNormalizerContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    int min_pi_nb_samples;

    min_pi_nb_samples = get_pi_samples(&s->cc[0], s->eof);
    for (int ch = 1; ch < inlink->ch_layout.nb_channels && min_pi_nb_samples > 0; ch++) {
        ChannelContext *cc = &s->cc[ch];

        min_pi_nb_samples = FFMIN(min_pi_nb_samples, get_pi_samples(cc, s->eof));
    }

    return min_pi_nb_samples;
}

static void consume_pi(ChannelContext *cc, int nb_samples)
{
    if (cc->pi_size >= nb_samples) {
        cc->pi_size -= nb_samples;
        cc->acc -= FFMIN(cc->acc, nb_samples);
    } else {
        av_assert1(0);
    }
}

static double next_gain(AVFilterContext *ctx, double pi_max_peak, int bypass, double state,
                        double pi_rms_sum, int pi_size, double scale)
{
    SpeechNormalizerContext *s = ctx->priv;
    const double compression = 1. / s->max_compression;
    const int type = s->invert ? pi_max_peak <= s->threshold_value : pi_max_peak >= s->threshold_value;
    const double ratio = s->peak_value / pi_max_peak;
    double expansion = FFMIN(s->max_expansion, ratio);

    if (s->rms_value > DBL_EPSILON)
        expansion = FFMIN(expansion, s->rms_value / sqrt(pi_rms_sum / pi_size));

    if (bypass) {
        return 1.;
    } else if (type) {
        const double raise_amount = s->raise_amount * scale;

        if (ratio > 1.0 && state < 1.0 && raise_amount == 0.0)
            state = 1.0;
        return FFMIN(expansion, state + raise_amount);
    } else {
        return FFMIN(expansion, FFMAX(compression, state - s->fall_amount * scale));
    }
}

static void next_pi(AVFilterContext *ctx, ChannelContext *cc, int bypass)
{
    av_assert1(cc->pi_size >= 0);
    if (cc->pi_size == 0) {
        av_unused SpeechNormalizerContext *s = ctx->priv;
        int start = cc->pi_start;
        double scale;

        av_assert1(cc->pi[start].size > 0);
        av_assert1(cc->pi[start].type > 0 || s->eof);
        cc->pi_size = cc->pi[start].size;
        cc->pi_rms_sum = cc->pi[start].rms_sum;
        cc->pi_max_peak = cc->pi[start].max_peak;
        av_assert1(cc->pi_start != cc->pi_end || s->eof);
        cc->pi[start].size = 0;
        cc->pi[start].type = 0;
        start++;
        if (start >= MAX_ITEMS)
            start = 0;
        cc->pi_start = start;
        scale = fmin(1.0, cc->pi_size / (double)ctx->inputs[0]->sample_rate);
        cc->gain_state = next_gain(ctx, cc->pi_max_peak, bypass, cc->gain_state,
                                   cc->pi_rms_sum, cc->pi_size, scale);
    }
}

static double min_gain(AVFilterContext *ctx, ChannelContext *cc, int max_size)
{
    SpeechNormalizerContext *s = ctx->priv;
    double min_gain = s->max_expansion;
    double gain_state = cc->gain_state;
    int size = cc->pi_size;
    int idx = cc->pi_start;
    double scale;

    min_gain = FFMIN(min_gain, gain_state);
    while (size <= max_size) {
        if (idx == cc->pi_end)
            break;
        scale = fmin(1.0, cc->pi[idx].size / (double)ctx->inputs[0]->sample_rate);
        gain_state = next_gain(ctx, cc->pi[idx].max_peak, 0, gain_state,
                               cc->pi[idx].rms_sum, cc->pi[idx].size, scale);
        min_gain = FFMIN(min_gain, gain_state);
        size += cc->pi[idx].size;
        idx++;
        if (idx >= MAX_ITEMS)
            idx = 0;
    }

    return min_gain;
}

#define DEPTH 32
#include "speechnorm_template.c"

#undef DEPTH
#define DEPTH 64
#include "speechnorm_template.c"

static int filter_frame(AVFilterContext *ctx)
{
    SpeechNormalizerContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    int ret;

    while (av_fifo_can_read(s->fifo) > 0) {
        int min_pi_nb_samples;
        AVFrame *in = NULL, *out;

        av_fifo_peek(s->fifo, &in, 1, 0);
        if (!in)
            break;

        min_pi_nb_samples = available_samples(ctx);
        if (min_pi_nb_samples < in->nb_samples && !s->eof)
            break;

        av_fifo_read(s->fifo, &in, 1);
        if (!in)
            break;

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

        s->filter_channels[s->link](ctx, in, out, in->nb_samples);

        s->pts = in->pts + av_rescale_q(in->nb_samples, av_make_q(1, outlink->sample_rate),
                                        outlink->time_base);

        if (out != in)
            ff_graph_frame_free(ctx, &in);
        return ff_filter_frame(outlink, out);
    }

    for (int f = 0; f < ff_inlink_queued_frames(inlink); f++) {
        AVFrame *in;

        ret = ff_inlink_consume_frame(inlink, &in);
        if (ret < 0)
            return ret;
        if (ret == 0)
            break;

        av_fifo_write(s->fifo, &in, 1);

        for (int ch = 0; ch < inlink->ch_layout.nb_channels; ch++) {
            ChannelContext *cc = &s->cc[ch];

            s->analyze_channel(ctx, cc, in->extended_data[ch], in->nb_samples);
        }
    }

    return 1;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    SpeechNormalizerContext *s = ctx->priv;
    int ret, status;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = filter_frame(ctx);
    if (ret <= 0)
        return ret;

    if (!s->eof && ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (status == AVERROR_EOF)
            s->eof = 1;
    }

    if (s->eof && ff_inlink_queued_samples(inlink) == 0 &&
        av_fifo_can_read(s->fifo) == 0) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
        return 0;
    }

    if (av_fifo_can_read(s->fifo) > 0) {
        const int nb_samples = available_samples(ctx);
        AVFrame *in;

        av_fifo_peek(s->fifo, &in, 1, 0);
        if (nb_samples >= in->nb_samples || s->eof) {
            ff_filter_set_ready(ctx, 10);
            return 0;
        }
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    SpeechNormalizerContext *s = ctx->priv;

    s->prev_gain = 1.;
    s->cc = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->cc));
    if (!s->cc)
        return AVERROR(ENOMEM);

    for (int ch = 0; ch < inlink->ch_layout.nb_channels; ch++) {
        ChannelContext *cc = &s->cc[ch];

        cc->state = -2;
        cc->gain_state = s->max_expansion;
    }

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->analyze_channel = analyze_channel_flt;
        s->filter_channels[0] = filter_channels_flt;
        s->filter_channels[1] = filter_link_channels_flt;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->analyze_channel = analyze_channel_dbl;
        s->filter_channels[0] = filter_channels_dbl;
        s->filter_channels[1] = filter_link_channels_dbl;
        break;
    default:
        av_assert1(0);
    }

    s->fifo = av_fifo_alloc2(1024, sizeof(AVFrame *), AV_FIFO_FLAG_AUTO_GROW);
    if (!s->fifo)
        return AVERROR(ENOMEM);

    return 0;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    SpeechNormalizerContext *s = ctx->priv;
    int link = s->link;
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;
    if (link != s->link)
        s->prev_gain = 1.;

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    SpeechNormalizerContext *s = ctx->priv;

    av_fifo_freep2(&s->fifo);
    av_freep(&s->cc);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_speechnorm = {
    .p.name          = "speechnorm",
    .p.description   = NULL_IF_CONFIG_SMALL("Speech Normalizer."),
    .p.priv_class    = &speechnorm_class,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .priv_size       = sizeof(SpeechNormalizerContext),
    .activate        = activate,
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .process_command = process_command,
};
