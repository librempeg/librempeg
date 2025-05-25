/*
 * Copyright (c) 2025 Paul B Mahol
 *
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

#include <float.h>

#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct AudioSpectralSubtractionContext {
    const AVClass *class;

    double reduction;
    double beta;
    int history;
    int rdft_size;
    int overlap;
    int channels;

    int trim_size;
    int flush_size;
    int64_t last_pts;

    void *window;
    void *st;

    AVFrame *in;

    AVChannelLayout ch_layout;

    void (*generate_window)(void *window, int size);
    int (*init)(AVFilterContext *ctx);
    void (*uninit)(AVFilterContext *ctx);
    int (*spectral_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, int ch);
    int (*spectral_flush_channel)(AVFilterContext *ctx,  AVFrame *out, int ch);
} AudioSpectralSubtractionContext;

#define OFFSET(x) offsetof(AudioSpectralSubtractionContext, x)
#define TFLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM
#define  FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption aspectralsub_options[] = {
    { "reduction", "set noise reduction",         OFFSET(reduction), AV_OPT_TYPE_DOUBLE,   {.dbl=0.05},  0,   1, TFLAGS },
    { "beta",      "set exponent factor",         OFFSET(beta),      AV_OPT_TYPE_DOUBLE,   {.dbl=30},    0, 100, TFLAGS },
    { "history",   "set look-back history",       OFFSET(history),   AV_OPT_TYPE_INT,      {.i64=12},    2,  64, FLAGS },
    { "channels",  "set channels to filter",      OFFSET(ch_layout), AV_OPT_TYPE_CHLAYOUT, {.str="24c"}, 0,   0, TFLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(aspectralsub);

#define DEPTH 32
#include "aspectralsub_template.c"

#undef DEPTH
#define DEPTH 64
#include "aspectralsub_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioSpectralSubtractionContext *s = ctx->priv;
    size_t sample_size;

    s->rdft_size = 1 << av_ceil_log2(outlink->sample_rate * 80 / 1000);
    s->overlap = s->rdft_size / 4;
    s->trim_size = s->rdft_size - s->overlap;
    s->flush_size = s->rdft_size - s->overlap;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        sample_size = sizeof(float);
        s->generate_window = generate_hann_window_fltp;
        s->spectral_channel = spectral_channel_fltp;
        s->spectral_flush_channel = spectral_flush_channel_fltp;
        s->init = init_fltp;
        s->uninit = uninit_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        sample_size = sizeof(double);
        s->generate_window = generate_hann_window_dblp;
        s->spectral_channel = spectral_channel_dblp;
        s->spectral_flush_channel = spectral_flush_channel_dblp;
        s->init = init_dblp;
        s->uninit = uninit_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    s->window = av_calloc(s->rdft_size, sample_size);
    if (!s->window)
        return AVERROR(ENOMEM);

    s->generate_window(s->window, s->rdft_size);

    s->channels = outlink->ch_layout.nb_channels;

    return s->init(ctx);
}

static int spectral_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSpectralSubtractionContext *s = ctx->priv;
    AVFrame *in = s->in;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->spectral_channel(ctx, in, out, ch);

    return 0;
}

static int spectral_flush_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSpectralSubtractionContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->spectral_flush_channel(ctx, out, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioSpectralSubtractionContext *s = ctx->priv;
    int extra_samples, nb_samples;
    AVFrame *out;

    extra_samples = in->nb_samples % s->overlap;
    if (extra_samples)
        extra_samples = FFMIN(s->overlap - extra_samples, s->flush_size);
    nb_samples = in->nb_samples;
    if (extra_samples > 0) {
        nb_samples += extra_samples;
        s->flush_size -= extra_samples;
    }

    out = ff_get_audio_buffer(outlink, nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    s->in = in;
    ff_filter_execute(ctx, spectral_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    out->pts = in->pts;
    out->pts -= av_rescale_q(s->rdft_size - s->overlap, av_make_q(1, outlink->sample_rate), outlink->time_base);

    s->last_pts = out->pts + out->duration;

    if (s->trim_size > 0 && s->trim_size < out->nb_samples) {
        for (int ch = 0; ch < out->ch_layout.nb_channels; ch++)
            out->extended_data[ch] += s->trim_size * av_get_bytes_per_sample(out->format);

        out->nb_samples -= s->trim_size;
        s->trim_size = 0;
    } else if (s->trim_size > 0) {
        s->trim_size -= out->nb_samples;
        av_frame_free(&out);
        av_frame_free(&in);

        ff_inlink_request_frame(inlink);

        return 0;
    }

    s->in = NULL;
    av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static int flush_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioSpectralSubtractionContext *s = ctx->priv;
    int ret = 0;

    while (s->flush_size > 0) {
        const int nb_samples = FFMIN(s->flush_size, s->overlap);
        AVFrame *out = ff_get_audio_buffer(outlink, nb_samples);

        if (!out)
            return AVERROR(ENOMEM);

        s->flush_size -= nb_samples;

        ff_filter_execute(ctx, spectral_flush_channels, out, NULL,
                          FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

        out->pts = s->last_pts;
        out->duration = av_rescale_q(out->nb_samples,
                                     (AVRational){1, outlink->sample_rate},
                                     outlink->time_base);
        s->last_pts += out->duration;

        ret = ff_filter_frame(outlink, out);
        if (ret < 0)
            break;
    }

    return ret;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AudioSpectralSubtractionContext *s = ctx->priv;
    int ret, status, available, wanted;
    AVFrame *in = NULL;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    available = ff_inlink_queued_samples(inlink);
    wanted = FFMAX(s->overlap, (available / s->overlap) * s->overlap);
    ret = ff_inlink_consume_samples(inlink, wanted, wanted, &in);
    if (ret < 0)
        return ret;

    if (ret > 0)
        return filter_frame(inlink, in);

    if (ff_inlink_queued_samples(inlink) >= s->overlap) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (s->flush_size > 0)
            ret = flush_frame(outlink);

        ff_outlink_set_status(outlink, status, pts);
        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioSpectralSubtractionContext *s = ctx->priv;

    if (s->uninit)
        s->uninit(ctx);

    av_freep(&s->window);
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_aspectralsub = {
    .p.name          = "aspectralsub",
    .p.description   = NULL_IF_CONFIG_SMALL("Audio Noise Reduction with Spectral Subtraction filter."),
    .p.priv_class    = &aspectralsub_class,
    .priv_size       = sizeof(AudioSpectralSubtractionContext),
    .uninit          = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_SLICE_THREADS,
    .activate        = activate,
    .process_command = ff_filter_process_command,
};
