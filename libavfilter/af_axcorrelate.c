/*
 * Copyright (c) 2019 Paul B Mahol
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

#include "libavutil/audio_fifo.h"
#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/opt.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "internal.h"

typedef struct AudioXCorrelateContext {
    const AVClass *class;

    int size;
    int algo;
    int64_t pts;

    AVAudioFifo *fifo[2];
    AVFrame *cache[2];
    AVFrame *mean_sum[2];
    AVFrame *num_sum;
    AVFrame *den_sum[2];
    int *used;
    int eof;

    void (*xcorrelate)(AVFilterContext *ctx, AVFrame *out, const int ch);
} AudioXCorrelateContext;

#define DEPTH 32
#include "axcorrelate_template.c"

#undef DEPTH
#define DEPTH 64
#include "axcorrelate_template.c"

static int filter_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AVFrame *out = arg;
    AudioXCorrelateContext *s = ctx->priv;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->xcorrelate(ctx, out, ch);

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AudioXCorrelateContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *frame = NULL;
    int ret, status;
    int available;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    for (int i = 0; i < 2 && !s->eof; i++) {
        ret = ff_inlink_consume_frame(ctx->inputs[i], &frame);
        if (ret > 0) {
            if (s->pts == AV_NOPTS_VALUE)
                s->pts = frame->pts;
            ret = av_audio_fifo_write(s->fifo[i], (void **)frame->extended_data,
                                      frame->nb_samples);
            av_frame_free(&frame);
            if (ret < 0)
                return ret;
        }
    }

    available = FFMIN(av_audio_fifo_size(s->fifo[0]), av_audio_fifo_size(s->fifo[1]));
    if (available > s->size) {
        const int out_samples = available - s->size;
        AVFrame *out;

        if (!s->cache[0] || s->cache[0]->nb_samples < available) {
            av_frame_free(&s->cache[0]);
            s->cache[0] = ff_get_audio_buffer(outlink, available);
            if (!s->cache[0])
                return AVERROR(ENOMEM);
        }

        if (!s->cache[1] || s->cache[1]->nb_samples < available) {
            av_frame_free(&s->cache[1]);
            s->cache[1] = ff_get_audio_buffer(outlink, available);
            if (!s->cache[1])
                return AVERROR(ENOMEM);
        }

        ret = av_audio_fifo_peek(s->fifo[0], (void **)s->cache[0]->extended_data, available);
        if (ret < 0)
            return ret;

        ret = av_audio_fifo_peek(s->fifo[1], (void **)s->cache[1]->extended_data, available);
        if (ret < 0)
            return ret;

        out = ff_get_audio_buffer(outlink, out_samples);
        if (!out)
            return AVERROR(ENOMEM);

        ff_filter_execute(ctx, filter_channels, out, NULL,
                          FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

        out->pts = s->pts;
        s->pts += out_samples;

        av_audio_fifo_drain(s->fifo[0], out_samples);
        av_audio_fifo_drain(s->fifo[1], out_samples);

        return ff_filter_frame(outlink, out);
    }

    for (int i = 0; i < 2 && !s->eof; i++) {
        if (ff_inlink_acknowledge_status(ctx->inputs[i], &status, &pts)) {
            AVFrame *silence = ff_get_audio_buffer(outlink, s->size);

            s->eof = 1;
            if (!silence)
                return AVERROR(ENOMEM);

            av_audio_fifo_write(s->fifo[0], (void **)silence->extended_data,
                                silence->nb_samples);

            av_audio_fifo_write(s->fifo[1], (void **)silence->extended_data,
                                silence->nb_samples);

            av_frame_free(&silence);
        }
    }

    if (s->eof &&
        (av_audio_fifo_size(s->fifo[0]) <= s->size ||
         av_audio_fifo_size(s->fifo[1]) <= s->size)) {
        ff_outlink_set_status(outlink, AVERROR_EOF, s->pts);
        return 0;
    }

    if ((av_audio_fifo_size(s->fifo[0]) > s->size &&
         av_audio_fifo_size(s->fifo[1]) > s->size) || s->eof) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    if (ff_outlink_frame_wanted(outlink) && !s->eof) {
        for (int i = 0; i < 2; i++) {
            if (av_audio_fifo_size(s->fifo[i]) > s->size)
                continue;
            ff_inlink_request_frame(ctx->inputs[i]);
            return 0;
        }
    }

    return FFERROR_NOT_READY;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioXCorrelateContext *s = ctx->priv;

    s->pts = AV_NOPTS_VALUE;

    s->used = av_calloc(outlink->ch_layout.nb_channels, sizeof(*s->used));
    s->fifo[0] = av_audio_fifo_alloc(outlink->format, outlink->ch_layout.nb_channels, s->size);
    s->fifo[1] = av_audio_fifo_alloc(outlink->format, outlink->ch_layout.nb_channels, s->size);
    if (!s->fifo[0] || !s->fifo[1] || !s->used)
        return AVERROR(ENOMEM);

    s->mean_sum[0] = ff_get_audio_buffer(outlink, 1);
    s->mean_sum[1] = ff_get_audio_buffer(outlink, 1);
    s->num_sum = ff_get_audio_buffer(outlink, 1);
    s->den_sum[0] = ff_get_audio_buffer(outlink, 1);
    s->den_sum[1] = ff_get_audio_buffer(outlink, 1);
    if (!s->mean_sum[0] || !s->mean_sum[1] || !s->num_sum ||
        !s->den_sum[0] || !s->den_sum[1])
        return AVERROR(ENOMEM);

    switch (s->algo) {
    case 0: s->xcorrelate = xcorrelate_slow_fltp; break;
    case 1: s->xcorrelate = xcorrelate_fast_fltp; break;
    case 2: s->xcorrelate = xcorrelate_best_fltp; break;
    }

    if (outlink->format == AV_SAMPLE_FMT_DBLP) {
        switch (s->algo) {
        case 0: s->xcorrelate = xcorrelate_slow_dblp; break;
        case 1: s->xcorrelate = xcorrelate_fast_dblp; break;
        case 2: s->xcorrelate = xcorrelate_best_dblp; break;
        }
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioXCorrelateContext *s = ctx->priv;

    av_freep(&s->used);
    av_audio_fifo_free(s->fifo[0]);
    av_audio_fifo_free(s->fifo[1]);
    av_frame_free(&s->cache[0]);
    av_frame_free(&s->cache[1]);
    av_frame_free(&s->mean_sum[0]);
    av_frame_free(&s->mean_sum[1]);
    av_frame_free(&s->num_sum);
    av_frame_free(&s->den_sum[0]);
    av_frame_free(&s->den_sum[1]);
}

static const AVFilterPad inputs[] = {
    {
        .name = "axcorrelate0",
        .type = AVMEDIA_TYPE_AUDIO,
    },
    {
        .name = "axcorrelate1",
        .type = AVMEDIA_TYPE_AUDIO,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define OFFSET(x) offsetof(AudioXCorrelateContext, x)

static const AVOption axcorrelate_options[] = {
    { "size", "set the segment size", OFFSET(size), AV_OPT_TYPE_INT, {.i64=256}, 2, 131072, AF },
    { "algo", "set the algorithm",    OFFSET(algo), AV_OPT_TYPE_INT, {.i64=2},   0,      2, AF, .unit = "algo" },
    { "slow", "slow algorithm",   0,            AV_OPT_TYPE_CONST, {.i64=0},   0,      0, AF, .unit = "algo" },
    { "fast", "fast algorithm",   0,            AV_OPT_TYPE_CONST, {.i64=1},   0,      0, AF, .unit = "algo" },
    { "best", "best algorithm",   0,            AV_OPT_TYPE_CONST, {.i64=2},   0,      0, AF, .unit = "algo" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(axcorrelate);

const AVFilter ff_af_axcorrelate = {
    .name           = "axcorrelate",
    .description    = NULL_IF_CONFIG_SMALL("Cross-correlate two audio streams."),
    .priv_size      = sizeof(AudioXCorrelateContext),
    .priv_class     = &axcorrelate_class,
    .activate       = activate,
    .uninit         = uninit,
    .flags          = AVFILTER_FLAG_SLICE_THREADS,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
};
