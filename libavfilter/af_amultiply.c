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

#include "libavutil/thread.h"
#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/mem.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct AudioMultiplyContext {
    const AVClass *class;

    AVFrame *frames[2];

    int planes;
    int channels;
} AudioMultiplyContext;

static int multiply_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioMultiplyContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (s->planes * jobnr) / nb_jobs;
    const int end = (s->planes * (jobnr+1)) / nb_jobs;
    const int plane_samples = FFMIN(s->frames[0]->nb_samples, s->frames[1]->nb_samples);

    if (av_get_packed_sample_fmt(ctx->inputs[0]->format) == AV_SAMPLE_FMT_FLT) {
        for (int i = start; i < end; i++) {
            const float *src0 = (const float *)s->frames[0]->extended_data[i];
            const float *src1 = (const float *)s->frames[1]->extended_data[i];
            float *dst = (float *)out->extended_data[i];

            for (int n = 0; n < plane_samples; n++)
                dst[n] = src0[n] * src1[n];
        }
    } else {
        for (int i = start; i < end; i++) {
            const double *src0 = (const double *)s->frames[0]->extended_data[i];
            const double *src1 = (const double *)s->frames[1]->extended_data[i];
            double *dst = (double *)out->extended_data[i];

            for (int n = 0; n < plane_samples; n++)
                dst[n] = src0[n] * src1[n];
        }
    }

    return 0;
}

static int filter_prepare(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AudioMultiplyContext *s = ctx->priv;
    int ret;

    if (s->frames[0] && s->frames[1]) {
        ff_graph_frame_free(ctx, &s->frames[0]);
        ff_graph_frame_free(ctx, &s->frames[1]);
    }

    ret = ff_outlink_get_status(outlink);
    if (ret) {
        for (int i = 0; i < 2; i++)
            ff_inlink_set_status(ctx->inputs[i], ret);
        return AVERROR_EOF;
    }

    if (!s->frames[0]) {
        ret = ff_inlink_consume_frame(ctx->inputs[0], &s->frames[0]);
        if (ret < 0)
            return ret;
    }

    if (s->frames[0] && !s->frames[1]) {
        ret = ff_inlink_consume_samples(ctx->inputs[1],
                                        s->frames[0]->nb_samples,
                                        s->frames[0]->nb_samples,
                                        &s->frames[1]);
        if (ret < 0)
            return ret;
    }

    if (s->frames[0] && s->frames[1])
        return 0;

    for (int i = 0; i < 2; i++) {
        int64_t pts;
        int status;

        if (ff_inlink_acknowledge_status(ctx->inputs[i], &status, &pts)) {
            ff_outlink_set_status(outlink, status, pts);
            return AVERROR_EOF;
        }
    }

    if (ff_outlink_frame_wanted(outlink)) {
        for (int i = 0; i < 2; i++) {
            if (s->frames[i])
                continue;

            ff_inlink_request_frame(ctx->inputs[i]);
            return AVERROR(EAGAIN);
        }
    }

    return AVERROR(EAGAIN);
}

#if CONFIG_AVFILTER_THREAD_FRAME
static int transfer_state(AVFilterContext *dst, const AVFilterContext *src)
{
    const AudioMultiplyContext *s_src = src->priv;
    AudioMultiplyContext       *s_dst = dst->priv;

    if (!ff_filter_is_frame_thread(dst) || ff_filter_is_frame_thread(src))
        return 0;

    av_frame_free(&s_dst->frames[0]);
    av_frame_free(&s_dst->frames[1]);

    if (s_src->frames[0]) {
        s_dst->frames[0] = ff_graph_frame_clone(dst, s_src->frames[0]);
        if (!s_dst->frames[0])
            return AVERROR(ENOMEM);
    }

    if (s_src->frames[1]) {
        s_dst->frames[1] = ff_graph_frame_clone(dst, s_src->frames[1]);
        if (!s_dst->frames[1])
            return AVERROR(ENOMEM);
    }

    return 0;
}
#endif

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AudioMultiplyContext *s = ctx->priv;
    AVFrame *out;
    int ret;

    if (!s->frames[0] || !s->frames[1])
        return AVERROR_EOF;

    if (ff_filter_disabled(ctx)) {
        out = s->frames[0];

        return ff_filter_frame(outlink, out);
    }

    out = ff_graph_frame_alloc(ctx);
    if (!out)
        return AVERROR(ENOMEM);

    out->nb_samples = s->frames[0]->nb_samples;
    ret = ff_filter_get_buffer(ctx, out);
    if (ret < 0) {
        av_frame_free(&out);
        return ret;
    }
    av_frame_copy_props(out, s->frames[0]);

    ff_filter_execute(ctx, multiply_channels, out, NULL,
                      FFMIN(s->planes, ff_filter_get_nb_threads(ctx)));

    return ff_filter_frame(outlink, out);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioMultiplyContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];

    s->channels = inlink->ch_layout.nb_channels;
    s->planes = av_sample_fmt_is_planar(inlink->format) ? inlink->ch_layout.nb_channels : 1;

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioMultiplyContext *s = ctx->priv;

    av_frame_free(&s->frames[0]);
    av_frame_free(&s->frames[1]);
}

static const AVFilterPad inputs[] = {
    {
        .name = "multiply0",
        .type = AVMEDIA_TYPE_AUDIO,
    },
    {
        .name = "multiply1",
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

const FFFilter ff_af_amultiply = {
    .p.name         = "amultiply",
    .p.description  = NULL_IF_CONFIG_SMALL("Multiply two audio streams."),
    .priv_size      = sizeof(AudioMultiplyContext),
    .uninit         = uninit,
    .filter_prepare = filter_prepare,
    .activate       = activate,
#if CONFIG_AVFILTER_THREAD_FRAME
    .transfer_state = transfer_state,
#endif
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_FLTP,
                      AV_SAMPLE_FMT_DBL, AV_SAMPLE_FMT_DBLP),
    .p.flags        = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                      AVFILTER_FLAG_FRAME_THREADS |
                      AVFILTER_FLAG_SLICE_THREADS,
};
