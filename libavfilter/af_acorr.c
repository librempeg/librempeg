/*
 * Copyright (c) 2026 Paul B Mahol
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

#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"

typedef struct AudioCorrContext {
    const AVClass *class;

    int size;
    int channels;

    AVFrame *in[2];
    int eof;
    int eof_status;
    int64_t eof_pts;

    void *state;

    void (*acorr)(AVFilterContext *ctx, AVFrame *out, const int ch);
    void (*uninit)(AVFilterContext *ctx);
} AudioCorrContext;

#define DEPTH 32
#include "acorr_template.c"

#undef DEPTH
#define DEPTH 64
#include "acorr_template.c"

static int filter_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AVFrame *out = arg;
    AudioCorrContext *s = ctx->priv;
    const int start = ff_slice_pos(out->ch_layout.nb_channels, jobnr, nb_jobs);
    const int end = ff_slice_pos(out->ch_layout.nb_channels, jobnr+1, nb_jobs);

    for (int ch = start; ch < end; ch++)
        s->acorr(ctx, out, ch);

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AudioCorrContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    if (!s->in[0] && !s->eof) {
        int ret;

        ret = ff_inlink_consume_samples(ctx->inputs[0], s->size, s->size, &s->in[0]);
        if (ret < 0)
            return ret;
    }

    if (s->in[0] && !s->eof && !s->in[1]) {
        const int ns = s->in[0]->nb_samples;
        int ret = ff_inlink_consume_samples(ctx->inputs[1], ns, ns, &s->in[1]);
        if (ret < 0)
            return ret;
    }

    if (s->in[0] && s->in[1]) {
        const int out_samples = s->in[0]->nb_samples;
        AVFrame *out;

        out = ff_get_audio_buffer(outlink, out_samples);
        if (!out) {
            av_frame_free(&s->in[0]);
            av_frame_free(&s->in[1]);
            return AVERROR(ENOMEM);
        }

        ff_filter_execute(ctx, filter_channels, out, NULL,
                          FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

        av_frame_copy_props(out, s->in[0]);

        ff_graph_frame_free(ctx, &s->in[0]);
        ff_graph_frame_free(ctx, &s->in[1]);

        return ff_filter_frame(outlink, out);
    }

    for (int i = 0; i < 2 && !s->eof; i++) {
        if (ff_inlink_acknowledge_status(ctx->inputs[i], &s->eof_status, &s->eof_pts))
            s->eof = 1;
    }

    if (s->eof && (!s->in[0] || !s->in[1])) {
        ff_outlink_set_status(outlink, s->eof_status, s->eof_pts);
        return 0;
    }

    if (ff_outlink_frame_wanted(outlink) && !s->eof) {
        for (int i = 0; i < 2; i++) {
            if (s->in[i])
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
    AudioCorrContext *s = ctx->priv;
    int ret;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->acorr = acorr_dblp;
        s->uninit = acorr_uninit_dblp;
        ret = acorr_init_dblp(ctx);
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->acorr = acorr_fltp;
        s->uninit = acorr_uninit_fltp;
        ret = acorr_init_fltp(ctx);
        break;
    default:
        return AVERROR_BUG;
    }

    return ret;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioCorrContext *s = ctx->priv;

    if (s->uninit)
        s->uninit(ctx);
}

static const AVFilterPad inputs[] = {
    {
        .name = "acorr0",
        .type = AVMEDIA_TYPE_AUDIO,
    },
    {
        .name = "acorr1",
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
#define OFFSET(x) offsetof(AudioCorrContext, x)

static const AVOption acorr_options[] = {
    { "size", "set the transform size", OFFSET(size), AV_OPT_TYPE_INT, {.i64=8192}, 128, 1<<17, AF },
    { NULL }
};

AVFILTER_DEFINE_CLASS(acorr);

const FFFilter ff_af_acorr = {
    .p.name         = "acorr",
    .p.description  = NULL_IF_CONFIG_SMALL("Calculate the correlation between two audio streams."),
    .p.priv_class   = &acorr_class,
    .priv_size      = sizeof(AudioCorrContext),
    .activate       = activate,
    .uninit         = uninit,
    .p.flags        = AVFILTER_FLAG_SLICE_THREADS,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
};
