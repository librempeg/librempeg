/*
 * Copyright (c) 2025 Paul B Mahol
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

#include <float.h> /* DBL_MAX */

#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "filters.h"
#include "avfilter.h"

typedef struct AudioAWienerContext {
    const AVClass *class;

    int look, hlook;
    AVChannelLayout ch_layout;
    double noise_var;
    int trim_size;
    int flush_size;
    int64_t last_pts;

    int nb_channels;
    AVFrame *in;

    void *st;

    int (*awiener_init)(AVFilterContext *ctx);
    void (*awiener_uninit)(AVFilterContext *ctx);
    int (*do_awiener)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch);
} AudioAWienerContext;

#define OFFSET(x) offsetof(AudioAWienerContext, x)
#define AF AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_AUDIO_PARAM
#define AFR AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption aawiener_options[] = {
    { "look", "set the look-ahead samples", OFFSET(look), AV_OPT_TYPE_INT, {.i64=1025}, 1, 1 << 20, AF },
    { "noise","set the noise variance",     OFFSET(noise_var), AV_OPT_TYPE_DOUBLE, {.dbl=0.0001}, 0, 100, AFR },
    { "channels", "set channels to filter", OFFSET(ch_layout), AV_OPT_TYPE_CHLAYOUT, {.str="24c"}, 0, 0, AFR },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aawiener);

#define DEPTH 32
#include "aawiener_template.c"

#undef DEPTH
#define DEPTH 64
#include "aawiener_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioAWienerContext *s = ctx->priv;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->do_awiener = do_awiener_fltp;
        s->awiener_uninit = awiener_uninit_fltp;
        s->awiener_init = awiener_init_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->do_awiener = do_awiener_dblp;
        s->awiener_uninit = awiener_uninit_dblp;
        s->awiener_init = awiener_init_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return s->awiener_init(ctx);
}

static int awiener_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioAWienerContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->do_awiener(ctx, s->in, out, ch);

    return 0;
}

static int filter_frame(AVFilterLink *outlink, AVFrame *in)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    AudioAWienerContext *s = ctx->priv;
    AVFrame *out;

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);
    out->pts -= av_rescale_q(s->hlook - s->trim_size, av_make_q(1, outlink->sample_rate), outlink->time_base);
    out->nb_samples -= s->trim_size;
    out->duration = av_rescale_q(out->nb_samples,
                                 (AVRational){1, outlink->sample_rate},
                                 outlink->time_base);

    s->last_pts = out->pts + out->duration;

    s->in = in;
    ff_filter_execute(ctx, awiener_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    if (s->trim_size > 0) {
        if (s->trim_size < in->nb_samples) {
            const size_t bps = av_get_bytes_per_sample(out->format);

            for (int ch = 0; ch < out->ch_layout.nb_channels; ch++)
                out->extended_data[ch] += s->trim_size * bps;

            s->trim_size = 0;
        } else {
            s->trim_size = FFMAX(s->trim_size - in->nb_samples, 0);
        }
    }

    if (s->trim_size > 0) {
        ff_inlink_request_frame(inlink);
        av_frame_free(&out);
    }

    av_frame_free(&in);
    s->in = NULL;
    if (out)
        return ff_filter_frame(outlink, out);
    return 0;
}

static int flush_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioAWienerContext *s = ctx->priv;
    int ret = 0;

    while (s->flush_size > 0) {
        const int nb_samples = s->flush_size;
        AVFrame *out = ff_get_audio_buffer(outlink, nb_samples);
        AVFrame *in = ff_get_audio_buffer(outlink, nb_samples);

        if (!out)
            return AVERROR(ENOMEM);

        s->flush_size -= nb_samples;

        s->in = in;
        ff_filter_execute(ctx, awiener_channels, out, NULL,
                          FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

        out->pts = s->last_pts;
        out->duration = av_rescale_q(out->nb_samples,
                                     (AVRational){1, outlink->sample_rate},
                                     outlink->time_base);
        s->last_pts += out->duration;

        av_frame_free(&in);
        s->in = NULL;
        ret = ff_filter_frame(outlink, out);
        if (ret < 0)
            break;
    }

    return ret;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];
    int ret, status;
    int64_t pts;
    AVFrame *in;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_frame(inlink, &in);
    if (ret < 0)
        return ret;
    if (ret > 0)
        return filter_frame(outlink, in);

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        AudioAWienerContext *s = ctx->priv;

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
    AudioAWienerContext *s = ctx->priv;

    if (s->awiener_uninit)
        s->awiener_uninit(ctx);
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_aawiener = {
    .p.name        = "aawiener",
    .p.description = NULL_IF_CONFIG_SMALL("Audio Noise Reduction with Adaptive Wiener filter."),
    .p.priv_class  = &aawiener_class,
    .priv_size     = sizeof(AudioAWienerContext),
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .process_command = ff_filter_process_command,
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                     AVFILTER_FLAG_SLICE_THREADS,
};
