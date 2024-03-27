/*
 * Copyright (c) 2023 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "formats.h"
#include "avfilter.h"
#include "filters.h"
#include "internal.h"

typedef struct AudioRDFTSRCContext {
    const AVClass *class;

    int quality;
    int sample_rate;
    int in_rdft_size;
    int out_rdft_size;
    int in_nb_samples;
    int tr_nb_samples;
    int taper_samples;
    int out_nb_samples;
    int channels;

    void *taper;

    AVFrame *in;
    AVFrame *over;
    AVFrame *rdft_in[2];
    AVFrame *rdft_out[2];

    int (*do_src)(AVFilterContext *ctx, AVFrame *in, AVFrame *out,
                  const int ch, const int soffset, const int doffset);

    AVTXContext **tx_ctx, **itx_ctx;
    av_tx_fn tx_fn, itx_fn;
} AudioRDFTSRCContext;

#define OFFSET(x) offsetof(AudioRDFTSRCContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption ardftsrc_options[] = {
    { "sample_rate", "set the sample rate", OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, FLAGS },
    { "quality", "set the quality", OFFSET(quality), AV_OPT_TYPE_INT, {.i64=1024}, 1, UINT16_MAX, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(ardftsrc);

static int query_formats(AVFilterContext *ctx)
{
    AudioRDFTSRCContext *s = ctx->priv;
    AVFilterFormats *formats = NULL;
    int ret, sample_rates[] = { s->sample_rate, -1 };

    ret = ff_add_format(&formats, AV_SAMPLE_FMT_FLTP);
    if (ret)
        return ret;
    ret = ff_add_format(&formats, AV_SAMPLE_FMT_DBLP);
    if (ret)
        return ret;
    ret = ff_set_common_formats(ctx, formats);
    if (ret)
        return ret;

    ret = ff_set_common_all_channel_counts(ctx);
    if (ret)
        return ret;

    if (!s->sample_rate)
        return ff_set_common_all_samplerates(ctx);

    if ((ret = ff_formats_ref(ff_all_samplerates(),
                              &ctx->inputs[0]->outcfg.samplerates)) < 0)
        return ret;

    return ff_formats_ref(ff_make_format_list(sample_rates),
                          &ctx->outputs[0]->incfg.samplerates);
}

#define DEPTH 32
#include "ardftsrc_template.c"

#undef DEPTH
#define DEPTH 64
#include "ardftsrc_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioRDFTSRCContext *s = ctx->priv;
    int ret, channels, factor;

    if (inlink->sample_rate == outlink->sample_rate)
        return 0;

    av_reduce(&s->in_nb_samples, &s->out_nb_samples,
              inlink->sample_rate, outlink->sample_rate, INT_MAX);

    factor = FFMIN(s->in_nb_samples, s->out_nb_samples);
    factor = 1 << av_ceil_log2((s->quality + factor - 1) / factor);
    s->in_nb_samples *= factor;
    s->out_nb_samples *= factor;

    s->in_rdft_size = s->in_nb_samples * 2;
    s->out_rdft_size = s->out_nb_samples * 2;
    s->tr_nb_samples = FFMIN(s->in_nb_samples, s->out_nb_samples);
    s->taper_samples = (s->tr_nb_samples + 19) / 20;
    av_log(ctx, AV_LOG_DEBUG, "%d: %d => %d\n", factor, s->in_rdft_size, s->out_rdft_size);

    s->over = ff_get_audio_buffer(inlink, s->out_rdft_size);

    s->rdft_in[0] = ff_get_audio_buffer(inlink, s->in_rdft_size + 2);
    s->rdft_in[1] = ff_get_audio_buffer(inlink, s->in_rdft_size + 2);

    s->rdft_out[0] = ff_get_audio_buffer(inlink, s->out_rdft_size + 2);
    s->rdft_out[1] = ff_get_audio_buffer(inlink, s->out_rdft_size + 2);

    channels = inlink->ch_layout.nb_channels;
    s->tx_ctx = av_calloc(channels, sizeof(*s->tx_ctx));
    s->itx_ctx = av_calloc(channels, sizeof(*s->itx_ctx));
    if (!s->tx_ctx || !s->itx_ctx || !s->over ||
        !s->rdft_in[0] || !s->rdft_in[1] ||
        !s->rdft_out[0] || !s->rdft_out[1])
        return AVERROR(ENOMEM);

    s->channels = channels;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->do_src = src_float;
        ret = src_tx_init_float(ctx);
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->do_src = src_double;
        ret = src_tx_init_double(ctx);
        break;
    }

    return ret;
}

static int src_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioRDFTSRCContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        for (int soffset = 0, doffset = 0; soffset < s->in->nb_samples;
             soffset += s->in_nb_samples, doffset += s->out_nb_samples)
            s->do_src(ctx, s->in, out, ch, soffset, doffset);
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioRDFTSRCContext *s = ctx->priv;
    const int offset = (s->in_rdft_size - s->in_nb_samples) >> 1;
    const int factor = FFMAX(1, in->nb_samples / s->in_nb_samples);
    AVFrame *out;
    int ret;

    if (inlink->sample_rate == outlink->sample_rate)
        return ff_filter_frame(outlink, in);

    out = ff_get_audio_buffer(outlink, s->out_nb_samples * factor);
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    s->in = in;
    av_frame_copy_props(out, in);
    ff_filter_execute(ctx, src_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    out->sample_rate = outlink->sample_rate;
    out->pts = in->pts;
    out->pts -= av_rescale_q(offset, av_make_q(1, inlink->sample_rate), inlink->time_base);
    out->pts = av_rescale_q(out->pts, outlink->time_base, inlink->time_base);
    ret = ff_filter_frame(outlink, out);
fail:
    av_frame_free(&in);
    s->in = NULL;
    return ret < 0 ? ret : 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AudioRDFTSRCContext *s = ctx->priv;
    AVFrame *in = NULL;
    int ret, status;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (inlink->sample_rate == outlink->sample_rate) {
        ret = ff_inlink_consume_frame(inlink, &in);
    } else {
        const int available = ff_inlink_queued_samples(inlink);
        const int wanted = FFMAX(s->in_nb_samples, (available / s->in_nb_samples) * s->in_nb_samples);

        ret = ff_inlink_consume_samples(inlink, wanted, wanted, &in);
    }
    if (ret < 0)
        return ret;

    if (ret > 0) {
        return filter_frame(inlink, in);
    } else if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        ff_outlink_set_status(outlink, status, pts);
        return 0;
    } else {
        if (inlink->sample_rate != outlink->sample_rate &&
            ff_inlink_queued_samples(inlink) >= s->in_nb_samples) {
            ff_filter_set_ready(ctx, 10);
        } else if (ff_outlink_frame_wanted(outlink)) {
            ff_inlink_request_frame(inlink);
        }
        return 0;
    }
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioRDFTSRCContext *s = ctx->priv;

    av_freep(&s->taper);

    av_frame_free(&s->over);
    av_frame_free(&s->rdft_in[0]);
    av_frame_free(&s->rdft_in[1]);
    av_frame_free(&s->rdft_out[0]);
    av_frame_free(&s->rdft_out[1]);

    for (int ch = 0; ch < s->channels; ch++) {
        if (s->tx_ctx)
            av_tx_uninit(&s->tx_ctx[ch]);
        if (s->itx_ctx)
            av_tx_uninit(&s->itx_ctx[ch]);
    }

    av_freep(&s->tx_ctx);
    av_freep(&s->itx_ctx);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const AVFilter ff_af_ardftsrc = {
    .name            = "ardftsrc",
    .description     = NULL_IF_CONFIG_SMALL("Audio Real Discrete Fourier Transform Sample Rate Conversion."),
    .priv_size       = sizeof(AudioRDFTSRCContext),
    .priv_class      = &ardftsrc_class,
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC(query_formats),
    .flags           = AVFILTER_FLAG_SLICE_THREADS,
    .activate        = activate,
};
