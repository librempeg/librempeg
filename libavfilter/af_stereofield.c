/*
 * Copyright (c) 2023 Paul B Mahol
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
#include "libavutil/opt.h"
#include "libavutil/mem.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

enum operation {
    OP_LEFT,
    OP_RIGHT,
    OP_STEREO,
    NB_OPERATION
};

typedef struct StereoFieldContext {
    const AVClass *class;

    double D, P;
    int mode;

    double *A;
    unsigned A_size;

    int fft_size;
    int overlap;

    int trim_size;
    int flush_size;
    int64_t last_pts;

    void *window;

    AVFrame *in;
    AVFrame *in_frame;
    AVFrame *out_dist_frame;
    AVFrame *windowed_frame;
    AVFrame *windowed_out;

    int (*sf_stereo)(AVFilterContext *ctx, AVFrame *out);
    int (*sf_flush)(AVFilterContext *ctx, AVFrame *out);

    AVTXContext *tx_ctx, *itx_ctx;
    av_tx_fn tx_fn, itx_fn;
} StereoFieldContext;

#define OFFSET(x) offsetof(StereoFieldContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_att = {.def="1 1", .size_min=2, .size_max=2, .sep=' '};

static const AVOption stereofield_options[] = {
    { "d", "set the depth", OFFSET(D), AV_OPT_TYPE_DOUBLE, {.dbl=0.}, -1, 1, FLAGS },
    { "a", "set the attenuation", OFFSET(A), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_att}, 0, 2, FLAGS },
    { "p", "set the panning", OFFSET(P), AV_OPT_TYPE_DOUBLE, {.dbl=0.}, -1, 1, FLAGS },
    { "o", "set the operating mode", OFFSET(mode), AV_OPT_TYPE_INT, {.i64=OP_STEREO}, 0, NB_OPERATION-1, FLAGS, "mode" },
    {  "l", "left",   0, AV_OPT_TYPE_CONST, {.i64=OP_LEFT},   0, 0, FLAGS, .unit = "mode"},
    {  "r", "right",  0, AV_OPT_TYPE_CONST, {.i64=OP_RIGHT},  0, 0, FLAGS, .unit = "mode"},
    {  "s", "stereo", 0, AV_OPT_TYPE_CONST, {.i64=OP_STEREO}, 0, 0, FLAGS, .unit = "mode"},
    {NULL}
};

AVFILTER_DEFINE_CLASS(stereofield);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVSampleFormat formats[] = {
        AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE,
    };
    static const AVChannelLayout layouts[] = {
        AV_CHANNEL_LAYOUT_STEREO,
        { .nb_channels = 0 },
    };
    int ret;

    ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, formats);
    if (ret < 0)
        return ret;

    return ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, layouts);
}

#define DEPTH 32
#include "stereofield_template.c"

#undef DEPTH
#define DEPTH 64
#include "stereofield_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    StereoFieldContext *s = ctx->priv;
    int ret;

    s->fft_size = 1 << av_ceil_log2((inlink->sample_rate + 19) / 20);
    s->overlap = (s->fft_size + 3) / 4;
    s->trim_size = s->fft_size - s->overlap;
    s->flush_size = s->fft_size - s->overlap;

    s->in_frame       = ff_get_audio_buffer(inlink, s->fft_size + 2);
    s->out_dist_frame = ff_get_audio_buffer(inlink, s->fft_size * 2);
    s->windowed_frame = ff_get_audio_buffer(inlink, s->fft_size + 2);
    s->windowed_out   = ff_get_audio_buffer(inlink, s->fft_size + 2);
    if (!s->in_frame || !s->windowed_out || !s->out_dist_frame || !s->windowed_frame)
        return AVERROR(ENOMEM);

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->sf_stereo = sf_stereo_float;
        s->sf_flush = sf_flush_float;
        ret = sf_tx_init_float(ctx);
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->sf_stereo = sf_stereo_double;
        s->sf_flush = sf_flush_double;
        ret = sf_tx_init_double(ctx);
        break;
    default:
        return AVERROR_BUG;
    }

    return ret;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    StereoFieldContext *s = ctx->priv;
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
    s->sf_stereo(ctx, out);

    out->pts -= av_rescale_q(s->fft_size - s->overlap, av_make_q(1, outlink->sample_rate), outlink->time_base);
    out->duration = av_rescale_q(out->nb_samples,
                                 (AVRational){1, outlink->sample_rate},
                                 outlink->time_base);

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
    StereoFieldContext *s = ctx->priv;
    int ret = 0;

    while (s->flush_size > 0) {
        const int nb_samples = FFMIN(s->flush_size, s->overlap);
        AVFrame *out = ff_get_audio_buffer(outlink, nb_samples);

        if (!out)
            return AVERROR(ENOMEM);

        s->flush_size -= nb_samples;

        s->sf_flush(ctx, out);

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
    StereoFieldContext *s = ctx->priv;
    AVFrame *in = NULL;
    int ret, status;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_samples(inlink, s->overlap, s->overlap, &in);
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
    StereoFieldContext *s = ctx->priv;

    av_freep(&s->window);

    av_frame_free(&s->in_frame);
    av_frame_free(&s->out_dist_frame);
    av_frame_free(&s->windowed_frame);
    av_frame_free(&s->windowed_out);

    av_tx_uninit(&s->tx_ctx);
    av_tx_uninit(&s->itx_ctx);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_stereofield = {
    .p.name          = "stereofield",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply Stereo Field effect."),
    .p.priv_class    = &stereofield_class,
    .priv_size       = sizeof(StereoFieldContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .activate        = activate,
    .process_command = ff_filter_process_command,
};
