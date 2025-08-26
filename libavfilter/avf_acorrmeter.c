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

/**
 * @file
 * audio to video multimedia acorrmeter filter
 */

#include <float.h>

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavutil/mem.h"
#include "libavutil/parseutils.h"
#include "libavutil/timestamp.h"
#include "libavutil/tx.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "audio.h"
#include "video.h"

typedef struct AudioCorrelationMeterContext {
    const AVClass *class;
    AVFrame *out, *in;
    int64_t last_pts;
    int do_video;
    int w, h;
    AVRational frame_rate;
    int contrast[4];
    uint8_t *mpc_str;
    uint8_t mpc[4];
    int draw_mean_correlation;
    int rdft_size;
    int nb_samples;
    float correlation;

    float *in_rdft[2];
    AVComplexFloat *out_rdft[2];
    AVTXContext *tx_ctx;
    av_tx_fn tx_fn;
} AudioCorrelationMeterContext;

#define OFFSET(x) offsetof(AudioCorrelationMeterContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption acorrmeter_options[] = {
    { "rate", "set video rate", OFFSET(frame_rate), AV_OPT_TYPE_VIDEO_RATE, {.str="25"}, 0, INT_MAX, FLAGS },
    { "r",    "set video rate", OFFSET(frame_rate), AV_OPT_TYPE_VIDEO_RATE, {.str="25"}, 0, INT_MAX, FLAGS },
    { "size", "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str="800x400"}, 0, 0, FLAGS },
    { "s",    "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str="800x400"}, 0, 0, FLAGS },
    { "rc", "set red contrast",   OFFSET(contrast[0]), AV_OPT_TYPE_INT, {.i64=2}, 0, 255, FLAGS },
    { "gc", "set green contrast", OFFSET(contrast[1]), AV_OPT_TYPE_INT, {.i64=7}, 0, 255, FLAGS },
    { "bc", "set blue contrast",  OFFSET(contrast[2]), AV_OPT_TYPE_INT, {.i64=1}, 0, 255, FLAGS },
    { "mpc", "set mean correlation color", OFFSET(mpc_str), AV_OPT_TYPE_STRING, {.str = "none"}, 0, 0, FLAGS },
    { "video", "set video output", OFFSET(do_video), AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(acorrmeter);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AudioCorrelationMeterContext *s = ctx->priv;
    AVFilterFormats *formats = NULL;
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_NONE };
    static const enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_RGBA, AV_PIX_FMT_NONE };
    static const AVChannelLayout layouts[] = {
        AV_CHANNEL_LAYOUT_STEREO,
        { .nb_channels = 0 },
    };
    int ret;

    formats = ff_make_format_list(sample_fmts);
    if ((ret = ff_formats_ref(formats, &cfg_in[0]->formats)) < 0 ||
        (ret = ff_formats_ref(formats, &cfg_out[0]->formats)) < 0)
        return ret;

    ret = ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, layouts);
    if (ret < 0)
        return ret;

    if (s->do_video) {
        formats = ff_make_format_list(pix_fmts);
        if ((ret = ff_formats_ref(formats, &cfg_out[1]->formats)) < 0)
            return ret;
    }

    return 0;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioCorrelationMeterContext *s = ctx->priv;
    float scale = 1.f;
    int ret;

    s->nb_samples = FFMAX(1, av_rescale(inlink->sample_rate, s->frame_rate.den, s->frame_rate.num));
    s->rdft_size = 1 << av_ceil_log2(s->nb_samples);

    ret = av_tx_init(&s->tx_ctx, &s->tx_fn, AV_TX_FLOAT_RDFT, 0, s->rdft_size, &scale, 0);
    if (ret < 0)
        return ret;

    for (int i = 0; i < 2; i++) {
        s->in_rdft[i] = av_calloc(s->rdft_size, sizeof(*s->in_rdft[0]));
        if (!s->in_rdft[i])
            return AVERROR(ENOMEM);

        s->out_rdft[i] = av_calloc(s->rdft_size+1, sizeof(*s->out_rdft[0]));
        if (!s->out_rdft[i])
            return AVERROR(ENOMEM);
    }

    return 0;
}

static int config_video_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioCorrelationMeterContext *s = ctx->priv;
    FilterLink *l = ff_filter_link(outlink);

    s->last_pts = AV_NOPTS_VALUE;

    outlink->w = s->w;
    outlink->h = s->h;
    outlink->sample_aspect_ratio = (AVRational){1,1};
    l->frame_rate = s->frame_rate;
    outlink->time_base = av_inv_q(l->frame_rate);

    if (!strcmp(s->mpc_str, "none"))
        s->draw_mean_correlation = 0;
    else if (av_parse_color(s->mpc, s->mpc_str, -1, ctx) >= 0)
        s->draw_mean_correlation = 1;
    else
        return AVERROR(EINVAL);

    return 0;
}

static inline int get_x(float correlation, int w)
{
  return correlation * (w - 1.f);
}

static inline void add_metadata(AVFrame *insamples, const char *key, char *value)
{
    char buf[128];

    snprintf(buf, sizeof(buf), "lavfi.acorrmeter.%s", key);
    av_dict_set(&insamples->metadata, buf, value, 0);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AudioCorrelationMeterContext *s = ctx->priv;
    AVFilterLink *outlink = s->do_video ? ctx->outputs[1] : NULL;
    AVFilterLink *aoutlink = ctx->outputs[0];
    AVDictionary **metadata;
    const int rc = s->contrast[0];
    const int gc = s->contrast[1];
    const int bc = s->contrast[2];
    float fcorrelation = 0;
    AVFrame *out;
    uint8_t *dst;
    int64_t new_pts;
    int ret;

    if (s->do_video && (!s->out || s->out->width  != outlink->w ||
                                   s->out->height != outlink->h)) {
        ff_graph_frame_free(ctx, &s->out);
        s->out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!s->out) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        out = s->out;
        for (int i = 0; i < outlink->h; i++)
            memset(out->data[0] + i * out->linesize[0], 0, outlink->w * 4);
    } else if (s->do_video) {
        ret = ff_inlink_make_frame_writable(outlink, &s->out);
        if (ret < 0)
            goto fail;
        out = s->out;
        for (int i = outlink->h - 1; i >= 10; i--)
            memmove(out->data[0] + (i  ) * out->linesize[0],
                    out->data[0] + (i-1) * out->linesize[0],
                    outlink->w * 4);
        for (int i = 0; i < outlink->w; i++)
            AV_WL32(out->data[0] + i * 4, 0);
    }

    for (int i = 0; i < 2; i++) {
        memset(s->in_rdft[i], 0, s->rdft_size * sizeof(float));
        memcpy(s->in_rdft[i], in->extended_data[i], s->nb_samples * sizeof(float));

        s->tx_fn(s->tx_ctx, s->out_rdft[i], s->in_rdft[i], sizeof(float));
    }

    for (int i = 0; i < s->rdft_size; i++) {
        AVComplexFloat lx = s->out_rdft[0][i];
        AVComplexFloat rx = s->out_rdft[1][i];
        const float re = lx.re * rx.re + lx.im * rx.im;
        const float im = lx.re * rx.im - lx.im * rx.re;
        const float correlation = 1.f-fabsf(atan2f(im, re) / M_PIf);
        const int x = get_x(correlation, s->w);

        if (s->do_video) {
            dst = out->data[0] + x * 4;
            dst[0] = FFMIN(255, dst[0] + rc);
            dst[1] = FFMIN(255, dst[1] + gc);
            dst[2] = FFMIN(255, dst[2] + bc);
            dst[3] = 255;
        }
        fcorrelation += correlation;
    }
    fcorrelation /= s->rdft_size;
    s->correlation = fcorrelation;

    if (s->do_video) {
        if (s->draw_mean_correlation) {
            dst = out->data[0] + get_x(fcorrelation, s->w) * 4;
            AV_WL32(dst, AV_RL32(s->mpc));
        }

        for (int i = 1; i < 10 && i < outlink->h; i++)
            memcpy(out->data[0] + i * out->linesize[0], out->data[0], outlink->w * 4);
    }

    metadata = &in->metadata;
    if (metadata) {
        uint8_t value[128];

        snprintf(value, sizeof(value), "%f", fcorrelation);
        add_metadata(in, "correlation", value);
    }

    if (s->do_video)
        new_pts = av_rescale_q(in->pts, inlink->time_base, outlink->time_base);
    if (s->do_video && new_pts != s->last_pts) {
        AVFrame *clone;

        s->out->pts = s->last_pts = new_pts;
        s->out->duration = 1;

        clone = ff_graph_frame_clone(ctx, s->out);
        if (!clone) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        ret = ff_filter_frame(outlink, clone);
        if (ret < 0)
            goto fail;
    }
    s->in = NULL;
    return ff_filter_frame(aoutlink, in);
fail:
    ff_graph_frame_free(ctx, &in);
    s->in = NULL;
    return ret;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AudioCorrelationMeterContext *s = ctx->priv;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);
    if (s->do_video)
        FF_FILTER_FORWARD_STATUS_BACK(ctx->outputs[1], inlink);

    if (!s->in) {
        ret = ff_inlink_consume_samples(inlink, s->nb_samples, s->nb_samples, &s->in);
        if (ret < 0)
            return ret;
        if (ret > 0)
            return filter_frame(inlink, s->in);
    }

    FF_FILTER_FORWARD_STATUS_ALL(inlink, ctx);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);
    if (s->do_video)
        FF_FILTER_FORWARD_WANTED(ctx->outputs[1], inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioCorrelationMeterContext *s = ctx->priv;

    for (int i = 0; i < 2; i++) {
        av_freep(&s->in_rdft[i]);
        av_freep(&s->out_rdft[i]);
    }
    av_tx_uninit(&s->tx_ctx);
    av_frame_free(&s->out);
}

static av_cold int init(AVFilterContext *ctx)
{
    AudioCorrelationMeterContext *s = ctx->priv;

    if (s->do_video) {
        AVFilterPad pad;

        pad = (AVFilterPad){
            .name         = "out1",
            .type         = AVMEDIA_TYPE_VIDEO,
            .config_props = config_video_output,
        };
        return ff_append_outpad(ctx, &pad);
    }

    return 0;
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_avf_acorrmeter = {
    .p.name        = "acorrmeter",
    .p.description = NULL_IF_CONFIG_SMALL("Convert input audio to correlation meter video output."),
    .p.priv_class  = &acorrmeter_class,
    .init          = init,
    .activate      = activate,
    .uninit        = uninit,
    .priv_size     = sizeof(AudioCorrelationMeterContext),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags       = AVFILTER_FLAG_DYNAMIC_OUTPUTS,
};
