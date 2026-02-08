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

#include <float.h>

#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "libavutil/mem.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct BinauralizerContext {
    const AVClass *class;

    double gain;

    double *x_size;
    unsigned nb_x_size;

    double *y_size;
    unsigned nb_y_size;

    double *x_offset;
    unsigned nb_x_offset;

    double *y_offset;
    unsigned nb_y_offset;

    double *i_gain;
    unsigned nb_i_gain;

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

    int (*ba_stereo)(AVFilterContext *ctx, AVFrame *out);
    int (*ba_flush)(AVFilterContext *ctx, AVFrame *out);

    AVTXContext *tx_ctx, *itx_ctx;
    av_tx_fn tx_fn, itx_fn;
} BinauralizerContext;

#define OFFSET(x) offsetof(BinauralizerContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_size  = {.def="1", .size_min=1, .sep=' '};
static const AVOptionArrayDef def_offset= {.def="0", .size_min=1, .sep=' '};

static const AVOption binauralizer_options[] = {
    { "gain",  "set the output gain",               OFFSET(gain),     AV_OPT_TYPE_DOUBLE,    {.dbl=1.},           0,10, FLAGS },
    { "xsize", "X-axis channel size values",        OFFSET(x_size),   AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_size},    0, 1, FLAGS },
    { "ysize", "Y-axis channel size values",        OFFSET(y_size),   AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_size},    0, 1, FLAGS },
    { "xoff",  "X-axis channel offset values",      OFFSET(x_offset), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_offset}, -1, 1, FLAGS },
    { "yoff",  "Y-axis channel offset values",      OFFSET(y_offset), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_offset}, -1, 1, FLAGS },
    { "igain", "set the input channel gain values", OFFSET(i_gain),   AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_size},    0,10, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(binauralizer);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVSampleFormat formats[] = {
        AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE,
    };
    AVFilterChannelLayouts *layouts = NULL;
    int ret;

    ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, formats);
    if (ret < 0)
        return ret;

    layouts = ff_all_channel_layouts();
    if (!layouts)
        return AVERROR(ENOMEM);

    ret = ff_channel_layouts_ref(layouts, &cfg_in[0]->channel_layouts);
    if (ret)
        return ret;

    layouts = NULL;
    ret = ff_add_channel_layout(&layouts, &(AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO);
    if (ret)
        return ret;

    return ff_channel_layouts_ref(layouts, &cfg_out[0]->channel_layouts);
}

#define DEPTH 32
#include "binauralizer_template.c"

#undef DEPTH
#define DEPTH 64
#include "binauralizer_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    BinauralizerContext *s = ctx->priv;
    int ret;

    s->fft_size = 1 << av_ceil_log2((inlink->sample_rate + 19) / 20);
    s->overlap = (s->fft_size + 3) / 4;
    s->trim_size = s->fft_size - s->overlap;
    s->flush_size = s->fft_size - s->overlap;

    s->in_frame       = ff_get_audio_buffer(inlink, s->fft_size + 2);
    s->out_dist_frame = ff_get_audio_buffer(outlink, s->fft_size * 2);
    s->windowed_frame = ff_get_audio_buffer(outlink, s->fft_size + 2);
    s->windowed_out   = ff_get_audio_buffer(outlink, s->fft_size + 2);
    if (!s->in_frame || !s->windowed_out || !s->out_dist_frame || !s->windowed_frame)
        return AVERROR(ENOMEM);

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->ba_stereo = ba_stereo_float;
        s->ba_flush = ba_flush_float;
        ret = ba_tx_init_float(ctx);
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->ba_stereo = ba_stereo_double;
        s->ba_flush = ba_flush_double;
        ret = ba_tx_init_double(ctx);
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
    BinauralizerContext *s = ctx->priv;
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
    s->ba_stereo(ctx, out);

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
        ff_graph_frame_free(ctx, &out);
        ff_graph_frame_free(ctx, &in);

        ff_inlink_request_frame(inlink);

        return 0;
    }

    s->in = NULL;
    ff_graph_frame_free(ctx, &in);
    return ff_filter_frame(outlink, out);
}

static int flush_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    BinauralizerContext *s = ctx->priv;
    int ret = 0;

    while (s->flush_size > 0) {
        const int nb_samples = FFMIN(s->flush_size, s->overlap);
        AVFrame *out = ff_get_audio_buffer(outlink, nb_samples);

        if (!out)
            return AVERROR(ENOMEM);

        s->flush_size -= nb_samples;

        s->ba_flush(ctx, out);

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
    BinauralizerContext *s = ctx->priv;
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
    BinauralizerContext *s = ctx->priv;

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

const FFFilter ff_af_binauralizer = {
    .p.name          = "binauralizer",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply Binauralizer effect."),
    .p.priv_class    = &binauralizer_class,
    .priv_size       = sizeof(BinauralizerContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .activate        = activate,
    .process_command = ff_filter_process_command,
};
