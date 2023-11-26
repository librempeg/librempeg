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

#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "internal.h"
#include "window_func.h"

#include <float.h>

typedef struct AudioCenterCutContext {
    const AVClass *class;

    double factor;

    int fft_size;
    int overlap;

    float *window;

    AVFrame *in;
    AVFrame *in_frame;
    AVFrame *out_dist_frame;
    AVFrame *windowed_frame;
    AVFrame *windowed_out;

    AVTXContext *tx_ctx[2], *itx_ctx[2];
    av_tx_fn tx_fn, itx_fn;
} AudioCenterCutContext;

#define OFFSET(x) offsetof(AudioCenterCutContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption acentercut_options[] = {
    { "factor", "set the center cut factor", OFFSET(factor), AV_OPT_TYPE_DOUBLE, {.dbl=1}, 0, 1, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(acentercut);

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layout = NULL;
    int ret;

    if ((ret = ff_add_format                 (&formats, AV_SAMPLE_FMT_FLTP )) < 0 ||
        (ret = ff_set_common_formats         (ctx     , formats            )) < 0 ||
        (ret = ff_add_channel_layout         (&layout , &(AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO)) < 0 ||
        (ret = ff_channel_layouts_ref(layout, &ctx->inputs[0]->outcfg.channel_layouts)) < 0 ||
        (ret = ff_channel_layouts_ref(layout, &ctx->outputs[0]->incfg.channel_layouts)) < 0)
        return ret;

    return ff_set_common_all_samplerates(ctx);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioCenterCutContext *s = ctx->priv;
    float scale = 1.f, iscale, overlap;
    int ret;

    s->fft_size = inlink->sample_rate > 100000 ? 8192 : inlink->sample_rate > 50000 ? 4096 : 2048;
    s->overlap = s->fft_size / 4;

    s->window = av_calloc(s->fft_size, sizeof(*s->window));
    if (!s->window)
        return AVERROR(ENOMEM);

    s->in_frame       = ff_get_audio_buffer(inlink, (s->fft_size + 2) * 2);
    s->out_dist_frame = ff_get_audio_buffer(inlink, (s->fft_size + 2) * 2);
    s->windowed_frame = ff_get_audio_buffer(inlink, (s->fft_size + 2) * 2);
    s->windowed_out   = ff_get_audio_buffer(inlink, (s->fft_size + 2) * 2);
    if (!s->in_frame || !s->windowed_out || !s->out_dist_frame || !s->windowed_frame)
        return AVERROR(ENOMEM);

    generate_window_func(s->window, s->fft_size, WFUNC_HANNING, &overlap);

    iscale = 1.f / (s->fft_size * 1.5f);

    ret = av_tx_init(&s->tx_ctx[0], &s->tx_fn, AV_TX_FLOAT_RDFT, 0, s->fft_size, &scale, 0);
    if (ret < 0)
        return ret;

    ret = av_tx_init(&s->tx_ctx[1], &s->tx_fn, AV_TX_FLOAT_RDFT, 0, s->fft_size, &scale, 0);
    if (ret < 0)
        return ret;

    ret = av_tx_init(&s->itx_ctx[0], &s->itx_fn, AV_TX_FLOAT_RDFT, 1, s->fft_size, &iscale, 0);
    if (ret < 0)
        return ret;

    ret = av_tx_init(&s->itx_ctx[1], &s->itx_fn, AV_TX_FLOAT_RDFT, 1, s->fft_size, &iscale, 0);
    if (ret < 0)
        return ret;

    return 0;
}

static void apply_window(AudioCenterCutContext *s,
                         const float *in_frame, float *out_frame, const int add_to_out_frame)
{
    const float *window = s->window;
    const int fft_size = s->fft_size;

    if (add_to_out_frame) {
        for (int i = 0; i < fft_size; i++)
            out_frame[i] += in_frame[i] * window[i];
    } else {
        for (int i = 0; i < fft_size; i++)
            out_frame[i] = in_frame[i] * window[i];
    }
}

static float sqrf(float x)
{
    return x * x;
}

static void center_cut(AVComplexFloat *left, AVComplexFloat *right, const int N, const float factor)
{
    for (int i = 0; i < N; i++) {
        const float l_re = left[i].re;
        const float l_im = left[i].im;
        const float r_re = right[i].re;
        const float r_im = right[i].im;
        const float sum_re = 0.5f * (l_re + r_re);
        const float sum_im = 0.5f * (l_im + r_im);
        const float diff_re = 0.5f * (l_re - r_re);
        const float diff_im = 0.5f * (l_im - r_im);
        const float a = (1.f - sqrtf((sqrf(diff_re) + sqrf(diff_im))/
                                     (sqrf(sum_re) + sqrf(sum_im) + FLT_EPSILON))) * factor;
        float c_im, c_re;

        c_re = a * sum_re;
        c_im = a * sum_im;

        left[i].re -= c_re;
        left[i].im -= c_im;
        right[i].re -= c_re;
        right[i].im -= c_im;
    }
}

static int cc_stereo(AVFilterContext *ctx, AVFrame *out)
{
    AudioCenterCutContext *s = ctx->priv;
    float *left_in         = (float *)s->in_frame->extended_data[0];
    float *right_in        = (float *)s->in_frame->extended_data[1];
    float *left_out        = (float *)s->out_dist_frame->extended_data[0];
    float *right_out       = (float *)s->out_dist_frame->extended_data[1];
    float *left_samples    = (float *)s->in->extended_data[0];
    float *right_samples   = (float *)s->in->extended_data[1];
    float *windowed_left   = (float *)s->windowed_frame->extended_data[0];
    float *windowed_right  = (float *)s->windowed_frame->extended_data[1];
    float *windowed_oleft  = (float *)s->windowed_out->extended_data[0];
    float *windowed_oright = (float *)s->windowed_out->extended_data[1];
    float *left_osamples   = (float *)out->extended_data[0];
    float *right_osamples  = (float *)out->extended_data[1];
    const int overlap = s->overlap;
    const int offset = s->fft_size - overlap;
    const int nb_samples = FFMIN(overlap, s->in->nb_samples);
    const float factor = s->factor;

    // shift in/out buffers
    memmove(left_in, &left_in[overlap], offset * sizeof(float));
    memmove(right_in, &right_in[overlap], offset * sizeof(float));
    memmove(left_out, &left_out[overlap], offset * sizeof(float));
    memmove(right_out, &right_out[overlap], offset * sizeof(float));

    memcpy(&left_in[offset], left_samples, nb_samples * sizeof(float));
    memcpy(&right_in[offset], right_samples, nb_samples * sizeof(float));
    memset(&left_out[offset], 0, overlap * sizeof(float));
    memset(&right_out[offset], 0, overlap * sizeof(float));

    apply_window(s, left_in,  windowed_left,  0);
    apply_window(s, right_in, windowed_right, 0);

    s->tx_fn(s->tx_ctx[0], windowed_oleft,  windowed_left,  sizeof(float));
    s->tx_fn(s->tx_ctx[1], windowed_oright, windowed_right, sizeof(float));

    center_cut((AVComplexFloat *)windowed_oleft, (AVComplexFloat *)windowed_oright,
                s->fft_size / 2 + 1, factor);

    s->itx_fn(s->itx_ctx[0], windowed_left, windowed_oleft, sizeof(AVComplexFloat));
    s->itx_fn(s->itx_ctx[1], windowed_right, windowed_oright, sizeof(AVComplexFloat));

    apply_window(s, windowed_left, left_out,  1);
    apply_window(s, windowed_right, right_out,  1);

    if (ctx->is_disabled) {
        memcpy(left_osamples, left_in, overlap * sizeof(float));
        memcpy(right_osamples, right_in, overlap * sizeof(float));
    } else {
        memcpy(left_osamples, left_out, overlap * sizeof(float));
        memcpy(right_osamples, right_out, overlap * sizeof(float));
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioCenterCutContext *s = ctx->priv;
    AVFrame *out;
    int ret;

    out = ff_get_audio_buffer(outlink, s->overlap);
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    s->in = in;
    cc_stereo(ctx, out);

    av_frame_copy_props(out, in);
    out->pts -= av_rescale_q(s->fft_size - s->overlap, av_make_q(1, outlink->sample_rate), outlink->time_base);
    out->nb_samples = in->nb_samples;
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
    AudioCenterCutContext *s = ctx->priv;
    AVFrame *in = NULL;
    int ret = 0, status;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_samples(inlink, s->overlap, s->overlap, &in);
    if (ret < 0)
        return ret;

    if (ret > 0) {
        return filter_frame(inlink, in);
    } else if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        ff_outlink_set_status(outlink, status, pts);
        return 0;
    } else {
        if (ff_inlink_queued_samples(inlink) >= s->overlap) {
            ff_filter_set_ready(ctx, 10);
        } else if (ff_outlink_frame_wanted(outlink)) {
            ff_inlink_request_frame(inlink);
        }
        return 0;
    }
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioCenterCutContext *s = ctx->priv;

    av_freep(&s->window);

    av_frame_free(&s->in_frame);
    av_frame_free(&s->out_dist_frame);
    av_frame_free(&s->windowed_frame);
    av_frame_free(&s->windowed_out);

    av_tx_uninit(&s->tx_ctx[0]);
    av_tx_uninit(&s->tx_ctx[1]);
    av_tx_uninit(&s->itx_ctx[0]);
    av_tx_uninit(&s->itx_ctx[1]);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const AVFilter ff_af_acentercut = {
    .name            = "acentercut",
    .description     = NULL_IF_CONFIG_SMALL("Audio Center Cut."),
    .priv_size       = sizeof(AudioCenterCutContext),
    .priv_class      = &acentercut_class,
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC(query_formats),
    .flags           = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL,
    .activate        = activate,
    .process_command = ff_filter_process_command,
};
