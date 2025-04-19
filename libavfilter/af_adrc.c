/*
 * Copyright (c) 2022 Paul B Mahol
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

#include "libavutil/eval.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"

static const char * const var_names[] = {
    "ch",           ///< the value of the current channel
    "sn",           ///< number of samples
    "nb_channels",
    "t",            ///< timestamp expressed in seconds
    "sr",           ///< sample rate
    "p",            ///< input power in dB for frequency bin
    "f",            ///< frequency in Hz
    "th",           ///< current threshold
    NULL
};

enum var_name {
    VAR_CH,
    VAR_SN,
    VAR_NB_CHANNELS,
    VAR_T,
    VAR_SR,
    VAR_P,
    VAR_F,
    VAR_TH,
    VAR_VARS_NB
};

typedef struct AudioDRCContext {
    const AVClass *class;

    double attack_ms;
    double release_ms;
    char *transfer_str;
    char *threshold_str;

    double attack;
    double release;

    int fft_size;
    int overlap;
    int channels;

    float fx;
    void *window;

    AVFrame *drc_frame;
    AVFrame *energy;
    AVFrame *envelope;
    AVFrame *factors;
    AVFrame *in;
    AVFrame *in_buffer;
    AVFrame *in_frame;
    AVFrame *out_dist_frame;
    AVFrame *spectrum_buf;
    AVFrame *target_gain;
    AVFrame *threshold;
    AVFrame *windowed_frame;

    AVChannelLayout ch_layout;

    AVTXContext **tx_ctx;
    av_tx_fn tx_fn;
    AVTXContext **itx_ctx;
    av_tx_fn itx_fn;

    AVExpr *transfer_expr, *threshold_expr;
    double var_values[VAR_VARS_NB];

    void (*generate_window)(void *window, int size);
    int (*drc_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, int ch);
} AudioDRCContext;

#define OFFSET(x) offsetof(AudioDRCContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption adrc_options[] = {
    { "transfer",  "set the transfer expression", OFFSET(transfer_str),  AV_OPT_TYPE_STRING, {.str="p"},   0,    0, FLAGS },
    { "threshold", "set the threshold expression",OFFSET(threshold_str), AV_OPT_TYPE_STRING, {.str="-30"}, 0,    0, FLAGS },
    { "attack",    "set the attack",              OFFSET(attack_ms),     AV_OPT_TYPE_DOUBLE, {.dbl=50.}, 0.1, 1000, FLAGS },
    { "release",   "set the release",             OFFSET(release_ms),    AV_OPT_TYPE_DOUBLE, {.dbl=100.},0.1, 2000, FLAGS },
    { "channels",  "set channels to filter",      OFFSET(ch_layout),   AV_OPT_TYPE_CHLAYOUT, {.str="24c"}, 0,    0, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(adrc);

#define DEPTH 32
#include "adrc_template.c"

#undef DEPTH
#define DEPTH 64
#include "adrc_template.c"

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioDRCContext *s = ctx->priv;
    union { double d; float f; } scale, iscale;
    enum AVTXType tx_type;
    size_t sample_size;
    int ret;

    s->fft_size = outlink->sample_rate > 100000 ? 1024 : outlink->sample_rate > 50000 ? 512 : 256;
    s->fx = outlink->sample_rate * 0.5f / (s->fft_size + 1);
    s->overlap = s->fft_size / 4;

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        scale.f  = 1.f / (s->fft_size * 2);
        iscale.f = 1.f / 1.5f;
        tx_type  = AV_TX_FLOAT_RDFT;
        sample_size = sizeof(float);
        s->generate_window = generate_hann_window_fltp;
        s->drc_channel = drc_channel_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        scale.d  = 1.0 / (s->fft_size * 2);
        iscale.d = 1.0 / 1.5;
        tx_type  = AV_TX_DOUBLE_RDFT;
        sample_size = sizeof(double);
        s->generate_window = generate_hann_window_dblp;
        s->drc_channel = drc_channel_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    s->window = av_calloc(s->fft_size, sample_size);
    if (!s->window)
        return AVERROR(ENOMEM);

    s->drc_frame      = ff_get_audio_buffer(outlink, s->fft_size * 2 + 2);
    s->energy         = ff_get_audio_buffer(outlink, s->fft_size + 1);
    s->envelope       = ff_get_audio_buffer(outlink, s->fft_size + 1);
    s->factors        = ff_get_audio_buffer(outlink, s->fft_size + 1);
    s->in_buffer      = ff_get_audio_buffer(outlink, s->fft_size * 2 + 2);
    s->in_frame       = ff_get_audio_buffer(outlink, s->fft_size * 2 + 2);
    s->out_dist_frame = ff_get_audio_buffer(outlink, s->fft_size * 2 + 2);
    s->spectrum_buf   = ff_get_audio_buffer(outlink, s->fft_size * 2 + 2);
    s->target_gain    = ff_get_audio_buffer(outlink, s->fft_size + 1);
    s->threshold      = ff_get_audio_buffer(outlink, s->fft_size + 1);
    s->windowed_frame = ff_get_audio_buffer(outlink, s->fft_size * 2 + 2);
    if (!s->in_buffer || !s->in_frame || !s->target_gain || !s->threshold ||
        !s->out_dist_frame || !s->windowed_frame || !s->envelope ||
        !s->drc_frame || !s->spectrum_buf || !s->energy || !s->factors)
        return AVERROR(ENOMEM);

    s->generate_window(s->window, s->fft_size);

    s->channels = outlink->ch_layout.nb_channels;

    s->tx_ctx = av_calloc(s->channels, sizeof(*s->tx_ctx));
    s->itx_ctx = av_calloc(s->channels, sizeof(*s->itx_ctx));
    if (!s->tx_ctx || !s->itx_ctx)
        return AVERROR(ENOMEM);

    for (int ch = 0; ch < s->channels; ch++) {
        switch (outlink->format) {
        case AV_SAMPLE_FMT_FLTP: {
            float *thr = (float *)s->threshold->extended_data[ch];

            for (int n = 0; n < s->threshold->nb_samples; n++)
                thr[n] = -321.f;
                                 }
            break;
        case AV_SAMPLE_FMT_DBLP: {
            double *thr = (double *)s->threshold->extended_data[ch];

            for (int n = 0; n < s->threshold->nb_samples; n++)
                thr[n] = -321.0;
                                 }
            break;
        }

        ret = av_tx_init(&s->tx_ctx[ch], &s->tx_fn, tx_type, 0, s->fft_size * 2, &scale, 0);
        if (ret < 0)
            return ret;

        ret = av_tx_init(&s->itx_ctx[ch], &s->itx_fn, tx_type, 1, s->fft_size * 2, &iscale, 0);
        if (ret < 0)
            return ret;
    }

    s->var_values[VAR_SR] = outlink->sample_rate;
    s->var_values[VAR_NB_CHANNELS] = s->channels;

    ret = av_expr_parse(&s->threshold_expr, s->threshold_str, var_names,
                        NULL, NULL, NULL, NULL, 0, ctx);
    if (ret < 0)
        return ret;
    return av_expr_parse(&s->transfer_expr, s->transfer_str, var_names,
                         NULL, NULL, NULL, NULL, 0, ctx);
}

static int drc_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioDRCContext *s = ctx->priv;
    AVFrame *in = s->in;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->drc_channel(ctx, in, out, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    FilterLink *outl = ff_filter_link(outlink);
    AudioDRCContext *s = ctx->priv;
    AVFrame *out;
    int ret;

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    s->var_values[VAR_SN] = outl->sample_count_in;
    s->var_values[VAR_T] = s->var_values[VAR_SN] / outlink->sample_rate;

    s->in = in;
    av_frame_copy_props(out, in);
    ff_filter_execute(ctx, drc_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    out->pts = in->pts;
    out->pts -= av_rescale_q(s->fft_size - s->overlap, av_make_q(1, outlink->sample_rate), outlink->time_base);
    out->nb_samples = in->nb_samples;
    ret = ff_filter_frame(outlink, out);
fail:
    av_frame_free(&in);
    s->in = NULL;
    return ret < 0 ? ret : 0;
}

static double get_coef(double x, double sr)
{
    return exp(-1.0 / (0.001 * x * sr));
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AudioDRCContext *s = ctx->priv;
    int ret, available, wanted;
    AVFrame *in = NULL;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    available = ff_inlink_queued_samples(inlink);
    wanted = FFMAX(s->overlap, (available / s->overlap) * s->overlap);
    ret = ff_inlink_consume_samples(inlink, wanted, wanted, &in);
    if (ret < 0)
        return ret;

    if (ret > 0) {
        s->attack  = get_coef(s->attack_ms, inlink->sample_rate);
        s->release = get_coef(s->release_ms, inlink->sample_rate);

        return filter_frame(inlink, in);
    }

    if (ff_inlink_queued_samples(inlink) >= s->overlap) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioDRCContext *s = ctx->priv;

    av_expr_free(s->transfer_expr);
    s->transfer_expr = NULL;

    av_expr_free(s->threshold_expr);
    s->threshold_expr = NULL;

    av_freep(&s->window);

    av_frame_free(&s->drc_frame);
    av_frame_free(&s->energy);
    av_frame_free(&s->envelope);
    av_frame_free(&s->factors);
    av_frame_free(&s->in_buffer);
    av_frame_free(&s->in_frame);
    av_frame_free(&s->out_dist_frame);
    av_frame_free(&s->spectrum_buf);
    av_frame_free(&s->target_gain);
    av_frame_free(&s->threshold);
    av_frame_free(&s->windowed_frame);

    for (int ch = 0; ch < s->channels; ch++) {
        if (s->tx_ctx)
            av_tx_uninit(&s->tx_ctx[ch]);
        if (s->itx_ctx)
            av_tx_uninit(&s->itx_ctx[ch]);
    }

    av_freep(&s->tx_ctx);
    av_freep(&s->itx_ctx);
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *args,
                           char *res, int res_len, int flags)
{
    AudioDRCContext *s = ctx->priv;
    char *old_transfer_str = av_strdup(s->transfer_str);
    char *old_threshold_str = av_strdup(s->threshold_str);
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret >= 0 && strcmp(old_transfer_str, s->transfer_str)) {
        ret = av_expr_parse(&s->transfer_expr, s->transfer_str, var_names,
                            NULL, NULL, NULL, NULL, 0, ctx);
    }
    if (ret >= 0 && strcmp(old_threshold_str, s->threshold_str)) {
        ret = av_expr_parse(&s->threshold_expr, s->threshold_str, var_names,
                            NULL, NULL, NULL, NULL, 0, ctx);
    }
    av_free(old_transfer_str);
    av_free(old_threshold_str);
    return ret;
}

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_adrc = {
    .p.name          = "adrc",
    .p.description   = NULL_IF_CONFIG_SMALL("Audio Spectral Dynamic Range Controller."),
    .p.priv_class    = &adrc_class,
    .priv_size       = sizeof(AudioDRCContext),
    .uninit          = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_FLTP),
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_SLICE_THREADS,
    .activate        = activate,
    .process_command = process_command,
};
