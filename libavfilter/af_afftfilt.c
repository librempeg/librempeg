/*
 * Copyright (c) 2016 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; either version 2.1 of the License,
 * or (at your option) any later version.
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

#include "libavutil/mem.h"
#include "libavutil/common.h"
#include "libavutil/cpu.h"
#include "libavutil/opt.h"
#include "libavutil/eval.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "filters.h"
#include "window_func.h"

typedef struct AFFTFiltContext {
    const AVClass *class;
    char **real_str;
    unsigned nb_real_str;
    char **imag_str;
    unsigned nb_imag_str;
    int tx_size;

    int trim_size;
    int flush_size;
    int64_t last_pts;

    AVTXContext **tx, **itx;
    av_tx_fn tx_fn, itx_fn;
    AVFrame *tx_in, *tx_out, *tx_temp;
    int nb_exprs;
    int channels;
    int win_size;
    AVExpr **real;
    AVExpr **imag;
    int hop_size;
    float overlap;
    AVFrame *window;
    AVFrame *buffer;
    AVFrame *out;
    int win_func;
    double win_gain;
    float *window_func_lut;

    int (*tx_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
} AFFTFiltContext;

static const char *const var_names[] = {            "sr",     "b",       "nb",        "ch",        "chs",   "pts",     "re",     "im", NULL };
enum                                   { VAR_SAMPLE_RATE, VAR_BIN, VAR_NBBINS, VAR_CHANNEL, VAR_CHANNELS, VAR_PTS, VAR_REAL, VAR_IMAG, VAR_VARS_NB };

#define OFFSET(x) offsetof(AFFTFiltContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_real = {.def="re",.size_min=1,.sep='|'};
static const AVOptionArrayDef def_imag = {.def="im",.size_min=1,.sep='|'};

static const AVOption afftfilt_options[] = {
    { "real", "set channels real expressions",       OFFSET(real_str), AV_OPT_TYPE_STRING|AR, {.arr = &def_real }, 0, 0, A },
    { "imag", "set channels imaginary expressions",  OFFSET(imag_str), AV_OPT_TYPE_STRING|AR, {.arr = &def_imag }, 0, 0, A },
    { "win_size", "set window size", OFFSET(tx_size), AV_OPT_TYPE_INT, {.i64=4096}, 16, 131072, A },
    WIN_FUNC_OPTION("win_func", OFFSET(win_func), A, WFUNC_HANNING),
    { "overlap", "set window overlap", OFFSET(overlap), AV_OPT_TYPE_FLOAT, {.dbl=0.75}, 0,  1, A },
    { NULL },
};

AVFILTER_DEFINE_CLASS(afftfilt);

static const char *const func2_names[]    = { "real", "imag", NULL };

#define DEPTH 32
#include "afftfilt_template.c"

#undef DEPTH
#define DEPTH 64
#include "afftfilt_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AFFTFiltContext *s = ctx->priv;
    enum AVTXType tx_type;
    float overlap;
    float scale_float = 1.f;
    double scale_double = 1.0;
    float iscale_float = 1.f;
    double iscale_double = 1.0;
    void *scale_ptr;
    void *iscale_ptr;
    int buf_size, ret = 0;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        scale_ptr = &scale_float;
        iscale_ptr = &iscale_float;
        tx_type = AV_TX_FLOAT_RDFT;
        s->tx_channels = tx_channels_float;
        s->filter_channels = filter_channels_float;
        break;
    case AV_SAMPLE_FMT_DBLP:
        scale_ptr = &scale_double;
        iscale_ptr = &iscale_double;
        tx_type = AV_TX_DOUBLE_RDFT;
        s->tx_channels = tx_channels_double;
        s->filter_channels = filter_channels_double;
        break;
    default:
        return AVERROR_BUG;
    }

    s->channels = inlink->ch_layout.nb_channels;
    s->tx  = av_calloc(s->channels, sizeof(*s->tx));
    s->itx = av_calloc(s->channels, sizeof(*s->itx));
    if (!s->tx || !s->itx)
        return AVERROR(ENOMEM);

    s->win_size = s->tx_size;
    buf_size = FFALIGN(s->win_size + 2, av_cpu_max_align());

    s->tx_in = ff_get_audio_buffer(inlink, buf_size);
    s->tx_out = ff_get_audio_buffer(inlink, buf_size);
    s->tx_temp = ff_get_audio_buffer(inlink, buf_size);
    if (!s->tx_in || !s->tx_out || !s->tx_temp)
        return AVERROR(ENOMEM);

    s->real = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->real));
    if (!s->real)
        return AVERROR(ENOMEM);

    s->imag = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->imag));
    if (!s->imag)
        return AVERROR(ENOMEM);

    for (int ch = 0; ch < inlink->ch_layout.nb_channels; ch++) {
        const unsigned idx = FFMIN(ch, s->nb_real_str-1);
        const char *arg = s->real_str[idx];

        ret = av_expr_parse(&s->real[ch], arg, var_names,
                            NULL, NULL, func2_names,
                            inlink->format == AV_SAMPLE_FMT_FLTP ? func2_float : func2_double,
                            0, ctx);
        if (ret < 0)
            goto fail;
        s->nb_exprs++;
    }

    for (int ch = 0; ch < inlink->ch_layout.nb_channels; ch++) {
        const unsigned idx = FFMIN(ch, s->nb_imag_str-1);
        const char *arg = s->imag_str[idx];

        ret = av_expr_parse(&s->imag[ch], arg, var_names,
                            NULL, NULL, func2_names,
                            inlink->format == AV_SAMPLE_FMT_FLTP ? func2_float : func2_double,
                            0, ctx);
        if (ret < 0)
            goto fail;
    }

    s->window_func_lut = av_realloc_f(s->window_func_lut, s->win_size,
                                      sizeof(*s->window_func_lut));
    if (!s->window_func_lut)
        return AVERROR(ENOMEM);
    generate_window_func(s->window_func_lut, s->win_size, s->win_func, &overlap);
    if (s->overlap == 1)
        s->overlap = overlap;

    s->hop_size = FFMAX(1, s->win_size * (1.f - s->overlap));
    s->trim_size = s->tx_size - s->hop_size;
    s->flush_size = s->tx_size - s->hop_size;

    s->window = ff_get_audio_buffer(inlink, s->win_size + 2);
    if (!s->window)
        return AVERROR(ENOMEM);

    s->buffer = ff_get_audio_buffer(inlink, s->win_size * 2);
    if (!s->buffer)
        return AVERROR(ENOMEM);

    {
        float max = 0.f, *temp_lut = av_calloc(s->win_size, sizeof(*temp_lut));
        if (!temp_lut)
            return AVERROR(ENOMEM);

        for (int j = 0; j < s->win_size; j += s->hop_size) {
            for (int i = 0; i < s->win_size; i++)
                temp_lut[(i + j) % s->win_size] += s->window_func_lut[i];
        }

        for (int i = 0; i < s->win_size; i++)
            max = fmaxf(temp_lut[i], max);
        av_freep(&temp_lut);

        s->win_gain = 1.0 / (max * s->win_size);
    }

    iscale_float  *= s->win_gain;
    iscale_double *= s->win_gain;

    for (int ch = 0; ch < s->channels; ch++) {
        ret = av_tx_init(&s->tx[ch], &s->tx_fn, tx_type, 0, s->tx_size, scale_ptr, 0);
        if (ret < 0)
            return ret;

        ret = av_tx_init(&s->itx[ch], &s->itx_fn, tx_type, 1, s->tx_size, iscale_ptr, 0);
        if (ret < 0)
            return ret;
    }

fail:
    return ret;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AFFTFiltContext *s = ctx->priv;
    const int win_size = s->win_size;
    int extra_samples, nb_samples;
    double values[VAR_VARS_NB];
    AVFrame *out;

    extra_samples = in->nb_samples % s->hop_size;
    if (extra_samples)
        extra_samples = FFMIN(s->hop_size - extra_samples, s->flush_size);
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

    values[VAR_PTS]         = in->pts;
    values[VAR_SAMPLE_RATE] = inlink->sample_rate;
    values[VAR_NBBINS]      = win_size / 2 + 1;
    values[VAR_CHANNELS]    = inlink->ch_layout.nb_channels;

    ff_filter_execute(ctx, s->tx_channels, in, NULL,
                      FFMIN(s->channels, ff_filter_get_nb_threads(ctx)));

    out->pts -= av_rescale_q(s->tx_size - s->hop_size, av_make_q(1, outlink->sample_rate), outlink->time_base);
    s->last_pts = out->pts + out->duration;
    s->out = out;
    av_frame_free(&in);

    ff_filter_execute(ctx, s->filter_channels, values, NULL,
                      FFMIN(s->channels, ff_filter_get_nb_threads(ctx)));
    s->out = NULL;

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

    return ff_filter_frame(outlink, out);
}

static int flush_frame(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AFFTFiltContext *s = ctx->priv;
    const int win_size = s->win_size;
    double values[VAR_VARS_NB];
    int ret = 0;

    values[VAR_SAMPLE_RATE] = inlink->sample_rate;
    values[VAR_NBBINS]      = win_size / 2 + 1;
    values[VAR_CHANNELS]    = inlink->ch_layout.nb_channels;

    while (s->flush_size > 0) {
        const int nb_samples = FFMIN(s->flush_size, s->hop_size);
        AVFrame *out = ff_get_audio_buffer(outlink, nb_samples);

        if (!out)
            return AVERROR(ENOMEM);

        s->flush_size -= nb_samples;

        values[VAR_PTS] = s->last_pts;

        ff_filter_execute(ctx, s->tx_channels, NULL, NULL,
                          FFMIN(s->channels, ff_filter_get_nb_threads(ctx)));

        out->pts = s->last_pts;
        out->duration = av_rescale_q(out->nb_samples,
                                     (AVRational){1, outlink->sample_rate},
                                     outlink->time_base);
        s->last_pts += out->duration;

        s->out = out;
        ff_filter_execute(ctx, s->filter_channels, values, NULL,
                          FFMIN(s->channels, ff_filter_get_nb_threads(ctx)));
        s->out = NULL;

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
    AFFTFiltContext *s = ctx->priv;
    AVFrame *in = NULL;
    int ret, status;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_samples(inlink, s->hop_size, s->hop_size, &in);
    if (ret < 0)
        return ret;

    if (ret > 0)
        return filter_frame(inlink, in);

    if (ff_inlink_queued_samples(inlink) >= s->hop_size) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        if (s->flush_size > 0)
            ret = flush_frame(inlink);

        ff_outlink_set_status(outlink, status, pts);
        return 0;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AFFTFiltContext *s = ctx->priv;

    for (int i = 0; i < s->channels; i++) {
        if (s->itx)
            av_tx_uninit(&s->itx[i]);
        if (s->tx)
            av_tx_uninit(&s->tx[i]);
    }

    av_freep(&s->tx);
    av_freep(&s->itx);

    av_frame_free(&s->tx_in);
    av_frame_free(&s->tx_out);
    av_frame_free(&s->tx_temp);

    for (int i = 0; i < s->nb_exprs; i++) {
        av_expr_free(s->real[i]);
        av_expr_free(s->imag[i]);
    }

    av_freep(&s->real);
    av_freep(&s->imag);
    av_frame_free(&s->buffer);
    av_frame_free(&s->window);
    av_freep(&s->window_func_lut);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_afftfilt = {
    .p.name          = "afftfilt",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply arbitrary expressions to samples in frequency domain."),
    .p.priv_class    = &afftfilt_class,
    .priv_size       = sizeof(AFFTFiltContext),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP),
    .activate        = activate,
    .uninit          = uninit,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_SLICE_THREADS,
};
