/*
 * Copyright (c) Paul B Mahol
 * Copyright (c) Laurent de Soras, 2005
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

#include "libavutil/channel_layout.h"
#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"

#define MAX_NB_COEFFS 16

typedef struct AFreqShift {
    const AVClass *class;

    double shift;
    double old_shift;
    double level;
    int nb_coeffs;
    int old_nb_coeffs;

    double cd[MAX_NB_COEFFS * 2];
    float cf[MAX_NB_COEFFS * 2];

    double k1, k2;

    void *state;

    int (*init_state)(AVFilterContext *ctx, int channels);
    void (*filter_channel)(AVFilterContext *ctx,
                           int channel,
                           AVFrame *in, AVFrame *out);
} AFreqShift;

static const enum AVSampleFormat sample_fmts[] = {
    AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_NONE
};

static void compute_transition_param(double *K, double *Q, double transition)
{
    double kksqrt, e, e2, e4, k, q;

    k  = tan((1. - transition * 2.) * M_PI / 4.);
    k *= k;
    kksqrt = pow(1 - k * k, 0.25);
    e = 0.5 * (1. - kksqrt) / (1. + kksqrt);
    e2 = e * e;
    e4 = e2 * e2;
    q = e * (1. + e4 * (2. + e4 * (15. + 150. * e4)));

    *Q = q;
    *K = k;
}

static double ipowp(double x, int64_t n)
{
    double z = 1.;

    while (n != 0) {
        if (n & 1)
            z *= x;
        n >>= 1;
        x *= x;
    }

    return z;
}

static double compute_acc_num(double q, int order, int c)
{
    int64_t i = 0;
    int j = 1;
    double acc = 0.;
    double q_ii1;

    do {
        q_ii1  = ipowp(q, i * (i + 1));
        q_ii1 *= sin((i * 2 + 1) * c * M_PI / order) * j;
        acc   += q_ii1;

        j = -j;
        i++;
    } while (fabs(q_ii1) > 1e-100);

    return acc;
}

static double compute_acc_den(double q, int order, int c)
{
    int64_t i = 1;
    int j = -1;
    double acc = 0.;
    double q_i2;

    do {
        q_i2  = ipowp(q, i * i);
        q_i2 *= cos(i * 2 * c * M_PI / order) * j;
        acc  += q_i2;

        j = -j;
        i++;
    } while (fabs(q_i2) > 1e-100);

    return acc;
}

static double compute_coef(int index, double k, double q, int order)
{
    const int    c    = index + 1;
    const double num  = compute_acc_num(q, order, c) * pow(q, 0.25);
    const double den  = compute_acc_den(q, order, c) + 0.5;
    const double ww   = num / den;
    const double wwsq = ww * ww;

    const double x    = sqrt((1 - wwsq * k) * (1 - wwsq / k)) / (1 + wwsq);
    const double coef = (1 - x) / (1 + x);

    return coef;
}

static void compute_coefs(double *coef_arrd, float *coef_arrf, int nbr_coefs,
                          double transition)
{
    const int order = nbr_coefs*2+1;
    const int N = nbr_coefs/2;
    double k, q;

    compute_transition_param(&k, &q, transition);

    for (int n = 0; n < N; n++) {
        coef_arrd[n+0] = compute_coef(2*n+0, k, q, order);
        coef_arrf[n+0] = coef_arrd[n+0];
        coef_arrd[n+N] = compute_coef(2*n+1, k, q, order);
        coef_arrf[n+N] = coef_arrd[n+N];
    }
}

static void compute_osc(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AFreqShift *s = ctx->priv;
    double w0 = 2.0*M_PI*fmod(s->shift/inlink->sample_rate, 0.49999);

    s->k1 = tan(0.5*w0);
    s->k2 = sin(w0);
    s->old_shift = s->shift;
}

#define DEPTH 32
#include "afreqshift_template.c"

#undef DEPTH
#define DEPTH 64
#include "afreqshift_template.c"

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AFreqShift *s = ctx->priv;
    int ret;

    if (s->old_nb_coeffs != s->nb_coeffs)
        compute_coefs(s->cd, s->cf, s->nb_coeffs * 2, 2. * 20. / inlink->sample_rate);
    s->old_nb_coeffs = s->nb_coeffs;

    if (s->old_shift != s->shift)
        compute_osc(ctx);

    if (inlink->format == AV_SAMPLE_FMT_DBLP) {
        s->init_state = init_state_dblp;
        if (!strcmp(ctx->filter->name, "afreqshift"))
            s->filter_channel = ffilter_channel_dblp;
        else
            s->filter_channel = pfilter_channel_dblp;
    } else {
        s->init_state = init_state_fltp;
        if (!strcmp(ctx->filter->name, "afreqshift"))
            s->filter_channel = ffilter_channel_fltp;
        else
            s->filter_channel = pfilter_channel_fltp;
    }

    ret = s->init_state(ctx, inlink->ch_layout.nb_channels);
    if (ret < 0)
        return ret;

    return 0;
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

static int filter_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AFreqShift *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->filter_channel(ctx, ch, in, out);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AFreqShift *s = ctx->priv;
    AVFrame *out;
    ThreadData td;

    if (s->old_nb_coeffs != s->nb_coeffs)
        compute_coefs(s->cd, s->cf, s->nb_coeffs * 2, 2. * 20. / inlink->sample_rate);
    s->old_nb_coeffs = s->nb_coeffs;

    if (s->old_shift != s->shift)
        compute_osc(ctx);

    if (av_frame_is_writable(in)) {
        out = in;
    } else {
        out = ff_get_audio_buffer(outlink, in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    td.in = in; td.out = out;
    ff_filter_execute(ctx, filter_channels, &td, NULL,
                      FFMIN(inlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    if (out != in)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AFreqShift *s = ctx->priv;

    av_freep(&s->state);
}

#define OFFSET(x) offsetof(AFreqShift, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption afreqshift_options[] = {
    { "shift", "set frequency shift", OFFSET(shift), AV_OPT_TYPE_DOUBLE, {.dbl=0}, -INT_MAX, INT_MAX, FLAGS },
    { "level", "set output level",    OFFSET(level), AV_OPT_TYPE_DOUBLE, {.dbl=1},      0.0,     1.0, FLAGS },
    { "order", "set filter order",    OFFSET(nb_coeffs),AV_OPT_TYPE_INT, {.i64=8},  1, MAX_NB_COEFFS, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(afreqshift);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_afreqshift = {
    .p.name          = "afreqshift",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply frequency shifting to input audio."),
    .p.priv_class    = &afreqshift_class,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                       AVFILTER_FLAG_SLICE_THREADS,
    .priv_size       = sizeof(AFreqShift),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS_ARRAY(sample_fmts),
    .process_command = ff_filter_process_command,
};

static const AVOption aphaseshift_options[] = {
    { "shift", "set phase shift", OFFSET(shift), AV_OPT_TYPE_DOUBLE, {.dbl=0}, -1.0, 1.0, FLAGS },
    { "level", "set output level",OFFSET(level), AV_OPT_TYPE_DOUBLE, {.dbl=1},  0.0, 1.0, FLAGS },
    { "order", "set filter order",OFFSET(nb_coeffs), AV_OPT_TYPE_INT,{.i64=8},    1, MAX_NB_COEFFS, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aphaseshift);

const FFFilter ff_af_aphaseshift = {
    .p.name          = "aphaseshift",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply phase shifting to input audio."),
    .p.priv_class    = &aphaseshift_class,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                       AVFILTER_FLAG_SLICE_THREADS,
    .priv_size       = sizeof(AFreqShift),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS_ARRAY(sample_fmts),
    .process_command = ff_filter_process_command,
};
