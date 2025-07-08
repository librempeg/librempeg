/*
 * Copyright (c) 2005 Boðaç Topaktaþ
 * Copyright (c) 2020 Paul B Mahol
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

typedef struct BiquadCoeffs {
    double a[3], b[3];
} BiquadCoeffs;

typedef struct ASuperCutContext {
    const AVClass *class;

    double cutoff;
    double qfactor;
    int order;

    int filter_count;
    int bypass;

    void *st[10];

    int (*init_state)(AVFilterContext *ctx, void **st,
                      const int nb_channels, const int block_samples, const int reset,
                      const double a[3], const double b[3], const double mix);

    void (*filter)(void *st, const void *ibuf, void *obuf, int len,
                   int ch, int disabled);
} ASuperCutContext;

#define CLIP_RESET 0
#define BIQUAD_DI 0
#define BIQUAD_DII 0
#define BIQUAD_TDI 0
#define BIQUAD_TDII 1
#define BIQUAD_LATT 0
#define BIQUAD_SVF 0
#define BIQUAD_WDF 0
#define BIQUAD_ZDF 0

#undef DEPTH
#define DEPTH 32
#include "biquads_template.c"

#undef DEPTH
#define DEPTH 64
#include "biquads_template.c"

static const enum AVSampleFormat sample_fmts[] = {
    AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_NONE
};

static void calc_q_factors(int n, double *q)
{
    for (int i = 0; i < n / 2; i++)
        q[i] = 1. / (-2. * cos(M_PI * (2. * (i + 1) + n - 1.) / (2. * n)));
}

static int get_coeffs(AVFilterContext *ctx)
{
    ASuperCutContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    const int nb_channels = inlink->ch_layout.nb_channels;
    double w0 = s->cutoff / inlink->sample_rate;
    double K = tan(M_PI * w0);
    double q[10];

    s->bypass = w0 >= 0.5;
    if (s->bypass)
        return 0;

    if (!strcmp(ctx->filter->name, "asubcut")) {
        s->filter_count = s->order / 2 + (s->order & 1);

        calc_q_factors(s->order, q);

        if (s->order & 1) {
            double omega = 2. * tan(M_PI * w0);
            BiquadCoeffs c;

            c.b[0] = 2. / (2. + omega);
            c.b[1] = -c.b[0];
            c.b[2] = 0.;
            c.a[1] = -(omega - 2.) / (2. + omega);
            c.a[2] = 0.;

            s->init_state(ctx, &s->st[0], nb_channels, 0, 0, c.a, c.b, 1.0);
        }

        for (int b = (s->order & 1); b < s->filter_count; b++) {
            const int idx = b - (s->order & 1);
            double norm = 1.0 / (1.0 + K / q[idx] + K * K);
            BiquadCoeffs c;

            c.b[0] = norm;
            c.b[1] = -2.0 * c.b[0];
            c.b[2] = c.b[0];
            c.a[1] = 2.0 * (K * K - 1.0) * norm;
            c.a[2] = (1.0 - K / q[idx] + K * K) * norm;

            s->init_state(ctx, &s->st[b], nb_channels, 0, 0, c.a, c.b, 1.0);
        }
    } else if (!strcmp(ctx->filter->name, "asupercut")) {
        s->filter_count = s->order / 2 + (s->order & 1);

        calc_q_factors(s->order, q);

        if (s->order & 1) {
            double omega = 2. * tan(M_PI * w0);
            BiquadCoeffs c;

            c.b[0] = omega / (2. + omega);
            c.b[1] = c.b[0];
            c.b[2] = 0.;
            c.a[1] = -(omega - 2.) / (2. + omega);
            c.a[2] = 0.;

            s->init_state(ctx, &s->st[0], nb_channels, 0, 0, c.a, c.b, 1.0);
        }

        for (int b = (s->order & 1); b < s->filter_count; b++) {
            const int idx = b - (s->order & 1);
            double norm = 1.0 / (1.0 + K / q[idx] + K * K);
            BiquadCoeffs c;

            c.b[0] = K * K * norm;
            c.b[1] = 2.0 * c.b[0];
            c.b[2] = c.b[0];
            c.a[1] = 2.0 * (K * K - 1.0) * norm;
            c.a[2] = (1.0 - K / q[idx] + K * K) * norm;

            s->init_state(ctx, &s->st[b], nb_channels, 0, 0, c.a, c.b, 1.0);
        }
    } else if (!strcmp(ctx->filter->name, "asuperpass")) {
        double alpha, beta, gamma, theta;
        double theta_0 = 2. * M_PI * (s->cutoff / inlink->sample_rate);
        double d_E;

        s->filter_count = s->order / 2;
        d_E = (2. * tan(theta_0 / (2. * s->qfactor))) / sin(theta_0);

        for (int b = 0; b < s->filter_count; b += 2) {
            double D = 2. * sin(((b + 1) * M_PI) / (2. * s->filter_count));
            double A = (1. + pow((d_E / 2.), 2)) / (D * d_E / 2.);
            double d = sqrt((d_E * D) / (A + sqrt(A * A - 1.)));
            double B = D * (d_E / 2.) / d;
            double W = B + sqrt(B * B - 1.);

            for (int j = 0; j < 2; j++) {
                BiquadCoeffs c;

                if (j == 1)
                    theta = 2. * atan(tan(theta_0 / 2.) / W);
                else
                    theta = 2. * atan(W * tan(theta_0 / 2.));

                beta = 0.5 * ((1. - (d / 2.) * sin(theta)) / (1. + (d / 2.) * sin(theta)));
                gamma = (0.5 + beta) * cos(theta);
                alpha = 0.5 * (0.5 - beta) * sqrt(1. + pow((W - (1. / W)) / d, 2.));

                c.a[1] = -2. * gamma;
                c.a[2] =  2. * beta;
                c.b[0] =  2. * alpha;
                c.b[1] =  0.;
                c.b[2] = -2. * alpha;

                s->init_state(ctx, &s->st[b+j], nb_channels, 0, 0, c.a, c.b, 1.0);
            }
        }
    } else if (!strcmp(ctx->filter->name, "asuperstop")) {
        double alpha, beta, gamma, theta;
        double theta_0 = 2. * M_PI * (s->cutoff / inlink->sample_rate);
        double d_E;

        s->filter_count = s->order / 2;
        d_E = (2. * tan(theta_0 / (2. * s->qfactor))) / sin(theta_0);

        for (int b = 0; b < s->filter_count; b += 2) {
            double D = 2. * sin(((b + 1) * M_PI) / (2. * s->filter_count));
            double A = (1. + pow((d_E / 2.), 2)) / (D * d_E / 2.);
            double d = sqrt((d_E * D) / (A + sqrt(A * A - 1.)));
            double B = D * (d_E / 2.) / d;
            double W = B + sqrt(B * B - 1.);

            for (int j = 0; j < 2; j++) {
                BiquadCoeffs c;

                if (j == 1)
                    theta = 2. * atan(tan(theta_0 / 2.) / W);
                else
                    theta = 2. * atan(W * tan(theta_0 / 2.));

                beta = 0.5 * ((1. - (d / 2.) * sin(theta)) / (1. + (d / 2.) * sin(theta)));
                gamma = (0.5 + beta) * cos(theta);
                alpha = 0.5 * (0.5 + beta) * ((1. - cos(theta)) / (1. - cos(theta_0)));

                c.a[1] = -2. * gamma;
                c.a[2] =  2. * beta;
                c.b[0] =  2. * alpha;
                c.b[1] = -4. * alpha * cos(theta_0);
                c.b[2] =  2. * alpha;

                s->init_state(ctx, &s->st[b+j], nb_channels, 0, 0, c.a, c.b, 1.0);
            }
        }
    }

    return 0;
}

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    ASuperCutContext *s = ctx->priv;

    switch (inlink->format) {
    case AV_SAMPLE_FMT_FLTP:
        s->filter = biquad_tdii_fltp;
        s->init_state = init_biquad_fltp;
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->filter = biquad_tdii_dblp;
        s->init_state = init_biquad_dblp;
        break;
    default:
        return AVERROR_BUG;
    }

    return get_coeffs(ctx);
}

static int filter_channels(AVFilterContext *ctx, void *arg,
                           int jobnr, int nb_jobs)
{
    ASuperCutContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int filter_count = s->filter_count;
    const int is_disabled = ff_filter_disabled(ctx);

    for (int ch = start; ch < end; ch++) {
        s->filter(s->st[0], in->extended_data[ch],
                  out->extended_data[ch],
                  out->nb_samples, ch, is_disabled);

        for (int b = 1; b < filter_count; b++) {
            s->filter(s->st[b], out->extended_data[ch],
                      out->extended_data[ch],
                      out->nb_samples, ch, is_disabled);
        }
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    ASuperCutContext *s = ctx->priv;
    ThreadData td;
    AVFrame *out;

    if (s->bypass)
        return ff_filter_frame(outlink, in);

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

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    int ret;

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    return get_coeffs(ctx);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ASuperCutContext *s = ctx->priv;

    for (int i = 0; i < FF_ARRAY_ELEMS(s->st); i++)
        av_freep(&s->st[i]);
}

#define OFFSET(x) offsetof(ASuperCutContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption asupercut_options[] = {
    { "cutoff", "set cutoff frequency", OFFSET(cutoff), AV_OPT_TYPE_DOUBLE, {.dbl=20000}, 20000, 192000, FLAGS },
    { "order",  "set filter order",     OFFSET(order),  AV_OPT_TYPE_INT,    {.i64=10},        3,     20, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(asupercut);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_af_asupercut = {
    .p.name          = "asupercut",
    .p.description   = NULL_IF_CONFIG_SMALL("Cut super frequencies."),
    .p.priv_class    = &asupercut_class,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                       AVFILTER_FLAG_SLICE_THREADS,
    .priv_size       = sizeof(ASuperCutContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS_ARRAY(sample_fmts),
    .process_command = process_command,
};

static const AVOption asubcut_options[] = {
    { "cutoff", "set cutoff frequency", OFFSET(cutoff), AV_OPT_TYPE_DOUBLE, {.dbl=20},  2, 200, FLAGS },
    { "order",  "set filter order",     OFFSET(order),  AV_OPT_TYPE_INT,    {.i64=10},  3,  20, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(asubcut);

const FFFilter ff_af_asubcut = {
    .p.name          = "asubcut",
    .p.description   = NULL_IF_CONFIG_SMALL("Cut subwoofer frequencies."),
    .p.priv_class    = &asubcut_class,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                       AVFILTER_FLAG_SLICE_THREADS,
    .priv_size       = sizeof(ASuperCutContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS_ARRAY(sample_fmts),
    .process_command = process_command,
};

static const AVOption asuperpass_asuperstop_options[] = {
    { "centerf","set center frequency", OFFSET(cutoff), AV_OPT_TYPE_DOUBLE, {.dbl=1000}, 2, 999999, FLAGS },
    { "order",  "set filter order",     OFFSET(order),  AV_OPT_TYPE_INT,    {.i64=4},    4,     20, FLAGS },
    { "qfactor","set Q-factor",         OFFSET(qfactor),AV_OPT_TYPE_DOUBLE, {.dbl=1.},0.01,   100., FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS_EXT(asuperpass_asuperstop, "asuperpass/asuperstop",
                          asuperpass_asuperstop_options);

const FFFilter ff_af_asuperpass = {
    .p.name          = "asuperpass",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply high order Butterworth band-pass filter."),
    .p.priv_class    = &asuperpass_asuperstop_class,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                       AVFILTER_FLAG_SLICE_THREADS,
    .priv_size       = sizeof(ASuperCutContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS_ARRAY(sample_fmts),
    .process_command = process_command,
};

const FFFilter ff_af_asuperstop = {
    .p.name          = "asuperstop",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply high order Butterworth band-stop filter."),
    .p.priv_class    = &asuperpass_asuperstop_class,
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                       AVFILTER_FLAG_SLICE_THREADS,
    .priv_size       = sizeof(ASuperCutContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS_ARRAY(sample_fmts),
    .process_command = process_command,
};
