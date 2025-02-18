/*
 * Copyright (c) 2005 Laurent de Soras
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

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "formats.h"
#include "avfilter.h"
#include "filters.h"

typedef int (*src_fun)(AVFilterContext *ctx, AVFrame *in, AVFrame *out,
                       const int ch);

typedef struct AudioIIRSRCContext {
    const AVClass *class;

    int sample_rate;
    int channels;
    int nbr_coefs;
    double att;
    double bw;

    AVFrame *in;

    void *state;

    double coefs[256];

    int (*do_src)(AVFilterContext *ctx, AVFrame *in, AVFrame *out,
                  const int ch);

    void (*src_uninit)(AVFilterContext *ctx);
} AudioIIRSRCContext;

#define OFFSET(x) offsetof(AudioIIRSRCContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM

static const AVOption aiirsrc_options[] = {
    { "attenuation", "set the attenuation", OFFSET(att), AV_OPT_TYPE_DOUBLE, {.dbl=110.0}, 0, 200,    FLAGS },
    { "bandwidth",   "set the bandwidth",   OFFSET(bw),  AV_OPT_TYPE_DOUBLE, {.dbl=0.95},  0, 0.9999, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(aiirsrc);

static int compute_order(const double attenuation, const double q)
{
    const double attn_p2 = pow(10.0, -attenuation / 10);
    const double a       = attn_p2 / (1.0 - attn_p2);
    int          order   = ceil(log(a * a / 16) / log(q));

    if ((order & 1) == 0)
        order++;
    if (order == 1)
        order = 3;

    return order;
}

static void compute_transition_param(double *k, double *q, const double transition)
{
    k[0]  = tan((1.0 - transition * 2.0) * M_PI_4);
    k[0] *= k[0];
    const double kksqrt = pow(1.0 - k[0] * k[0], 0.25);
    const double e = 0.5 * (1.0 - kksqrt) / (1.0 + kksqrt);
    const double e2 = e * e;
    const double e4 = e2 * e2;

    q[0] = e * (1 + e4 * (2 + e4 * (15 + 150 * e4)));
}

static double ipowp(double x, long n)
{
    double z = 1;

    while (n != 0) {
        if ((n & 1) != 0)
            z *= x;
        n >>= 1;
        x *= x;
    }

    return z;
}

static double compute_acc_num(const double q, const int order, const int c)
{
    int    i   = 0;
    int    j   = 1;
    double acc = 0.0;
    double q_ii1;

    do {
        q_ii1  = ipowp(q, i * (i + 1));
        q_ii1 *= sin((i * 2 + 1) * c * M_PI / order) * j;
        acc   += q_ii1;

        j = -j;
        ++i;
    } while (fabs(q_ii1) > 1e-100);

    return acc;
}

static double compute_acc_den(const double q, const int order, const int c)
{
    int    i   =  1;
    int    j   = -1;
    double acc =  0.0;
    double q_i2;

    do {
        q_i2  = ipowp(q, i * i);
        q_i2 *= cos(i * 2 * c * M_PI / order) * j;
        acc  += q_i2;

        j = -j;
        ++i;
    } while (fabs(q_i2) > 1e-100);

    return acc;
}

static double compute_coef(const int index,
                           const double k, const double q, const int order)
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

static int compute_coefs(double coef_arr[], const double attenuation, const double transition)
{
    double k, q;

    compute_transition_param(&k, &q, transition);

    const int order     = compute_order(attenuation, q);
    const int nbr_coefs = (order - 1) / 2;

    for (int i = 0; i < nbr_coefs; i++)
        coef_arr[i] = compute_coef(i, k, q, order);

    return nbr_coefs;
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_S16P,
        AV_SAMPLE_FMT_S32P,
        AV_SAMPLE_FMT_FLTP,
        AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE
    };
    int ret, sample_rates[] = { 0, -1 };
    AVFilterFormats *formats;

    if (!ctx->inputs[0]->incfg.samplerates ||
        ctx->inputs[0]->incfg.samplerates->nb_formats < 1)
        return AVERROR(EAGAIN);

    formats = ff_make_format_list(sample_fmts);
    if ((ret = ff_formats_ref(formats, &cfg_in[0]->formats)) < 0)
        return ret;

    if ((ret = ff_formats_ref(formats, &cfg_out[0]->formats)) < 0)
        return ret;

    if (ctx->inputs[0]->incfg.samplerates->formats[0] > INT_MAX/2)
        return AVERROR(EINVAL);

    sample_rates[0] = 2 * ctx->inputs[0]->incfg.samplerates->formats[0];
    return ff_formats_ref(ff_make_format_list(sample_rates),
                          &cfg_out[0]->samplerates);
}

#undef DEPTH
#define DEPTH 16
#include "aiirsrc_template.c"

#undef DEPTH
#define DEPTH 32
#include "aiirsrc_template.c"

#undef DEPTH
#define DEPTH 33
#include "aiirsrc_template.c"

#undef DEPTH
#define DEPTH 65
#include "aiirsrc_template.c"

static src_fun src_funs[4][16] = {
#undef DEPTH
#define DEPTH 16
#include "aiirsrc_template_entry.c"

#undef DEPTH
#define DEPTH 32
#include "aiirsrc_template_entry.c"

#undef DEPTH
#define DEPTH 33
#include "aiirsrc_template_entry.c"

#undef DEPTH
#define DEPTH 65
#include "aiirsrc_template_entry.c"
};

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioIIRSRCContext *s = ctx->priv;
    int ret;

    if (inlink->sample_rate == outlink->sample_rate)
        return 0;

    outlink->time_base = (AVRational) {1, outlink->sample_rate};

    s->nbr_coefs = compute_coefs(s->coefs, s->att, (1.0 - s->bw) * 0.5);
    if (s->nbr_coefs <= 0 || s->nbr_coefs > 16)
        return AVERROR(EINVAL);

    switch (inlink->format) {
    case AV_SAMPLE_FMT_S16P:
        s->do_src = src_funs[0][s->nbr_coefs-1];
        s->src_uninit = src_uninit_s16p;
        ret = src_init_s16p(ctx);
        break;
    case AV_SAMPLE_FMT_S32P:
        s->do_src = src_funs[1][s->nbr_coefs-1];
        s->src_uninit = src_uninit_s32p;
        ret = src_init_s32p(ctx);
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->do_src = src_funs[2][s->nbr_coefs-1];
        s->src_uninit = src_uninit_fltp;
        ret = src_init_fltp(ctx);
        break;
    case AV_SAMPLE_FMT_DBLP:
        s->do_src = src_funs[3][s->nbr_coefs-1];
        s->src_uninit = src_uninit_dblp;
        ret = src_init_dblp(ctx);
        break;
    default:
        return AVERROR_BUG;
    }

    return ret;
}

static int src_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioIIRSRCContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        s->do_src(ctx, s->in, out, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioIIRSRCContext *s = ctx->priv;
    AVFrame *out;
    int ret;

    if (inlink->sample_rate == outlink->sample_rate)
        return ff_filter_frame(outlink, in);

    out = ff_get_audio_buffer(outlink, av_rescale(in->nb_samples, outlink->sample_rate, inlink->sample_rate));
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    s->in = in;
    av_frame_copy_props(out, in);
    out->sample_rate = outlink->sample_rate;

    ff_filter_execute(ctx, src_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    out->pts = av_rescale_q(in->pts, inlink->time_base, outlink->time_base);

    out->duration = av_rescale_q(out->nb_samples,
                                 (AVRational){1, outlink->sample_rate},
                                 outlink->time_base);

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
    int ret, status;
    int64_t pts;
    AVFrame *in;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_frame(inlink, &in);
    if (ret < 0)
        return ret;

    if (ret > 0)
        return filter_frame(inlink, in);

    if (ff_inlink_queued_frames(inlink) >= 1) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        pts = av_rescale_q(pts, inlink->time_base, outlink->time_base);
        ff_outlink_set_status(outlink, status, pts);
        return ret;
    }

    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioIIRSRCContext *s = ctx->priv;

    if (s->src_uninit)
        s->src_uninit(ctx);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const FFFilter ff_af_aiirsrc = {
    .p.name          = "aiirsrc",
    .p.description   = NULL_IF_CONFIG_SMALL("Audio IIR Polyphase Sample Rate Conversion."),
    .p.priv_class    = &aiirsrc_class,
    .priv_size       = sizeof(AudioIIRSRCContext),
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags         = AVFILTER_FLAG_SLICE_THREADS,
    .activate        = activate,
};
