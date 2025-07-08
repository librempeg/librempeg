/*
 * Copyright (c) 2002 Naoki Shibata
 * Copyright (c) 2017 Paul B Mahol
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
#include "libavutil/tx.h"

#include "audio.h"
#include "avfilter.h"
#include "filters.h"

#define M 15

typedef struct EqParameter {
    float lower, upper, gain;
} EqParameter;

typedef struct SuperEqualizerContext {
    const AVClass *class;

    unsigned nb_params;
    EqParameter *params;

    float *bands;
    unsigned nb_bands;

    float *gains;
    unsigned nb_gains;

    float fact[M + 1];
    float aa;
    float iza;
    float *ires, *irest;
    float *fsamples, *fsamples_out;
    int winlen, tabsize;

    AVFrame *in, *out;
    AVTXContext *rdft, *irdft;
    av_tx_fn tx_fn, itx_fn;
} SuperEqualizerContext;

static float izero(SuperEqualizerContext *s, float x)
{
    float ret = 1;
    int m;

    for (m = 1; m <= M; m++) {
        float t;

        t = pow(x / 2, m) / s->fact[m];
        ret += t*t;
    }

    return ret;
}

static float hn_lpf(int n, float f, float fs)
{
    float t = 1 / fs;
    float omega = 2 * M_PI * f;

    if (n * omega * t == 0)
        return 2 * f * t;
    return 2 * f * t * sinf(n * omega * t) / (n * omega * t);
}

static float hn_imp(int n)
{
    return n == 0 ? 1.f : 0.f;
}

static float hn(int n, EqParameter *param, float fs, const int nb_params)
{
    float ret, lhn;
    int i;

    lhn = hn_lpf(n, param[0].upper, fs);
    ret = param[0].gain*lhn;

    for (i = 1; i < nb_params && param[i].upper < fs / 2; i++) {
        float lhn2 = hn_lpf(n, param[i].upper, fs);
        ret += param[i].gain * (lhn2 - lhn);
        lhn = lhn2;
    }

    ret += param[i].gain * (hn_imp(n) - lhn);

    return ret;
}

static float alpha(float a)
{
    if (a <= 21)
        return 0;
    if (a <= 50)
        return .5842f * pow(a - 21, 0.4f) + 0.07886f * (a - 21);
    return .1102f * (a - 8.7f);
}

static float win(SuperEqualizerContext *s, float n, int N)
{
    return izero(s, alpha(s->aa) * sqrtf(1 - 4 * n * n / ((N - 1) * (N - 1)))) / s->iza;
}

static void process_param(const float *bc, EqParameter *param, float fs,
                          const float *bands, const unsigned nb_bands)
{
    int i;

    for (i = 0; i < nb_bands; i++) {
        param[i].lower = i == 0 ? 0 : bands[i - 1];
        param[i].upper = i == (nb_bands-1) ? fs : bands[i];
        param[i].gain  = bc[i];
    }
}

static int equ_init(SuperEqualizerContext *s, int wb)
{
    float scale = 1.f, iscale = 1.f;
    int i, j, ret;

    ret = av_tx_init(&s->rdft, &s->tx_fn, AV_TX_FLOAT_RDFT, 0, 1 << wb, &scale, 0);
    if (ret < 0)
        return ret;

    ret = av_tx_init(&s->irdft, &s->itx_fn, AV_TX_FLOAT_RDFT, 1, 1 << wb, &iscale, 0);
    if (ret < 0)
        return ret;

    s->aa = 96;
    s->winlen = (1 << (wb-1))-1;
    s->tabsize  = 1 << wb;

    s->ires     = av_calloc(s->tabsize + 2, sizeof(float));
    s->irest    = av_calloc(s->tabsize, sizeof(float));
    s->fsamples = av_calloc(s->tabsize, sizeof(float));
    s->fsamples_out = av_calloc(s->tabsize + 2, sizeof(float));
    if (!s->ires || !s->irest || !s->fsamples || !s->fsamples_out)
        return AVERROR(ENOMEM);

    for (i = 0; i <= M; i++) {
        s->fact[i] = 1;
        for (j = 1; j <= i; j++)
            s->fact[i] *= j;
    }

    s->iza = izero(s, alpha(s->aa));

    return 0;
}

static void make_fir(SuperEqualizerContext *s, const float *lbc, EqParameter *param, float fs)
{
    const int winlen = s->winlen;
    const int tabsize = s->tabsize;
    int i;

    if (fs <= 0)
        return;

    process_param(lbc, param, fs, s->bands, s->nb_params);

    for (i = 0; i < winlen; i++)
        s->irest[i] = hn(i - winlen / 2, param, fs, s->nb_params) * win(s, i - winlen / 2, winlen);
    for (; i < tabsize; i++)
        s->irest[i] = 0;

    s->tx_fn(s->rdft, s->ires, s->irest, sizeof(float));
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    SuperEqualizerContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    const float *ires = s->ires;
    float *fsamples_out = s->fsamples_out;
    float *fsamples = s->fsamples;
    int ch, i;

    AVFrame *out = ff_get_audio_buffer(outlink, in->nb_samples);
    float *src, *dst, *ptr;

    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }

    for (ch = 0; ch < in->ch_layout.nb_channels; ch++) {
        ptr = (float *)out->extended_data[ch];
        dst = (float *)s->out->extended_data[ch];
        src = (float *)in->extended_data[ch];

        for (i = 0; i < in->nb_samples; i++)
            fsamples[i] = src[i];
        for (; i < s->tabsize; i++)
            fsamples[i] = 0;

        s->tx_fn(s->rdft, fsamples_out, fsamples, sizeof(float));

        for (i = 0; i <= s->tabsize / 2; i++) {
            float re, im;

            re = ires[i*2  ] * fsamples_out[i*2] - ires[i*2+1] * fsamples_out[i*2+1];
            im = ires[i*2+1] * fsamples_out[i*2] + ires[i*2  ] * fsamples_out[i*2+1];

            fsamples_out[i*2  ] = re;
            fsamples_out[i*2+1] = im;
        }

        s->itx_fn(s->irdft, fsamples, fsamples_out, sizeof(AVComplexFloat));

        for (i = 0; i < s->winlen; i++)
            dst[i] += fsamples[i] / s->tabsize;
        for (i = s->winlen; i < s->tabsize; i++)
            dst[i]  = fsamples[i] / s->tabsize;
        for (i = 0; i < out->nb_samples; i++)
            ptr[i] = dst[i];
        for (i = 0; i < s->winlen; i++)
            dst[i] = dst[i+s->winlen];
    }

    out->pts = in->pts;
    av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    SuperEqualizerContext *s = ctx->priv;
    AVFrame *in = NULL;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_samples(inlink, s->winlen, s->winlen, &in);
    if (ret < 0)
        return ret;
    if (ret > 0)
        return filter_frame(inlink, in);

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold int init(AVFilterContext *ctx)
{
    SuperEqualizerContext *s = ctx->priv;

    return equ_init(s, 14);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    SuperEqualizerContext *s = ctx->priv;

    s->out = ff_get_audio_buffer(inlink, s->tabsize);
    if (!s->out)
        return AVERROR(ENOMEM);

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    SuperEqualizerContext *s = ctx->priv;

    s->nb_params = FFMIN(s->nb_bands, s->nb_gains);

    s->params = av_calloc(s->nb_params, sizeof(*s->params));
    if (!s->params)
        return AVERROR(ENOMEM);

    make_fir(s, s->gains, s->params, outlink->sample_rate);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    SuperEqualizerContext *s = ctx->priv;

    av_frame_free(&s->out);
    av_freep(&s->params);
    av_freep(&s->irest);
    av_freep(&s->ires);
    av_freep(&s->fsamples);
    av_freep(&s->fsamples_out);
    av_tx_uninit(&s->rdft);
    av_tx_uninit(&s->irdft);
}

static const AVFilterPad superequalizer_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

static const AVFilterPad superequalizer_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define OFFSET(x) offsetof(SuperEqualizerContext, x)
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_gains = {.def="1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0",.size_min=2,.sep=' '};
static const AVOptionArrayDef def_bands = {.def="65.406392 92.498606 130.81278 184.99721 261.62557 369.99442 523.25113 739.9884 1046.5023 1479.9768 2093.0045 2959.9536 4186.0091 5919.9072 8372.0181 11839.814 16744.036 20000",.size_min=2,.sep=' '};

static const AVOption superequalizer_options[] = {
    { "gains", "set bands gain", OFFSET(gains), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_gains}, 0, 20, AF },
    { "bands", "set bands freq", OFFSET(bands), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_bands}, 1, INT_MAX, AF },
    { NULL }
};

AVFILTER_DEFINE_CLASS(superequalizer);

const FFFilter ff_af_superequalizer = {
    .p.name        = "superequalizer",
    .p.description = NULL_IF_CONFIG_SMALL("Apply X band equalization filter."),
    .p.priv_class  = &superequalizer_class,
    .priv_size     = sizeof(SuperEqualizerContext),
    .init          = init,
    .activate      = activate,
    .uninit        = uninit,
    FILTER_INPUTS(superequalizer_inputs),
    FILTER_OUTPUTS(superequalizer_outputs),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_FLTP),
};
