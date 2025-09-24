/*
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

#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define MAX_BANDS 3

#include "afdelay_template.c"

static int fn(init_ambiophonic)(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AmbiophonicContext *s = ctx->priv;
    double a[3], m[3], g, k, Q = 0.5 * (s->lcut + s->hcut) / (s->hcut - s->lcut);
    int ret;

    s->nb_channels = inlink->ch_layout.nb_channels;
    s->max_delay = (inlink->sample_rate + 1000-1) / 1000;
    s->delay[0] = (s->delay_us / 1000000.) * inlink->sample_rate;
    s->delay[1] = s->delay[0];

    ret = fn(init_afdelay)(ctx, &s->st, s->delay, 2,
                           s->max_delay, s->nb_channels);
    if (ret < 0)
        return ret;

    g = tan(M_PI_2 * (s->lcut + s->hcut) / inlink->sample_rate);
    k = 1. / Q;
    a[0] = 1. / (1. + g * (g + k));
    a[1] = g * a[0];
    a[2] = g * a[1];
    m[0] = 0.;
    m[1] = 1.;
    m[2] = 0.;

    return s->init_biquad(ctx, &s->biquad_st, s->nb_channels, 0, 0, a, m, 1.0);
}

static void fn(uninit_ambiophonic)(AVFilterContext *ctx)
{
    AmbiophonicContext *s = ctx->priv;

    av_freep(&s->biquad_st);
    fn(uninit_afdelay)(ctx, &s->st, s->nb_channels);
}

static void fn(ambiophonic)(AVFilterContext *ctx, AVFrame *in, AVFrame *out)
{
    AmbiophonicContext *s = ctx->priv;
    const ftype *lsrc = (const ftype *)in->extended_data[0];
    const ftype *rsrc = (const ftype *)in->extended_data[1];
    ftype *ldst = (ftype *)out->extended_data[0];
    ftype *rdst = (ftype *)out->extended_data[1];
    const int is_disabled = ff_filter_disabled(ctx);
    const int nb_samples = in->nb_samples;
    const ftype att = -s->att;
    void *biquad_st = s->biquad_st;
    void *st = s->st;
    ftype lt = s->tmp[0];
    ftype rt = s->tmp[1];

    for (int n = 0; n < nb_samples; n++) {
        const ftype ls = lsrc[n], rs = rsrc[n];
        ftype out[2], bin[2], bout[2], bo[2];

        s->biquad_filter(biquad_st, &lsrc[n], &out[0], 1, 0, 0);
        s->biquad_filter(biquad_st, &rsrc[n], &out[1], 1, 1, 0);

        bin[0] = out[0];
        bin[1] = out[1];

        bin[0] += rt;
        bin[1] += lt;

        bo[0] = bin[0];
        bo[1] = bin[1];

        bin[0] *= att;
        bin[1] *= att;

        fn(afdelay_channel)(ctx, st, 1, (const uint8_t *)&bin[0],
                            (uint8_t *)&bout[0], 0);
        fn(afdelay_channel)(ctx, st, 1, (const uint8_t *)&bin[1],
                            (uint8_t *)&bout[1], 1);

        if (is_disabled) {
            ldst[n] = ls;
            rdst[n] = rs;
        } else {
            lt = bout[0];
            rt = bout[1];

            ldst[n] = ls + rt + bo[0];
            rdst[n] = rs + lt + bo[1];
        }
    }

    s->tmp[0] = lt;
    s->tmp[1] = rt;
}
