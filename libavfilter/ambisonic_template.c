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

#include "avfilter.h"
#include "internal.h"
#include "audio.h"

#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define ftype float
#else
#define SAMPLE_FORMAT double
#define ftype double
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(scale)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AmbisonicContext *s = ctx->priv;
    AVFrame *in = arg;
    const int nb_channels = in->ch_layout.nb_channels;
    const int start = (nb_channels * jobnr) / nb_jobs;
    const int end = (nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = in->nb_samples;
    AVFrame *out = s->sframe;

    for (int ch = start; ch < end; ch++) {
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        ftype mul = s->norm_tab[s->scaling_norm][s->seq_map[ch]];

#if DEPTH == 32
        s->fdsp->vector_fmul_scalar(dst, src, mul, FFALIGN(nb_samples, 16));
#else
        s->fdsp->vector_dmul_scalar(dst, src, mul, FFALIGN(nb_samples, 16));
#endif
    }

    return 0;
}

static int fn(transform)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AVFrame *input = arg;
    AmbisonicContext *s = ctx->priv;
    const int nb_channels = FFMIN(input->ch_layout.nb_channels, s->max_channels);
    const int start = (nb_channels * jobnr) / nb_jobs;
    const int end = (nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = input->nb_samples;
    AVFrame *out = s->rframe;
    AVFrame *in = s->sframe;

    for (int ch = start; ch < end; ch++) {
        const ftype *src = (const ftype *)in->extended_data[0];
        ftype *dst = (ftype *)out->extended_data[ch];
        ftype mul = s->transform_mat[s->seq_map[ch]][s->seq_map[0]];

#if DEPTH == 32
        s->fdsp->vector_fmul_scalar(dst, src, mul, FFALIGN(nb_samples, 16));
#else
        s->fdsp->vector_dmul_scalar(dst, src, mul, FFALIGN(nb_samples, 16));
#endif
        for (int ch2 = 1; ch2 < nb_channels; ch2++) {
            const ftype *src = (const ftype *)in->extended_data[ch2];
            ftype mul = s->transform_mat[s->seq_map[ch]][s->seq_map[ch2]];

#if DEPTH == 32
            s->fdsp->vector_fmac_scalar(dst, src, mul, FFALIGN(nb_samples, 16));
#else
            s->fdsp->vector_dmac_scalar(dst, src, mul, FFALIGN(nb_samples, 16));
#endif
        }
    }

    return 0;
}

static int fn(multiply)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AmbisonicContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const double *gains_tab = td->gains_tab;
    const int nb_channels = td->nb_channels;
    const int outputs = ambisonic_tab[s->layout].speakers;
    const int inputs = ambisonic_tab[s->layout].inputs;
    const int start = (outputs * jobnr) / nb_jobs;
    const int end = (outputs * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        ftype *dst = (ftype *)out->extended_data[ch];

        for (int ch2 = 0; ch2 < FFMIN3(nb_channels, s->max_channels, inputs); ch2++) {
            const int index = FFMIN(s->seq_map[ch2], nb_channels - 1);
            const ftype *src = (const ftype *)in->extended_data[index];
            const ftype gain = gains_tab ? gains_tab[ch2] : 1.f;
            const ftype mul = s->norm_decode_mat[ch][ch2] * gain;

#if DEPTH == 32
            s->fdsp->vector_fmac_scalar(dst, src, mul, FFALIGN(in->nb_samples, 16));
#else
            s->fdsp->vector_dmac_scalar(dst, src, mul, FFALIGN(in->nb_samples, 16));
#endif
        }
    }

    return 0;
}

static void fn(nfield1_process)(NearField *nf, AVFrame *frame, int ch, int add,
                                double gain)
{
    ftype *dst = (ftype *)frame->extended_data[ch];
    ftype z0, d0, d1;

    z0 = nf->z[0];
    d0 = nf->d[0] * gain;
    d1 = nf->d[1];

    for (int n = 0; n < frame->nb_samples; n++) {
        ftype x = dst[n] * d0 - d1 * z0;
        dst[n] = x + (add ? dst[n] : 0.f);
        z0 += x;
    }

    nf->z[0] = z0;
}

static void fn(xover_process)(Xover *xover, const ftype *src, ftype *dst, int nb_samples)
{
    ftype b0 = xover->b[0];
    ftype b1 = xover->b[1];
    ftype b2 = xover->b[2];
    ftype a1 = xover->a[1];
    ftype a2 = xover->a[2];
    ftype w0 = xover->w[0];
    ftype w1 = xover->w[1];

    for (int i = 0; i < nb_samples; i++) {
        ftype in = src[i];
        ftype out = b0 * in + w0;

        w0 = b1 * in + w1 + a1 * out;
        w1 = b2 * in + a2 * out;

        dst[i] = out;
    }

    xover->w[0] = w0;
    xover->w[1] = w1;
}

static void fn(xover)(AmbisonicContext *s,
                      AVFrame *in, AVFrame *lf, AVFrame *hf)
{
    for (int ch = 0; ch < in->ch_layout.nb_channels; ch++) {
        fn(xover_process)(&s->xover[0][ch],
                          (const ftype *)in->extended_data[ch],
                          (ftype *)lf->extended_data[ch], in->nb_samples);

        fn(xover_process)(&s->xover[1][ch],
                          (const ftype *)in->extended_data[ch],
                          (ftype *)hf->extended_data[ch], in->nb_samples);
    }
}

static int fn(level)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AmbisonicContext *s = ctx->priv;
    AVFrame *out = arg;
    const int nb_channels = out->ch_layout.nb_channels;
    const int start = (nb_channels * jobnr) / nb_jobs;
    const int end = (nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = out->nb_samples;

    for (int ch = start; ch < end; ch++) {
        ftype *dst = (ftype *)out->extended_data[ch];
        const ftype mul = s->level_tab[ch];

#if DEPTH == 32
        s->fdsp->vector_fmul_scalar(dst, dst, mul, FFALIGN(nb_samples, 16));
#else
        s->fdsp->vector_dmul_scalar(dst, dst, mul, FFALIGN(nb_samples, 16));
#endif
    }

    return 0;
}

static void fn(process)(AVFilterContext *ctx,
                        AVFrame *in, AVFrame *out)
{
    AmbisonicContext *s = ctx->priv;
    const int nb_in_channels = in->ch_layout.nb_channels;
    const int nb_out_channels = FFMIN(nb_in_channels, s->max_channels);
    const int nb_threads = ff_filter_get_nb_threads(ctx);
    ThreadData td;

    ff_filter_execute(ctx, fn(scale), in, NULL, FFMIN(nb_in_channels, nb_threads));

    ff_filter_execute(ctx, fn(transform), in, NULL, FFMIN(nb_out_channels, nb_threads));

    if (s->near_field == NF_IN)
        near_field(s, s->rframe, 0, 0);

    if (s->xover_freq > 0.) {
        fn(xover)(s, s->rframe, s->frame2, s->rframe);

        td.gains_tab = s->gains_tab[0];
        td.nb_channels = in->ch_layout.nb_channels;
        td.in = s->frame2;
        td.out = out;

        ff_filter_execute(ctx, fn(multiply), &td, NULL, FFMIN(out->ch_layout.nb_channels, nb_threads));
    }

    td.gains_tab = s->xover_freq > 0. ? s->gains_tab[1] : NULL;
    td.nb_channels = in->ch_layout.nb_channels;
    td.in = s->rframe;
    td.out = out;

    ff_filter_execute(ctx, fn(multiply), &td, NULL, FFMIN(out->ch_layout.nb_channels, nb_threads));

    if (s->near_field == NF_OUT)
        near_field(s, out, 1, 1);

    ff_filter_execute(ctx, fn(level), out, NULL, FFMIN(out->ch_layout.nb_channels, nb_threads));
}
