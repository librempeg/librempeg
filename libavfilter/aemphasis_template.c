/*
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

static void fn(biquad_process)(BiquadCoeffs *bq, ftype *dst, const ftype *src, const int nb_samples,
                               ftype *w, ftype level_in, ftype level_out, const int disabled)
{
    const ftype b0 = bq->b0;
    const ftype b1 = bq->b1;
    const ftype b2 = bq->b2;
    const ftype a1 = -bq->a1;
    const ftype a2 = -bq->a2;
    ftype w1 = w[0];
    ftype w2 = w[1];

    for (int i = 0; i < nb_samples; i++) {
        ftype in = src[i] * level_in;
        ftype out = b0 * in + w1;
        w1 = b1 * in + w2 + a1 * out;
        w2 = b2 * in + a2 * out;

        dst[i] = disabled ? src[i] : out * level_out;
    }

    w[0] = isnormal(w1) ? w1 : F(0.0);
    w[1] = isnormal(w2) ? w2 : F(0.0);
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    const int disabled = ff_filter_disabled(ctx);
    AudioEmphasisContext *s = ctx->priv;
    const ftype level_out = s->level_out;
    const ftype level_in = s->level_in;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *w = (ftype *)s->w->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];

        fn(biquad_process)(&s->rc.r1, dst, src, in->nb_samples, w, level_in, level_out, disabled);
    }

    return 0;
}
