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
#undef FMAX
#undef SAMPLE_FORMAT
#undef ZERO
#undef ONE
#undef CLIP
#undef SQRT
#undef SMALL
#if DEPTH == 32
#define SAMPLE_FORMAT fltp
#define FMAX fmaxf
#define ftype float
#define ZERO 0.f
#define ONE 1.f
#define CLIP av_clipf
#define SQRT sqrtf
#define SMALL 1e-6f
#else
#define SAMPLE_FORMAT dblp
#define FMAX fmax
#define ftype double
#define ZERO 0.0
#define ONE 1.0
#define CLIP av_clipd
#define SQRT sqrt
#define SMALL 1e-9
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static ftype fn(mean_sum)(const ftype *in,
                          const int size)
{
    ftype mean_sum = ZERO;

    for (int i = 0; i < size; i++)
        mean_sum += in[i];

    return mean_sum;
}

static ftype fn(square_sum)(const ftype *x,
                        const ftype *y,
                        const int size)
{
    ftype square_sum = ZERO;

    for (int i = 0; i < size; i++)
        square_sum += x[i] * y[i];

    return square_sum;
}

static ftype fn(xcorrelate)(const ftype *x,
                            const ftype *y,
                            ftype sumx,
                            ftype sumy, const int size)
{
    const ftype xm = sumx / size, ym = sumy / size;
    ftype num = ZERO, den, den0 = ZERO, den1 = ZERO;

    for (int i = 0; i < size; i++) {
        ftype xd = x[i] - xm;
        ftype yd = y[i] - ym;

        num += xd * yd;
        den0 += xd * xd;
        den1 += yd * yd;
    }

    num /= size;
    den  = SQRT((den0 * den1) / size / size);

    return den <= SMALL ? ZERO : num / den;
}

static int fn(xcorrelate_slow)(AVFilterContext *ctx,
                           AVFrame *out, int available)
{
    AudioXCorrelateContext *s = ctx->priv;
    const int size = s->size;
    int used;

    for (int ch = 0; ch < out->ch_layout.nb_channels; ch++) {
        const ftype *x = (const ftype *)s->cache[0]->extended_data[ch];
        const ftype *y = (const ftype *)s->cache[1]->extended_data[ch];
        ftype *sumx = (ftype *)s->mean_sum[0]->extended_data[ch];
        ftype *sumy = (ftype *)s->mean_sum[1]->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];

        used = s->used;
        if (!used) {
            sumx[0] = fn(mean_sum)(x, size);
            sumy[0] = fn(mean_sum)(y, size);
            used = 1;
        }

        for (int n = 0; n < out->nb_samples; n++) {
            const int idx = n + size;

            dst[n] = fn(xcorrelate)(x + n, y + n,
                                    sumx[0], sumy[0],
                                    size);

            sumx[0] -= x[n];
            sumx[0] += x[idx];
            sumy[0] -= y[n];
            sumy[0] += y[idx];
        }
    }

    return used;
}

static int fn(xcorrelate_fast)(AVFilterContext *ctx, AVFrame *out,
                               int available)
{
    AudioXCorrelateContext *s = ctx->priv;
    const int size = s->size;
    int used;

    for (int ch = 0; ch < out->ch_layout.nb_channels; ch++) {
        const ftype *x = (const ftype *)s->cache[0]->extended_data[ch];
        const ftype *y = (const ftype *)s->cache[1]->extended_data[ch];
        ftype *num_sum = (ftype *)s->num_sum->extended_data[ch];
        ftype *den_sumx = (ftype *)s->den_sum[0]->extended_data[ch];
        ftype *den_sumy = (ftype *)s->den_sum[1]->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];

        used = s->used;
        if (!used) {
            num_sum[0]  = fn(square_sum)(x, y, size);
            den_sumx[0] = fn(square_sum)(x, x, size);
            den_sumy[0] = fn(square_sum)(y, y, size);
            used = 1;
        }

        for (int n = 0; n < out->nb_samples; n++) {
            const int idx = n + size;
            ftype num, den;

            num = num_sum[0] / size;
            den = SQRT((den_sumx[0] * den_sumy[0]) / size / size);

            dst[n] = den <= SMALL ? ZERO : CLIP(num / den, -ONE, ONE);

            num_sum[0]  -= x[n] * y[n];
            num_sum[0]  += x[idx] * y[idx];
            den_sumx[0] -= x[n] * x[n];
            den_sumx[0] += x[idx] * x[idx];
            den_sumx[0]  = FFMAX(den_sumx[0], ZERO);
            den_sumy[0] -= y[n] * y[n];
            den_sumy[0] += y[idx] * y[idx];
            den_sumy[0]  = FFMAX(den_sumy[0], ZERO);
        }
    }

    return used;
}

static int fn(xcorrelate_best)(AVFilterContext *ctx, AVFrame *out,
                               int available)
{
    AudioXCorrelateContext *s = ctx->priv;
    const int size = s->size;
    int used;

    for (int ch = 0; ch < out->ch_layout.nb_channels; ch++) {
        const ftype *x = (const ftype *)s->cache[0]->extended_data[ch];
        const ftype *y = (const ftype *)s->cache[1]->extended_data[ch];
        ftype *mean_sumx = (ftype *)s->mean_sum[0]->extended_data[ch];
        ftype *mean_sumy = (ftype *)s->mean_sum[1]->extended_data[ch];
        ftype *num_sum = (ftype *)s->num_sum->extended_data[ch];
        ftype *den_sumx = (ftype *)s->den_sum[0]->extended_data[ch];
        ftype *den_sumy = (ftype *)s->den_sum[1]->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];

        used = s->used;
        if (!used) {
            num_sum[0]  = fn(square_sum)(x, y, size);
            den_sumx[0] = fn(square_sum)(x, x, size);
            den_sumy[0] = fn(square_sum)(y, y, size);
            mean_sumx[0] = fn(mean_sum)(x, size);
            mean_sumy[0] = fn(mean_sum)(y, size);
            used = 1;
        }

        for (int n = 0; n < out->nb_samples; n++) {
            const int idx = n + size;
            ftype num, den, xm, ym;

            xm = mean_sumx[0] / size;
            ym = mean_sumy[0] / size;
            num = num_sum[0] - size * xm * ym;
            den = SQRT(FMAX(den_sumx[0] - size * xm * xm, ZERO)) *
                  SQRT(FMAX(den_sumy[0] - size * ym * ym, ZERO));

            dst[n] = den <= SMALL ? ZERO : CLIP(num / den, -ONE, ONE);

            mean_sumx[0]-= x[n];
            mean_sumx[0]+= x[idx];
            mean_sumy[0]-= y[n];
            mean_sumy[0]+= y[idx];
            num_sum[0]  -= x[n] * y[n];
            num_sum[0]  += x[idx] * y[idx];
            den_sumx[0] -= x[n] * x[n];
            den_sumx[0] += x[idx] * x[idx];
            den_sumx[0]  = FMAX(den_sumx[0], ZERO);
            den_sumy[0] -= y[n] * y[n];
            den_sumy[0] += y[idx] * y[idx];
            den_sumy[0]  = FMAX(den_sumy[0], ZERO);
        }
    }

    return used;
}
