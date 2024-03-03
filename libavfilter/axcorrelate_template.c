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

static void fn(xcorrelate_slow)(AVFilterContext *ctx,
                                AVFrame *out, const int ch)
{
    AudioXCorrelateContext *s = ctx->priv;
    const ftype *x = (const ftype *)s->cache[0]->extended_data[ch];
    const ftype *y = (const ftype *)s->cache[1]->extended_data[ch];
    ftype *sumxp = (ftype *)s->mean_sum[0]->extended_data[ch];
    ftype *sumyp = (ftype *)s->mean_sum[1]->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    const int size = s->size;
    ftype sumx, sumy;

    if (!s->used[ch]) {
        sumx = fn(mean_sum)(x, size);
        sumy = fn(mean_sum)(y, size);
    } else {
        sumx = sumxp[0];
        sumy = sumyp[0];
    }

    for (int n = 0; n < out->nb_samples; n++) {
        const int idx = n + size;

        dst[n] = fn(xcorrelate)(x + n, y + n,
                                sumx, sumy,
                                size);

        sumx -= x[n];
        sumx += x[idx];
        sumy -= y[n];
        sumy += y[idx];
    }

    sumxp[0] = sumx;
    sumyp[0] = sumy;

    s->used[ch] = 1;
}

static void fn(xcorrelate_fast)(AVFilterContext *ctx,
                                AVFrame *out, const int ch)
{
    AudioXCorrelateContext *s = ctx->priv;
    const ftype *x = (const ftype *)s->cache[0]->extended_data[ch];
    const ftype *y = (const ftype *)s->cache[1]->extended_data[ch];
    ftype *num_sump = (ftype *)s->num_sum->extended_data[ch];
    ftype *den_sumxp = (ftype *)s->den_sum[0]->extended_data[ch];
    ftype *den_sumyp = (ftype *)s->den_sum[1]->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    ftype num_sum, den_sumx, den_sumy;
    const int size = s->size;

    if (!s->used[ch]) {
        num_sum  = fn(square_sum)(x, y, size);
        den_sumx = fn(square_sum)(x, x, size);
        den_sumy = fn(square_sum)(y, y, size);
    } else {
        num_sum = num_sump[0];
        den_sumx = den_sumxp[0];
        den_sumy = den_sumyp[0];
    }

    for (int n = 0; n < out->nb_samples; n++) {
        const int idx = n + size;
        const ftype xidx = x[idx];
        const ftype yidx = y[idx];
        const ftype xn = x[n];
        const ftype yn = y[n];
        ftype num, den;

        num = num_sum / size;
        den = SQRT((den_sumx * den_sumy) / size / size);

        dst[n] = den <= SMALL ? ZERO : CLIP(num / den, -ONE, ONE);

        num_sum  -= xn * yn;
        num_sum  += xidx * yidx;
        den_sumx -= xn * xn;
        den_sumx += xidx * xidx;
        den_sumx  = FFMAX(den_sumx, ZERO);
        den_sumy -= yn * yn;
        den_sumy += yidx * yidx;
        den_sumy  = FFMAX(den_sumy, ZERO);
    }

    num_sump[0] = num_sum;
    den_sumxp[0] = den_sumx;
    den_sumyp[0] = den_sumy;

    s->used[ch] = 1;
}

static void fn(xcorrelate_best)(AVFilterContext *ctx, AVFrame *out,
                                const int ch)
{
    AudioXCorrelateContext *s = ctx->priv;
    const ftype *x = (const ftype *)s->cache[0]->extended_data[ch];
    const ftype *y = (const ftype *)s->cache[1]->extended_data[ch];
    ftype *mean_sumxp = (ftype *)s->mean_sum[0]->extended_data[ch];
    ftype *mean_sumyp = (ftype *)s->mean_sum[1]->extended_data[ch];
    ftype *num_sump = (ftype *)s->num_sum->extended_data[ch];
    ftype *den_sumxp = (ftype *)s->den_sum[0]->extended_data[ch];
    ftype *den_sumyp = (ftype *)s->den_sum[1]->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    ftype mean_sumx, mean_sumy, num_sum, den_sumx, den_sumy;
    const int size = s->size;

    if (!s->used[ch]) {
        num_sum  = fn(square_sum)(x, y, size);
        den_sumx = fn(square_sum)(x, x, size);
        den_sumy = fn(square_sum)(y, y, size);
        mean_sumx = fn(mean_sum)(x, size);
        mean_sumy = fn(mean_sum)(y, size);
    } else {
        num_sum = num_sump[0];
        den_sumx = den_sumxp[0];
        den_sumy = den_sumyp[0];
        mean_sumx = mean_sumxp[0];
        mean_sumy = mean_sumyp[0];
    }

    for (int n = 0; n < out->nb_samples; n++) {
        const int idx = n + size;
        ftype num, den, xm, ym;
        const ftype xidx = x[idx];
        const ftype yidx = y[idx];
        const ftype xn = x[n];
        const ftype yn = y[n];

        xm = mean_sumx / size;
        ym = mean_sumy / size;
        num = num_sum - size * xm * ym;
        den = SQRT(FMAX(den_sumx - size * xm * xm, ZERO)) *
              SQRT(FMAX(den_sumy - size * ym * ym, ZERO));

        dst[n] = den <= SMALL ? ZERO : CLIP(num / den, -ONE, ONE);

        mean_sumx-= xn;
        mean_sumx+= xidx;
        mean_sumy-= yn;
        mean_sumy+= yidx;
        num_sum  -= xn * yn;
        num_sum  += xidx * yidx;
        den_sumx -= xn * xn;
        den_sumx += xidx * xidx;
        den_sumx  = FMAX(den_sumx, ZERO);
        den_sumy -= yn * yn;
        den_sumy += yidx * yidx;
        den_sumy  = FMAX(den_sumy, ZERO);
    }

    num_sump[0] = num_sum;
    den_sumxp[0] = den_sumx;
    den_sumyp[0] = den_sumy;
    mean_sumxp[0] = mean_sumx;
    mean_sumyp[0] = mean_sumy;

    s->used[ch] = 1;
}
