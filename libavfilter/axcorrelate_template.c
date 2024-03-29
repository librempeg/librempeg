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
#undef EPS
#if DEPTH == 32
#define SAMPLE_FORMAT fltp
#define FMAX fmaxf
#define ftype float
#define ZERO 0.f
#define ONE 1.f
#define CLIP av_clipf
#define SQRT sqrtf
#define EPS FLT_EPSILON
#else
#define SAMPLE_FORMAT dblp
#define FMAX fmax
#define ftype double
#define ZERO 0.0
#define ONE 1.0
#define CLIP av_clipd
#define SQRT sqrt
#define EPS DBL_EPSILON
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(ChannelState) {
    ftype num_sum;
    ftype mean_sumx;
    ftype mean_sumy;
    ftype den_sumx;
    ftype den_sumy;
} fn(ChannelState);

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

    den = SQRT(den0) * SQRT(den1);
    den = den < EPS ? EPS : den;

    return num / den;
}

static void fn(xcorrelate_slow)(AVFilterContext *ctx,
                                AVFrame *out, const int ch)
{
    AudioXCorrelateContext *s = ctx->priv;
    const ftype *x = (const ftype *)s->cache[0]->extended_data[ch];
    const ftype *y = (const ftype *)s->cache[1]->extended_data[ch];
    fn(ChannelState) *ch_state = s->ch_state;
    fn(ChannelState) *state = &ch_state[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    const int size = s->size;
    ftype sumx, sumy;

    if (!s->used[ch]) {
        sumx = fn(mean_sum)(x, size);
        sumy = fn(mean_sum)(y, size);
    } else {
        sumx = state->mean_sumx;
        sumy = state->mean_sumy;
    }

    for (int n = 0; n < out->nb_samples; n++) {
        const int idx = n + size;

        dst[n] = fn(xcorrelate)(x + n, y + n,
                                sumx, sumy,
                                size);

        sumx += x[idx];
        sumx -= x[n];
        sumy += y[idx];
        sumy -= y[n];
    }

    state->mean_sumx = sumx;
    state->mean_sumy = sumy;

    s->used[ch] = 1;
}

static void fn(xcorrelate_fast)(AVFilterContext *ctx,
                                AVFrame *out, const int ch)
{
    AudioXCorrelateContext *s = ctx->priv;
    const ftype *x = (const ftype *)s->cache[0]->extended_data[ch];
    const ftype *y = (const ftype *)s->cache[1]->extended_data[ch];
    fn(ChannelState) *ch_state = s->ch_state;
    fn(ChannelState) *state = &ch_state[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    ftype num_sum, den_sumx, den_sumy;
    const int size = s->size;

    if (!s->used[ch]) {
        num_sum  = fn(square_sum)(x, y, size);
        den_sumx = fn(square_sum)(x, x, size);
        den_sumy = fn(square_sum)(y, y, size);
    } else {
        num_sum = state->num_sum;
        den_sumx = state->den_sumx;
        den_sumy = state->den_sumy;
    }

    for (int n = 0; n < out->nb_samples; n++) {
        const int idx = n + size;
        const ftype xidx = x[idx];
        const ftype yidx = y[idx];
        const ftype xn = x[n];
        const ftype yn = y[n];
        ftype num, den;

        num = num_sum;
        den = SQRT(den_sumx) * SQRT(den_sumy);
        den = den < EPS ? EPS : den;

        dst[n] = CLIP(num / den, -ONE, ONE);

        num_sum  += xidx * yidx;
        num_sum  -= xn * yn;
        den_sumx += xidx * xidx;
        den_sumx -= xn * xn;
        den_sumx  = FFMAX(den_sumx, ZERO);
        den_sumy += yidx * yidx;
        den_sumy -= yn * yn;
        den_sumy  = FFMAX(den_sumy, ZERO);
    }

    state->num_sum = num_sum;
    state->den_sumx = den_sumx;
    state->den_sumy = den_sumy;

    s->used[ch] = 1;
}

static void fn(xcorrelate_best)(AVFilterContext *ctx, AVFrame *out,
                                const int ch)
{
    AudioXCorrelateContext *s = ctx->priv;
    const ftype *x = (const ftype *)s->cache[0]->extended_data[ch];
    const ftype *y = (const ftype *)s->cache[1]->extended_data[ch];
    fn(ChannelState) *ch_state = s->ch_state;
    fn(ChannelState) *state = &ch_state[ch];
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
        num_sum = state->num_sum;
        den_sumx = state->den_sumx;
        den_sumy = state->den_sumy;
        mean_sumx = state->mean_sumx;
        mean_sumy = state->mean_sumy;
    }

    for (int n = 0; n < out->nb_samples; n++) {
        const int idx = n + size;
        const ftype xidx = x[idx];
        const ftype yidx = y[idx];
        const ftype xn = x[n];
        const ftype yn = y[n];
        ftype num, den;

        num = num_sum * size - mean_sumx * mean_sumy;
        den = SQRT(FMAX(size * den_sumx - mean_sumx * mean_sumx, ZERO)) *
              SQRT(FMAX(size * den_sumy - mean_sumy * mean_sumy, ZERO));
        den = den < EPS ? EPS : den;

        dst[n] = CLIP(num / den, -ONE, ONE);

        mean_sumx+= xidx;
        mean_sumx-= xn;
        mean_sumy+= yidx;
        mean_sumy-= yn;
        num_sum  += xidx * yidx;
        num_sum  -= xn * yn;
        den_sumx += xidx * xidx;
        den_sumx -= xn * xn;
        den_sumx  = FMAX(den_sumx, ZERO);
        den_sumy += yidx * yidx;
        den_sumy -= yn * yn;
        den_sumy  = FMAX(den_sumy, ZERO);
    }

    state->num_sum = num_sum;
    state->den_sumx = den_sumx;
    state->den_sumy = den_sumy;
    state->mean_sumx = mean_sumx;
    state->mean_sumy = mean_sumy;

    s->used[ch] = 1;
}
