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
    int used;
    ftype num_sum;
    ftype mean_sumx;
    ftype mean_sumy;
    ftype den_sumx;
    ftype den_sumy;
} fn(ChannelState);

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
    int used = state->used;
    ftype sumx, sumy;

    sumx = state->mean_sumx;
    sumy = state->mean_sumy;

    for (int n = 0; n < out->nb_samples; n++) {
        const int idx = n + size;

        sumx += x[idx];
        sumy += y[idx];

        dst[n] = fn(xcorrelate)(x + n, y + n,
                                sumx, sumy, size);

        used++;
        if (used >= size) {
            sumx -= x[n];
            sumy -= y[n];
        }
    }

    state->mean_sumx = sumx;
    state->mean_sumy = sumy;
    state->used = FFMIN(used, size);
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
    int used = state->used;

    num_sum = state->num_sum;
    den_sumx = state->den_sumx;
    den_sumy = state->den_sumy;

    for (int n = 0; n < out->nb_samples; n++) {
        const int idx = n + size;
        const ftype xidx = x[idx];
        const ftype yidx = y[idx];
        const ftype xn = x[n];
        const ftype yn = y[n];
        ftype num, den;

        num_sum  += xidx * yidx;
        den_sumx += xidx * xidx;
        den_sumy += yidx * yidx;

        num = num_sum;
        den = SQRT(den_sumx) * SQRT(den_sumy);
        den = den < EPS ? EPS : den;

        dst[n] = CLIP(num / den, -ONE, ONE);

        used++;
        if (used >= size) {
            num_sum  -= xn * yn;
            den_sumx -= xn * xn;
            den_sumy -= yn * yn;

            den_sumx  = FFMAX(den_sumx, ZERO);
            den_sumy  = FFMAX(den_sumy, ZERO);
        }
    }

    state->num_sum = num_sum;
    state->den_sumx = den_sumx;
    state->den_sumy = den_sumy;
    state->used = FFMIN(used, size);
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
    int used = state->used;

    num_sum = state->num_sum;
    den_sumx = state->den_sumx;
    den_sumy = state->den_sumy;
    mean_sumx = state->mean_sumx;
    mean_sumy = state->mean_sumy;

    for (int n = 0; n < out->nb_samples; n++) {
        const int idx = n + size;
        const ftype xidx = x[idx];
        const ftype yidx = y[idx];
        const ftype xn = x[n];
        const ftype yn = y[n];
        ftype num, den;

        mean_sumx+= xidx;
        mean_sumy+= yidx;
        num_sum  += xidx * yidx;
        den_sumx += xidx * xidx;
        den_sumy += yidx * yidx;

        num = num_sum * size - mean_sumx * mean_sumy;
        den = SQRT(FMAX(size * den_sumx - mean_sumx * mean_sumx, ZERO)) *
              SQRT(FMAX(size * den_sumy - mean_sumy * mean_sumy, ZERO));
        den = den < EPS ? EPS : den;

        dst[n] = CLIP(num / den, -ONE, ONE);

        used++;
        if (used >= size) {
            num_sum  -= xn * yn;
            mean_sumx-= xn;
            mean_sumy-= yn;
            den_sumx -= xn * xn;
            den_sumy -= yn * yn;

            den_sumx  = FMAX(den_sumx, ZERO);
            den_sumy  = FMAX(den_sumy, ZERO);
        }
    }

    state->num_sum = num_sum;
    state->den_sumx = den_sumx;
    state->den_sumy = den_sumy;
    state->mean_sumx = mean_sumx;
    state->mean_sumy = mean_sumy;
    state->used = FFMIN(used, size);
}
