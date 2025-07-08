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

#undef ROUND
#undef LRINT
#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ROUND roundf
#define LRINT lrintf
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define ROUND round
#define LRINT lrint
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(FDelayContext) {
    unsigned N;
    ftype *a;
    ftype *b;
    ftype *c;
    ftype *t;
    ftype *y;
} fn(FDelayContext);

static int fn(update_afdelay)(AVFilterContext *ctx, void *st,
                              const double *delays, const int nb_delays,
                              const int max_delay, const int nb_channels, const int reset)
{
    fn(FDelayContext) *stc = st;

    if (nb_delays == 0)
        return 0;

    for (int ch = 0; ch < nb_channels; ch++) {
        const int idx = FFMIN(ch, nb_delays-1);
        fn(FDelayContext) *st = &stc[ch];
        const ftype D = delays[idx];
        const unsigned N = FFMAX(LRINT(ROUND(D)),1);

        st->N = N;

        st->a = av_realloc_f(st->a, max_delay+1, sizeof(*st->a));
        st->b = av_realloc_f(st->b, max_delay+1, sizeof(*st->b));
        st->c = av_realloc_f(st->c, max_delay+1, sizeof(*st->c));
        st->t = av_realloc_f(st->t, max_delay+1, sizeof(*st->t));
        st->y = av_realloc_f(st->y, max_delay+1, sizeof(*st->y));
        if (!st->a || !st->b || !st->c || !st->t || !st->y)
            return AVERROR(ENOMEM);

        for (int n = 0; n < N+1; n++) {
            st->a[n] = D-F(n+1)+F(1.0);
            st->b[n] = -F(1.0)/(D+F(n+1));
            st->c[n] = F(2.0)*F(n+1)-F(1.0);
            if (reset) {
                st->t[n] = F(0.0);
                st->y[n] = F(0.0);
            }
            av_log(ctx, AV_LOG_DEBUG, "[%d]: %g %g %g\n",
                   n, st->a[n], st->b[n], st->c[n]);
        }
    }

    return 0;
}

static void fn(uninit_afdelay)(AVFilterContext *ctx, void **state,
                               const int nb_channels)
{
    fn(FDelayContext) *stc = *state;

    for (int ch = 0; ch < nb_channels && stc; ch++) {
        fn(FDelayContext) *st = &stc[ch];

        av_freep(&st->a);
        av_freep(&st->b);
        av_freep(&st->c);
        av_freep(&st->t);
        av_freep(&st->y);
    }

    av_freep(state);
}

static int fn(init_afdelay)(AVFilterContext *ctx, void **state,
                            const double *delays, const int nb_delays,
                            const int max_delay, const int nb_channels)
{
    *state = av_calloc(nb_channels, sizeof(fn(FDelayContext)));
    if (!*state)
        return AVERROR(ENOMEM);

    return fn(update_afdelay)(ctx, *state, delays, nb_delays, max_delay,
                              nb_channels, 1);
}

static void fn(afdelay_channel)(AVFilterContext *ctx, void *state, const int nb_samples,
                                const uint8_t *ssrc, uint8_t *ddst, const int ch)
{
    fn(FDelayContext) *stc = state;
    fn(FDelayContext) *st = &stc[ch];
    const ftype *restrict src = (const ftype *restrict)ssrc;
    ftype *restrict dst = (ftype *restrict)ddst;
    const ftype *restrict a = st->a;
    const ftype *restrict b = st->b;
    const ftype *restrict c = st->c;
    ftype *restrict y = st->y;
    ftype *restrict t = st->t;
    const unsigned N = st->N;
    const unsigned L = N-1;

    for (int i = 0; i < nb_samples; i++) {
        const ftype in = src[i];
        ftype v, u;

        u = in;
        for (int n = 0; n < N; n++) {
            t[n] += y[n] * c[n];
            v = a[n] * u + t[n];
            v *= b[n];
            u = v;
            y[n] = v + v;
        }

        for (int n = 0; n < N; n++) {
            if (!isnormal(t[n]))
                t[n] = F(0.0);
            if (!isnormal(y[n]))
                y[n] = F(0.0);
        }

        for (int n = L-1; n >= 0; n--)
            y[n] += y[n+1];

        dst[i] = y[0] + in;
    }
}
