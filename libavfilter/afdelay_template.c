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

typedef struct fn(StateContext) {
    unsigned N;
    ftype a[MAX_DELAY+1];
    ftype b[MAX_DELAY+1];
    ftype c[MAX_DELAY+1];
    ftype t[MAX_DELAY+1];
    ftype y[MAX_DELAY+1];
} fn(StateContext);

static int fn(update_state)(AVFilterContext *ctx, const int reset)
{
    AudioFDelayContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;

    if (s->nb_delays == 0)
        return 0;

    for (int ch = 0; ch < s->nb_channels; ch++) {
        const int idx = FFMIN(ch, s->nb_delays-1);
        fn(StateContext) *st = &stc[ch];
        const ftype D = s->delays_opt[idx];
        const unsigned N = FFMAX(LRINT(ROUND(D)),1);
        const unsigned L = N-1;

        av_assert0(D > L);

        st->N = N;
        for (int n = 0; n < N+1; n++) {
            st->a[n] = D-F(n+1)+F(1.0);
            st->b[n] = -F(1.0)/(D+F(n+1));
            st->c[n] = F(2.0)*F(n+1)-F(1.0);
            if (reset)
                st->t[n] = F(0.0);
            av_log(ctx, AV_LOG_DEBUG, "[%d]: %g %g %g\n",
                   n, st->a[n], st->b[n], st->c[n]);
        }
    }

    return 0;
}

static int fn(init_state)(AVFilterContext *ctx)
{
    AudioFDelayContext *s = ctx->priv;
    fn(StateContext) *stc;

    s->st = av_calloc(s->nb_channels, sizeof(*stc));
    if (!s->st)
        return AVERROR(ENOMEM);

    return fn(update_state)(ctx, 1);
}

static void fn(uninit_state)(AVFilterContext *ctx)
{
    AudioFDelayContext *s = ctx->priv;

    av_freep(&s->st);
}

static void fn(filter_channel)(AVFilterContext *ctx, const int nb_samples,
                               const uint8_t *ssrc, uint8_t *ddst, const int ch)
{
    AudioFDelayContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;
    fn(StateContext) *st = &stc[ch];
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
