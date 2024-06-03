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

typedef struct fn(DelayContext) {
    ftype a, b, c;
    ftype u, y, t;
} fn(DelayContext);

typedef struct fn(StateContext) {
    unsigned N;
    fn(DelayContext) *dc;
} fn(StateContext);

static int fn(update_state)(AVFilterContext *ctx)
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

        st->dc = av_realloc_f(st->dc, N+1, sizeof(*st->dc));
        if (!st->dc)
            return AVERROR(ENOMEM);

        st->N = N;
        for (int n = 0; n < N+1; n++) {
            fn(DelayContext) *dc = &st->dc[n];

            dc->a = D-F(n+1)+F(1.0);
            dc->b = -F(1.0)/(D+F(n+1));
            dc->c = F(2.0)*F(n+1)-F(1.0);
            dc->u = F(0.0);
            dc->y = F(0.0);
            dc->t = F(0.0);
            av_log(ctx, AV_LOG_DEBUG, "[%d]: %g %g %g\n", n, dc->a, dc->b, dc->c);
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

    return fn(update_state)(ctx);
}

static void fn(uninit_state)(AVFilterContext *ctx)
{
    AudioFDelayContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;

    for (int ch = 0; ch < s->nb_channels; ch++) {
        fn(StateContext) *st = &stc[ch];

        av_freep(&st->dc);
    }

    av_freep(&s->st);
}

static void fn(filter_channel)(AVFilterContext *ctx, const int nb_samples,
                               const uint8_t *ssrc, uint8_t *ddst, const int ch)
{
    AudioFDelayContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;
    fn(StateContext) *st = &stc[ch];
    const ftype *src = (const ftype *)ssrc;
    ftype *dst = (ftype *)ddst;
    fn(DelayContext) *sdc = st->dc;
    const unsigned N = st->N;
    const unsigned L = N-1;

    for (int i = 0; i < nb_samples; i++) {
        fn(DelayContext) *d0 = &sdc[0];
        const ftype in = src[i];
        ftype v;

        d0->u = in;
        for (int n = 0; n < N; n++) {
            fn(DelayContext) *dn = &sdc[n+1];
            fn(DelayContext) *dc = &sdc[n];

            dc->t += dc->y * dc->c;
            v = dc->a * dc->u + dc->t;
            v *= dc->b;
            dn->u = v;
            dc->y = v * F(2.0);

            dc->t = isnormal(dc->t) ? dc->t : F(0.0);
            dc->y = isnormal(dc->y) ? dc->y : F(0.0);
            dn->u = isnormal(dn->u) ? dn->u : F(0.0);

            if (n == L) {
                for (int j = n-1; j >= 0; j--)
                    sdc[j].y += sdc[j+1].y;
            }
        }

        dst[i] = d0->y + in;
    }
}
