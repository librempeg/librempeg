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

#undef FEXP
#undef FSQRT
#undef ftype
#undef ctype
#undef FPI
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define FSQRT sqrtf
#define FEXP expf
#define ftype float
#define FPI M_PIf
#define SAMPLE_FORMAT fltp
#else
#define FSQRT sqrt
#define FEXP exp
#define ftype double
#define FPI M_PI
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define HYPOT(x, y) FSQRT((x)*(x) + (y)*(y))
typedef struct fn(ctype) { ftype re, im; } fn(ctype);

typedef struct fn(aswift_plan) {
    ftype a_fast, a_slow;
    fn(ctype) wf, ws;

    fn(ctype) p_fast;
    fn(ctype) p_slow;
    fn(ctype) x_fast;
    fn(ctype) x_slow;
} fn(aswift_plan);

av_always_inline static fn(ctype) fn(make_complex)(const ftype re, const ftype im)
{
    return (fn(ctype)){ re, im };
}

av_always_inline static fn(ctype) fn(aswift_add)(const fn(ctype) x, const fn(ctype) y)
{
    return fn(make_complex)(x.re + y.re, x.im + y.im);
}

av_always_inline static fn(ctype) fn(aswift_mul)(const fn(ctype) x, const fn(ctype) y)
{
    return fn(make_complex)(x.re * y.re - x.im * y.im, x.re * y.im + x.im * y.re);
}

av_always_inline static fn(ctype) fn(aswift_mul_real)(const fn(ctype) x, const ftype y)
{
    return fn(make_complex)(x.re * y, x.im * y);
}

av_always_inline static fn(ctype) fn(polar)(const ftype r, const ftype t)
{
    return fn(make_complex)(r * cos(t), r * sin(t));
}

typedef struct fn(StateContext) {
    fn(aswift_plan) plan;
} fn(StateContext);

static int fn(init)(AVFilterContext *ctx)
{
    ToneDetectContext *s = ctx->priv;
    fn(StateContext) *state;

    if (!s->state)
        s->state = av_calloc(s->channels, sizeof(*state));
    if (!s->state)
        return AVERROR(ENOMEM);
    state = s->state;

    for (int ch = 0; ch < s->channels; ch++) {
        fn(StateContext) *stc = &state[ch];

        stc->plan.a_fast = FEXP(-F(1.0) / 10);
        stc->plan.a_slow = FEXP(-F(1.0) / 50);

        stc->plan.wf = fn(polar)(F(1.0), s->frequency * F(-2.0)*FPI / s->sample_rate);
        stc->plan.wf = fn(aswift_mul_real)(stc->plan.wf, stc->plan.a_fast);
        stc->plan.ws = fn(polar)(F(1.0), s->frequency * F(-2.0)*FPI / s->sample_rate);
        stc->plan.ws = fn(aswift_mul_real)(stc->plan.ws, stc->plan.a_slow);
    }

    return 0;
}

static void fn(uninit)(AVFilterContext *ctx)
{
    ToneDetectContext *s = ctx->priv;

    av_freep(&s->state);
}

static int fn(tone_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, int ch)
{
    ToneDetectContext *s = ctx->priv;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    const ftype *src = (const ftype *)in->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    fn(aswift_plan) *plan = &stc->plan;
    const fn(ctype) wf = plan->wf;
    const fn(ctype) ws = plan->ws;
    fn(ctype) fast = plan->x_fast;
    fn(ctype) slow = plan->x_slow;
    fn(ctype) pfast = plan->p_fast;
    fn(ctype) pslow = plan->p_slow;
    const int nb_samples = in->nb_samples;

    for (int n = 0; n < nb_samples; n++) {
        const fn(ctype) csample = fn(polar)(src[n], F(0.0));
        fn(ctype) adst;

        fast = fn(aswift_add)(fn(aswift_mul)(pfast, wf), csample);
        slow = fn(aswift_add)(fn(aswift_mul)(pslow, ws), csample);

        adst.re = slow.re - fast.re;
        adst.im = slow.im - fast.im;

        pfast.re = isnormal(fast.re) ? fast.re : F(0.0);
        pfast.im = isnormal(fast.im) ? fast.im : F(0.0);
        pslow.re = isnormal(slow.re) ? slow.re : F(0.0);
        pslow.im = isnormal(slow.im) ? slow.im : F(0.0);

        dst[n] = HYPOT(adst.re, adst.im);
    }

    plan->x_fast = fast;
    plan->x_slow = slow;
    plan->p_fast = pfast;
    plan->p_slow = pslow;

    return 0;
}
