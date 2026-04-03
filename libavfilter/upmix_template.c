/*
 * Copyright (c) 2026 Paul B Mahol
 *
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
#undef ATAN2
#undef FABS
#undef FSQRT
#undef FEXP
#undef FEXP10
#undef EPS
#undef CLIP
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ftype float
#define ATAN2 atan2f
#define FABS fabsf
#define FSQRT sqrtf
#define FEXP expf
#define FEXP10 ff_exp10f
#define CLIP av_clipf
#define SAMPLE_FORMAT fltp
#define EPS FLT_EPSILON
#else
#define ftype double
#define ATAN2 atan2
#define FABS fabs
#define FSQRT sqrt
#define FEXP exp
#define FEXP10 ff_exp10
#define CLIP av_clipd
#define SAMPLE_FORMAT dblp
#define EPS DBL_EPSILON
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define HYPOT(x, y) FSQRT((x)*(x) + (y)*(y))

typedef struct fn(ctype) { ftype re, im; } fn(ctype);

typedef struct fn(aswift_plan) {
    int size;
    ftype a_fast, a_slow;
    fn(ctype) *wf, *ws;

    fn(ctype) *p_fast;
    fn(ctype) *p_slow;
    fn(ctype) *x_fast;
    fn(ctype) *x_slow;
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
    fn(ctype) *aswift, *iaswift;
} fn(StateContext);

static void fn(upmix_uninit)(AVFilterContext *ctx)
{
    AUpmixContext *s = ctx->priv;

    if (s->state) {
        fn(StateContext) *state = s->state;

        for (int ch = 0; ch < s->channels; ch++) {
            fn(StateContext) *stc = &state[ch];

            av_freep(&stc->aswift);
            av_freep(&stc->iaswift);

            av_freep(&stc->plan.wf);
            av_freep(&stc->plan.ws);
            av_freep(&stc->plan.p_fast);
            av_freep(&stc->plan.p_slow);
            av_freep(&stc->plan.x_fast);
            av_freep(&stc->plan.x_slow);
        }
    }

    av_freep(&s->state);
    av_freep(&s->x_pos);
    av_freep(&s->y_pos);
}

static int fn(upmix_init)(AVFilterContext *ctx)
{
    AUpmixContext *s = ctx->priv;
    const int channels = FFMAX(ctx->inputs[0]->ch_layout.nb_channels,
                               ctx->outputs[0]->ch_layout.nb_channels);
    fn(StateContext) *state;

    s->x_pos = av_calloc(s->aswift_size, sizeof(ftype));
    if (!s->x_pos)
        return AVERROR(ENOMEM);

    s->y_pos = av_calloc(s->aswift_size, sizeof(ftype));
    if (!s->y_pos)
        return AVERROR(ENOMEM);

    s->state = av_calloc(channels, sizeof(*state));
    if (!s->state)
        return AVERROR(ENOMEM);
    s->channels = channels;
    state = s->state;

    for (int ch = 0; ch < channels; ch++) {
        fn(StateContext) *stc = &state[ch];

        stc->aswift = av_calloc(s->aswift_size, sizeof(*stc->aswift));
        if (!stc->aswift)
            return AVERROR(ENOMEM);

        stc->iaswift = av_calloc(s->aswift_size, sizeof(*stc->iaswift));
        if (!stc->iaswift)
            return AVERROR(ENOMEM);

        stc->plan.size = s->aswift_size;
        stc->plan.wf = av_calloc(stc->plan.size, sizeof(*stc->plan.wf));
        if (!stc->plan.wf)
            return AVERROR(ENOMEM);

        stc->plan.ws = av_calloc(stc->plan.size, sizeof(*stc->plan.ws));
        if (!stc->plan.ws)
            return AVERROR(ENOMEM);

        stc->plan.a_fast = FEXP(-F(1.0) / 10);
        stc->plan.a_slow = FEXP(-F(1.0) / 50);

        stc->plan.x_fast = av_calloc(s->aswift_size, sizeof(*stc->plan.x_fast));
        if (!stc->plan.x_fast)
            return AVERROR(ENOMEM);

        stc->plan.x_slow = av_calloc(s->aswift_size, sizeof(*stc->plan.x_slow));
        if (!stc->plan.x_slow)
            return AVERROR(ENOMEM);

        stc->plan.p_fast = av_calloc(s->aswift_size, sizeof(*stc->plan.p_fast));
        if (!stc->plan.p_fast)
            return AVERROR(ENOMEM);

        stc->plan.p_slow = av_calloc(s->aswift_size, sizeof(*stc->plan.p_slow));
        if (!stc->plan.p_slow)
            return AVERROR(ENOMEM);

        for (int n = 0; n < stc->plan.size; n++) {
            stc->plan.wf[n] = fn(polar)(F(1.0), (n+1) * F(-2.0)*F(M_PI) / stc->plan.size);
            stc->plan.wf[n] = fn(aswift_mul_real)(stc->plan.wf[n], stc->plan.a_fast);
            stc->plan.ws[n] = fn(polar)(F(1.0), (n+1) * F(-2.0)*F(M_PI) / stc->plan.size);
            stc->plan.ws[n] = fn(aswift_mul_real)(stc->plan.ws[n], stc->plan.a_slow);
        }
    }

    return 0;
}

static int fn(angle)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AUpmixContext *s = ctx->priv;
    const int N = s->aswift_size;
    const int start = (N * jobnr) / nb_jobs;
    const int end = (N * (jobnr+1)) / nb_jobs;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc0 = &state[0];
    fn(StateContext) *stc1 = &state[1];
    fn(ctype) *aswift0 = stc0->aswift;
    fn(ctype) *aswift1 = stc1->aswift;
    const ftype smooth = s->smooth;
    const ftype asmooth = F(1.0) - smooth;
    ftype *x_pos = s->x_pos;
    ftype *y_pos = s->y_pos;

    for (int n = start; n < end; n++) {
        ftype l_re = aswift0[n].re, r_re = aswift1[n].re;
        ftype l_im = aswift0[n].im, r_im = aswift1[n].im;
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ftype x = (r_mag-l_mag)/(r_mag+l_mag+EPS);
        ftype re, im, a, y;

        re = l_re * r_re + l_im * r_im;
        im = r_re * l_im - r_im * l_re;
        a = ATAN2(im, re);
        y = F(1.0) - FABS(a * F(M_2_PI));
        x = CLIP(x, F(-1.0), F(1.0));
        y = CLIP(y, F(-1.0), F(1.0));

        x = x_pos[n] * asmooth + x * smooth;
        y = y_pos[n] * asmooth + y * smooth;
        x_pos[n] = isnormal(x) ? x : F(0.0);
        y_pos[n] = isnormal(y) ? y : F(0.0);
    }

    return 0;
}

static int fn(upmix)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AUpmixContext *s = ctx->priv;
    const int nb_out_channels = s->out_ch_layout.nb_channels;
    const int N = s->aswift_size;
    const int start = (N * jobnr) / nb_jobs;
    const int end = (N * (jobnr+1)) / nb_jobs;

    for (int ch = 0; ch < nb_out_channels; ch++) {
        const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);
        const int sc_chan = sc_map[chan];
        fn(StateContext) *state = s->state;
        fn(StateContext) *stc0 = &state[0];
        fn(StateContext) *stc1 = &state[1];
        fn(StateContext) *stc = &state[ch];
        const fn(ctype) *srcl = stc0->aswift;
        const fn(ctype) *srcr = stc1->aswift;
        const ftype ch_x = sc_ch_pos[sc_chan][0];
        const ftype ch_y = sc_ch_pos[sc_chan][1];
        const ftype sign0 = sc_ch_pos[sc_chan][3];
        const ftype sign1 = sc_ch_pos[sc_chan][4];
        fn(ctype) *dst = stc->iaswift;
        const ftype *x = s->x_pos;
        const ftype *y = s->y_pos;

        if (chan == AV_CHAN_NONE)
            continue;

        for (int n = start; n < end; n++) {
            const ftype xx = (FABS(ch_x - x[n]) + FABS(ch_y - y[n])) * F(-0.5);
            const ftype xf = FEXP10(xx);
            const fn(ctype) l = srcl[n];
            const fn(ctype) r = srcr[n];
            fn(ctype) ret;

            ret.re = sign0 * xf * l.re + xf * r.re * sign1;
            ret.im = sign0 * xf * l.im + xf * r.im * sign1;

            dst[n] = ret;
        }
    }

    return 0;
}

static int fn(upmix_in)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AUpmixContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *in = td->frame;
    const int idx = td->idx;
    const int nb_in_channels = in->ch_layout.nb_channels;
    const int N = s->aswift_size;
    const int start = (N * jobnr) / nb_jobs;
    const int end = (N * (jobnr+1)) / nb_jobs;
    fn(StateContext) *state = s->state;

    for (int ch = 0; ch < nb_in_channels; ch++) {
        fn(StateContext) *stc = &state[ch];
        const ftype *src = (const ftype *)in->extended_data[ch];
        fn(aswift_plan) *plan = &stc->plan;
        fn(ctype) *out = stc->aswift;
        const fn(ctype) *wf = plan->wf;
        const fn(ctype) *ws = plan->ws;
        fn(ctype) *fast = plan->x_fast;
        fn(ctype) *slow = plan->x_slow;
        fn(ctype) *pfast = plan->p_fast;
        fn(ctype) *pslow = plan->p_slow;
        const fn(ctype) csample = fn(polar)(src[idx], F(0.0));

        for (int n = start; n < end; n++) {
            fast[n] = fn(aswift_add)(fn(aswift_mul)(pfast[n], wf[n]), csample);
            slow[n] = fn(aswift_add)(fn(aswift_mul)(pslow[n], ws[n]), csample);

            out[n].re = slow[n].re - fast[n].re;
            out[n].im = slow[n].im - fast[n].im;

            pfast[n].re = isnormal(fast[n].re) ? fast[n].re : F(0.0);
            pfast[n].im = isnormal(fast[n].im) ? fast[n].im : F(0.0);
            pslow[n].re = isnormal(slow[n].re) ? slow[n].re : F(0.0);
            pslow[n].im = isnormal(slow[n].im) ? slow[n].im : F(0.0);
        }
    }

    return 0;
}

static int fn(upmix_out)(AVFilterContext *ctx, AVFrame *out, const int ch,
                         const int idx)
{
    AUpmixContext *s = ctx->priv;
    const ftype gain = s->gain;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    fn(aswift_plan) *plan = &stc->plan;
    fn(ctype) *in = stc->iaswift;
    const int N = plan->size;
    ftype acc = F(0.0);

    for (int n = 0; n < N; n += 2) {
        acc += in[n+0].re;
        acc -= in[n+1].re;
    }

    dst[idx] = acc * gain / N;

    return 0;
}
