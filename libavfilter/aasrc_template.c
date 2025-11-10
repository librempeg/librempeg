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
#undef itype
#undef FLOG2
#undef FSIN
#undef FCOS
#undef FEXP2
#undef FLOOR
#undef LRINT
#undef SAMPLE_FORMAT
#if DEPTH == 16
#define ftype float
#define itype int16_t
#define FLOG2 log2f
#define FSIN sinf
#define FCOS cosf
#define FEXP2 exp2f
#define FLOOR floorf
#define LRINT lrintf
#define SAMPLE_FORMAT s16p
#elif DEPTH == 32
#define ftype float
#define itype float
#define FLOG2 log2f
#define FSIN sinf
#define FCOS cosf
#define FEXP2 exp2f
#define FLOOR floorf
#define LRINT lrintf
#define SAMPLE_FORMAT fltp
#else
#define ftype double
#define itype double
#define FLOG2 log2
#define FSIN sin
#define FCOS cos
#define FEXP2 exp2
#define FLOOR floor
#define LRINT lrint
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))
#undef isnormal
#define isnormal(x) (1)

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define MAX_NB_POLES 8

typedef struct fn(complex_ftype) {
    ftype re, im;
} fn(complex_ftype);

#define ctype fn(complex_ftype)

typedef struct fn(StateContext) {
    int   K1;

    ftype scale_factor;
    ftype Log2MagP[MAX_NB_POLES];
    ftype thetaP[MAX_NB_POLES];
    ftype Log2MagP_Fsf[MAX_NB_POLES];
    ftype thetaP_Fsf[MAX_NB_POLES];
    ctype pInv[MAX_NB_POLES];
    ctype pFixed[MAX_NB_POLES];
    ctype rFixed[MAX_NB_POLES];

    int   reset_index;
    int   reset_phasors;
    int   in_idx;
    int   out_idx;
    int   nb_poles;
    ftype delta_t;
    ftype t_inc_frac;
    int   t_inc_int;
    ctype *pAdv;
    ctype pCur[MAX_NB_POLES];
    ctype pAdvUp[MAX_NB_POLES];
    ctype pAdvDown[MAX_NB_POLES];
    ctype filter_state[MAX_NB_POLES];

    ctype *hat;
    int   hat_samples;
} fn(StateContext);

static void fn(complex_exponential)(ctype *x,
                                    const ftype *Log2MagP,
                                    const ftype *theta,
                                    const ftype delta_t,
                                    const int N)
{
    for (int n = 0; n < N; n++) {
        ftype mag = FEXP2(Log2MagP[n] * delta_t);
        ftype re, im;

        re = mag * FCOS(theta[n] * delta_t);
        im = mag * FSIN(theta[n] * delta_t);

        x[n].re = isnormal(re) ? re : F(0.0);
        x[n].im = isnormal(im) ? im : F(0.0);
    }
}

static void fn(vector_mul_complex)(ctype *x,
                                   const ctype *a,
                                   const ctype *b,
                                   const int N)
{
    for (int n = 0; n < N; n++) {
        ftype re = a[n].re * b[n].re - a[n].im * b[n].im;
        ftype im = a[n].re * b[n].im + a[n].im * b[n].re;

        x[n].re = isnormal(re) ? re : F(0.0);
        x[n].im = isnormal(im) ? im : F(0.0);
    }
}

static void fn(aasrc_prepare)(AVFilterContext *ctx, fn(StateContext) *stc,
                              const double t_inc)
{
    stc->scale_factor = (t_inc > 1.0) ? F(1.0) / t_inc : F(1.0);
    stc->K1 = 32;
    stc->out_idx = 0;
    stc->in_idx = 0;
    stc->delta_t = F(0.0);
    stc->reset_index = 0;
    stc->t_inc_frac = t_inc - FLOOR(t_inc);
    stc->t_inc_int = LRINT(t_inc - stc->t_inc_frac);
    stc->reset_phasors = 1;
    stc->nb_poles = FF_ARRAY_ELEMS(ps1);

    for (int n = 0; n < stc->nb_poles; n++) {
        ftype re, im;

        stc->pCur[n].re = F(1.0);
        stc->pCur[n].im = F(0.0);
        re = rs1[n][0] * stc->scale_factor;
        im = rs1[n][1] * stc->scale_factor;
        stc->rFixed[n].re = isnormal(re) ? re : F(0.0);
        stc->rFixed[n].im = isnormal(im) ? im : F(0.0);
    }

    fn(vector_mul_complex)(stc->pCur, stc->pCur, stc->rFixed, stc->nb_poles);

    for (int n = 0; n < stc->nb_poles; n++) {
        ftype a, b;

        stc->Log2MagP[n] = FLOG2(ps1[n][0]);
        stc->thetaP[n] = ps1[n][1];
        a = stc->Log2MagP[n] * stc->scale_factor;
        b = stc->thetaP[n] * stc->scale_factor;
        stc->Log2MagP_Fsf[n] = isnormal(a) ? a : F(0.0);
        stc->thetaP_Fsf[n] = isnormal(b) ? b : F(0.0);
    }

    stc->pAdv = NULL;

    for (int n = 0; n < stc->nb_poles; n++) {
        const ftype pInvMag = FEXP2(-stc->Log2MagP_Fsf[n]);
        const ftype pCos = FCOS(stc->thetaP_Fsf[n]);
        const ftype pSin = FSIN(stc->thetaP_Fsf[n]);
        const ftype pMag = FEXP2(stc->Log2MagP_Fsf[n]);
        ftype re, im;

        re = pInvMag *  pCos;
        im = pInvMag * -pSin;

        stc->pInv[n].re = isnormal(re) ? re : F(0.0);
        stc->pInv[n].im = isnormal(im) ? im : F(0.0);

        re = pMag * pCos;
        im = pMag * pSin;

        stc->pFixed[n].re = isnormal(re) ? re : F(0.0);
        stc->pFixed[n].im = isnormal(im) ? im : F(0.0);
    }

    fn(complex_exponential)(stc->pAdvDown, stc->Log2MagP_Fsf, stc->thetaP_Fsf, stc->t_inc_frac, stc->nb_poles);
    fn(vector_mul_complex)(stc->pAdvUp, stc->pAdvDown, stc->pInv, stc->nb_poles);

    memset(stc->filter_state, 0, sizeof(stc->filter_state));
}

static int fn(aasrc_init)(AVFilterContext *ctx)
{
    AASRCContext *s = ctx->priv;
    fn(StateContext) *state;

    s->state = av_calloc(s->channels, sizeof(*state));
    if (!s->state)
        return AVERROR(ENOMEM);
    state = s->state;

    for (int ch = 0; ch < s->channels; ch++) {
        fn(StateContext) *stc = &state[ch];

        fn(aasrc_prepare)(ctx, stc, s->t_inc);
    }

    return 0;
}

static void fn(aasrc)(AVFilterContext *ctx, AVFrame *in, AVFrame *out,
                      const int ch)
{
    AASRCContext *s = ctx->priv;
    const itype *src = (const itype *)in->extended_data[ch];
    itype *dst = (itype *)out->extended_data[ch];
    const int n_out_samples = out->nb_samples;
    const int n_in_samples = in->nb_samples;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    const ftype t_inc_frac = stc->t_inc_frac;
    const int t_inc_int = stc->t_inc_int;
    const int nb_poles = stc->nb_poles;
    ftype delta_t = stc->delta_t;
    int in_idx = stc->in_idx;
    const ctype *pAdvDown = stc->pAdvDown;
    const ctype *pAdvUp = stc->pAdvUp;
    const ctype *pAdv = stc->pAdv;

    if (stc->hat_samples < n_in_samples) {
        stc->hat = av_realloc_f(stc->hat, n_in_samples, nb_poles * sizeof(*stc->hat));
        if (!stc->hat)
            return;

        stc->hat_samples = n_in_samples;
    }

    for (int n = 0; n < nb_poles; n++) {
        ctype *h = stc->hat + n * n_in_samples;
        const ctype a = stc->pFixed[n];
        ctype z = stc->filter_state[n];

        for (int i = 0; i < n_in_samples; i++) {
            ftype re, im, x;

#if DEPTH == 16
            x = src[i] / F(1<<(DEPTH-1));
#else
            x = src[i];
#endif
            re = z.re * a.re - z.im * a.im + x;
            im = z.re * a.im + z.im * a.re;

            h[i].re = z.re = isnormal(re) ? re : F(0.0);
            h[i].im = z.im = isnormal(im) ? im : F(0.0);
        }

        stc->filter_state[n].re = z.re;
        stc->filter_state[n].im = z.im;
    }

    for (int n = 0; n < n_out_samples && in_idx < n_in_samples; n++) {
        ftype delta_t_frac, y = F(0.0);
        const ctype *h = stc->hat;
        int frac_carry;

        if (pAdv)
            fn(vector_mul_complex)(stc->pCur, stc->pCur, pAdv, nb_poles);

        {
            const ctype *const pCur = stc->pCur;

            for (int i = 0; i < nb_poles; i++) {
                const ftype re = h[in_idx].re;
                const ftype im = h[in_idx].im;
                const ftype cre = pCur[i].re;
                const ftype cim = pCur[i].im;

                y += re * cre - im * cim;
                h += n_in_samples;
            }
        }
#if DEPTH == 16
        dst[n] = LRINT(y * F(1<<(DEPTH-1)));
#else
        dst[n] = y;
#endif

        if (stc->reset_phasors) {
            if (stc->reset_index >= stc->K1) {
                fn(complex_exponential)(stc->pCur, stc->Log2MagP_Fsf, stc->thetaP_Fsf, delta_t, nb_poles);
                fn(vector_mul_complex)(stc->pCur, stc->rFixed, stc->pCur, nb_poles);
                stc->reset_index = 0;
            }

            stc->reset_index++;
        }

        delta_t += t_inc_frac;
        delta_t_frac = delta_t - FLOOR(delta_t);
        frac_carry = LRINT(delta_t - delta_t_frac);
        in_idx += frac_carry + t_inc_int;
        delta_t = delta_t_frac;

        pAdv = (frac_carry == 0) ? pAdvDown : pAdvUp;

        stc->out_idx = n+1;
    }

    stc->pAdv = (ctype *)pAdv;
    stc->delta_t = delta_t;
    stc->in_idx = FFMAX(0, in_idx - n_in_samples);
}

static int fn(nb_output_samples)(AVFilterContext *ctx)
{
    AASRCContext *s = ctx->priv;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[0];

    return stc->out_idx;
}

static void fn(aasrc_uninit)(AVFilterContext *ctx)
{
    AASRCContext *s = ctx->priv;

    if (s->state) {
        fn(StateContext) *state = s->state;

        for (int ch = 0; ch < s->channels; ch++) {
            fn(StateContext) *stc = &state[ch];

            av_freep(&stc->hat);
        }

        av_freep(&s->state);
    }
}
