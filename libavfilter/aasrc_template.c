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

#undef ctype
#undef ftype
#undef itype
#undef FLOG
#undef FSIN
#undef FCOS
#undef FEXP
#undef FLOOR
#undef LRINT
#undef CLIP
#undef dsp_vector_mul_real
#undef dsp_vector_mul_complex
#undef dsp_vector_mul_complex_add
#undef SAMPLE_FORMAT
#if DEPTH == 16
#define ctype complex_float
#define ftype float
#define itype int16_t
#define FLOG logf
#define FSIN sinf
#define FCOS cosf
#define FEXP expf
#define FLOOR floorf
#define LRINT lrintf
#define CLIP av_clip_int16
#define dsp_vector_mul_real dsp->vector_fmul_real
#define dsp_vector_mul_complex dsp->vector_fmul_complex
#define dsp_vector_mul_complex_add dsp->vector_fmul_complex_add
#define SAMPLE_FORMAT s16p
#elif DEPTH == 32
#define ctype complex_double
#define ftype double
#define itype int32_t
#define FLOG log
#define FSIN sin
#define FCOS cos
#define FEXP exp
#define FLOOR floor
#define LRINT lrint
#define CLIP av_clipl_int32
#define dsp_vector_mul_real dsp->vector_dmul_real
#define dsp_vector_mul_complex dsp->vector_dmul_complex
#define dsp_vector_mul_complex_add dsp->vector_dmul_complex_add
#define SAMPLE_FORMAT s32p
#elif DEPTH == 33
#define ctype complex_float
#define ftype float
#define itype float
#define FLOG logf
#define FSIN sinf
#define FCOS cosf
#define FEXP expf
#define FLOOR floorf
#define LRINT lrintf
#define dsp_vector_mul_real dsp->vector_fmul_real
#define dsp_vector_mul_complex dsp->vector_fmul_complex
#define dsp_vector_mul_complex_add dsp->vector_fmul_complex_add
#define SAMPLE_FORMAT fltp
#else
#define ctype complex_double
#define ftype double
#define itype double
#define FLOG log
#define FSIN sin
#define FCOS cos
#define FEXP exp
#define FLOOR floor
#define LRINT lrint
#define dsp_vector_mul_real dsp->vector_dmul_real
#define dsp_vector_mul_complex dsp->vector_dmul_complex
#define dsp_vector_mul_complex_add dsp->vector_dmul_complex_add
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))
#undef isnormal
#define isnormal(x) (1)

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define fnc3(a,b)   a##_##b
#define fnc2(a,b)   fn3(a, b)
#define fnc(a)      fnc2(a, ftype)

#define K1 32
#define MAX_NB_POLES 12
#define MAX_HISTORY 4

typedef struct fn(StateContext) {
    ftype scale_factor;
    ftype log_mag[MAX_NB_POLES];
    ftype angle[MAX_NB_POLES];
    ftype log_mag_scaled[MAX_NB_POLES];
    ftype angle_scaled[MAX_NB_POLES];
    ctype inv[MAX_NB_POLES];
    ctype p_fixed[MAX_NB_POLES];
    ctype r_fixed[MAX_NB_POLES];

    int   reset_index;
    int   in_idx;
    int   out_idx;
    int   nb_poles;
    ftype delta_t;
    ftype reset_delta_t;
    ftype t_inc_frac;
    int   t_inc_int;
    const ctype *adv_ptr;
    ctype one[MAX_NB_POLES];
    ctype cur[MAX_NB_POLES];
    ctype adv[2][MAX_NB_POLES];
    ctype h[MAX_NB_POLES];

    int   prev_index;
    ftype prev_delta_t[MAX_HISTORY];
    ctype prev_cur[MAX_HISTORY][MAX_NB_POLES];
} fn(StateContext);

static void fn(complex_exponential)(fn(StateContext) *stc,
                                    ctype *x,
                                    const ftype *log_mag,
                                    const ftype *theta,
                                    const ftype delta_t,
                                    const int N)
{
    ftype *prev_delta_t = stc->prev_delta_t;

    for (int n = 0; n < MAX_HISTORY; n++) {
        if (prev_delta_t[n] == delta_t) {
            memcpy(x, stc->prev_cur[n], MAX_NB_POLES * sizeof(*x));
            return;
        }
    }

    for (int n = 0; n < N; n++) {
        ftype mag = FEXP(log_mag[n] * delta_t);
        ftype re, im, w;

        w = theta[n] * delta_t;

        re = mag * FCOS(w);
        im = mag * FSIN(w);

        x[n].re = isnormal(re) ? re : F(0.0);
        x[n].im = isnormal(im) ? im : F(0.0);
    }

    prev_delta_t[stc->prev_index] = delta_t;
    memcpy(stc->prev_cur[stc->prev_index], x, MAX_NB_POLES * sizeof(*x));

    stc->prev_index++;
    if (stc->prev_index >= MAX_HISTORY)
        stc->prev_index = 0;
}

static void fn(vector_mul_complex)(ctype *x,
                                   const ctype *a,
                                   const ctype *b,
                                   const int N)
{
    for (int n = 0; n < N; n++) {
        const ftype are = a[n].re;
        const ftype aim = a[n].im;
        const ftype bre = b[n].re;
        const ftype bim = b[n].im;
        ftype re = are * bre - aim * bim;
        ftype im = are * bim + aim * bre;

        x[n].re = isnormal(re) ? re : F(0.0);
        x[n].im = isnormal(im) ? im : F(0.0);
    }
}

#if DEPTH == 33 || DEPTH == 65
static ftype fn(vector_mul_real)(const ctype *cur,
                                 const ctype *h,
                                 const int N)
{
    ftype y = F(0.0);

    for (int n = 0; n < N; n++) {
        const ftype cre = cur[n].re;
        const ftype cim = cur[n].im;
        const ftype re = h[n].re;
        const ftype im = h[n].im;

        y += re * cre - im * cim;
    }

    return y;
}

static void fn(vector_mul_complex_add)(const ftype src,
                                       const ctype *fixed,
                                       const ctype *in,
                                       ctype *out,
                                       const int N)
{
    for (int n = 0; n < N; n++) {
        const ctype a = fixed[n];
        ctype z = in[n];
        ftype re, im;

        re = z.re * a.re - z.im * a.im + src;
        im = z.re * a.im + z.im * a.re;

        z.re = re;
        z.im = im;

        out[n] = z;
    }
}
#endif

static int fn(aasrc_prepare)(AVFilterContext *ctx, fn(StateContext) *stc,
                             const double t_inc)
{
    AASRCContext *s = ctx->priv;
    const double (*ps)[2];
    const double (*rs)[2];

    stc->scale_factor = (t_inc > 1.0) ? F(1.0) / t_inc : F(1.0);
    stc->out_idx = 0;
    stc->in_idx = 0;
    stc->delta_t = F(0.0);
    stc->reset_index = 0;
    stc->t_inc_frac = t_inc - FLOOR(t_inc);
    stc->t_inc_int = LRINT(t_inc - stc->t_inc_frac);

    switch (s->coeffs) {
    case 0:
        stc->nb_poles = FF_ARRAY_ELEMS(ps0);
        ps = ps0;
        rs = rs0;
        break;
    default:
        return AVERROR(EINVAL);
    }

    for (int n = 0; n < MAX_HISTORY; n++)
        stc->prev_delta_t[n] = F(-1.0);

    for (int n = 0; n < stc->nb_poles; n++) {
        ftype inv_mag, p_cos, p_sin, mag;
        ftype re, im, a, b;

        stc->log_mag[n] = FLOG(ps[n][0]);
        stc->angle[n] = ps[n][1];
        a = stc->log_mag[n] * stc->scale_factor;
        b = stc->angle[n] * stc->scale_factor;
        stc->log_mag_scaled[n] = isnormal(a) ? a : F(0.0);
        stc->angle_scaled[n] = isnormal(b) ? b : F(0.0);

        stc->one[n].re = F(1.0);
        stc->one[n].im = F(0.0);
        stc->cur[n] = stc->one[n];
        re = rs[n][0] * stc->scale_factor;
        im = rs[n][1] * stc->scale_factor;
        stc->r_fixed[n].re = isnormal(re) ? re : F(0.0);
        stc->r_fixed[n].im = isnormal(im) ? im : F(0.0);

        inv_mag = FEXP(-stc->log_mag_scaled[n]);
        p_cos = FCOS(stc->angle_scaled[n]);
        p_sin = FSIN(stc->angle_scaled[n]);
        mag = FEXP(stc->log_mag_scaled[n]);

        re = inv_mag *  p_cos;
        im = inv_mag * -p_sin;

        stc->inv[n].re = isnormal(re) ? re : F(0.0);
        stc->inv[n].im = isnormal(im) ? im : F(0.0);

        re = mag * p_cos;
        im = mag * p_sin;

        stc->p_fixed[n].re = isnormal(re) ? re : F(0.0);
        stc->p_fixed[n].im = isnormal(im) ? im : F(0.0);
    }

    fn(vector_mul_complex)(stc->cur, stc->cur, stc->r_fixed, stc->nb_poles);

    stc->adv_ptr = stc->one;

    fn(complex_exponential)(stc, stc->adv[0], stc->log_mag_scaled, stc->angle_scaled, stc->t_inc_frac, stc->nb_poles);
    fn(vector_mul_complex)(stc->adv[1], stc->adv[0], stc->inv, stc->nb_poles);

    return 0;
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
        int ret;

        ret = fn(aasrc_prepare)(ctx, stc, s->t_inc);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static void fn(aasrc)(AVFilterContext *ctx, AVFrame *in, AVFrame *out,
                      const int ch)
{
    AASRCContext *s = ctx->priv;
    const AudioASRCDSPContext *const dsp = &s->dsp;
    const itype *src = (const itype *)in->extended_data[ch];
    itype *dst = (itype *)out->extended_data[ch];
    const int n_out_samples = out->nb_samples;
    const int n_in_samples = in->nb_samples;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    const ftype t_inc_frac = stc->t_inc_frac;
    ftype reset_delta_t = stc->reset_delta_t;
    const int t_inc_int = stc->t_inc_int;
    const int nb_poles = stc->nb_poles;
    ftype delta_t = stc->delta_t;
    int in_idx = stc->in_idx;
    int reset_index = stc->reset_index;
    const ctype *p_fixed = stc->p_fixed;
    const ctype (*adv)[MAX_NB_POLES] = stc->adv;
    const ctype *adv_ptr = stc->adv_ptr;
    ctype *cur = stc->cur;
    int prev_in_idx = -1;
    ctype *h = stc->h;
    ftype x;
    int n;

    ftype (*vector_mul_real)(const ctype *cur,
                             const ctype *h,
                             const int N) = dsp_vector_mul_real;

    void (*vector_mul_complex)(ctype *x,
                               const ctype *a,
                               const ctype *b,
                               const int N) = dsp_vector_mul_complex;

    void (*vector_mul_complex_add)(const ftype src,
                                   const ctype *fixed,
                                   const ctype *in,
                                   ctype *out,
                                   const int N) = dsp_vector_mul_complex_add;

    n = 0;
repeat:
    if (reset_index >= K1) {
        fn(complex_exponential)(stc, cur, stc->log_mag_scaled, stc->angle_scaled, reset_delta_t, nb_poles);
        vector_mul_complex(cur, stc->r_fixed, cur, nb_poles);
        reset_index = 0;
    }

    while (n < n_out_samples && in_idx < n_in_samples && reset_index < K1) {
        int frac_carry, idx_inc;
        ftype delta_t_frac, y;

        if (prev_in_idx < in_idx) {
            x = src[in_idx];
#if DEPTH == 16 || DEPTH == 32
            x /= F(1<<(DEPTH-1));
#endif
            vector_mul_complex_add(x, p_fixed, h, h, nb_poles);
            prev_in_idx = in_idx;
        }

        vector_mul_complex(cur, cur, adv_ptr, nb_poles);

        y = vector_mul_real(cur, h, nb_poles);

#if DEPTH == 16 || DEPTH == 32
        dst[n] = CLIP(LRINT(y * F(1<<(DEPTH-1))));
#else
        dst[n] = y;
#endif

        reset_delta_t = delta_t;
        reset_index++;

        delta_t += t_inc_frac;
        delta_t_frac = delta_t - FLOOR(delta_t);
        frac_carry = LRINT(delta_t - delta_t_frac);
        idx_inc = frac_carry + t_inc_int;
        delta_t = delta_t_frac;

        adv_ptr = adv[frac_carry];

        for (int i = 1; i < idx_inc; i++) {
            x = src[in_idx+i];
#if DEPTH == 16 || DEPTH == 32
            x /= F(1<<(DEPTH-1));
#endif
            vector_mul_complex_add(x, p_fixed, h, h, nb_poles);
        }

        in_idx += idx_inc;
        n++;
    }

    if (n < n_out_samples && in_idx < n_in_samples)
        goto repeat;

    stc->out_idx = n;
    stc->adv_ptr = adv_ptr;
    stc->delta_t = delta_t;
    stc->reset_index = reset_index;
    stc->reset_delta_t = reset_delta_t;
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

    av_freep(&s->state);
}
