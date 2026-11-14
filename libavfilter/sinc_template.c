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
#undef FCOS
#undef FSIN
#undef FLOG
#undef SQRT
#undef FPOW
#undef CEIL
#undef FMINV
#undef HYPOT
#undef LRINT
#undef ftype
#undef ctype
#undef TX_TYPE
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define FEXP expf
#define FCOS cosf
#define FSIN sinf
#define FLOG logf
#define FPOW powf
#define CEIL ceilf
#define SQRT sqrtf
#define FMINV FLT_MIN
#define HYPOT hypotf
#define LRINT lrintf
#define ftype float
#define ctype AVComplexFloat
#define TX_TYPE AV_TX_FLOAT_RDFT
#define SAMPLE_FORMAT flt
#else
#define FEXP exp
#define FCOS cos
#define FSIN sin
#define FLOG log
#define FPOW pow
#define CEIL ceil
#define SQRT sqrt
#define FMINV DBL_MIN
#define HYPOT hypot
#define LRINT lrint
#define ftype double
#define ctype AVComplexDouble
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define SAMPLE_FORMAT dbl
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static ftype *fn(make_lpf)(int num_taps, ftype Fc, ftype beta,
                           ftype rho, ftype scale)
{
    int m = num_taps - 1;
    ftype mult = scale / av_bessel_i0(beta), mult1 = F(1.0) / (F(0.5) * m + rho);
    ftype *h = av_calloc(num_taps, sizeof(*h));

    if (!h)
        return NULL;

    av_assert0(Fc >= 0 && Fc <= 1);

    for (int i = 0; i <= m / 2; i++) {
        ftype z = i - F(0.5) * m, x = z * M_PI, y = z * mult1;
        h[i] = x ? FSIN(Fc * x) / x : Fc;
        h[i] *= av_bessel_i0(beta * SQRT(F(1.0) - y * y)) * mult;
        if (m - i != i)
            h[m - i] = h[i];
    }

    return h;
}

static ftype fn(kaiser_beta)(ftype att, ftype tr_bw)
{
    if (att >= F(60.0)) {
        static const ftype coefs[][4] = {
            {-6.784957e-10, 1.02856e-05, 0.1087556, -0.8988365 + .001},
            {-6.897885e-10, 1.027433e-05, 0.10876, -0.8994658 + .002},
            {-1.000683e-09, 1.030092e-05, 0.1087677, -0.9007898 + .003},
            {-3.654474e-10, 1.040631e-05, 0.1087085, -0.8977766 + .006},
            {8.106988e-09, 6.983091e-06, 0.1091387, -0.9172048 + .015},
            {9.519571e-09, 7.272678e-06, 0.1090068, -0.9140768 + .025},
            {-5.626821e-09, 1.342186e-05, 0.1083999, -0.9065452 + .05},
            {-9.965946e-08, 5.073548e-05, 0.1040967, -0.7672778 + .085},
            {1.604808e-07, -5.856462e-05, 0.1185998, -1.34824 + .1},
            {-1.511964e-07, 6.363034e-05, 0.1064627, -0.9876665 + .18},
        };
        ftype realm = FLOG(tr_bw / F(.0005)) / FLOG(2.0);
        ftype const *c0 = coefs[av_clip((int)realm, 0, FF_ARRAY_ELEMS(coefs) - 1)];
        ftype const *c1 = coefs[av_clip(1 + (int)realm, 0, FF_ARRAY_ELEMS(coefs) - 1)];
        ftype b0 = ((c0[0] * att + c0[1]) * att + c0[2]) * att + c0[3];
        ftype b1 = ((c1[0] * att + c1[1]) * att + c1[2]) * att + c1[3];

        return b0 + (b1 - b0) * (realm - (int)realm);
    }
    if (att > F(50.0))
        return F(0.1102) * (att - F(8.7));
    if (att > F(20.96))
        return F(0.58417) * FPOW(att - F(20.96), F(0.4)) + F(0.07886) * (att - F(20.96));
    return 0;
}

static void fn(kaiser_params)(ftype att, ftype Fc, ftype tr_bw, double *beta, int *num_taps)
{
    *beta = *beta < F(0.0) ? fn(kaiser_beta)(att, tr_bw): *beta;
    att = att < F(60.0) ? (att - F(7.95)) / (F(2.285) * M_PI * F(2.0)) :
        ((F(0.0007528358)-F(1.577737e-05) * *beta) * *beta + F(0.6248022)) * *beta + F(.06186902);
    *num_taps = !*num_taps ? CEIL(att/tr_bw + 1) : *num_taps;
}

static ftype *fn(lpf)(ftype Fn, ftype Fc, ftype tbw, int *num_taps, ftype att, double *beta, int round)
{
    int n = *num_taps;

    if ((Fc /= Fn) <= F(0.0) || Fc >= F(1.0)) {
        *num_taps = 0;
        return NULL;
    }

    fn(kaiser_params)(att, Fc, (tbw ? tbw : F(0.05)), beta, num_taps);

    if (!n) {
        n = *num_taps;
        if (round)
            *num_taps = 1 + 2 * (int)((int)((*num_taps / 2) * Fc + F(0.5)) / Fc + F(0.5));
    }

    return fn(make_lpf)(*num_taps |= 1, Fc, *beta, F(0.0), F(1.0));
}

static void fn(invert)(ftype *h, int n)
{
    for (int i = 0; i < n; i++)
        h[i] = -h[i];

    h[(n - 1) / 2] += F(1.0);
}

static void fn(gain)(ftype *h, int n, const ftype g, const ftype G)
{
    for (int i = 0; i < n; i++)
        h[i] *= g - F(1.0);

    h[(n - 1) / 2] += F(1.0);

    for (int i = 0; i < n; i++)
        h[i] *= G;
}

static ctype fn(cexp)(const ctype z)
{
    ftype x = z.re, y = z.im;
    ftype c, s;

    c = FCOS(y);
    s = FSIN(y);

    ftype ex = FEXP(x);
    return (ctype){ ex * c, ex * s };
}

static int fn(fir_to_phase)(const AVFilterContext *ctx, ftype *dst, const ftype *src, const int nb_taps, const ftype phase)
{
    const ftype a = F(1.0) - (F(1.0) + phase) * F(0.5);
    const int oversample = 4;
    const int fft_size = oversample << av_ceil_log2(nb_taps);
    const int fft_mid = fft_size/2;
    AVTXContext *tx_ctx = NULL, *itx_ctx;
    const ftype threshold = FMINV;
    const ftype logt = FLOG(threshold);
    const ftype iscale = F(1.0) / fft_size;
    const ftype scale = F(1.0);
    av_tx_fn tx_fn, itx_fn;
    ctype *fft_out = NULL;
    ftype *fft_in = NULL;
    int ret;

    ret = av_tx_init(&tx_ctx, &tx_fn, TX_TYPE, 0, fft_size, &scale, 0);
    if (ret < 0)
        return ret;

    ret = av_tx_init(&itx_ctx, &itx_fn, TX_TYPE, 1, fft_size, &iscale, 0);
    if (ret < 0)
        goto fail;

    fft_in = av_calloc(fft_size, sizeof(*fft_in));
    if (!fft_in) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    fft_out = av_calloc(fft_mid+1, sizeof(*fft_out));
    if (!fft_out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    for (int i = 0; i < nb_taps; i++)
        fft_in[i] = src[i];

    tx_fn(tx_ctx, fft_out, fft_in, sizeof(*fft_in));

    for (int i = 0; i < fft_mid+1; i++) {
        const ftype mag = HYPOT(fft_out[i].re, fft_out[i].im);

        fft_out[i].re = mag < threshold ? logt : FLOG(mag);
        fft_out[i].im = F(0.0);
    }

    itx_fn(itx_ctx, fft_in, fft_out, sizeof(*fft_out));

    for (int i = 1; i < fft_mid; i++)
        fft_in[i] *= F(2.0) * a;
    for (int i = fft_mid+1; i < fft_size; i++)
        fft_in[i] *= F(2.0) * (F(1.0) - a);

    tx_fn(tx_ctx, fft_out, fft_in, sizeof(*fft_in));

    for (int i = 0; i < fft_mid+1; i++) {
        const ftype re = fft_out[i].re;
        const ftype im = fft_out[i].im;
        ctype v;

        v.re = re;
        v.im = im;

        fft_out[i] = fn(cexp)(v);
    }

    itx_fn(itx_ctx, fft_in, fft_out, sizeof(*fft_out));

    int shift = LRINT((F(1.0) - a) * (nb_taps - 1));
    for (int i = 0; i < nb_taps; i++) {
        const int idx = (i - shift) % fft_size;

        dst[i] = fft_in[(idx < 0) ? idx + fft_size : idx];
    }

fail:
    av_freep(&fft_out);
    av_freep(&fft_in);

    av_tx_uninit(&itx_ctx);
    av_tx_uninit(&tx_ctx);

    return ret;
}

static int fn(generate)(AVFilterContext *ctx)
{
    SincContext *s = ctx->priv;
    ftype Fn = s->sample_rate * F(0.5);
    ftype *h[2], *coeffs;
    const ftype g = s->gain;
    int ret = 0, n, longer;

    if (s->Fc0 >= Fn || s->Fc1 >= Fn) {
        av_log(ctx, AV_LOG_ERROR,
               "filter frequency must be less than %d/2.\n", s->sample_rate);
        return AVERROR(EINVAL);
    }

    h[0] = fn(lpf)(Fn, s->Fc0, s->tbw0, &s->num_taps[0], s->att, &s->beta, s->round);
    h[1] = fn(lpf)(Fn, s->Fc1, s->tbw1, &s->num_taps[1], s->att, &s->beta, s->round);
    if (!h[0] && !h[1])
        return AVERROR(ENOMEM);

    if (h[0])
        fn(invert)(h[0], s->num_taps[0]);

    longer = s->num_taps[1] > s->num_taps[0];
    n = s->num_taps[longer];

    if (h[0] && h[1]) {
        for (int i = 0; i < s->num_taps[!longer]; i++)
            h[longer][i + (n - s->num_taps[!longer]) / 2] += h[!longer][i];

        if (s->Fc0 < s->Fc1) {
            fn(invert)(h[longer], n);
        } else if (g > F(0.0)) {
            fn(gain)(h[longer], n, F(1.0) / g, g);
        }

        av_free(h[!longer]);
    } else if (g > F(0.0)) {
        fn(gain)(h[longer], n, g, F(1.0));
    }

    if (s->phase != F(50.0)) {
        ret = fn(fir_to_phase)(ctx, h[longer], h[longer], n, s->phase * F(0.02) - F(1.0));
        if (ret < 0)
            goto fail;
    }

    s->n = 1 << av_ceil_log2(n);
    s->coeffs = coeffs = av_calloc(s->n, s->sample_size);
    if (!s->coeffs) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    for (int i = 0; i < n; i++)
        coeffs[i] = h[longer][i];

fail:
    av_free(h[longer]);

    return ret;
}
