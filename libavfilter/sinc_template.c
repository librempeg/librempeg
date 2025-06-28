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

#undef FABS
#undef FEXP
#undef FCOS
#undef FSIN
#undef FLOG
#undef SQRT
#undef FPOW
#undef CEIL
#undef ATAN2
#undef FMINV
#undef ftype
#undef ctype
#undef TX_TYPE
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define FABS fabsf
#define FEXP expf
#define FCOS cosf
#define FSIN sinf
#define FLOG logf
#define FPOW powf
#define CEIL ceilf
#define SQRT sqrtf
#define ATAN2 atan2f
#define FMINV FLT_MIN
#define ftype float
#define ctype AVComplexFloat
#define TX_TYPE AV_TX_FLOAT_RDFT
#define SAMPLE_FORMAT flt
#else
#define FABS fabs
#define FEXP exp
#define FCOS cos
#define FSIN sin
#define FLOG log
#define FPOW pow
#define CEIL ceil
#define SQRT sqrt
#define ATAN2 atan2
#define FMINV DBL_MIN
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

#define SQR(a) ((a) * (a))

static ftype fn(safe_log)(ftype x)
{
    av_assert0(x >= F(0.0));
    if (x > FMINV)
        return FLOG(x);
    return FLOG(FMINV);
}

static int fn(fir_to_phase)(SincContext *s, ftype **h, int *len, int *post_len, ftype phase)
{
    ftype *pi_wraps, *work, phase1 = (phase > F(50.0) ? F(100.0) - phase : phase) / F(50.0);
    int work_len, begin, end, imp_peak = 0, peak = 0, ret;
    ftype imp_sum = 0, peak_imp_sum = 0, scale = F(1.0), iscale;
    ftype prev_angle2 = 0, cum_2pi = 0, prev_angle1 = 0, cum_1pi = 0;

    work_len = 4 << av_ceil_log2(*len);

    /* The first part is for work (+2 for (UN)PACK), the latter for pi_wraps. */
    work = av_calloc((work_len + 2) + ((work_len + 2) / 2), sizeof(ftype));
    if (!work)
        return AVERROR(ENOMEM);
    pi_wraps = &work[work_len + 2];

    memcpy(work, *h, *len * sizeof(*work));

    iscale = F(1.0) / work_len;

    av_tx_uninit(&s->tx);
    av_tx_uninit(&s->itx);
    ret = av_tx_init(&s->tx,  &s->tx_fn,  TX_TYPE, 0, work_len, &scale, AV_TX_INPLACE);
    if (ret < 0)
        goto fail;
    ret = av_tx_init(&s->itx, &s->itx_fn, TX_TYPE, 1, work_len, &iscale, AV_TX_INPLACE);
    if (ret < 0)
        goto fail;

    s->tx_fn(s->tx, work, work, sizeof(ftype));   /* Cepstral: */

    for (int i = 0; i <= work_len; i += 2) {
        ftype angle = ATAN2(work[i + 1], work[i]);
        ftype detect = 2 * M_PI;
        ftype delta = angle - prev_angle2;
        ftype adjust = detect * ((delta < -detect * F(0.7)) - (delta > detect * F(0.7)));

        prev_angle2 = angle;
        cum_2pi += adjust;
        angle += cum_2pi;
        detect = M_PI;
        delta = angle - prev_angle1;
        adjust = detect * ((delta < -detect * F(0.7)) - (delta > detect * F(0.7)));
        prev_angle1 = angle;
        cum_1pi += FABS(adjust);        /* fabs for when 2pi and 1pi have combined */
        pi_wraps[i >> 1] = cum_1pi;

        work[i] = fn(safe_log)(SQRT(SQR(work[i]) + SQR(work[i + 1])));
        work[i + 1] = F(0.0);
    }

    s->itx_fn(s->itx, work, work, sizeof(ctype));

    for (int i = 1; i < work_len/2; i++) { /* Window to reject acausal components */
        work[i] *= F(2.0);
        work[i + work_len/2] = F(0.0);
    }
    s->tx_fn(s->tx, work, work, sizeof(ftype));

    for (int i = 2; i < work_len; i += 2)   /* Interpolate between linear & min phase */
        work[i + 1] = phase1 * i / work_len * pi_wraps[work_len >> 1] + (F(1.0) - phase1) * (work[i + 1] + pi_wraps[i >> 1]) - pi_wraps[i >> 1];

    work[0] = FEXP(work[0]);
    work[1] = F(0.0);
    for (int i = 2; i < work_len; i += 2) {
        ftype x = FEXP(work[i]);

        work[i    ] = x * FCOS(work[i + 1]);
        work[i + 1] = x * FSIN(work[i + 1]);
    }
    work[work_len] = FEXP(work[work_len]);
    work[work_len+1] = F(0.0);

    s->itx_fn(s->itx, work, work, sizeof(ctype));

    /* Find peak pos. */
    for (int i = 0; i <= lrintf(pi_wraps[work_len >> 1] / M_PI); i++) {
        imp_sum += work[i];
        if (FABS(imp_sum) > FABS(peak_imp_sum)) {
            peak_imp_sum = imp_sum;
            peak = i;
        }
        if (work[i] > work[imp_peak])   /* For debug check only */
            imp_peak = i;
    }

    while (peak && FABS(work[peak - 1]) > FABS(work[peak]) && (work[peak - 1] * work[peak] > 0)) {
        peak--;
    }

    if (phase == F(0.0)) {
        begin = 0;
    } else if (phase == F(100.0)) {
        begin = peak - *len / 2;
    } else {
        begin = (F(0.997) - (2 - phase1) * F(0.22)) * *len + F(0.5);
        end = (F(0.997) + (0 - phase1) * F(0.22)) * *len + F(0.5);
        begin = peak - (begin & ~3);
        end = peak + 1 + ((end + 3) & ~3);
        *len = end - begin;
        *h = av_realloc_f(*h, *len, sizeof(**h));
        if (!*h) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }

    for (int i = 0; i < *len; i++)
        (*h)[i] = work[(begin + (phase > F(50.0) ? *len - 1 - i : i) + work_len) & (work_len - 1)];
    *post_len = phase > F(50.0) ? peak - begin : begin + *len - (peak + 1);

    av_log(s, AV_LOG_DEBUG, "%d nPI=%g peak-sum@%i=%g (val@%i=%g); len=%i post=%i (%g%%)\n",
           work_len, pi_wraps[work_len >> 1] / M_PI, peak, peak_imp_sum, imp_peak,
           work[imp_peak], *len, *post_len, F(100.0) - F(100.0) * *post_len / (*len - 1));

fail:
    av_free(work);

    return ret;
}

static int fn(generate)(AVFilterContext *ctx)
{
    SincContext *s = ctx->priv;
    ftype Fn = s->sample_rate * F(0.5);
    ftype *h[2], *coeffs;
    int ret = 0, n, post_peak, longer;

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

        if (s->Fc0 < s->Fc1)
            fn(invert)(h[longer], n);

        av_free(h[!longer]);
    }

    if (s->phase != F(50.0)) {
        ret = fn(fir_to_phase)(s, &h[longer], &n, &post_peak, s->phase);
        if (ret < 0)
            goto fail;
    } else {
        post_peak = n >> 1;
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
    av_tx_uninit(&s->tx);
    av_tx_uninit(&s->itx);

    return ret;
}
