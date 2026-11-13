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
#undef FABS
#undef FSIN
#undef FCOS
#undef FEXP
#undef FEXP10
#undef FLOG10
#undef FLOG
#undef FMINV
#undef FMAX
#undef LRINT
#undef SAMPLE_FORMAT
#undef TX_TYPE
#if DEPTH == 32
#define FABS fabsf
#define FSIN sinf
#define FCOS cosf
#define FEXP expf
#define FEXP10 ff_exp10f
#define FLOG logf
#define FLOG10 log10f
#define FMINV FLT_MIN
#define FMAX fmaxf
#define SAMPLE_FORMAT flt
#define LRINT lrintf
#define ftype float
#define ctype AVComplexFloat
#define TX_TYPE AV_TX_FLOAT_RDFT
#else
#define FABS fabs
#define FSIN sin
#define FCOS cos
#define FEXP exp
#define FEXP10 ff_exp10
#define FLOG log
#define FLOG10 log10
#define FMINV DBL_MIN
#define FMAX fmax
#define LRINT lrint
#define SAMPLE_FORMAT dbl
#define ftype double
#define ctype AVComplexDouble
#define TX_TYPE AV_TX_DOUBLE_RDFT
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define F(x) ((ftype)(x))

static ftype fn(analog_mag_db)(const ftype f, const double *tau)
{
    ftype w = F(2.0) * F(M_PI) * f;
    ftype a = w * tau[2];
    ftype b = w * tau[1];
    ftype db = F(10.0) * (FLOG10(F(1.0) + b*b) - FLOG10(F(1.0) + a*a));

    if (isinf(tau[0])) {
        db -= F(20.0) * FLOG10(w);
    } else {
        ftype c = w * tau[0];
        db -= F(10.0) * FLOG10(F(1.0) + c*c);
    }

    return db;
}

static av_cold int fn(config_output)(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioFIRWSourceContext *s = ctx->priv;
    const ftype percentage = s->percentage * F(0.01);
    const ftype sgn = s->invert ? F(-1.0) : F(1.0);
    int fft_size, fft_mid, ret;
    ctype *complexf;
    ftype *realf;
    ftype *taps;
    ftype scale;

    fft_size = 1 << av_ceil_log2(s->nb_taps+1);
    fft_mid = fft_size/2;
    s->realf = realf = av_calloc(fft_size, sizeof(*realf));
    if (!s->realf)
        return AVERROR(ENOMEM);

    s->complexf = complexf = av_calloc(fft_mid+1, sizeof(*complexf));
    if (!s->complexf)
        return AVERROR(ENOMEM);

    scale = F(1.0) / fft_size;
    ret = av_tx_init(&s->itx_ctx, &s->itx_fn, TX_TYPE, 1, fft_size, &scale, 0);
    if (ret < 0)
        return ret;

    s->taps = taps = av_calloc(s->nb_taps, sizeof(*taps));
    if (!s->taps)
        return AVERROR(ENOMEM);

    const ftype center = sgn * percentage * fn(analog_mag_db)(s->normf, w_curve[s->curve].tau);
    for (int i = 0; i < fft_mid+1; i++) {
        const ftype f = FMAX(F(1.0), i * F(s->sample_rate * F(0.5)) / F(fft_mid));
        const ftype mag = sgn * percentage * fn(analog_mag_db)(f, w_curve[s->curve].tau) - center;

        complexf[i].re = FEXP10(mag / F(20.0));
    }

    if (s->phase != F(0.0)) {
        const ftype a = F(1.0) - (F(1.0) + s->phase) * F(0.5);
        const ftype threshold = FMINV;
        const ftype logt = FLOG(threshold);

        scale = F(1.0);
        ret = av_tx_init(&s->tx_ctx, &s->tx_fn, TX_TYPE, 0, fft_size, &scale, 0);
        if (ret < 0)
            return ret;

        for (int i = 0; i < fft_mid+1; i++)
            complexf[i].re = complexf[i].re < threshold ? logt : logf(complexf[i].re);

        s->itx_fn(s->itx_ctx, realf, complexf, sizeof(ctype));

        for (int i = 1; i < fft_mid; i++)
            realf[i] *= F(2.0) * a;
        for (int i = fft_mid+1; i < fft_size; i++)
            realf[i] *= F(2.0) * (F(1.0) - a);

        s->tx_fn(s->tx_ctx, complexf, realf, sizeof(ftype));

        for (int i = 0; i < fft_mid+1; i++) {
            ftype eR = FEXP(complexf[i].re);

            complexf[i].re = eR * FCOS(complexf[i].im);
            complexf[i].im = eR * FSIN(complexf[i].im);
        }

        s->itx_fn(s->itx_ctx, realf, complexf, sizeof(ctype));

        int shift = LRINT((F(1.0) - a) * (s->nb_taps - 1));
        for (int i = 0; i < s->nb_taps; i++) {
            const int idx = (i - shift) % fft_size;

            taps[i] = realf[(idx < 0) ? idx + fft_size : idx];
        }
    } else {
        s->itx_fn(s->itx_ctx, realf, complexf, sizeof(ctype));

        int middle = s->nb_taps / 2;
        for (int i = 0; i < middle; i++) {
            taps[middle - i] = realf[i];
            taps[middle + i] = realf[i];
        }
    }

    s->pts = 0;

    return 0;
}
