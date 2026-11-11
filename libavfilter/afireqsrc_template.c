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
#undef FLOG
#undef FMINV
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
#define FMINV FLT_MIN
#define LRINT lrintf
#define SAMPLE_FORMAT flt
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
#define FMINV DBL_MIN
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

static void fn(eq_interp)(ctype *complexf,
                          const ftype *freq,
                          const ftype *magnitude,
                          int m, int interp, int minterp,
                          const ftype factor)
{
    for (int i = 0; i < minterp; i++) {
        for (int j = 0; j < m; j++) {
            const ftype x = factor * i;

            if (x <= freq[j+1]) {
                ftype g = F(1.0);

                if (interp == INTERP_LINEAR) {
                    const ftype d  = freq[j+1] - freq[j];
                    const ftype d0 = x - freq[j];
                    const ftype d1 = freq[j+1] - x;
                    const ftype g0 = magnitude[j];
                    const ftype g1 = magnitude[j+1];

                    if (d0 && d1) {
                        g = (d0 * g1 + d1 * g0) / d;
                    } else if (d0) {
                        g = g1;
                    } else {
                        g = g0;
                    }
                } else if (interp == INTERP_CUBIC) {
                    if (x <= freq[j]) {
                        g = magnitude[j];
                    } else {
                        ftype x1, x2, x3;
                        ftype a, b, c, d;
                        ftype m0, m1, m2, msum;
                        const ftype unit = freq[j+1] - freq[j];

                        m0 = j != 0 ? unit * (magnitude[j] - magnitude[j-1]) / (freq[j] - freq[j-1]) : 0;
                        m1 = magnitude[j+1] - magnitude[j];
                        m2 = j != minterp - 1 ? unit * (magnitude[j+2] - magnitude[j+1]) / (freq[j+2] - freq[j+1]) : 0;

                        msum = FABS(m0) + FABS(m1);
                        m0 = msum > F(0.0) ? (FABS(m0) * m1 + FABS(m1) * m0) / msum : F(0.0);
                        msum = FABS(m1) + FABS(m2);
                        m1 = msum > F(0.0) ? (FABS(m1) * m2 + FABS(m2) * m1) / msum : F(0.0);

                        d = magnitude[j];
                        c = m0;
                        b = F(3.0) * magnitude[j+1] - m1 - F(2.0) * c - F(3.0) * d;
                        a = magnitude[j+1] - b - c - d;

                        x1 = (x - freq[j]) / unit;
                        x2 = x1 * x1;
                        x3 = x2 * x1;

                        g = a * x3 + b * x2 + c * x1 + d;
                    }
                } else if (interp == INTERP_COSINE) {
                    const ftype fx = (x < freq[j]) ? freq[j] : x;
                    const ftype t = FLOG((fx + FMINV) / (freq[j] + FMINV)) / FLOG(freq[j+1] / (freq[j] + FMINV));
                    const ftype w = F(0.5) * (F(1.0) - FCOS(M_PI * t));
                    const ftype g0 = magnitude[j];
                    const ftype g1 = magnitude[j+1];

                    g = g0 + (g1 - g0) * w;
                } else if (interp == INTERP_SMOOTH) {
                    const ftype fx = (x < freq[j]) ? freq[j] : x;
                    const ftype t = FLOG((fx + FMINV) / (freq[j] + FMINV)) / FLOG(freq[j+1] / (freq[j] + FMINV));
                    const ftype w = t * t * (F(3.0) - F(2.0) * t);
                    const ftype g0 = magnitude[j];
                    const ftype g1 = magnitude[j+1];

                    g = g0 + (g1 - g0) * w;
                }

                complexf[i].re = g;
                complexf[i].im = 0;

                break;
            }
        }
    }
}

static av_cold int fn(config_output)(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioFIRSourceContext *s = ctx->priv;
    int fft_size, fft_mid, ret;
    ftype scale, factor;
    ftype *magnitude;
    ctype *complexf;
    ftype *realf;
    ftype *freq;
    ftype *taps;

    if (s->preset >= 0) {
        const int nb_magnitude = FF_ARRAY_ELEMS(eq_presets[s->preset].gains);

        if (s->nb_freq != nb_magnitude) {
            av_log(ctx, AV_LOG_ERROR, "Number of bands and gains must be same and >= 2.\n");
            return AVERROR(EINVAL);
        }

        s->freq = freq = av_calloc(s->nb_freq+1, sizeof(*freq));
        s->magnitude = magnitude = av_calloc(nb_magnitude+1, sizeof(*magnitude));
        if (!s->magnitude || !s->freq)
            return AVERROR(ENOMEM);

        for (int i = 0; i < s->nb_freq; i++)
            freq[i] = s->freq_points_opt[i];
        for (int i = 0; i < nb_magnitude; i++)
            magnitude[i] = eq_presets[s->preset].gains[i];
    } else {
        if (s->nb_freq != s->nb_magnitude || s->nb_freq < 2) {
            av_log(ctx, AV_LOG_ERROR, "Number of bands and gains must be same and >= 2.\n");
            return AVERROR(EINVAL);
        }

        s->freq = freq = av_calloc(s->nb_magnitude+1, sizeof(*freq));
        s->magnitude = magnitude = av_calloc(s->nb_magnitude+1, sizeof(*magnitude));
        if (!s->magnitude || !s->freq)
            return AVERROR(ENOMEM);

        for (int i = 0; i < s->nb_magnitude; i++)
            freq[i] = s->freq_points_opt[i];
        for (int i = 0; i < s->nb_magnitude; i++)
            magnitude[i] = s->magnitude_opt[i];
    }

    freq[s->nb_freq] = outlink->sample_rate * F(0.5);
    magnitude[s->nb_freq] = magnitude[s->nb_freq-1];

    fft_size = 1 << av_ceil_log2(s->nb_taps+1);
    fft_mid = fft_size/2;
    factor = FFMIN(outlink->sample_rate * F(0.5), freq[s->nb_freq - 1]) / (ftype)s->nb_taps;
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

    fn(eq_interp)(complexf, freq, magnitude, s->nb_freq, s->interp, s->nb_taps, factor);

    for (int i = 0; i < fft_mid+1; i++)
        complexf[i].re = FEXP10(complexf[i].re / F(20.0));

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
