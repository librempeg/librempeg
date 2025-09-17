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

#include "libavutil/mem.h"
#include "libavutil/tx.h"
#include "avfilter.h"
#include "audio.h"

#undef ctype
#undef ftype
#undef FPOW
#undef ATAN2
#undef HYPOT
#undef FABS
#undef FCOS
#undef FSIN
#undef FEXP
#undef FLOG
#undef SAMPLE_FORMAT
#undef TX_TYPE
#undef MPI
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define FPOW powf
#define ATAN2 atan2f
#define HYPOT hypotf
#define FABS fabsf
#define FCOS cosf
#define FSIN sinf
#define FEXP expf
#define FLOG logf
#define ctype AVComplexFloat
#define ftype float
#define TX_TYPE AV_TX_FLOAT_FFT
#define MPI M_PIf
#else
#define SAMPLE_FORMAT double
#define FPOW pow
#define ATAN2 atan2
#define HYPOT hypot
#define FABS fabs
#define FCOS cos
#define FSIN sin
#define FEXP exp
#define FLOG log
#define ctype AVComplexDouble
#define ftype double
#define TX_TYPE AV_TX_DOUBLE_FFT
#define MPI M_PI
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static ftype fn(unwrap)(ftype previous_angle, ftype new_angle, const ftype angle)
{
    ftype d = new_angle - previous_angle;
    d = d > MPI ? d - F(2.0) * MPI : (d < -MPI ? d + F(2.0) * MPI : d);

    return angle + d;
}

static int fn(rephase)(AVFilterContext *ctx, AVFrame *out, const int ch)
{
    AudioFIRPhaseContext *s = ctx->priv;
    const ftype phase = s->phase;
    const int nb_taps = s->in->nb_samples;
    const int oversample = 4;
    const int fft_size = oversample << av_ceil_log2(nb_taps);
    AVTXContext *tx_ctx = NULL, *itx_ctx;
    const ftype *src = (const ftype *)s->in->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    const ftype threshold = FPOW(F(10.), -F(100.0 / 20.0));
    const ftype logt = FLOG(threshold);
    const ftype scale = F(1.0) / sqrt(fft_size);
    const ftype unused = F(1.0);
    av_tx_fn tx_fn, itx_fn;
    ftype *linear_phase = NULL;
    ftype *magnitude = NULL;
    ctype *fft_out = NULL;
    ctype *fft_in = NULL;
    int ret;

    ret = av_tx_init(&tx_ctx, &tx_fn, TX_TYPE, 0, fft_size, &unused, 0);
    if (ret < 0)
        return ret;

    ret = av_tx_init(&itx_ctx, &itx_fn, TX_TYPE, 1, fft_size, &unused, 0);
    if (ret < 0)
        goto fail;

    linear_phase = av_calloc(fft_size, sizeof(*linear_phase));
    if (!linear_phase) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    magnitude = av_calloc(fft_size, sizeof(*magnitude));
    if (!magnitude) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    fft_in = av_calloc(fft_size, sizeof(*fft_in));
    if (!fft_in) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    fft_out = av_calloc(fft_size, sizeof(*fft_out));
    if (!fft_out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    for (int i = 0; i < nb_taps; i++) {
        fft_in[i].re = src[i];
        fft_in[i].im = F(0.0);
    }

    tx_fn(tx_ctx, fft_out, fft_in, sizeof(*fft_in));

    for (int i = 0; i < fft_size; i++) {
        fft_out[i].re = HYPOT(fft_out[i].re * scale, fft_out[i].im * scale);
        fft_out[i].im = F(0.0);
    }

    itx_fn(itx_ctx, fft_in, fft_out, sizeof(*fft_out));

    if (phase == F(0.0)) {
        const int mid = nb_taps/2;

        for (int i = 0; i < mid; i++) {
            dst[mid-i] = fft_in[i].re * scale;
            dst[mid+i] = fft_in[i].re * scale;
        }
    } else {
        const ftype aphase = FABS(phase);
        const ftype inter = FPOW(aphase, aphase > F(0.5) ? aphase/F(0.5) : F(0.5)/aphase);
        const int mid = nb_taps/2;

        memset(fft_out, 0, sizeof(*fft_out) * fft_size);
        for (int i = 0; i < mid; i++) {
            fft_out[mid-i].re = fft_in[i].re * scale;
            fft_out[mid+i].re = fft_in[i].re * scale;
            fft_out[mid-i].im = F(0.0);
            fft_out[mid+i].im = F(0.0);
        }

        tx_fn(tx_ctx, fft_in, fft_out, sizeof(*fft_out));

        for (int i = 0; i < fft_size; i++) {
            const ftype re = fft_in[i].re * scale;
            const ftype im = fft_in[i].im * scale;

            linear_phase[i] = ATAN2(im, re);
            magnitude[i] = HYPOT(re, im);
        }

        {
            ftype prev = linear_phase[0];
            for (int i = 1; i < fft_size; i++) {
                ftype next_prev = linear_phase[i];

                linear_phase[i] = fn(unwrap)(prev, linear_phase[i], linear_phase[i-1]);
                prev = next_prev;
            }
        }

        for (int i = 0; i < fft_size; i++) {
            fft_in[i].re = magnitude[i] < threshold ? logt : FLOG(magnitude[i]);
            fft_in[i].im = F(0.0);
        }

        itx_fn(itx_ctx, fft_out, fft_in, sizeof(*fft_in));
        for (int i = 0; i < fft_size; i++) {
            fft_out[i].re *= scale;
            fft_out[i].im *= scale;
        }

        for (int i = 1; i < nb_taps; i++) {
            fft_out[i].re += fft_out[fft_size - i].re;
            fft_out[i].im -= fft_out[fft_size - i].im;
            fft_out[fft_size - i].re = F(0.0);
            fft_out[fft_size - i].im = F(0.0);
        }
        fft_out[nb_taps - 1].im *= F(-1.0);

        tx_fn(tx_ctx, fft_in, fft_out, sizeof(*fft_out));

        {
            ftype prev = fft_in[0].im * scale, prev_min_phase = fft_in[0].im * scale;
            const ftype eR = FEXP(fft_in[0].re * scale);

            fft_in[0].re = eR * FCOS(prev);
            fft_in[0].im = eR * FSIN(prev);
            for (int i = 1; i < fft_size; i++) {
                const ftype re = fft_in[i].re * scale;
                const ftype im = fft_in[i].im * scale;
                const ftype eR = FEXP(re);
                const ftype min_phase = fn(unwrap)(prev, im, prev_min_phase);
                const ftype phase = (F(1.0) - inter) * linear_phase[i] + inter * min_phase;

                prev = im;
                prev_min_phase = min_phase;
                fft_in[i].re = eR * FCOS(phase);
                fft_in[i].im = eR * FSIN(phase);
            }
        }

        itx_fn(itx_ctx, fft_out, fft_in, sizeof(*fft_in));

        if (phase > F(0.0)) {
            for (int i = 0; i < nb_taps; i++)
                dst[i] = fft_out[nb_taps-i-1].re * scale;
        } else {
            for (int i = 0; i < nb_taps; i++)
                dst[i] = fft_out[i].re * scale;
        }
    }

fail:
    av_freep(&magnitude);
    av_freep(&linear_phase);
    av_freep(&fft_out);
    av_freep(&fft_in);

    av_tx_uninit(&itx_ctx);
    av_tx_uninit(&tx_ctx);

    return ret;
}
