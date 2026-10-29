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
#undef HYPOT
#undef FCOS
#undef FSIN
#undef FEXP
#undef FLOG
#undef LRINT
#undef F_MIN
#undef SAMPLE_FORMAT
#undef TX_TYPE
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define HYPOT hypotf
#define FCOS cosf
#define FSIN sinf
#define FEXP expf
#define FLOG logf
#define LRINT lrintf
#define F_MIN FLT_MIN
#define ctype AVComplexFloat
#define ftype float
#define TX_TYPE AV_TX_FLOAT_FFT
#else
#define SAMPLE_FORMAT double
#define HYPOT hypot
#define FCOS cos
#define FSIN sin
#define FEXP exp
#define FLOG log
#define LRINT lrint
#define F_MIN DBL_MIN
#define ctype AVComplexDouble
#define ftype double
#define TX_TYPE AV_TX_DOUBLE_FFT
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static ctype fn(cexp)(ctype z)
{
    ftype x = z.re, y = z.im;
    ftype c, s;

    if (y == 0.0)
        return (ctype){ FEXP(x), y };

    c = FCOS(y);
    s = FSIN(y);

    ftype ex = FEXP(x);
    return (ctype){ ex * c, ex * s };
}

static int fn(rephase)(AVFilterContext *ctx, AVFrame *out, const int ch)
{
    AudioFIRPhaseContext *s = ctx->priv;
    const ftype phase = s->phase;
    const ftype a = F(1.0) - (F(1.0) + phase) * F(0.5);
    const int nb_taps = s->in->nb_samples;
    const int oversample = 4;
    const int fft_size = oversample << av_ceil_log2(nb_taps);
    const int fft_mid = fft_size/2;
    AVTXContext *tx_ctx = NULL, *itx_ctx;
    const ftype *src = (const ftype *)s->in->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    const ftype threshold = F_MIN;
    const ftype logt = FLOG(threshold);
    const ftype scale = F(1.0) / fft_size;
    const ftype unused = F(1.0);
    av_tx_fn tx_fn, itx_fn;
    ctype *fft_out = NULL;
    ctype *fft_in = NULL;
    int ret;

    ret = av_tx_init(&tx_ctx, &tx_fn, TX_TYPE, 0, fft_size, &unused, 0);
    if (ret < 0)
        return ret;

    ret = av_tx_init(&itx_ctx, &itx_fn, TX_TYPE, 1, fft_size, &unused, 0);
    if (ret < 0)
        goto fail;

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
        const ftype mag = HYPOT(fft_out[i].re, fft_out[i].im);

        fft_out[i].re = mag < threshold ? logt : FLOG(mag);
        fft_out[i].im = F(0.0);
    }

    itx_fn(itx_ctx, fft_in, fft_out, sizeof(*fft_out));

    for (int i = 0; i < fft_size; i++) {
        fft_in[i].re *= scale;
        fft_in[i].im  = F(0.0);
    }

    for (int i = 1; i < fft_mid; i++)
        fft_in[i].re *= F(2.0) * a;
    for (int i = fft_mid+1; i < fft_size; i++)
        fft_in[i].re *= F(2.0) * (F(1.0) - a);

    tx_fn(tx_ctx, fft_out, fft_in, sizeof(*fft_in));

    for (int i = 0; i < fft_size; i++) {
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

        dst[i] = fft_in[(idx < 0) ? idx + fft_size : idx].re * scale;
    }

fail:
    av_freep(&fft_out);
    av_freep(&fft_in);

    av_tx_uninit(&itx_ctx);
    av_tx_uninit(&tx_ctx);

    return ret;
}
