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

#include "libavutil/mem.h"
#include "libavutil/tx.h"
#include "avfilter.h"
#include "audio.h"

#undef ctype
#undef ftype
#undef HYPOT
#undef SAMPLE_FORMAT
#undef TX_TYPE
#undef EPSILON
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define HYPOT hypotf
#define ctype AVComplexFloat
#define ftype float
#define TX_TYPE AV_TX_FLOAT_RDFT
#define EPSILON FLT_EPSILON
#else
#define SAMPLE_FORMAT double
#define HYPOT hypot
#define ctype AVComplexDouble
#define ftype double
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define EPSILON DBL_EPSILON
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(phase)(AVFilterContext *ctx, AVFrame *out, const int ch)
{
    AudioFIRPhaseContext *s = ctx->priv;
    const int nb_taps = s->in->nb_samples;
    const int oversample = 16;
    const int rdft_size = oversample << av_ceil_log2(nb_taps);
    AVTXContext *tx_ctx = NULL, *itx_ctx;
    const ftype iscale = F(1.0) / nb_taps;
    ftype *dst = (ftype *)out->extended_data[ch];
    const ftype scale = F(1.0);
    av_tx_fn tx_fn, itx_fn;
    ctype *rdft_out = NULL;
    ftype *rdft_in = NULL;
    int ret;

    ret = av_tx_init(&tx_ctx, &tx_fn, TX_TYPE, 0, rdft_size, &scale, 0);
    if (ret < 0)
        return ret;

    ret = av_tx_init(&itx_ctx, &itx_fn, TX_TYPE, 1, rdft_size, &iscale, 0);
    if (ret < 0)
        goto fail;

    rdft_in = av_calloc(rdft_size+1, sizeof(*rdft_in));
    if (!rdft_in) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    rdft_out = av_calloc(rdft_size+1, sizeof(*rdft_out));
    if (!rdft_out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    memcpy(rdft_in, s->in->extended_data[ch], nb_taps * sizeof(*rdft_in));

    tx_fn(tx_ctx, rdft_out, rdft_in, sizeof(*rdft_in));

    for (int i = 0; i < rdft_size/2+1; i++) {
        rdft_out[i].re = HYPOT(rdft_out[i].re, rdft_out[i].im);
        rdft_out[i].im = F(0.0);
    }

    itx_fn(itx_ctx, rdft_in, rdft_out, sizeof(*rdft_out));

    dst[nb_taps/2] = rdft_in[0] / F(oversample);
    for (int i = 1; i < nb_taps/2; i++) {
        dst[nb_taps/2-i] = rdft_in[i] / F(oversample);
        dst[nb_taps/2+i] = rdft_in[i] / F(oversample);
    }

fail:
    av_freep(&rdft_out);
    av_freep(&rdft_in);
    av_tx_uninit(&itx_ctx);
    av_tx_uninit(&tx_ctx);

    return ret;
}
