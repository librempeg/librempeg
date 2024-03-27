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

#include "libavutil/tx.h"
#include "avfilter.h"
#include "internal.h"
#include "audio.h"

#undef ctype
#undef ftype
#undef COS
#undef SAMPLE_FORMAT
#undef TX_TYPE
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define COS cosf
#define ctype AVComplexFloat
#define ftype float
#define TX_TYPE AV_TX_FLOAT_RDFT
#else
#define SAMPLE_FORMAT double
#define COS cos
#define ctype AVComplexDouble
#define ftype double
#define TX_TYPE AV_TX_DOUBLE_RDFT
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(src_tx_init)(AVFilterContext *ctx)
{
    AudioRDFTSRCContext *s = ctx->priv;
    const int taper_samples = s->taper_samples;
    ctype *taper;
    int ret;

    for (int ch = 0; ch < s->channels; ch++) {
        ftype scale = F(1.0) / s->out_rdft_size;

        ret = av_tx_init(&s->tx_ctx[ch], &s->tx_fn, TX_TYPE, 0, s->in_rdft_size, &scale, 0);
        if (ret < 0)
            return ret;

        scale = (F(1.0) * s->out_rdft_size) / s->in_rdft_size;
        ret = av_tx_init(&s->itx_ctx[ch], &s->itx_fn, TX_TYPE, 1, s->out_rdft_size, &scale, 0);
        if (ret < 0)
            return ret;
    }

    s->taper = av_calloc(taper_samples, sizeof(*taper));
    if (!s->taper)
        return AVERROR(ENOMEM);
    taper = s->taper;
    for (int n = 0; n < taper_samples; n++)
        taper[n].re = taper[n].im = (F(1.0) + COS(F(M_PI) * (n + F(1.0)) / taper_samples)) * F(0.5);

    return 0;
}

static int fn(src)(AVFilterContext *ctx, AVFrame *in, AVFrame *out,
                   const int ch, const int soffset, const int doffset)
{
    AudioRDFTSRCContext *s = ctx->priv;
    const ftype *src = ((const ftype *)in->extended_data[ch]) + soffset;
    ftype *over = (ftype *)s->over->extended_data[ch];
    ftype *dst = ((ftype *)out->extended_data[ch]) + doffset;
    ftype *rdft0 = (ftype *)s->rdft_in[0]->extended_data[ch];
    ctype *rdft1 = (ctype *)s->rdft_in[1]->extended_data[ch];
    ctype *irdft0 = (ctype *)s->rdft_out[0]->extended_data[ch];
    ftype *irdft1 = (ftype *)s->rdft_out[1]->extended_data[ch];
    const int out_nb_samples = s->out_nb_samples;
    const int in_nb_samples = s->in_nb_samples;
    const int tr_nb_samples = s->tr_nb_samples;
    const int taper_samples = s->taper_samples;
    const int in_offset = (s->in_rdft_size - in_nb_samples) >> 1;
    const int offset = tr_nb_samples - taper_samples;
    const int copy_samples = FFMIN(s->in_nb_samples, in->nb_samples-soffset);
    const ctype *taper = s->taper;

    memset(rdft0, 0, in_offset * sizeof(*rdft0));
    memset(rdft0 + s->in_rdft_size - in_offset, 0, in_offset * sizeof(*rdft0));
    memcpy(rdft0 + in_offset, src, copy_samples * sizeof(*rdft0));

    s->tx_fn(s->tx_ctx[ch], rdft1, rdft0, sizeof(*rdft0));

    memset(irdft0 + tr_nb_samples, 0, (s->out_rdft_size / 2 + 1 - tr_nb_samples) * sizeof(*irdft0));
    memcpy(irdft0, rdft1, tr_nb_samples * sizeof(*irdft0));
    for (int n = 0, m = offset; n < taper_samples; n++, m++) {
        irdft0[m].re *= taper[n].re;
        irdft0[m].im *= taper[n].im;
    }

    s->itx_fn(s->itx_ctx[ch], irdft1, irdft0, sizeof(*irdft0));

    memcpy(dst, irdft1, out_nb_samples * sizeof(*dst));
    for (int n = 0; n < out_nb_samples; n++)
        dst[n] += over[n];
    memcpy(over, irdft1 + out_nb_samples, sizeof(*over) * out_nb_samples);

    return 0;
}
