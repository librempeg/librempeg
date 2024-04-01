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
#undef stype
#undef itype
#undef ftype
#undef ttype
#undef COS
#undef SAMPLE_FORMAT
#undef TX_TYPE
#if DEPTH == 16
#define COS cosf
#define ctype AVComplexFloat
#define ftype float
#define itype int16_t
#define SAMPLE_FORMAT s16
#define stype float
#define ttype AVComplexFloat
#define TX_TYPE AV_TX_FLOAT_RDFT
#elif DEPTH == 32
#define COS cosf
#define ctype AVComplexFloat
#define ftype float
#define itype float
#define SAMPLE_FORMAT fltp
#define stype float
#define ttype AVComplexFloat
#define TX_TYPE AV_TX_FLOAT_RDFT
#else
#define COS cos
#define ctype AVComplexDouble
#define ftype double
#define itype double
#define SAMPLE_FORMAT dblp
#define stype double
#define ttype AVComplexDouble
#define TX_TYPE AV_TX_DOUBLE_RDFT
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    stype *over;
    stype *rdft_in0;
    ctype *rdft_in1;
    ctype *rdft_out0;
    stype *rdft_out1;

    AVTXContext *tx_ctx, *itx_ctx;
    av_tx_fn tx_fn, itx_fn;
} fn(StateContext);

static ftype fn(get_scale)(ftype in_size, ftype out_size)
{
    return F(1.0) / out_size;
}

static ftype fn(get_iscale)(ftype in_size, ftype out_size)
{
    return F(1.0) * out_size / in_size;
}

static void fn(src_uninit)(AVFilterContext *ctx)
{
    AudioRDFTSRCContext *s = ctx->priv;

    if (s->state) {
        fn(StateContext) *state = s->state;

        for (int ch = 0; ch < s->channels; ch++) {
            fn(StateContext) *stc = &state[ch];

            av_freep(&stc->over);
            av_tx_uninit(&stc->tx_ctx);
            av_tx_uninit(&stc->itx_ctx);
            av_freep(&stc->rdft_in0);
            av_freep(&stc->rdft_out0);
            av_freep(&stc->rdft_in1);
            av_freep(&stc->rdft_out1);
        }
    }

    av_freep(&s->state);
    av_freep(&s->taper);
}

static int fn(src_init)(AVFilterContext *ctx)
{
    AudioRDFTSRCContext *s = ctx->priv;
    const int taper_samples = s->taper_samples;
    ftype iscale = fn(get_iscale)(s->in_rdft_size, s->out_rdft_size);
    ftype scale = fn(get_scale)(s->in_rdft_size, s->out_rdft_size);
    fn(StateContext) *state;
    ttype *taper;
    int ret;

    s->state = av_calloc(s->channels, sizeof(*state));
    if (!s->state)
        return AVERROR(ENOMEM);
    state = s->state;

    for (int ch = 0; ch < s->channels; ch++) {
        fn(StateContext) *stc = &state[ch];

        ret = av_tx_init(&stc->tx_ctx, &stc->tx_fn, TX_TYPE, 0, s->in_rdft_size, &scale, 0);
        if (ret < 0)
            return ret;

        ret = av_tx_init(&stc->itx_ctx, &stc->itx_fn, TX_TYPE, 1, s->out_rdft_size, &iscale, 0);
        if (ret < 0)
            return ret;

        stc->over = av_calloc(s->out_rdft_size, sizeof(*stc->over));
        if (!stc->over)
            return AVERROR(ENOMEM);

        stc->rdft_in0 = av_calloc(s->in_rdft_size+2, sizeof(*stc->rdft_in0));
        if (!stc->rdft_in0)
            return AVERROR(ENOMEM);

        stc->rdft_in1 = av_calloc(s->in_rdft_size+2, sizeof(*stc->rdft_in1));
        if (!stc->rdft_in1)
            return AVERROR(ENOMEM);

        stc->rdft_out0 = av_calloc(s->out_rdft_size+2, sizeof(*stc->rdft_out0));
        if (!stc->rdft_out0)
            return AVERROR(ENOMEM);

        stc->rdft_out1 = av_calloc(s->out_rdft_size+2, sizeof(*stc->rdft_out1));
        if (!stc->rdft_out1)
            return AVERROR(ENOMEM);
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
    const itype *src = ((const itype *)in->extended_data[ch]) + soffset;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    stype *over = stc->over;
    itype *dst = ((itype *)out->extended_data[ch]) + doffset;
    stype *rdft0 = stc->rdft_in0;
    ctype *rdft1 = stc->rdft_in1;
    ctype *irdft0 = stc->rdft_out0;
    stype *irdft1 = stc->rdft_out1;
    const int out_nb_samples = s->out_nb_samples;
    const int in_nb_samples = s->in_nb_samples;
    const int tr_nb_samples = s->tr_nb_samples;
    const int taper_samples = s->taper_samples;
    const int in_offset = (s->in_rdft_size - in_nb_samples) >> 1;
    const int offset = tr_nb_samples - taper_samples;
    const int copy_samples = FFMIN(s->in_nb_samples, in->nb_samples-soffset);
    const ttype *taper = s->taper;

    memset(rdft0, 0, in_offset * sizeof(*rdft0));
    memset(rdft0 + s->in_rdft_size - in_offset, 0, in_offset * sizeof(*rdft0));
#if DEPTH == 16
    for (int n = 0; n < copy_samples; n++)
        rdft0[in_offset+n] = src[n] / F(1<<(DEPTH-1));
#else
    memcpy(rdft0 + in_offset, src, copy_samples * sizeof(*rdft0));
#endif

    stc->tx_fn(stc->tx_ctx, rdft1, rdft0, sizeof(*rdft0));

    memset(irdft0 + tr_nb_samples, 0, (s->out_rdft_size / 2 + 1 - tr_nb_samples) * sizeof(*irdft0));
    memcpy(irdft0, rdft1, tr_nb_samples * sizeof(*irdft0));
    for (int n = 0, m = offset; n < taper_samples; n++, m++) {
        irdft0[m].re *= taper[n].re;
        irdft0[m].im *= taper[n].im;
    }

    stc->itx_fn(stc->itx_ctx, irdft1, irdft0, sizeof(*irdft0));

#if DEPTH == 16
    for (int n = 0; n < out_nb_samples; n++) {
        dst[n] = irdft1[n] * F(1<<(DEPTH-1));
        dst[n] += over[n] * F(1<<(DEPTH-1));
    }
#else
    memcpy(dst, irdft1, out_nb_samples * sizeof(*dst));
    for (int n = 0; n < out_nb_samples; n++)
        dst[n] += over[n];
#endif
    memcpy(over, irdft1 + out_nb_samples, sizeof(*over) * out_nb_samples);

    return 0;
}
