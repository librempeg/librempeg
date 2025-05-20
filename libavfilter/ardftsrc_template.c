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
#include "audio.h"

#undef ctype
#undef itype
#undef ftype
#undef ttype
#undef FABS
#undef FEXP
#undef FPOW
#undef FCOS
#undef FSIN
#undef SAMPLE_FORMAT
#undef TX_TYPE
#if DEPTH == 8
#define FCOS cosf
#define FSIN sinf
#define FPOW powf
#define FABS fabsf
#define FEXP expf
#define ctype AVComplexFloat
#define ftype float
#define itype uint8_t
#define SAMPLE_FORMAT u8p
#define ttype AVComplexFloat
#define TX_TYPE AV_TX_FLOAT_RDFT
#elif DEPTH == 16
#define FCOS cosf
#define FSIN sinf
#define FPOW powf
#define FABS fabsf
#define FEXP expf
#define ctype AVComplexFloat
#define ftype float
#define itype int16_t
#define SAMPLE_FORMAT s16p
#define ttype AVComplexFloat
#define TX_TYPE AV_TX_FLOAT_RDFT
#elif DEPTH == 32
#define FCOS cos
#define FSIN sin
#define FPOW pow
#define FABS fabs
#define FEXP exp
#define ctype AVComplexDouble
#define ftype double
#define itype int32_t
#define SAMPLE_FORMAT s32p
#define ttype AVComplexDouble
#define TX_TYPE AV_TX_DOUBLE_RDFT
#elif DEPTH == 33
#define FCOS cosf
#define FSIN sinf
#define FPOW powf
#define FABS fabsf
#define FEXP expf
#define ctype AVComplexFloat
#define ftype float
#define itype float
#define SAMPLE_FORMAT fltp
#define ttype AVComplexFloat
#define TX_TYPE AV_TX_FLOAT_RDFT
#else
#define FCOS cos
#define FSIN sin
#define FPOW pow
#define FABS fabs
#define FEXP exp
#define ctype AVComplexDouble
#define ftype double
#define itype double
#define SAMPLE_FORMAT dblp
#define ttype AVComplexDouble
#define TX_TYPE AV_TX_DOUBLE_RDFT
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ftype *over;
    ftype *temp;
    ftype *rdft_in;
    ctype *rdft_complex;
    ftype *rdft_out;

    AVTXContext *tx_ctx, *itx_ctx;
    av_tx_fn tx_fn, itx_fn;
} fn(StateContext);

static ftype fn(get_scale)(ftype in_size, ftype out_size)
{
    return F(1.0);
}

static ftype fn(get_iscale)(ftype in_size, ftype out_size)
{
    return F(1.0) / in_size;
}

static void fn(src_uninit)(AVFilterContext *ctx)
{
    AudioRDFTSRCContext *s = ctx->priv;

    if (s->state) {
        fn(StateContext) *state = s->state;

        for (int ch = 0; ch < s->channels; ch++) {
            fn(StateContext) *stc = &state[ch];

            av_freep(&stc->over);
            av_freep(&stc->temp);
            av_tx_uninit(&stc->tx_ctx);
            av_tx_uninit(&stc->itx_ctx);
            av_freep(&stc->rdft_in);
            av_freep(&stc->rdft_complex);
            av_freep(&stc->rdft_out);
        }
    }

    av_freep(&s->state);
    av_freep(&s->taper);
    av_freep(&s->phase);
}

static int fn(src_init)(AVFilterContext *ctx)
{
    AudioRDFTSRCContext *s = ctx->priv;
    const int channels = ctx->inputs[0]->ch_layout.nb_channels;
    const int taper_samples = s->taper_samples;
    ftype iscale = fn(get_iscale)(s->in_rdft_size, s->out_rdft_size);
    ftype scale = fn(get_scale)(s->in_rdft_size, s->out_rdft_size);
    const int rdft_size = FFMAX(s->in_rdft_size, s->out_rdft_size) / 2;
    fn(StateContext) *state;
    ttype *taper;
    ctype *phase;
    int ret;

    s->state = av_calloc(channels, sizeof(*state));
    if (!s->state)
        return AVERROR(ENOMEM);
    s->channels = channels;
    state = s->state;

    for (int ch = 0; ch < channels; ch++) {
        fn(StateContext) *stc = &state[ch];

        ret = av_tx_init(&stc->tx_ctx, &stc->tx_fn, TX_TYPE, 0, s->in_rdft_size, &scale, 0);
        if (ret < 0)
            return ret;

        ret = av_tx_init(&stc->itx_ctx, &stc->itx_fn, TX_TYPE, 1, s->out_rdft_size, &iscale, 0);
        if (ret < 0)
            return ret;

        stc->over = av_calloc(s->out_nb_samples, sizeof(*stc->over));
        if (!stc->over)
            return AVERROR(ENOMEM);

        stc->temp = av_calloc(s->out_nb_samples, sizeof(*stc->temp));
        if (!stc->temp)
            return AVERROR(ENOMEM);

        stc->rdft_in = av_calloc(s->in_rdft_size+2, sizeof(*stc->rdft_in));
        if (!stc->rdft_in)
            return AVERROR(ENOMEM);

        stc->rdft_complex = av_calloc(rdft_size+2, sizeof(*stc->rdft_complex));
        if (!stc->rdft_complex)
            return AVERROR(ENOMEM);

        stc->rdft_out = av_calloc(s->out_rdft_size+2, sizeof(*stc->rdft_out));
        if (!stc->rdft_out)
            return AVERROR(ENOMEM);
    }

    s->taper = av_calloc(taper_samples, sizeof(*taper));
    if (!s->taper)
        return AVERROR(ENOMEM);
    taper = s->taper;
    for (int n = 0; n < taper_samples-1; n++) {
        const ftype t = taper_samples;
        const ftype zbk = t/((t-n)-F(1.0)) - t/(n+F(1.0));
        const ftype v = F(1.0)/(FEXP(zbk)+F(1.0));

        taper[n].re = taper[n].im = isnormal(v) ? v : F(0.0);
    }

    s->phase = av_calloc(s->tr_nb_samples, sizeof(*phase));
    if (!s->phase)
        return AVERROR(ENOMEM);
    phase = s->phase;
    for (int n = 0; n < s->tr_nb_samples; n++) {
        const ftype aphase = FABS(s->phaset);
        const ftype sgn = s->phaset < F(0.0) ? F(-1.0) : F(1.0);
        const ftype inter = FPOW(aphase, aphase > F(0.5) ? aphase/F(0.5) : F(0.5)/aphase);
        const ftype x = F(n+1) / s->tr_nb_samples;
        const ftype z = F(4.0 * M_PI) * FPOW(x, F(4.0) * F(M_PI));
        const ftype w = inter * z;

        phase[n].re = FCOS(w * sgn);
        phase[n].im = FSIN(w * sgn);
    }

    return 0;
}

static int fn(src_out)(AVFilterContext *ctx, AVFrame *out, const int ch,
                       const int doffset, const int mode)
{
    AudioRDFTSRCContext *s = ctx->priv;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    ftype *over = mode ? s->over : stc->over;
    ftype *irdft = mode ? stc->temp : stc->rdft_out;
    const int out_nb_samples = s->out_nb_samples;
    const int write_samples = FFMIN(out_nb_samples, out->nb_samples - doffset);

    if (mode)
        over += ch * out_nb_samples;

    if (s->out_planar) {
        itype *dst = ((itype *)out->extended_data[ch]) + doffset;
#if DEPTH == 8
        for (int n = 0; n < write_samples; n++)
            dst[n] = av_clip_uint8(lrintf(0x80 + (irdft[n] + over[n]) * F(1<<(DEPTH-1))));
#elif DEPTH == 16
        for (int n = 0; n < write_samples; n++)
            dst[n] = av_clip_int16(lrintf((irdft[n] + over[n]) * F(1<<(DEPTH-1))));
#elif DEPTH == 32
        for (int n = 0; n < write_samples; n++)
            dst[n] = av_clipl_int32(llrint((irdft[n] + over[n]) * F(1LL<<(DEPTH-1))));
#else
        memcpy(dst, irdft, write_samples * sizeof(*dst));
        for (int n = 0; n < write_samples; n++)
            dst[n] += over[n];
#endif
    } else {
        const int nb_channels = ctx->outputs[0]->ch_layout.nb_channels;
        itype *dst = ((itype *)out->data[0]) + doffset * nb_channels;
#if DEPTH == 8
        for (int n = 0, m = ch; n < write_samples; n++, m += nb_channels)
            dst[m] = av_clip_uint8(lrintf(0x80 + (irdft[n] + over[n]) * F(1<<(DEPTH-1))));
#elif DEPTH == 16
        for (int n = 0, m = ch; n < write_samples; n++, m += nb_channels)
            dst[m] = av_clip_int16(lrintf((irdft[n] + over[n]) * F(1<<(DEPTH-1))));
#elif DEPTH == 32
        for (int n = 0, m = ch; n < write_samples; n++, m += nb_channels)
            dst[m] = av_clipl_int32(llrint((irdft[n] + over[n]) * F(1LL<<(DEPTH-1))));
#else
        for (int n = 0, m = ch; n < write_samples; n++, m += nb_channels)
            dst[m] = irdft[n] + over[n];
#endif
    }

    return 0;
}

static int fn(src_in)(AVFilterContext *ctx, AVFrame *in, AVFrame *out,
                      const int ch, const int soffset, const int doffset)
{
    AudioRDFTSRCContext *s = ctx->priv;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    ftype *rdft = stc->rdft_in;
    ctype *rdftc = stc->rdft_complex;
    ftype *irdft = stc->rdft_out;
    ftype *over = stc->over;
    ftype *temp = stc->temp;
    const int in_nb_samples = s->in_nb_samples;
    const int tr_nb_samples = s->tr_nb_samples;
    const int taper_samples = s->taper_samples;
    const int in_offset = s->in_offset;
    const int offset = tr_nb_samples - taper_samples;
    const int out_nb_samples = s->out_nb_samples;
    const int copy_samples = FFMIN(in_nb_samples, in->nb_samples - soffset);
    const int write_samples = FFMIN(out_nb_samples, out->nb_samples - doffset);
    const ttype *taper = s->taper;
    const ctype *phase = s->phase;

    if (s->in_planar) {
        const itype *src = ((const itype *)in->extended_data[ch]) + soffset;
        ftype *rdft0o = rdft + in_offset;
#if DEPTH == 8
        for (int n = 0; n < copy_samples; n++)
            rdft0o[n] = (src[n] - 0x80) * (F(1.0)/F(1<<(DEPTH-1)));
#elif DEPTH == 16
        for (int n = 0; n < copy_samples; n++)
            rdft0o[n] = src[n] * (F(1.0)/F(1<<(DEPTH-1)));
#elif DEPTH == 32
        for (int n = 0; n < copy_samples; n++)
            rdft0o[n] = src[n] * (F(1.0)/F(1LL<<(DEPTH-1)));
#else
        memcpy(rdft0o, src, copy_samples * sizeof(*rdft));
#endif
    } else {
        const int nb_channels = ctx->inputs[0]->ch_layout.nb_channels;
        const itype *src = ((const itype *)in->data[0]) + soffset * nb_channels;
        ftype *rdft0o = rdft + in_offset;
#if DEPTH == 8
        for (int n = 0, m = ch; n < copy_samples; n++, m += nb_channels)
            rdft0o[n] = (src[m] - 0x80) * (F(1.0)/F(1<<(DEPTH-1)));
#elif DEPTH == 16
        for (int n = 0, m = ch; n < copy_samples; n++, m += nb_channels)
            rdft0o[n] = src[m] * (F(1.0)/F(1<<(DEPTH-1)));
#elif DEPTH == 32
        for (int n = 0, m = ch; n < copy_samples; n++, m += nb_channels)
            rdft0o[n] = src[m] * (F(1.0)/F(1LL<<(DEPTH-1)));
#else
        for (int n = 0, m = ch; n < copy_samples; n++, m += nb_channels)
            rdft0o[n] = src[m];
#endif
    }
    memset(rdft + in_offset+copy_samples, 0, (in_nb_samples-copy_samples) * sizeof(*rdft));

    stc->tx_fn(stc->tx_ctx, rdftc, rdft, sizeof(*rdft));

    memset(rdftc + tr_nb_samples, 0, (s->out_rdft_size / 2 + 1 - tr_nb_samples) * sizeof(*rdftc));

    if (s->phaset != F(0.0)) {
        for (int n = 0; n < tr_nb_samples; n++) {
            const ftype re = rdftc[n].re;
            const ftype im = rdftc[n].im;
            const ftype cre = phase[n].re;
            const ftype cim = phase[n].im;

            rdftc[n].re = re * cre - im * cim;
            rdftc[n].im = re * cim + im * cre;
        }
    }

    for (int n = 0, m = offset; n < taper_samples; n++, m++) {
        rdftc[m].re *= taper[n].re;
        rdftc[m].im *= taper[n].im;
    }

    stc->itx_fn(stc->itx_ctx, irdft, rdftc, sizeof(*rdftc));

    if (doffset > 0)
        fn(src_out)(ctx, out, ch, doffset, 0);

    memcpy(over, irdft + write_samples, sizeof(*over) * out_nb_samples);

    if (soffset == 0)
        memcpy(temp, irdft, sizeof(*temp) * out_nb_samples);

    return 0;
}

static void fn(copy_over)(AVFilterContext *ctx)
{
    const int nb_channels = ctx->outputs[0]->ch_layout.nb_channels;
    AudioRDFTSRCContext *s = ctx->priv;
    const int out_nb_samples = s->out_nb_samples;
    fn(StateContext) *state = s->state;
    ftype *over = s->over;

    for (int ch = 0; ch < nb_channels; ch++) {
        fn(StateContext) *stc = &state[ch];

        memcpy(over, stc->over, out_nb_samples * sizeof(*over));

        over += out_nb_samples;
    }
}

static int fn(flush)(AVFilterContext *ctx, AVFrame *out, const int ch)
{
    const int nb_samples = out->nb_samples;
    AudioRDFTSRCContext *s = ctx->priv;
    const int out_nb_samples = s->out_nb_samples;
    ftype *over = s->over;

    over += ch * out_nb_samples;
    if (s->out_planar) {
        itype *dst = ((itype *)out->extended_data[ch]);
#if DEPTH == 8
        for (int n = 0; n < nb_samples; n++)
            dst[n] = av_clip_uint8(lrintf(0x80 + over[n] * F(1<<(DEPTH-1))));
#elif DEPTH == 16
        for (int n = 0; n < nb_samples; n++)
            dst[n] = av_clip_int16(lrintf(over[n] * F(1<<(DEPTH-1))));
#elif DEPTH == 32
        for (int n = 0; n < nb_samples; n++)
            dst[n] = av_clipl_int32(llrint(over[n] * F(1LL<<(DEPTH-1))));
#else
        memcpy(dst, over, nb_samples * sizeof(*dst));
#endif
    } else {
        const int nb_channels = ctx->outputs[0]->ch_layout.nb_channels;
        itype *dst = (itype *)out->data[0];
#if DEPTH == 8
        for (int n = 0, m = ch; n < nb_samples; n++, m += nb_channels)
            dst[m] = av_clip_uint8(lrintf(0x80 + over[n] * F(1<<(DEPTH-1))));
#elif DEPTH == 16
        for (int n = 0, m = ch; n < nb_samples; n++, m += nb_channels)
            dst[m] = av_clip_int16(lrintf(over[n] * F(1<<(DEPTH-1))));
#elif DEPTH == 32
        for (int n = 0, m = ch; n < nb_samples; n++, m += nb_channels)
            dst[m] = av_clipl_int32(llrint(over[n] * F(1LL<<(DEPTH-1))));
#else
        for (int n = 0, m = ch; n < nb_samples; n++, m += nb_channels)
            dst[m] = over[n];
#endif
    }

    return 0;
}
