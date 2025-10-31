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
    ftype *rdft_in_last;

    int done_start;
    int done_stop;

    ftype error;

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
            av_freep(&stc->rdft_in_last);
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

        stc->rdft_in_last = av_calloc(s->in_nb_samples*2, sizeof(*stc->rdft_in_last));
        if (!stc->rdft_in_last)
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
        const ftype x = F(n) / s->tr_nb_samples;
        const ftype z = F(0.5) * FEXP((x - F(0.5)) * F(2.0 * M_PI));
        const ftype w = aphase * z * sgn;

        phase[n].re = FCOS(w);
        phase[n].im = FSIN(w);
    }

    return 0;
}

static int fn(src_out)(AVFilterContext *ctx, AVFrame *out, const int ch,
                       const int doffset, const int mode)
{
    AudioRDFTSRCContext *s = ctx->priv;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    const ftype *over = mode ? s->over : stc->over;
    ftype *irdft = mode ? stc->temp : stc->rdft_out;
    const int out_nb_samples = s->out_nb_samples;
    const int write_samples = FFMIN(out_nb_samples, out->nb_samples - doffset);
    const int shape = s->shape;

    if (mode)
        over += ch * out_nb_samples;

    for (int n = 0; n < write_samples; n++)
        irdft[n] += over[n];

    if (s->out_planar) {
        if (s->out_depth == 8) {
            uint8_t *dst = ((uint8_t *)out->extended_data[ch]) + doffset;

            if (shape) {
                ftype error = stc->error;

                for (int n = 0; n < write_samples; n++) {
                    ftype sample = irdft[n] * F(1<<(8-1));
                    int rsample = lrintf(sample + error);

                    error += sample - rsample;
                    dst[n] = av_clip_uint8(0x80 + rsample);
                }
            } else {
                for (int n = 0; n < write_samples; n++)
                    dst[n] = av_clip_uint8(lrintf(0x80 + irdft[n] * F(1<<(8-1))));
            }
        } else if (s->out_depth == 16) {
            int16_t *dst = ((int16_t *)out->extended_data[ch]) + doffset;

            if (shape) {
                ftype error = stc->error;

                for (int n = 0; n < write_samples; n++) {
                    ftype sample = irdft[n] * F(1<<(16-1));
                    int rsample = lrintf(sample + error);

                    error += sample - rsample;
                    dst[n] = av_clip_int16(rsample);
                }
            } else {
                for (int n = 0; n < write_samples; n++)
                    dst[n] = av_clip_int16(lrintf(irdft[n] * F(1<<(16-1))));
            }
        } else if (s->out_depth == 32) {
            int32_t *dst = ((int32_t *)out->extended_data[ch]) + doffset;
            for (int n = 0; n < write_samples; n++)
                dst[n] = av_clipl_int32(llrint(irdft[n] * F(1LL<<(32-1))));
        } else if (s->out_depth == 33) {
            float *dst = ((float *)out->extended_data[ch]) + doffset;
            for (int n = 0; n < write_samples; n++)
                dst[n] = irdft[n];
        } else {
            double *dst = ((double *)out->extended_data[ch]) + doffset;
            for (int n = 0; n < write_samples; n++)
                dst[n] = irdft[n];
        }
    } else {
        const int nb_channels = ctx->outputs[0]->ch_layout.nb_channels;
        if (s->out_depth == 8) {
            uint8_t *dst = ((uint8_t *)out->data[0]) + doffset * nb_channels;

            if (shape) {
                ftype error = stc->error;

                for (int n = 0, m = ch; n < write_samples; n++, m += nb_channels) {
                    ftype sample = irdft[n] * F(1<<(8-1));
                    int rsample = lrintf(sample + error);

                    error += sample - rsample;
                    dst[m] = av_clip_uint8(0x80 + rsample);
                }
            } else {
                for (int n = 0, m = ch; n < write_samples; n++, m += nb_channels)
                    dst[m] = av_clip_uint8(lrintf(0x80 + irdft[n] * F(1<<(8-1))));
            }
        } else if (s->out_depth == 16) {
            int16_t *dst = ((int16_t *)out->data[0]) + doffset * nb_channels;

            if (shape) {
                ftype error = stc->error;

                for (int n = 0, m = ch; n < write_samples; n++, m += nb_channels) {
                    ftype sample = irdft[n] * F(1<<(16-1));
                    int rsample = lrintf(sample + error);

                    error += sample - rsample;
                    dst[m] = av_clip_int16(rsample);
                }
            } else {
                for (int n = 0, m = ch; n < write_samples; n++, m += nb_channels)
                    dst[m] = av_clip_int16(lrintf(irdft[n] * F(1<<(16-1))));
            }
        } else if (s->out_depth == 32) {
            int32_t *dst = ((int32_t *)out->data[0]) + doffset * nb_channels;
            for (int n = 0, m = ch; n < write_samples; n++, m += nb_channels)
                dst[m] = av_clipl_int32(llrint(irdft[n] * F(1LL<<(32-1))));
        } else if (s->out_depth == 33) {
            float *dst = ((float *)out->data[0]) + doffset * nb_channels;
            for (int n = 0, m = ch; n < write_samples; n++, m += nb_channels)
                dst[m] = irdft[n];
        } else {
            double *dst = ((double *)out->data[0]) + doffset * nb_channels;
            for (int n = 0, m = ch; n < write_samples; n++, m += nb_channels)
                dst[m] = irdft[n];
        }
    }

    return 0;
}

static void fn(autocorr)(const ftype *data, const int N, double *ac, const int m)
{
    for (int j = 0; j <= m; j++) {
        double d = 0.0;

        for (int n = j; n < N; n++)
            d += (double)data[n] * (double)data[n-j];

        ac[j] = d;
    }
}

static int fn(do_lpc)(const double *ac, double *lpc, const int lpc_order)
{
    double r, error, epsilon;
    int max_order = lpc_order;

    error = ac[0] * (1.+1e-10);
    epsilon = 1e-9 * ac[0] + 1e-10;

    for (int i = 0; i < lpc_order; i++) {
        if (error < epsilon) {
            memset(&lpc[i], 0, (lpc_order - i) * sizeof(lpc[0]));
            max_order = i;
            break;
        }

        r = -ac[i+1];
        for (int j = 0; j < i; j++)
            r -= lpc[j] * ac[i-j];
        r /= error;

        lpc[i] = r;
        for (int j = 0; j < i/2; j++) {
            const double tmp = lpc[j];

            lpc[j    ] += r * lpc[i-1-j];
            lpc[i-1-j] += r * tmp;
        }

        if (i & 1)
            lpc[i/2] += lpc[i/2]*r;

        error *= 1.0 - r*r;
    }

    {
        const double g = F(0.999);
        double damp = g;

        for (int j = 0; j < max_order; j++) {
            lpc[j] *= damp;
            damp *= g;
        }
    }

    if (max_order == 0) {
        max_order = 1;
        lpc[0] = F(-1.0);
    }

    return max_order;
}

static void fn(extrapolate)(ftype *data0, const size_t N,
                            const int extra, const double *lpc,
                            const int O, const int dir)
{
    if (dir) {
        ftype *data = data0 - 1 + O;

        for (int n = 0; n < extra; n++) {
            ftype sum = F(0.0);

            for (int j = 0; j < O; j++)
                sum -= data[-n-j] * lpc[O-1-j];

            data[-O-n] = sum;
        }
    } else {
        ftype *data = data0 + N - O;

        for (int n = 0; n < extra; n++) {
            ftype sum = F(0.0);

            for (int j = 0; j < O; j++)
                sum -= data[n+j] * lpc[O-1-j];

            data[O+n] = sum;
        }
    }
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
    ftype *oover = s->over;
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

    oover += s->out_nb_samples * ch;

redo:
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

    if (stc->done_start == 0 && s->first_pts == in->pts) {
        const int extra_samples = copy_samples/2;
        const int lpc_order = FFMIN(64, (copy_samples+1)/2);
        ftype *rdft0o = rdft + in_offset;
        double ac[64+1] = { 0 };
        double lpc[64] = { 0 };
        int order;

        fn(autocorr)(rdft0o, copy_samples, ac, lpc_order);
        order = fn(do_lpc)(ac, lpc, lpc_order);
        fn(extrapolate)(rdft0o, copy_samples, extra_samples, lpc, order, 1);
        memmove(rdft0o+copy_samples-extra_samples, rdft0o-extra_samples, extra_samples * sizeof(*rdft0o));
        memset(rdft0o-extra_samples, 0, copy_samples * sizeof(*rdft0o));

        stc->done_start = 1;
    } else if (stc->done_start == 1) {
        stc->done_start = 2;
    } else if (stc->done_stop == 0 && copy_samples < in_nb_samples) {
        const int pad_samples = in_nb_samples-copy_samples;
        const int lpc_order = FFMIN(64, in_nb_samples/2);
        ftype *rdft0o = rdft + in_offset;
        ftype *last = stc->rdft_in_last;
        double ac[64+1] = { 0 };
        double lpc[64] = { 0 };
        int order;

        memmove(last, last+copy_samples, pad_samples * sizeof(*last));
        memcpy(last+pad_samples, rdft0o, copy_samples * sizeof(*last));
        memcpy(rdft, last, in_nb_samples * sizeof(*rdft));
        memset(rdft+in_nb_samples, 0, in_nb_samples * sizeof(*rdft));
        fn(autocorr)(rdft, in_nb_samples, ac, lpc_order);
        order = fn(do_lpc)(ac, lpc, lpc_order);
        fn(extrapolate)(rdft, in_nb_samples, in_nb_samples, lpc, order, 0);
        memcpy(last, rdft+in_nb_samples + pad_samples, copy_samples * sizeof(*rdft));
        memset(last+copy_samples, 0, pad_samples * sizeof(*last));
        memmove(rdft0o, rdft + pad_samples, in_nb_samples * sizeof(*rdft0o));
        memset(rdft, 0, in_offset * sizeof(*rdft));
        memset(rdft0o+in_nb_samples, 0, in_offset * sizeof(*rdft0o));

        stc->done_stop = 1;
    } else {
        const ftype *src = rdft + in_offset;
        ftype *dst = stc->rdft_in_last;

        memcpy(dst, src, in_nb_samples * sizeof(*dst));
    }

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

    if (stc->done_start == 1)
        memcpy(oover, irdft + write_samples, sizeof(*over) * out_nb_samples);
    memcpy(over, irdft + write_samples, sizeof(*over) * out_nb_samples);

    if (soffset == 0)
        memcpy(temp, irdft, sizeof(*temp) * out_nb_samples);

    if (stc->done_start == 1)
        goto redo;

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
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    const int out_nb_samples = s->out_nb_samples;
    ftype *over = s->over;

    over += ch * out_nb_samples;

    if (stc->done_stop == 0) {
        const int tr_nb_samples = s->tr_nb_samples;
        const int taper_samples = s->taper_samples;
        const int offset = tr_nb_samples - taper_samples;
        const int in_offset = s->in_offset;
        const ttype *taper = s->taper;
        const ctype *phase = s->phase;
        ftype *irdft = stc->rdft_out;
        ftype *rdft = stc->rdft_in;
        ctype *rdftc = stc->rdft_complex;
        const int in_nb_samples = s->in_nb_samples;
        const int lpc_order = FFMIN(64, in_nb_samples/2);
        ftype *rdft0o = rdft + in_offset;
        ftype *last = stc->rdft_in_last;
        double ac[64+1] = { 0 };
        double lpc[64] = { 0 };
        int order;

        fn(autocorr)(last, in_nb_samples, ac, lpc_order);
        order = fn(do_lpc)(ac, lpc, lpc_order);
        fn(extrapolate)(last, in_nb_samples, in_offset, lpc, order, 0);
        memcpy(rdft0o, last + in_nb_samples, in_nb_samples * sizeof(*rdft0o));

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

        for (int n = 0; n < nb_samples; n++)
            over[n] += irdft[n];
    }

    if (s->out_planar) {
        if (s->out_depth == 8) {
            uint8_t *dst = ((uint8_t *)out->extended_data[ch]);
            for (int n = 0; n < nb_samples; n++)
                dst[n] = av_clip_uint8(lrintf(0x80 + over[n] * F(1<<(8-1))));
        } else if (s->out_depth == 16) {
            int16_t *dst = ((int16_t *)out->extended_data[ch]);
            for (int n = 0; n < nb_samples; n++)
                dst[n] = av_clip_int16(lrintf(over[n] * F(1<<(16-1))));
        } else if (s->out_depth == 32) {
            int32_t *dst = ((int32_t *)out->extended_data[ch]);
            for (int n = 0; n < nb_samples; n++)
                dst[n] = av_clipl_int32(llrint(over[n] * F(1LL<<(32-1))));
        } else if (s->out_depth == 33) {
            float *dst = ((float *)out->extended_data[ch]);
            for (int n = 0; n < nb_samples; n++)
                dst[n] = over[n];
        } else {
            double *dst = ((double *)out->extended_data[ch]);
            for (int n = 0; n < nb_samples; n++)
                dst[n] = over[n];
        }
    } else {
        const int nb_channels = ctx->outputs[0]->ch_layout.nb_channels;
        if (s->out_depth == 8) {
            uint8_t *dst = (uint8_t *)out->data[0];
            for (int n = 0, m = ch; n < nb_samples; n++, m += nb_channels)
                dst[m] = av_clip_uint8(lrintf(0x80 + over[n] * F(1<<(8-1))));
        } else if (s->out_depth == 16) {
            int16_t *dst = (int16_t *)out->data[0];
            for (int n = 0, m = ch; n < nb_samples; n++, m += nb_channels)
                dst[m] = av_clip_int16(lrintf(over[n] * F(1<<(16-1))));
        } else if (s->out_depth == 32) {
            int32_t *dst = (int32_t *)out->data[0];
            for (int n = 0, m = ch; n < nb_samples; n++, m += nb_channels)
                dst[m] = av_clipl_int32(llrint(over[n] * F(1LL<<(32-1))));
        } else if (s->out_depth == 33) {
            float *dst = (float *)out->data[0];
            for (int n = 0, m = ch; n < nb_samples; n++, m += nb_channels)
                dst[m] = over[n];
        } else {
            double *dst = (double *)out->data[0];
            for (int n = 0, m = ch; n < nb_samples; n++, m += nb_channels)
                dst[m] = over[n];
        }
    }

    return 0;
}
