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

#undef SAMPLE_FORMAT
#undef ctype
#undef ftype
#undef TX_TYPE
#undef ATAN2
#undef SIN
#undef COS
#undef FABS
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define ctype AVComplexFloat
#define ftype float
#define TX_TYPE AV_TX_FLOAT_RDFT
#define ATAN2 atan2f
#define SIN sinf
#define COS cosf
#define FABS fabsf
#else
#define SAMPLE_FORMAT double
#define ctype AVComplexDouble
#define ftype double
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define ATAN2 atan2
#define SIN sin
#define COS cos
#define FABS fabs
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(sf_tx_init)(AVFilterContext *ctx)
{
    StereoFieldContext *s = ctx->priv;
    ftype scale = F(1.0), iscale = F(0.5) / s->fft_size;
    ftype *window;
    int ret;

    s->window = av_calloc(s->fft_size, sizeof(*window));
    if (!s->window)
        return AVERROR(ENOMEM);
    window = s->window;
    for (int n = 0; n < s->fft_size; n++)
        window[n] = SIN(M_PI*n/(s->fft_size-1));

    ret = av_tx_init(&s->tx_ctx, &s->tx_fn, TX_TYPE, 0, s->fft_size, &scale, 0);
    if (ret < 0)
        return ret;

    ret = av_tx_init(&s->itx_ctx, &s->itx_fn, TX_TYPE, 1, s->fft_size, &iscale, 0);
    if (ret < 0)
        return ret;

    return 0;
}

static void fn(apply_window)(StereoFieldContext *s,
                             const ftype *in_frame, ftype *out_frame, const int add_to_out_frame)
{
    const ftype *window = s->window;
    const int size = s->fft_size;

    if (add_to_out_frame) {
        for (int i = 0; i < size; i++)
            out_frame[i] += in_frame[i] * window[i];
    } else {
        for (int i = 0; i < size; i++)
            out_frame[i] = in_frame[i] * window[i];
    }
}

static void fn(stereofield)(ctype *fl, ctype *fr, const int N,
                            const ftype d, const ftype a0, const ftype a1,
                            const ftype p, const int M)
{
    const ftype lx = (p > F(0.0)) ? F(1.0)-p : F(1.0);
    const ftype rx = (p < F(0.0)) ? F(1.0)+p : F(1.0);
    const ftype m = (d < F(0.0)) ? d : F(0.0);
    const ftype s = (d > F(0.0)) ? d : F(0.0);

    switch (M) {
    case OP_LEFT:
        for (int i = 0; i < N; i++) {
            const ftype l_re = fl[i].re;
            const ftype l_im = fl[i].im;
            const ftype r_re = fr[i].re;
            const ftype r_im = fr[i].im;
            const ftype re = l_re * r_re + l_im * r_im;
            const ftype im = l_re * r_im - l_im * r_re;
            const ftype a = ATAN2(im, re);
            const ftype aa = FABS(a);
            const ftype lf = m * -a + s * (F(M_PI) - aa);
            const ftype lc = COS(lf);
            const ftype ls = SIN(lf);
            const ftype bx = a1 * aa * F(M_1_PI) + a0 * ((F(M_PI) - aa) * F(M_1_PI));

            fl[i].re = l_re * lc - l_im * ls;
            fl[i].im = l_re * ls + l_im * lc;
            fr[i].re = r_re;
            fr[i].im = r_im;

            fl[i].re *= bx;
            fl[i].im *= bx;
            fr[i].re *= bx;
            fr[i].im *= bx;

            fl[i].re *= lx;
            fl[i].im *= lx;
            fr[i].re *= rx;
            fr[i].im *= rx;
        }
        break;
    case OP_RIGHT:
        for (int i = 0; i < N; i++) {
            const ftype l_re = fl[i].re;
            const ftype l_im = fl[i].im;
            const ftype r_re = fr[i].re;
            const ftype r_im = fr[i].im;
            const ftype re = l_re * r_re + l_im * r_im;
            const ftype im = l_re * r_im - l_im * r_re;
            const ftype a = ATAN2(im, re);
            const ftype aa = FABS(a);
            const ftype lf = m * a - s * (F(M_PI) - aa);
            const ftype lc = COS(lf);
            const ftype ls = SIN(lf);
            const ftype bx = a1 * aa * F(M_1_PI) + a0 * ((F(M_PI) - aa) * F(M_1_PI));

            fl[i].re = l_re;
            fl[i].im = l_im;
            fr[i].re = r_re * lc - r_im * ls;
            fr[i].im = r_re * ls + r_im * lc;

            fl[i].re *= bx;
            fl[i].im *= bx;
            fr[i].re *= bx;
            fr[i].im *= bx;

            fl[i].re *= lx;
            fl[i].im *= lx;
            fr[i].re *= rx;
            fr[i].im *= rx;
        }
        break;
    case OP_STEREO:
        for (int i = 0; i < N; i++) {
            const ftype l_re = fl[i].re;
            const ftype l_im = fl[i].im;
            const ftype r_re = fr[i].re;
            const ftype r_im = fr[i].im;
            const ftype re = l_re * r_re + l_im * r_im;
            const ftype im = l_re * r_im - l_im * r_re;
            const ftype a = ATAN2(im, re) * F(0.5);
            const ftype aa = FABS(a);
            const ftype lfl = m * -a + s * (F(M_PI) - aa);
            const ftype lfr = m *  a + s * aa;
            const ftype lcl = COS(lfl);
            const ftype lsl = SIN(lfl);
            const ftype lcr = COS(lfr);
            const ftype lsr = SIN(lfr);
            const ftype bx = a1 * aa * F(M_2_PI) + a0 * ((F(M_PI_2) - aa) * F(M_2_PI));

            fl[i].re = l_re * lcl - l_im * lsl;
            fl[i].im = l_re * lsl + l_im * lcl;
            fr[i].re = r_re * lcr - r_im * lsr;
            fr[i].im = r_re * lsr + r_im * lcr;

            fl[i].re *= bx;
            fl[i].im *= bx;
            fr[i].re *= bx;
            fr[i].im *= bx;

            fl[i].re *= lx;
            fl[i].im *= lx;
            fr[i].re *= rx;
            fr[i].im *= rx;
        }
        break;
    }
}

static int fn(sf_stereo)(AVFilterContext *ctx, AVFrame *out)
{
    StereoFieldContext *s = ctx->priv;
    ftype *left_in         = (ftype *)s->in_frame->extended_data[0];
    ftype *right_in        = (ftype *)s->in_frame->extended_data[1];
    ftype *left_out        = (ftype *)s->out_dist_frame->extended_data[0];
    ftype *right_out       = (ftype *)s->out_dist_frame->extended_data[1];
    ftype *left_samples    = (ftype *)s->in->extended_data[0];
    ftype *right_samples   = (ftype *)s->in->extended_data[1];
    ftype *windowed_left   = (ftype *)s->windowed_frame->extended_data[0];
    ftype *windowed_right  = (ftype *)s->windowed_frame->extended_data[1];
    ctype *windowed_oleft  = (ctype *)s->windowed_out->extended_data[0];
    ctype *windowed_oright = (ctype *)s->windowed_out->extended_data[1];
    ftype *left_osamples   = (ftype *)out->extended_data[0];
    ftype *right_osamples  = (ftype *)out->extended_data[1];
    const int overlap = s->overlap;
    const int offset = s->fft_size - overlap;
    const int nb_samples = FFMIN(overlap, s->in->nb_samples);
    const int M = s->mode;
    const ftype A0 = s->A[0];
    const ftype A1 = s->A[1];
    const ftype D = s->D;
    const ftype P = s->P;

    // shift in/out buffers
    memmove(left_in, &left_in[overlap], offset * sizeof(*left_in));
    memmove(right_in, &right_in[overlap], offset * sizeof(*right_in));

    memcpy(&left_in[offset], left_samples, nb_samples * sizeof(*left_in));
    memcpy(&right_in[offset], right_samples, nb_samples * sizeof(*right_in));
    memset(&left_in[offset + nb_samples], 0, (overlap - nb_samples) * sizeof(*left_in));
    memset(&right_in[offset + nb_samples], 0, (overlap - nb_samples) * sizeof(*right_in));

    fn(apply_window)(s, left_in,  windowed_left,  0);
    fn(apply_window)(s, right_in, windowed_right, 0);

    s->tx_fn(s->tx_ctx, windowed_oleft,  windowed_left,  sizeof(ftype));
    s->tx_fn(s->tx_ctx, windowed_oright, windowed_right, sizeof(ftype));

    fn(stereofield)(windowed_oleft, windowed_oright,
                    s->fft_size/2 + 1, D, A0, A1, P, M);

    s->itx_fn(s->itx_ctx, windowed_left, windowed_oleft, sizeof(ctype));
    s->itx_fn(s->itx_ctx, windowed_right, windowed_oright, sizeof(ctype));

    memmove(left_out, &left_out[overlap], offset * sizeof(*left_out));
    memmove(right_out, &right_out[overlap], offset * sizeof(*right_out));
    memset(&left_out[offset], 0, overlap * sizeof(*left_out));
    memset(&right_out[offset], 0, overlap * sizeof(*right_out));

    fn(apply_window)(s, windowed_left,  left_out,  1);
    fn(apply_window)(s, windowed_right, right_out, 1);

    if (ff_filter_disabled(ctx)) {
        memcpy(left_osamples, left_in, out->nb_samples * sizeof(*left_osamples));
        memcpy(right_osamples, right_in, out->nb_samples * sizeof(*right_osamples));
    } else {
        memcpy(left_osamples, left_out, out->nb_samples * sizeof(*left_osamples));
        memcpy(right_osamples, right_out, out->nb_samples * sizeof(*right_osamples));
    }

    return 0;
}

static int fn(sf_flush)(AVFilterContext *ctx, AVFrame *out)
{
    StereoFieldContext *s = ctx->priv;
    ftype *left_in         = (ftype *)s->in_frame->extended_data[0];
    ftype *right_in        = (ftype *)s->in_frame->extended_data[1];
    ftype *left_out        = (ftype *)s->out_dist_frame->extended_data[0];
    ftype *right_out       = (ftype *)s->out_dist_frame->extended_data[1];
    ftype *windowed_left   = (ftype *)s->windowed_frame->extended_data[0];
    ftype *windowed_right  = (ftype *)s->windowed_frame->extended_data[1];
    ctype *windowed_oleft  = (ctype *)s->windowed_out->extended_data[0];
    ctype *windowed_oright = (ctype *)s->windowed_out->extended_data[1];
    ftype *left_osamples   = (ftype *)out->extended_data[0];
    ftype *right_osamples  = (ftype *)out->extended_data[1];
    const int overlap = s->overlap;
    const int offset = s->fft_size - overlap;
    const int nb_samples = 0;
    const int M = s->mode;
    const ftype A0 = s->A[0];
    const ftype A1 = s->A[1];
    const ftype D = s->D;
    const ftype P = s->P;

    // shift in/out buffers
    memmove(left_in, &left_in[overlap], offset * sizeof(*left_in));
    memmove(right_in, &right_in[overlap], offset * sizeof(*right_in));

    memset(&left_in[offset + nb_samples], 0, (overlap - nb_samples) * sizeof(*left_in));
    memset(&right_in[offset + nb_samples], 0, (overlap - nb_samples) * sizeof(*right_in));

    fn(apply_window)(s, left_in,  windowed_left,  0);
    fn(apply_window)(s, right_in, windowed_right, 0);

    s->tx_fn(s->tx_ctx, windowed_oleft,  windowed_left,  sizeof(ftype));
    s->tx_fn(s->tx_ctx, windowed_oright, windowed_right, sizeof(ftype));

    fn(stereofield)(windowed_oleft, windowed_oright,
                    s->fft_size/2 + 1, D, A0, A1, P, M);

    s->itx_fn(s->itx_ctx, windowed_left, windowed_oleft, sizeof(ctype));
    s->itx_fn(s->itx_ctx, windowed_right, windowed_oright, sizeof(ctype));

    memmove(left_out, &left_out[overlap], offset * sizeof(*left_out));
    memmove(right_out, &right_out[overlap], offset * sizeof(*right_out));
    memset(&left_out[offset], 0, overlap * sizeof(*left_out));
    memset(&right_out[offset], 0, overlap * sizeof(*right_out));

    fn(apply_window)(s, windowed_left,  left_out,  1);
    fn(apply_window)(s, windowed_right, right_out, 1);

    if (ff_filter_disabled(ctx)) {
        memcpy(left_osamples, left_in, out->nb_samples * sizeof(*left_osamples));
        memcpy(right_osamples, right_in, out->nb_samples * sizeof(*right_osamples));
    } else {
        memcpy(left_osamples, left_out, out->nb_samples * sizeof(*left_osamples));
        memcpy(right_osamples, right_out, out->nb_samples * sizeof(*right_osamples));
    }

    return 0;
}
