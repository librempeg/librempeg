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
#undef ftype
#undef SAMPLE_FORMAT
#undef TX_TYPE
#undef ATAN2
#undef SIN
#undef COS
#undef CLIP
#undef FABS
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define ctype AVComplexFloat
#define ftype float
#define TX_TYPE AV_TX_FLOAT_RDFT
#define ATAN2 atan2f
#define SIN sinf
#define COS cosf
#define CLIP av_clipf
#define FABS fabsf
#else
#define SAMPLE_FORMAT double
#define ctype AVComplexDouble
#define ftype double
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define ATAN2 atan2
#define SIN sin
#define COS cos
#define CLIP av_clipd
#define FABS fabs
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(sf_tx_init)(AVFilterContext *ctx)
{
    StereoFieldContext *s = ctx->priv;
    ftype scale = F(1.0), iscale = F(1.0) / (s->fft_size * 2 * F(2.0));
    ftype *window;
    int ret;

    s->window = av_calloc(s->fft_size, sizeof(*window));
    if (!s->window)
        return AVERROR(ENOMEM);
    window = s->window;
    for (int n = 0; n < s->fft_size; n++)
        window[n] = SIN(M_PI*n/(s->fft_size-1));

    ret = av_tx_init(&s->tx_ctx, &s->tx_fn, TX_TYPE, 0, s->fft_size * 2, &scale, 0);
    if (ret < 0)
        return ret;

    ret = av_tx_init(&s->itx_ctx, &s->itx_fn, TX_TYPE, 1, s->fft_size * 2, &iscale, 0);
    if (ret < 0)
        return ret;

    return 0;
}

static void fn(apply_window)(StereoFieldContext *s,
                             const ftype *in_frame, ftype *out_frame, const int add_to_out_frame)
{
    const ftype *window = s->window;
    const int fft_size = s->fft_size;

    if (add_to_out_frame) {
        for (int i = 0; i < fft_size; i++)
            out_frame[i] += in_frame[i] * window[i];
    } else {
        for (int i = 0; i < fft_size; i++)
            out_frame[i] = in_frame[i] * window[i];
    }
}

static void fn(stereofield)(ctype *fl, ctype *fr, const int N,
                            const ftype d)
{
    const ftype m = (d < F(0.0)) ? d : F(0.0);
    const ftype s = (d > F(0.0)) ? d : F(0.0);

    for (int i = 0; i < N; i++) {
        const ftype l_re = fl[i].re;
        const ftype l_im = fl[i].im;
        const ftype r_re = fr[i].re;
        const ftype r_im = fr[i].im;
        const ftype re = l_re * r_re + l_im * r_im;
        const ftype im = l_re * r_im - l_im * r_re;
        const ftype a = FABS(ATAN2(im, re));
        const ftype lf = m * a + s * (F(M_PI) - a);
        const ftype lc = COS(lf);
        const ftype ls = SIN(lf);

        fl[i].re = l_re * lc - l_im * ls;
        fl[i].im = l_re * ls + l_im * lc;
        fr[i].re = r_re;
        fr[i].im = r_im;
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
    const ftype D = s->D;

    // shift in/out buffers
    memmove(left_in, &left_in[overlap], offset * sizeof(*left_in));
    memmove(right_in, &right_in[overlap], offset * sizeof(*right_in));
    memmove(left_out, &left_out[overlap], offset * sizeof(*left_out));
    memmove(right_out, &right_out[overlap], offset * sizeof(*right_out));

    memcpy(&left_in[offset], left_samples, nb_samples * sizeof(*left_in));
    memcpy(&right_in[offset], right_samples, nb_samples * sizeof(*right_in));
    memset(&left_out[offset], 0, overlap * sizeof(*left_out));
    memset(&right_out[offset], 0, overlap * sizeof(*right_out));

    fn(apply_window)(s, left_in,  windowed_left,  0);
    fn(apply_window)(s, right_in, windowed_right, 0);

    s->tx_fn(s->tx_ctx, windowed_oleft,  windowed_left,  sizeof(ftype));
    s->tx_fn(s->tx_ctx, windowed_oright, windowed_right, sizeof(ftype));

    fn(stereofield)(windowed_oleft, windowed_oright,
                    s->fft_size + 1, D);

    s->itx_fn(s->itx_ctx, windowed_left, windowed_oleft, sizeof(ctype));
    s->itx_fn(s->itx_ctx, windowed_right, windowed_oright, sizeof(ctype));

    fn(apply_window)(s, windowed_left,  left_out,  1);
    fn(apply_window)(s, windowed_right, right_out, 1);

    if (ctx->is_disabled) {
        memcpy(left_osamples, left_in, overlap * sizeof(*left_osamples));
        memcpy(right_osamples, right_in, overlap * sizeof(*right_osamples));
    } else {
        memcpy(left_osamples, left_out, overlap * sizeof(*left_osamples));
        memcpy(right_osamples, right_out, overlap * sizeof(*right_osamples));
    }

    return 0;
}
