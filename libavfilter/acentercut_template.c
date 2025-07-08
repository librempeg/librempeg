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
#undef ftype
#undef SQRT
#undef SAMPLE_FORMAT
#undef TX_TYPE
#undef SIN
#undef EPSILON
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define SQRT sqrtf
#define ctype AVComplexFloat
#define ftype float
#define TX_TYPE AV_TX_FLOAT_RDFT
#define SIN sinf
#define EPSILON FLT_EPSILON
#else
#define SAMPLE_FORMAT double
#define SQRT sqrt
#define ctype AVComplexDouble
#define ftype double
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define SIN sin
#define EPSILON DBL_EPSILON
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(cc_tx_init)(AVFilterContext *ctx)
{
    AudioCenterCutContext *s = ctx->priv;
    ftype scale = F(1.0), iscale = F(1.0) / (s->fft_size * F(1.5));
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

static void fn(apply_window)(AudioCenterCutContext *s,
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

static ftype fn(sqr)(ftype x)
{
    return x * x;
}

static void fn(center_cut)(ctype *left, ctype *right, const int N, const ftype factor)
{
    for (int i = 0; i < N; i++) {
        const ftype l_re = left[i].re;
        const ftype l_im = left[i].im;
        const ftype r_re = right[i].re;
        const ftype r_im = right[i].im;
        const ftype sum_re = l_re + r_re;
        const ftype sum_im = l_im + r_im;
        const ftype diff_re = l_re - r_re;
        const ftype diff_im = l_im - r_im;
        const ftype a = F(0.5) * (F(1.0) - SQRT((fn(sqr)(diff_re) + fn(sqr)(diff_im)) /
                                                (fn(sqr)(sum_re) + fn(sqr)(sum_im) + EPSILON))) * factor;
        ftype c_im, c_re;

        c_re = a * sum_re;
        c_im = a * sum_im;

        left[i].re -= c_re;
        left[i].im -= c_im;
        right[i].re -= c_re;
        right[i].im -= c_im;
    }
}

static int fn(cc_stereo)(AVFilterContext *ctx, AVFrame *out)
{
    AudioCenterCutContext *s = ctx->priv;
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
    const ftype factor = s->factor;

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

    fn(center_cut)(windowed_oleft, windowed_oright,
                   s->fft_size/2 + 1, factor);

    s->itx_fn(s->itx_ctx, windowed_left, windowed_oleft, sizeof(ctype));
    s->itx_fn(s->itx_ctx, windowed_right, windowed_oright, sizeof(ctype));

    fn(apply_window)(s, windowed_left, left_out,  1);
    fn(apply_window)(s, windowed_right, right_out,  1);

    if (ff_filter_disabled(ctx)) {
        memcpy(left_osamples, left_in, overlap * sizeof(*left_osamples));
        memcpy(right_osamples, right_in, overlap * sizeof(*right_osamples));
    } else {
        memcpy(left_osamples, left_out, overlap * sizeof(*left_osamples));
        memcpy(right_osamples, right_out, overlap * sizeof(*right_osamples));
    }

    return 0;
}

static int fn(cc_flush)(AVFilterContext *ctx, AVFrame *out)
{
    AudioCenterCutContext *s = ctx->priv;
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
    const ftype factor = s->factor;

    // shift in/out buffers
    memmove(left_in, &left_in[overlap], offset * sizeof(*left_in));
    memmove(right_in, &right_in[overlap], offset * sizeof(*right_in));
    memmove(left_out, &left_out[overlap], offset * sizeof(*left_out));
    memmove(right_out, &right_out[overlap], offset * sizeof(*right_out));

    memset(&left_out[offset], 0, overlap * sizeof(*left_out));
    memset(&right_out[offset], 0, overlap * sizeof(*right_out));

    fn(apply_window)(s, left_in,  windowed_left,  0);
    fn(apply_window)(s, right_in, windowed_right, 0);

    s->tx_fn(s->tx_ctx, windowed_oleft,  windowed_left,  sizeof(ftype));
    s->tx_fn(s->tx_ctx, windowed_oright, windowed_right, sizeof(ftype));

    fn(center_cut)(windowed_oleft, windowed_oright,
                   s->fft_size/2 + 1, factor);

    s->itx_fn(s->itx_ctx, windowed_left, windowed_oleft, sizeof(ctype));
    s->itx_fn(s->itx_ctx, windowed_right, windowed_oright, sizeof(ctype));

    fn(apply_window)(s, windowed_left, left_out,  1);
    fn(apply_window)(s, windowed_right, right_out,  1);

    if (ff_filter_disabled(ctx)) {
        memcpy(left_osamples, left_in, overlap * sizeof(*left_osamples));
        memcpy(right_osamples, right_in, overlap * sizeof(*right_osamples));
    } else {
        memcpy(left_osamples, left_out, overlap * sizeof(*left_osamples));
        memcpy(right_osamples, right_out, overlap * sizeof(*right_osamples));
    }

    return 0;
}
