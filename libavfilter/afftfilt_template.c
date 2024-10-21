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
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define ctype AVComplexFloat
#define ftype float
#define TX_TYPE AV_TX_FLOAT_RDFT
#else
#define SAMPLE_FORMAT double
#define ctype AVComplexDouble
#define ftype double
#define TX_TYPE AV_TX_DOUBLE_RDFT
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static double fn(getreal)(void *priv, double x, double ch)
{
    AFFTFiltContext *s = priv;
    ctype *tx_out;
    int ich, ix;

    ich = av_clip(ch, 0, s->nb_exprs - 1);
    ix = av_clip(x, 0, s->win_size/2+1);
    tx_out = (ctype *)s->tx_out->extended_data[ich];

    return tx_out[ix].re;
}

static double fn(getimag)(void *priv, double x, double ch)
{
    AFFTFiltContext *s = priv;
    ctype *tx_out;
    int ich, ix;

    ich = av_clip(ch, 0, s->nb_exprs - 1);
    ix = av_clip(x, 0, s->win_size/2+1);
    tx_out = (ctype *)s->tx_out->extended_data[ich];

    return tx_out[ix].im;
}

static void fn(apply_window)(AFFTFiltContext *s,
                             const ftype *in_frame, ftype *out_frame,
                             const int add_to_out_frame)
{
    const float *window = s->window_func_lut;
    const int fft_size = s->win_size;

    if (add_to_out_frame) {
        for (int i = 0; i < fft_size; i++)
            out_frame[i] += in_frame[i] * window[i];
    } else {
        for (int i = 0; i < fft_size; i++)
            out_frame[i] = in_frame[i] * window[i];
    }
}

static double (*const fn(func2)[])(void *, double, double) = {  fn(getreal),  fn(getimag), NULL };

static int fn(tx_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AFFTFiltContext *s = ctx->priv;
    AVFrame *in = arg;
    const int channels = s->channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        const int offset = s->win_size - s->hop_size;
        ftype *src = (ftype *)s->window->extended_data[ch];
        ctype *tx_out = (ctype *)s->tx_out->extended_data[ch];
        ftype *tx_in = (ftype *)s->tx_in->extended_data[ch];

        memmove(src, &src[s->hop_size], offset * sizeof(ftype));
        memcpy(&src[offset], in->extended_data[ch], in->nb_samples * sizeof(ftype));
        memset(&src[offset + in->nb_samples], 0, (s->hop_size - in->nb_samples) * sizeof(ftype));

        fn(apply_window)(s, src, tx_in, 0);

        s->tx_fn(s->tx[ch], tx_out, tx_in, sizeof(*tx_in));
    }

    return 0;
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AFFTFiltContext *s = ctx->priv;
    const int win_size = s->win_size;
    const int channels = s->channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;
    double values[VAR_VARS_NB];
    AVFrame *out = s->out;

    memcpy(values, arg, sizeof(values));

    for (int ch = start; ch < end; ch++) {
        ctype *tx_out = (ctype *)s->tx_out->extended_data[ch];
        ctype *tx_temp = (ctype *)s->tx_temp->extended_data[ch];
        ftype *buf = (ftype *)s->buffer->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        ftype *tx_in = (ftype *)s->tx_in->extended_data[ch];

        values[VAR_CHANNEL] = ch;

        if (ff_filter_disabled(ctx)) {
            for (int n = 0; n <= win_size/2; n++) {
                tx_temp[n].re = tx_out[n].re;
                tx_temp[n].im = tx_out[n].im;
            }
        } else {
            for (int n = 0; n <= win_size/2; n++) {
                ftype fr, fi;

                values[VAR_BIN] = n;
                values[VAR_REAL] = tx_out[n].re;
                values[VAR_IMAG] = tx_out[n].im;

                fr = av_expr_eval(s->real[ch], values, s);
                fi = av_expr_eval(s->imag[ch], values, s);

                tx_temp[n].re = fr;
                tx_temp[n].im = fi;
            }
        }

        s->itx_fn(s->itx[ch], tx_in, tx_temp, sizeof(*tx_temp));

        memmove(buf, buf + s->hop_size, win_size * sizeof(ftype));

        fn(apply_window)(s, tx_in, buf, 1);

        memcpy(dst, buf, s->hop_size * sizeof(ftype));
    }

    return 0;
}
