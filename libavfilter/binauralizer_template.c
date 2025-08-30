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

static int fn(ba_tx_init)(AVFilterContext *ctx)
{
    BinauralizerContext *s = ctx->priv;
    const int nb_in_channels = ctx->inputs[0]->ch_layout.nb_channels;;
    ftype scale = F(1.0), iscale = F(0.5) / (s->fft_size * nb_in_channels);
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

static void fn(apply_window)(BinauralizerContext *s,
                             const ftype *in_frame, ftype *out_frame, const int add_to_out_frame,
                             const ftype gain)
{
    const ftype *window = s->window;
    const int size = s->fft_size;

    if (add_to_out_frame) {
        for (int i = 0; i < size; i++)
            out_frame[i] += in_frame[i] * window[i] * gain;
    } else {
        for (int i = 0; i < size; i++)
            out_frame[i] = in_frame[i] * window[i] * gain;
    }
}

static void fn(binaural)(ctype *fl, ctype *fr, ctype *in,
                         const ftype angle, const ftype depth, const int N)
{
    const ftype hd = depth * F(0.5);
    const ftype lcd = COS(-hd);
    const ftype lsd = SIN(-hd);
    const ftype rcd = COS(hd);
    const ftype rsd = SIN(hd);
    const ftype ca = COS(angle);
    const ftype sa = SIN(angle);

    for (int i = 0; i < N; i++) {
        const ftype in_re = in[i].re;
        const ftype in_im = in[i].im;
        const ftype l_re = in_re * ca;
        const ftype l_im = in_im * ca;
        const ftype r_re = in_re * sa;
        const ftype r_im = in_im * sa;

        fl[i].re += (l_re * lcd - l_im * lsd);
        fl[i].im += (l_re * lsd + l_im * lcd);
        fr[i].re += (r_re * rcd - r_im * rsd);
        fr[i].im += (r_re * rsd + r_im * rcd);
    }
}

static void fn(get_angle_depth)(const AVChannelLayout *ch_layout, const int ch,
                                const double *x, const double *y,
                                const unsigned nb_x_size, const unsigned nb_y_size,
                                ftype *angle, ftype *depth)
{
    const ftype x_size = x[FFMIN(ch, nb_x_size-1)];
    const ftype y_size = x[FFMIN(ch, nb_y_size-1)];
    const int chan = av_channel_layout_channel_from_index(ch_layout, ch);
    ftype a, d;

    switch (chan) {
    case AV_CHAN_FRONT_LEFT:            a = F( 0.5) + (F(1.0) - x_size) * F(44.5), d = (F(1.0) - y_size) * F(90.0); break;
    case AV_CHAN_FRONT_RIGHT:           a = F(89.5) - (F(1.0) - x_size) * F(44.5), d = (F(1.0) - y_size) * F(90.0); break;
    case AV_CHAN_FRONT_CENTER:          a = F(45.0),                               d = (F(1.0) - y_size) * F(90.0); break;
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:       a = F(45.0),                               d = F(90.0) * y_size;            break;
    case AV_CHAN_BACK_LEFT:             a = F( 0.5) + (F(1.0) - x_size) * F(44.5), d = F(90.0) + y_size * F(89.0);  break;
    case AV_CHAN_BACK_RIGHT:            a = F(89.5) - (F(1.0) - x_size) * F(44.5), d = F(90.0) + y_size * F(89.0);  break;
    case AV_CHAN_BACK_CENTER:           a = F(45.0),                               d = F(90.0) + y_size * F(89.0);  break;
    case AV_CHAN_SIDE_LEFT:             a = F( 0.5) + (F(1.0) - x_size) * F(44.5), d = F(85.0) * y_size;            break;
    case AV_CHAN_SIDE_RIGHT:            a = F(89.5) - (F(1.0) - x_size) * F(44.5), d = F(85.0) * y_size;            break;
    case AV_CHAN_FRONT_LEFT_OF_CENTER:  a = F(15.0),                               d = F(20.0) * y_size;            break;
    case AV_CHAN_FRONT_RIGHT_OF_CENTER: a = F(75.0),                               d = F(20.0) * y_size;            break;
    case AV_CHAN_WIDE_LEFT:             a = F( 0.5) + (F(1.0) - x_size) * F(44.5), d = F(90.0) * y_size;            break;
    case AV_CHAN_WIDE_RIGHT:            a = F(89.5) - (F(1.0) - x_size) * F(44.5), d = F(90.0) * y_size;            break;
    case AV_CHAN_SURROUND_DIRECT_LEFT:  a = F( 0.5) + (F(1.0) - x_size) * F(44.5), d = F(85.0) * y_size;            break;
    case AV_CHAN_SURROUND_DIRECT_RIGHT: a = F(89.5) - (F(1.0) - x_size) * F(44.5), d = F(85.0) * y_size;            break;
    case AV_CHAN_STEREO_LEFT:           a = F( 0.5) + (F(1.0) - x_size) * F(44.5), d = F(30.0) * y_size;            break;
    case AV_CHAN_STEREO_RIGHT:          a = F(89.5) - (F(1.0) - x_size) * F(44.5), d = F(30.0) * y_size;            break;
    default:                            a = F(45.0),                               d = F(0.0);                      break;
    }

    *angle = a * F(M_PI/180.0);
    *depth = d * F(M_PI/180.0);
}

static int fn(ba_stereo)(AVFilterContext *ctx, AVFrame *out)
{
    BinauralizerContext *s = ctx->priv;
    const AVChannelLayout *ch_layout = &ctx->inputs[0]->ch_layout;
    ftype *left_out        = (ftype *)s->out_dist_frame->extended_data[0];
    ftype *right_out       = (ftype *)s->out_dist_frame->extended_data[1];
    ftype *windowed_left   = (ftype *)s->windowed_frame->extended_data[0];
    ftype *windowed_right  = (ftype *)s->windowed_frame->extended_data[1];
    ctype *windowed_oleft  = (ctype *)s->windowed_out->extended_data[0];
    ctype *windowed_oright = (ctype *)s->windowed_out->extended_data[1];
    ftype *left_osamples   = (ftype *)out->extended_data[0];
    ftype *right_osamples  = (ftype *)out->extended_data[1];
    const int nb_in_channels = ch_layout->nb_channels;;
    const int overlap = s->overlap;
    const int offset = s->fft_size - overlap;
    const int nb_samples = FFMIN(overlap, s->in->nb_samples);
    const unsigned nb_x_size = s->nb_x_size;
    const unsigned nb_y_size = s->nb_y_size;
    const unsigned nb_i_gain = s->nb_i_gain;
    const double *x_size = s->x_size;
    const double *y_size = s->y_size;
    const double *i_gain = s->i_gain;
    const int N = s->fft_size/2 + 1;
    const ftype G = s->gain;

    memset(windowed_oleft, 0, N * sizeof(*windowed_oleft));
    memset(windowed_oright, 0, N * sizeof(*windowed_oright));

    for (int ch = 0; ch < nb_in_channels; ch++) {
        const ftype igain = i_gain[FFMIN(ch, nb_i_gain-1)];
        ftype *in = (ftype *)s->in_frame->extended_data[ch];
        ftype *iwindowed = (ftype *)s->windowed_frame->extended_data[0];
        ctype *owindowed = (ctype *)s->windowed_frame->extended_data[1];
        const ftype *samples = (const ftype *)s->in->extended_data[ch];
        ftype angle, depth;

        fn(get_angle_depth)(ch_layout, ch, x_size, y_size,
                            nb_x_size, nb_y_size,
                            &angle, &depth);

        // shift in/out buffers
        memmove(in, &in[overlap], offset * sizeof(*in));

        memcpy(&in[offset], samples, nb_samples * sizeof(*in));
        memset(&in[offset + nb_samples], 0, (overlap - nb_samples) * sizeof(*in));

        fn(apply_window)(s, in, iwindowed, 0, igain);

        s->tx_fn(s->tx_ctx, owindowed, iwindowed, sizeof(ftype));

        fn(binaural)(windowed_oleft, windowed_oright, owindowed, angle, depth, N);
    }

    s->itx_fn(s->itx_ctx, windowed_left, windowed_oleft, sizeof(ctype));
    s->itx_fn(s->itx_ctx, windowed_right, windowed_oright, sizeof(ctype));

    memmove(left_out, &left_out[overlap], offset * sizeof(*left_out));
    memmove(right_out, &right_out[overlap], offset * sizeof(*right_out));
    memset(&left_out[offset], 0, overlap * sizeof(*left_out));
    memset(&right_out[offset], 0, overlap * sizeof(*right_out));

    fn(apply_window)(s, windowed_left,  left_out,  1, G);
    fn(apply_window)(s, windowed_right, right_out, 1, G);

    memcpy(left_osamples, left_out, out->nb_samples * sizeof(*left_osamples));
    memcpy(right_osamples, right_out, out->nb_samples * sizeof(*right_osamples));

    return 0;
}

static int fn(ba_flush)(AVFilterContext *ctx, AVFrame *out)
{
    BinauralizerContext *s = ctx->priv;
    const AVChannelLayout *ch_layout = &ctx->inputs[0]->ch_layout;
    ftype *left_out        = (ftype *)s->out_dist_frame->extended_data[0];
    ftype *right_out       = (ftype *)s->out_dist_frame->extended_data[1];
    ftype *windowed_left   = (ftype *)s->windowed_frame->extended_data[0];
    ftype *windowed_right  = (ftype *)s->windowed_frame->extended_data[1];
    ctype *windowed_oleft  = (ctype *)s->windowed_out->extended_data[0];
    ctype *windowed_oright = (ctype *)s->windowed_out->extended_data[1];
    ftype *left_osamples   = (ftype *)out->extended_data[0];
    ftype *right_osamples  = (ftype *)out->extended_data[1];
    const int nb_in_channels = ch_layout->nb_channels;;
    const int overlap = s->overlap;
    const int offset = s->fft_size - overlap;
    const unsigned nb_x_size = s->nb_x_size;
    const unsigned nb_y_size = s->nb_y_size;
    const unsigned nb_i_gain = s->nb_i_gain;
    const double *x_size = s->x_size;
    const double *y_size = s->y_size;
    const double *i_gain = s->i_gain;
    const int N = s->fft_size/2 + 1;
    const int nb_samples = 0;
    const ftype G = s->gain;

    memset(windowed_oleft, 0, N * sizeof(*windowed_oleft));
    memset(windowed_oright, 0, N * sizeof(*windowed_oright));

    for (int ch = 0; ch < nb_in_channels; ch++) {
        const ftype igain = i_gain[FFMIN(ch, nb_i_gain-1)];
        ftype *in = (ftype *)s->in_frame->extended_data[ch];
        ftype *iwindowed = (ftype *)s->windowed_frame->extended_data[0];
        ctype *owindowed = (ctype *)s->windowed_frame->extended_data[1];
        ftype angle, depth;

        fn(get_angle_depth)(ch_layout, ch, x_size, y_size,
                            nb_x_size, nb_y_size,
                            &angle, &depth);

        // shift in/out buffers
        memmove(in, &in[overlap], offset * sizeof(*in));

        memset(&in[offset + nb_samples], 0, (overlap - nb_samples) * sizeof(*in));

        fn(apply_window)(s, in, iwindowed, 0, igain);

        s->tx_fn(s->tx_ctx, owindowed, iwindowed, sizeof(ftype));

        fn(binaural)(windowed_oleft, windowed_oright, owindowed, angle, depth, N);
    }

    s->itx_fn(s->itx_ctx, windowed_left, windowed_oleft, sizeof(ctype));
    s->itx_fn(s->itx_ctx, windowed_right, windowed_oright, sizeof(ctype));

    memmove(left_out, &left_out[overlap], offset * sizeof(*left_out));
    memmove(right_out, &right_out[overlap], offset * sizeof(*right_out));
    memset(&left_out[offset], 0, overlap * sizeof(*left_out));
    memset(&right_out[offset], 0, overlap * sizeof(*right_out));

    fn(apply_window)(s, windowed_left,  left_out,  1, G);
    fn(apply_window)(s, windowed_right, right_out, 1, G);

    memcpy(left_osamples, left_out, out->nb_samples * sizeof(*left_osamples));
    memcpy(right_osamples, right_out, out->nb_samples * sizeof(*right_osamples));

    return 0;
}
