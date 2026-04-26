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
#undef SIN
#undef COS
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define ctype AVComplexFloat
#define ftype float
#define TX_TYPE AV_TX_FLOAT_RDFT
#define SIN sinf
#define COS cosf
#else
#define SAMPLE_FORMAT double
#define ctype AVComplexDouble
#define ftype double
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define SIN sin
#define COS cos
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateOutContext) {
    ftype *out_dist_frame;
    ftype *windowed_frame;
    ctype *windowed_out;

    AVTXContext *itx_ctx;
    av_tx_fn itx_fn;
} fn(StateOutContext);

typedef struct fn(StateInContext) {
    ftype *in_frame;
    ftype *iwindowed_frame;
    ctype *owindowed_frame;
    ctype *windowed_outl;
    ctype *windowed_outr;

    AVTXContext *tx_ctx;
    av_tx_fn tx_fn;
} fn(StateInContext);

static int fn(ba_init)(AVFilterContext *ctx)
{
    BinauralizerContext *s = ctx->priv;
    fn(StateOutContext) *state_out;
    fn(StateInContext) *state_in;
    const int nb_in_channels = ctx->inputs[0]->ch_layout.nb_channels;
    ftype scale = F(1.0), iscale = F(1.0) / (s->rdft_size * (F(1.5 / 2.0) * nb_in_channels));
    ftype *window;
    int ret;

    s->state_in = av_calloc(s->nb_in_channels, sizeof(*state_in));
    if (!s->state_in)
        return AVERROR(ENOMEM);
    state_in = s->state_in;

    s->state_out = av_calloc(2, sizeof(*state_out));
    if (!s->state_out)
        return AVERROR(ENOMEM);
    state_out = s->state_out;

    s->window = av_calloc(s->rdft_size, sizeof(*window));
    if (!s->window)
        return AVERROR(ENOMEM);
    window = s->window;
    for (int n = 0; n < s->rdft_size; n++)
        window[n] = SIN(F(M_PI)*(n+F(0.5))/s->rdft_size);

    for (int ch = 0; ch < s->nb_in_channels; ch++) {
        fn(StateInContext) *stc = &state_in[ch];

        stc->in_frame = av_calloc(s->rdft_size + 2, sizeof(*stc->in_frame));
        stc->iwindowed_frame = av_calloc(s->rdft_size + 2, sizeof(*stc->iwindowed_frame));
        stc->owindowed_frame = av_calloc(s->rdft_size + 2, sizeof(*stc->owindowed_frame));
        stc->windowed_outl = av_calloc(s->rdft_size + 2, sizeof(*stc->windowed_outl));
        stc->windowed_outr = av_calloc(s->rdft_size + 2, sizeof(*stc->windowed_outr));
        if (!stc->in_frame || !stc->iwindowed_frame || !stc->owindowed_frame ||
            !stc->windowed_outl || !stc->windowed_outr)
            return AVERROR(ENOMEM);

        ret = av_tx_init(&stc->tx_ctx, &stc->tx_fn, TX_TYPE, 0, s->rdft_size, &scale, 0);
        if (ret < 0)
            return ret;
    }

    for (int ch = 0; ch < 2; ch++) {
        fn(StateOutContext) *stc = &state_out[ch];

        stc->out_dist_frame = av_calloc(s->rdft_size * 2, sizeof(*stc->out_dist_frame));
        stc->windowed_frame = av_calloc(s->rdft_size + 2, sizeof(*stc->windowed_frame));
        stc->windowed_out = av_calloc(s->rdft_size + 2, sizeof(*stc->windowed_out));
        if (!stc->out_dist_frame || !stc->windowed_frame || !stc->windowed_out)
            return AVERROR(ENOMEM);

        ret = av_tx_init(&stc->itx_ctx, &stc->itx_fn, TX_TYPE, 1, s->rdft_size, &iscale, 0);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static void fn(apply_window)(BinauralizerContext *s,
                             const ftype *in_frame, ftype *out_frame, const int add_to_out_frame,
                             const ftype gain)
{
    const ftype *window = s->window;
    const int size = s->rdft_size;

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
    const ftype rcd =  lcd;
    const ftype rsd = -lsd;
    const ftype ca = COS(angle);
    const ftype sa = SIN(angle);

    for (int i = 0; i < N; i++) {
        const ftype in_re = in[i].re;
        const ftype in_im = in[i].im;
        const ftype l_re = in_re * ca;
        const ftype l_im = in_im * ca;
        const ftype r_re = in_re * sa;
        const ftype r_im = in_im * sa;

        fl[i].re = (l_re * lcd - l_im * lsd);
        fl[i].im = (l_re * lsd + l_im * lcd);
        fr[i].re = (r_re * rcd - r_im * rsd);
        fr[i].im = (r_re * rsd + r_im * rcd);
    }
}

static void fn(get_angle_depth)(const AVChannelLayout *ch_layout, const int ch,
                                const double *x, const double *y,
                                const unsigned nb_x_size, const unsigned nb_y_size,
                                const double *xoff, const double *yoff,
                                const unsigned nb_x_offset, const unsigned nb_y_offset,
                                ftype *angle, ftype *depth)
{
    const ftype x_off = xoff[FFMIN(ch, nb_x_offset-1)];
    const ftype y_off = yoff[FFMIN(ch, nb_y_offset-1)];
    const ftype x_size = x[FFMIN(ch, nb_x_size-1)];
    const ftype y_size = y[FFMIN(ch, nb_y_size-1)];
    const int chan = av_channel_layout_channel_from_index(ch_layout, ch);
    ftype a, d;

    switch (chan) {
    case AV_CHAN_FRONT_LEFT:            a = F( 0.5) + (F(1.0) - x_size) * F(44.5), d = F(0.0);                      break;
    case AV_CHAN_FRONT_RIGHT:           a = F(89.5) - (F(1.0) - x_size) * F(44.5), d = F(0.0);                      break;
    case AV_CHAN_FRONT_CENTER:          a = F(45.0),                               d = F(0.0);                      break;
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:       a = F(45.0),                               d = F(90.0) * y_size;            break;
    case AV_CHAN_BACK_LEFT:             a = F( 0.5) + (F(1.0) - x_size) * F(44.5), d = y_size * F(189.0);           break;
    case AV_CHAN_BACK_RIGHT:            a = F(89.5) - (F(1.0) - x_size) * F(44.5), d = y_size * F(189.0);           break;
    case AV_CHAN_BACK_CENTER:           a = F(45.0),                               d = y_size * F(189.0);           break;
    case AV_CHAN_SIDE_LEFT:             a = F( 0.5) + (F(1.0) - x_size) * F(44.5), d = F(85.0) * y_size;            break;
    case AV_CHAN_SIDE_RIGHT:            a = F(89.5) - (F(1.0) - x_size) * F(44.5), d = F(85.0) * y_size;            break;
    case AV_CHAN_FRONT_LEFT_OF_CENTER:  a = F(15.0),                               d = F(0.0);                      break;
    case AV_CHAN_FRONT_RIGHT_OF_CENTER: a = F(75.0),                               d = F(0.0);                      break;
    case AV_CHAN_WIDE_LEFT:             a = F( 0.5) + (F(1.0) - x_size) * F(44.5), d = F(90.0) * y_size;            break;
    case AV_CHAN_WIDE_RIGHT:            a = F(89.5) - (F(1.0) - x_size) * F(44.5), d = F(90.0) * y_size;            break;
    case AV_CHAN_SURROUND_DIRECT_LEFT:  a = F( 0.5) + (F(1.0) - x_size) * F(44.5), d = F(85.0) * y_size;            break;
    case AV_CHAN_SURROUND_DIRECT_RIGHT: a = F(89.5) - (F(1.0) - x_size) * F(44.5), d = F(85.0) * y_size;            break;
    case AV_CHAN_STEREO_LEFT:           a = F( 0.5) + (F(1.0) - x_size) * F(44.5), d = F(30.0) * y_size;            break;
    case AV_CHAN_STEREO_RIGHT:          a = F(89.5) - (F(1.0) - x_size) * F(44.5), d = F(30.0) * y_size;            break;
    default:                            a = F(45.0),                               d = F(0.0);                      break;
    }

    *angle = (a + x_off * F(90.0)) * F(M_PI/180.0);
    *depth = (d + y_off * F(90.0)) * F(M_PI/180.0);
}

static int fn(ba_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    BinauralizerContext *s = ctx->priv;
    fn(StateInContext) *state_in = s->state_in;
    const AVChannelLayout *ch_layout = &ctx->inputs[0]->ch_layout;
    const int nb_in_channels = ch_layout->nb_channels;
    const int start = (nb_in_channels * jobnr) / nb_jobs;
    const int end = (nb_in_channels * (jobnr+1)) / nb_jobs;
    const unsigned nb_x_offset = s->nb_x_offset;
    const unsigned nb_y_offset = s->nb_y_offset;
    const unsigned nb_x_size = s->nb_x_size;
    const unsigned nb_y_size = s->nb_y_size;
    const unsigned nb_i_gain = s->nb_i_gain;
    const double *x_offset = s->x_offset;
    const double *y_offset = s->y_offset;
    const double *x_size = s->x_size;
    const double *y_size = s->y_size;
    const double *i_gain = s->i_gain;
    const int overlap = s->overlap;
    const int offset = s->rdft_size - overlap;
    const int N = s->rdft_size/2 + 1;
    ThreadData *td = arg;
    const int nb_samples = td->nb_samples;
    const int doffset = td->offset;

    for (int ch = start; ch < end; ch++) {
        fn(StateInContext) *stc = &state_in[ch];
        const ftype igain = i_gain[FFMIN(ch, nb_i_gain-1)];
        ctype *windowed_outl = stc->windowed_outl;
        ctype *windowed_outr = stc->windowed_outr;
        ftype *iwindowed = stc->iwindowed_frame;
        ctype *owindowed = stc->owindowed_frame;
        ftype *in = stc->in_frame;
        ftype angle, depth;

        fn(get_angle_depth)(ch_layout, ch, x_size, y_size,
                            nb_x_size, nb_y_size,
                            x_offset, y_offset,
                            nb_x_offset, nb_y_offset,
                            &angle, &depth);

        // shift in/out buffers
        memmove(in, &in[overlap], offset * sizeof(*in));

        if (nb_samples > 0) {
            const ftype *samples = ((const ftype *)td->in->extended_data[ch]) + doffset;
            memcpy(&in[offset], samples, nb_samples * sizeof(*in));
        }
        memset(&in[offset + nb_samples], 0, (overlap - nb_samples) * sizeof(*in));

        fn(apply_window)(s, in, iwindowed, 0, igain);

        stc->tx_fn(stc->tx_ctx, owindowed, iwindowed, sizeof(ftype));

        fn(binaural)(windowed_outl, windowed_outr, owindowed, angle, depth, N);
    }

    return 0;
}

static int fn(ba_stereo)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int doffset)
{
    BinauralizerContext *s = ctx->priv;
    fn(StateOutContext) *state_out = s->state_out;
    fn(StateInContext) *state_in = s->state_in;
    const AVChannelLayout *ch_layout = &ctx->inputs[0]->ch_layout;
    ftype *left_out        = state_out[0].out_dist_frame;
    ftype *right_out       = state_out[1].out_dist_frame;
    ftype *windowed_left   = state_out[0].windowed_frame;
    ftype *windowed_right  = state_out[1].windowed_frame;
    ctype *windowed_oleft  = state_out[0].windowed_out;
    ctype *windowed_oright = state_out[1].windowed_out;
    ftype *left_osamples   = ((ftype *)out->extended_data[0]) + doffset;
    ftype *right_osamples  = ((ftype *)out->extended_data[1]) + doffset;
    const int nb_in_channels = ch_layout->nb_channels;
    const int overlap = s->overlap;
    const int nb_samples = FFMIN(overlap, in->nb_samples - doffset);
    const int out_nb_samples = FFMIN(overlap, out->nb_samples - doffset);
    const int offset = s->rdft_size - overlap;
    const int N = s->rdft_size/2 + 1;
    const ftype G = s->gain;
    ThreadData td;

    memset(windowed_oleft, 0, N * sizeof(*windowed_oleft));
    memset(windowed_oright, 0, N * sizeof(*windowed_oright));

    td.offset = doffset;
    td.nb_samples = nb_samples;
    td.in = in;
    ff_filter_execute(ctx, fn(ba_channels), &td, NULL,
                      FFMIN(nb_in_channels, ff_filter_get_nb_threads(ctx)));

    for (int ch = 0; ch < nb_in_channels; ch++) {
        fn(StateInContext) *stc = &state_in[ch];
        const ctype *windowed_outl = stc->windowed_outl;
        const ctype *windowed_outr = stc->windowed_outr;

        for (int n = 0; n < N; n++) {
            windowed_oleft[n].re  += windowed_outl[n].re;
            windowed_oleft[n].im  += windowed_outl[n].im;
            windowed_oright[n].re += windowed_outr[n].re;
            windowed_oright[n].im += windowed_outr[n].im;
        }
    }

    state_out[0].itx_fn(state_out[0].itx_ctx, windowed_left, windowed_oleft, sizeof(ctype));
    state_out[1].itx_fn(state_out[1].itx_ctx, windowed_right, windowed_oright, sizeof(ctype));

    memmove(left_out, &left_out[overlap], offset * sizeof(*left_out));
    memmove(right_out, &right_out[overlap], offset * sizeof(*right_out));
    memset(&left_out[offset], 0, overlap * sizeof(*left_out));
    memset(&right_out[offset], 0, overlap * sizeof(*right_out));

    fn(apply_window)(s, windowed_left,  left_out,  1, G);
    fn(apply_window)(s, windowed_right, right_out, 1, G);

    memcpy(left_osamples, left_out, out_nb_samples * sizeof(*left_osamples));
    memcpy(right_osamples, right_out, out_nb_samples * sizeof(*right_osamples));

    return 0;
}

static int fn(ba_flush)(AVFilterContext *ctx, AVFrame *out, const int doffset)
{
    BinauralizerContext *s = ctx->priv;
    fn(StateOutContext) *state_out = s->state_out;
    const AVChannelLayout *ch_layout = &ctx->inputs[0]->ch_layout;
    ftype *left_out        = state_out[0].out_dist_frame;
    ftype *right_out       = state_out[1].out_dist_frame;
    ftype *windowed_left   = state_out[0].windowed_frame;
    ftype *windowed_right  = state_out[1].windowed_frame;
    ctype *windowed_oleft  = state_out[0].windowed_out;
    ctype *windowed_oright = state_out[1].windowed_out;
    ftype *left_osamples   = (ftype *)out->extended_data[0];
    ftype *right_osamples  = (ftype *)out->extended_data[1];
    const int nb_in_channels = ch_layout->nb_channels;
    const int overlap = s->overlap;
    const int offset = s->rdft_size - overlap;
    const int N = s->rdft_size/2 + 1;
    const int nb_samples = 0;
    const ftype G = s->gain;
    ThreadData td;

    memset(windowed_oleft, 0, N * sizeof(*windowed_oleft));
    memset(windowed_oright, 0, N * sizeof(*windowed_oright));

    td.offset = doffset;
    td.nb_samples = nb_samples;
    td.in = NULL;
    ff_filter_execute(ctx, fn(ba_channels), &td, NULL,
                      FFMIN(nb_in_channels, ff_filter_get_nb_threads(ctx)));

    for (int ch = 0; ch < nb_in_channels; ch++) {
        fn(StateInContext) *state_in = s->state_in;
        fn(StateInContext) *stc = &state_in[ch];
        const ctype *windowed_outl = stc->windowed_outl;
        const ctype *windowed_outr = stc->windowed_outr;

        for (int n = 0; n < N; n++) {
            windowed_oleft[n].re  += windowed_outl[n].re;
            windowed_oleft[n].im  += windowed_outl[n].im;
            windowed_oright[n].re += windowed_outr[n].re;
            windowed_oright[n].im += windowed_outr[n].im;
        }
    }

    state_out[0].itx_fn(state_out[0].itx_ctx, windowed_left, windowed_oleft, sizeof(ctype));
    state_out[1].itx_fn(state_out[1].itx_ctx, windowed_right, windowed_oright, sizeof(ctype));

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

static void fn(ba_uninit)(AVFilterContext *ctx)
{
    BinauralizerContext *s = ctx->priv;

    if (s->state_in) {
        fn(StateInContext) *state = s->state_in;

        for (int ch = 0; ch < s->nb_in_channels; ch++) {
            fn(StateInContext) *stc = &state[ch];

            av_freep(&stc->in_frame);
            av_freep(&stc->iwindowed_frame);
            av_freep(&stc->owindowed_frame);
            av_freep(&stc->windowed_outl);
            av_freep(&stc->windowed_outr);

            av_tx_uninit(&stc->tx_ctx);
        }
    }

    if (s->state_out) {
        fn(StateOutContext) *state = s->state_out;

        for (int ch = 0; ch < 2; ch++) {
            fn(StateOutContext) *stc = &state[ch];

            av_freep(&stc->out_dist_frame);
            av_freep(&stc->windowed_frame);
            av_freep(&stc->windowed_out);

            av_tx_uninit(&stc->itx_ctx);
        }
    }

    av_freep(&s->state_out);
    av_freep(&s->state_in);
}
