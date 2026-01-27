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

#undef MPI
#undef M_2PI
#undef MPI4
#undef MSQRT1_2
#undef ftype
#undef ctype
#undef COPYSIGN
#undef HYPOT
#undef ATAN2
#undef FABS
#undef FEXP
#undef FMA
#undef SIN
#undef COS
#undef POW
#undef TAN
#undef SQRT
#undef FMIN
#undef FMAX
#undef LRINT
#undef EPSILON
#undef CLIP
#undef SAMPLE_FORMAT
#undef TX_TYPE

#if DEPTH == 32
#define MPI M_PIf
#define M_2PI M_2_PIf
#define MPI4 M_PI_4f
#define MSQRT1_2 M_SQRT1_2f
#define ftype float
#define ctype AVComplexFloat
#define COPYSIGN copysignf
#define HYPOT hypotf
#define ATAN2 atan2f
#define FABS fabsf
#define FEXP expf
#define FMA fmaf
#define SIN sinf
#define COS cosf
#define POW powf
#define TAN tanf
#define SQRT sqrtf
#define FMIN fminf
#define FMAX fmaxf
#define LRINT lrintf
#define EPSILON FLT_EPSILON
#define CLIP av_clipf
#define SAMPLE_FORMAT fltp
#define TX_TYPE AV_TX_FLOAT_RDFT
#else
#define MPI M_PI
#define M_2PI M_2_PI
#define MPI4 M_PI_4
#define MSQRT1_2 M_SQRT1_2
#define ftype double
#define ctype AVComplexDouble
#define COPYSIGN copysign
#define HYPOT hypot
#define ATAN2 atan2
#define FABS fabs
#define FEXP exp
#define FMA fma
#define SIN sin
#define COS cos
#define POW pow
#define TAN tan
#define SQRT sqrt
#define FMIN fmin
#define FMAX fmax
#define LRINT lrint
#define EPSILON FLT_EPSILON /* to keep similar output with float */
#define CLIP av_clipd
#define SAMPLE_FORMAT dblp
#define TX_TYPE AV_TX_DOUBLE_RDFT
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define F(x) ((ftype)(x))

#undef AVFILTER_WINDOW_FUNC_H
#include "window_func.h"

static void fn(set_input_levels)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *input_levels = s->input_levels;

    for (int ch = 0; ch < s->nb_in_channels; ch++) {
        const int fch = FFMIN(ch, s->nb_f_i-1);

        input_levels[ch] = s->f_i[fch];
    }
}

static void fn(set_output_levels)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *output_levels = s->output_levels;

    for (int ch = 0; ch < s->nb_out_channels; ch++) {
        const int fch = FFMIN(ch, s->nb_f_o-1);

        output_levels[ch] = s->f_o[fch];
    }
}

static void fn(set_smooth_levels)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *smooth_levels = s->smooth_levels;
    const int rdft_size = s->rdft_size;
    const ftype fs = ctx->inputs[0]->sample_rate;
    const ftype scale = F(0.5) * fs / rdft_size;

    for (int ch = 0; ch < s->nb_out_channels; ch++) {
        const ftype S = s->smooth[FFMIN(ch, s->nb_smooth-1)];
        ftype *smooth = smooth_levels + ch * rdft_size;

        for (int n = 0; n < rdft_size; n++) {
            const ftype rk = S / ((n+F(1.0)) * scale);

            smooth[n] = F(1.0) - FEXP(F(-1.0)/(rk*fs));
        }
    }
}

static ftype fn(get_angle)(const ctype cor)
{
    return ATAN2(cor.im, cor.re);
}

static void fn(stereo_position)(const ftype l, const ftype r,
                                const ctype cor,
                                ftype *x, ftype *y, ftype *z)
{
    ftype x0 = (r-l)/(r+l+EPSILON);
    ftype a0 = fn(get_angle)(cor);
    ftype y0 = F(1.0)-FABS(a0 * M_2PI);
    ftype z0 = COPYSIGN(F(1.0)-F(2.0)*FABS(FABS(y0)-F(0.5)), a0);

    x0 = isnormal(x0) ? x0 : F(0.0);
    y0 = isnormal(y0) ? y0 : F(0.0);
    z0 = isnormal(z0) ? z0 : F(0.0);

    x[0] = CLIP(x0, F(-1.0), F(1.0));
    y[0] = CLIP(y0, F(-1.0), F(1.0));
    z[0] = CLIP(z0, F(-1.0), F(1.0));
}

static inline void fn(get_lfe)(int output_lfe, int n, ftype lowcut, ftype highcut,
                               ctype *lfe, ctype sum, ctype *osum, int lfe_mode)
{
    if (output_lfe && n < highcut) {
        const ftype f = F(0.5) * (n < lowcut ? F(1.0) : F(0.5)*(F(1.0)+COS(MPI*(lowcut-n)/(lowcut-highcut))));

        lfe->re    = f;
        lfe->im    = f;
        lfe->re   *= sum.re;
        lfe->im   *= sum.im;
        if (lfe_mode) {
            osum->re -= lfe->re;
            osum->im -= lfe->im;
        }
    } else {
        lfe->re = F(0.0);
        lfe->im = F(0.0);
    }
}

static void fn(filter_stereo)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    const ctype *srcl = (const ctype *)s->input->extended_data[0];
    const ctype *srcr = (const ctype *)s->input->extended_data[1];
    const int output_lfe = s->output_lfe && s->create_lfe;
    const int rdft_size = s->rdft_size;
    const int lfe_mode = s->lfe_mode;
    const ftype highcut = s->highcut;
    const ftype lowcut = s->lowcut;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;
    ctype *osum = s->sum;
    ctype *odif = s->dif;
    ctype *olfe = s->lfe;

    for (int n = 0; n < rdft_size; n++) {
        ftype l_re = srcl[n].re, r_re = srcr[n].re;
        ftype l_im = srcl[n].im, r_im = srcr[n].im;
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ctype sum, dif, lfe, cor;
        ftype x, y, z;

        cor.re = l_re * r_re + l_im * r_im;
        cor.im = r_re * l_im - r_im * l_re;

        sum.re = (l_re + r_re) * F(0.5);
        sum.im = (l_im + r_im) * F(0.5);
        dif.re = (l_re - r_re) * F(0.5);
        dif.im = (l_im - r_im) * F(0.5);

        fn(stereo_position)(l_mag, r_mag, cor, &x, &y, &z);
        fn(get_lfe)(output_lfe, n, lowcut, highcut, &lfe, sum, &sum, lfe_mode);

        xpos[n] = x;
        ypos[n] = y;
        zpos[n] = z;
        osum[n] = sum;
        odif[n] = dif;
        olfe[n] = lfe;
    }
}

static void fn(filter_2_1)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    const ctype *srcl = (const ctype *)s->input->extended_data[0];
    const ctype *srcr = (const ctype *)s->input->extended_data[1];
    const ctype *srclfe = (const ctype *)s->input->extended_data[2];
    const int rdft_size = s->rdft_size;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;
    ctype *osum = s->sum;
    ctype *odif = s->dif;
    ctype *olfe = s->lfe;

    for (int n = 0; n < rdft_size; n++) {
        ftype l_re = srcl[n].re, r_re = srcr[n].re;
        ftype l_im = srcl[n].im, r_im = srcr[n].im;
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ctype sum, dif, cor;
        ftype x, y, z;

        cor.re = l_re * r_re + l_im * r_im;
        cor.im = r_re * l_im - r_im * l_re;

        sum.re = (l_re + r_re) * F(0.5);
        sum.im = (l_im + r_im) * F(0.5);
        dif.re = (l_re - r_re) * F(0.5);
        dif.im = (l_im - r_im) * F(0.5);

        fn(stereo_position)(l_mag, r_mag, cor, &x, &y, &z);

        xpos[n] = x;
        ypos[n] = y;
        zpos[n] = z;
        osum[n] = sum;
        odif[n] = dif;
        olfe[n] = srclfe[n];
    }
}

static void fn(filter_surround)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    const ctype *srcl = (const ctype *)s->input->extended_data[0];
    const ctype *srcr = (const ctype *)s->input->extended_data[1];
    const ctype *srcc = (const ctype *)s->input->extended_data[2];
    const int output_lfe = s->output_lfe && s->create_lfe;
    const int rdft_size = s->rdft_size;
    const int lfe_mode = s->lfe_mode;
    const ftype highcut = s->highcut;
    const ftype lowcut = s->lowcut;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;
    ctype *osum = s->sum;
    ctype *odif = s->dif;
    ctype *ocnt = s->cnt;
    ctype *olfe = s->lfe;

    for (int n = 0; n < rdft_size; n++) {
        ftype l_re = srcl[n].re, r_re = srcr[n].re;
        ftype l_im = srcl[n].im, r_im = srcr[n].im;
        ftype c_re = srcc[n].re, c_im = srcc[n].im;
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ctype sum, dif, cnt, lfe, cor;
        ftype x, y, z;

        cor.re = l_re * r_re + l_im * r_im;
        cor.im = r_re * l_im - r_im * l_re;

        sum.re = (l_re + r_re) * F(0.5);
        sum.im = (l_im + r_im) * F(0.5);
        dif.re = (l_re - r_re) * F(0.5);
        dif.im = (l_im - r_im) * F(0.5);

        cnt.re = c_re;
        cnt.im = c_im;

        fn(stereo_position)(l_mag, r_mag, cor, &x, &y, &z);
        fn(get_lfe)(output_lfe, n, lowcut, highcut, &lfe, cnt, &sum, lfe_mode);

        xpos[n] = x;
        ypos[n] = y;
        zpos[n] = z;
        osum[n] = sum;
        odif[n] = dif;
        ocnt[n] = cnt;
        olfe[n] = lfe;
    }
}

static void fn(filter_3_1)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    const ctype *srcl = (const ctype *)s->input->extended_data[0];
    const ctype *srcr = (const ctype *)s->input->extended_data[1];
    const ctype *srcc = (const ctype *)s->input->extended_data[2];
    const ctype *srclfe = (const ctype *)s->input->extended_data[3];
    const int rdft_size = s->rdft_size;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;
    ctype *osum = s->sum;
    ctype *odif = s->dif;
    ctype *ocnt = s->cnt;
    ctype *olfe = s->lfe;

    for (int n = 0; n < rdft_size; n++) {
        ftype l_re = srcl[n].re, r_re = srcr[n].re;
        ftype l_im = srcl[n].im, r_im = srcr[n].im;
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ctype sum, dif, cor;
        ftype x, y, z;

        cor.re = l_re * r_re + l_im * r_im;
        cor.im = r_re * l_im - r_im * l_re;

        sum.re = (l_re + r_re) * F(0.5);
        sum.im = (l_im + r_im) * F(0.5);
        dif.re = (l_re - r_re) * F(0.5);
        dif.im = (l_im - r_im) * F(0.5);

        sum.re = isnormal(sum.re) ? sum.re : F(0.0);
        sum.im = isnormal(sum.im) ? sum.im : F(0.0);
        dif.re = isnormal(dif.re) ? dif.re : F(0.0);
        dif.im = isnormal(dif.im) ? dif.im : F(0.0);

        fn(stereo_position)(l_mag, r_mag, cor, &x, &y, &z);

        xpos[n] = x;
        ypos[n] = y;
        zpos[n] = z;
        osum[n] = sum;
        odif[n] = dif;
        ocnt[n] = srcc[n];
        olfe[n] = srclfe[n];
    }
}

static int fn(config_output)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];

    s->irdft = av_calloc(outlink->ch_layout.nb_channels, sizeof(*s->irdft));
    if (!s->irdft)
        return AVERROR(ENOMEM);
    s->nb_out_channels = outlink->ch_layout.nb_channels;

    for (int ch = 0; ch < outlink->ch_layout.nb_channels; ch++) {
        ftype iscale = F(1.0) / s->win_size;
        int ret;

        ret = av_tx_init(&s->irdft[ch], &s->itx_fn, TX_TYPE,
                         1, s->win_size, &iscale, 0);
        if (ret < 0)
            return ret;
    }

    s->smooth_levels = av_malloc_array(s->nb_out_channels, sizeof(ftype) * s->rdft_size);
    s->output_levels = av_malloc_array(s->nb_out_channels, sizeof(ftype));
    if (!s->output_levels || !s->smooth_levels)
        return AVERROR(ENOMEM);

    fn(set_output_levels)(ctx);
    fn(set_smooth_levels)(ctx);

    s->factors = ff_get_audio_buffer(outlink, s->rdft_size);
    s->sfactors = ff_get_audio_buffer(outlink, s->rdft_size);
    s->output_sum = ff_get_audio_buffer(outlink, s->rdft_size * 2);
    s->output_dif = ff_get_audio_buffer(outlink, s->rdft_size * 2);
    s->output_out = ff_get_audio_buffer(outlink, s->win_size + 1);
    s->output = ff_get_audio_buffer(outlink, s->rdft_size * 2);
    s->overlap_buffer = ff_get_audio_buffer(outlink, s->win_size * 2);
    s->x_out = ff_get_audio_buffer(outlink, s->rdft_size);
    s->y_out = ff_get_audio_buffer(outlink, s->rdft_size);
    s->z_out = ff_get_audio_buffer(outlink, s->rdft_size);
    if (!s->overlap_buffer || !s->factors || !s->sfactors ||
        !s->output_sum || !s->output_dif ||
        !s->output || !s->output_out ||
        !s->x_out || !s->y_out || !s->z_out)
        return AVERROR(ENOMEM);

    s->x_pos = av_calloc(s->rdft_size, sizeof(ftype));
    s->y_pos = av_calloc(s->rdft_size, sizeof(ftype));
    s->z_pos = av_calloc(s->rdft_size, sizeof(ftype));
    s->sum = av_calloc(s->rdft_size, sizeof(ctype));
    s->dif = av_calloc(s->rdft_size, sizeof(ctype));
    s->lfe = av_calloc(s->rdft_size, sizeof(ctype));
    s->cnt = av_calloc(s->rdft_size, sizeof(ctype));
    if (!s->x_pos || !s->y_pos || !s->z_pos || !s->sum || !s->dif ||
        !s->lfe || !s->cnt)
        return AVERROR(ENOMEM);

    return 0;
}

static int fn(bypass_channel)(AVFilterContext *ctx, AVFrame *out, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);
    const int idx = av_channel_layout_index_from_channel(&s->in_ch_layout, chan);
    const ftype *window_func_lut = s->window_func_lut;
    const ftype *output_levels = s->output_levels;
    const ftype level_out = output_levels[ch] * s->win_gain;
    const int win_size = s->win_size;
    ftype *dst, *ptr;


    dst = (ftype *)s->output_out->extended_data[ch];
    ptr = (ftype *)s->overlap_buffer->extended_data[ch];
    if (idx < 0)
        memset(dst, 0, win_size * sizeof(ftype));
    else
        s->itx_fn(s->irdft[ch], dst, (ftype *)s->input->extended_data[ch], sizeof(ctype));

    memmove(ptr, ptr + s->hop_size, win_size * sizeof(ftype));

    for (int n = 0; n < win_size; n++)
        ptr[n] = FMA(dst[n], window_func_lut[n] * level_out, ptr[n]);

    dst = (ftype *)out->extended_data[ch];
    memcpy(dst, ptr, s->hop_size * sizeof(ftype));

    return 0;
}

static int fn(ifft_channel)(AVFilterContext *ctx, AVFrame *out, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const ftype *window_func_lut = s->window_func_lut;
    const ftype *output_levels = s->output_levels;
    const ftype level_out = output_levels[ch] * s->win_gain;
    const int win_size = s->win_size;
    ftype *dst, *ptr;

    if (ff_filter_disabled(ctx))
        return fn(bypass_channel)(ctx, out, ch);

    dst = (ftype *)s->output_out->extended_data[ch];
    ptr = (ftype *)s->overlap_buffer->extended_data[ch];
    s->itx_fn(s->irdft[ch], dst, (ftype *)s->output->extended_data[ch], sizeof(ctype));

    memmove(ptr, ptr + s->hop_size, win_size * sizeof(ftype));

    for (int n = 0; n < win_size; n++)
        ptr[n] = FMA(dst[n], window_func_lut[n] * level_out, ptr[n]);

    dst = (ftype *)out->extended_data[ch];
    memcpy(dst, ptr, s->hop_size * sizeof(ftype));

    return 0;
}

static int fn(fft_channel)(AVFilterContext *ctx, AVFrame *in, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *src = (ftype *)s->input_in->extended_data[ch];
    ftype *win = (ftype *)s->window->extended_data[ch];
    const ftype *window_func_lut = s->window_func_lut;
    const int offset = s->input_in->nb_samples - s->hop_size;
    const ftype *input_levels = s->input_levels;
    const ftype level_in = input_levels[ch];
    const int win_size = s->win_size;

    memmove(src, &src[s->hop_size], offset * sizeof(ftype));
    if (in) {
        memcpy(&src[offset], in->extended_data[ch], in->nb_samples * sizeof(ftype));
        memset(&src[offset + in->nb_samples], 0, (s->hop_size - in->nb_samples) * sizeof(ftype));
    } else {
        memset(&src[offset], 0, s->hop_size * sizeof(ftype));
    }

    for (int n = 0; n < win_size; n++)
        win[n] = src[n] * window_func_lut[n] * level_in;

    s->tx_fn(s->rdft[ch], (ftype *)s->input->extended_data[ch], win, sizeof(ftype));

    return 0;
}

static void fn(angle_transform)(ftype *x, ftype *y, ftype a)
{
    if (a == F(90.0))
        return;

    a /= F(90.0);
    a -= F(1.0);

    y[0] = CLIP(y[0] + FABS(x[0]) * (y[0]+F(1.0)) * a, F(-1.0), F(1.0));
}

static void fn(shift_transform)(ftype *y, const ftype shift)
{
    if (shift == F(0.0))
        return;

    y[0] = CLIP(y[0] + shift, F(-1.0), F(1.0));
}

static void fn(depth_transform)(ftype *y, const ftype depth)
{
    if (depth == F(0.0))
        return;

    if (depth < F(0.0) && y[0] > F(0.0))
        return;

    if (depth > F(0.0) && y[0] < F(0.0))
        return;

    y[0] = CLIP(FMA(y[0], depth, y[0]), F(-1.0), F(1.0));
}

static void fn(focus_transform)(ftype *x, ftype focus)
{
    if (focus == F(0.0))
        return;

    if (focus > F(0.0))
        focus = F(1.0) / (F(1.0) +  focus * F(10.0));
    if (focus < F(0.0))
        focus = F(1.0) * (F(1.0) + -focus * F(10.0));

    x[0] = CLIP(COPYSIGN(POW(FABS(x[0]), focus), x[0]), F(-1.0), F(1.0));
}

static void fn(power_factors)(AVFilterContext *ctx, const int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const ftype f_x = -s->f_x[FFMIN(ch, s->nb_f_x-1)];
    const ftype f_y = -s->f_y[FFMIN(ch, s->nb_f_y-1)];
    const ftype f_z = -s->f_z[FFMIN(ch, s->nb_f_z-1)];
    const ftype *xin = (const ftype *)s->x_out->extended_data[ch];
    const ftype *yin = (const ftype *)s->y_out->extended_data[ch];
    const ftype *zin = (const ftype *)s->z_out->extended_data[ch];
    ftype *factor = (ftype *)s->factors->extended_data[ch];
    const ftype num_x = F(1.0) + FEXP(f_x * F(0.5));
    const ftype num_y = F(1.0) + FEXP(f_y * F(0.5));
    const ftype num_z = F(1.0) + FEXP(f_z * F(0.5));
    const ftype num = num_x * num_y * num_z;
    const int rdft_size = s->rdft_size;

    for (int n = 0; n < rdft_size; n++) {
        ftype x = xin[n];
        ftype y = yin[n];
        ftype z = zin[n];

        x = F(1.0) + FEXP(f_x * (x - F(0.5)));
        y = F(1.0) + FEXP(f_y * (y - F(0.5)));
        z = F(1.0) + FEXP(f_z * (z - F(0.5)));

        factor[n] = num / (x*y*z);
        factor[n] = isnormal(factor[n]) ? factor[n] : F(0.0);
    }
}

static void fn(calculate_factors)(AVFilterContext *ctx, int ch, int chan)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *x_out = (ftype *)s->x_out->extended_data[ch];
    ftype *y_out = (ftype *)s->y_out->extended_data[ch];
    ftype *z_out = (ftype *)s->z_out->extended_data[ch];
    const int rdft_size = s->rdft_size;
    const ftype *x = s->x_pos;
    const ftype *y = s->y_pos;
    const ftype *z = s->z_pos;

    if (chan == AV_CHAN_NONE)
        return;

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
    case AV_CHAN_BOTTOM_FRONT_CENTER:
        for (int n = 0; n < rdft_size; n++)
            x_out[n] = F(1.0) - FMIN(FABS(x[n]*F(2.0)), F(1.0));
        break;
    case AV_CHAN_BOTTOM_FRONT_LEFT:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_TOP_SIDE_LEFT:
    case AV_CHAN_BACK_LEFT:
        for (int n = 0; n < rdft_size; n++)
            x_out[n] = FMAX(-x[n], F(0.0));
        break;
    case AV_CHAN_BOTTOM_FRONT_RIGHT:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_BACK_RIGHT:
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_TOP_SIDE_RIGHT:
    case AV_CHAN_BACK_RIGHT:
        for (int n = 0; n < rdft_size; n++)
            x_out[n] = FMAX(x[n], F(0.0));
        break;
    case AV_CHAN_FRONT_LEFT_OF_CENTER:
        for (int n = 0; n < rdft_size; n++)
            x_out[n] = F(1.0) - FMIN(FABS(x[n]+F(0.5)), F(1.0));
        break;
    case AV_CHAN_FRONT_RIGHT_OF_CENTER:
        for (int n = 0; n < rdft_size; n++)
            x_out[n] = F(1.0) - FMIN(FABS(x[n]-F(0.5)), F(1.0));
        break;
    default:
        for (int n = 0; n < rdft_size; n++)
            x_out[n] = x[n];
        break;
    }

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_BOTTOM_FRONT_CENTER:
    case AV_CHAN_BOTTOM_FRONT_LEFT:
    case AV_CHAN_BOTTOM_FRONT_RIGHT:
    case AV_CHAN_FRONT_LEFT_OF_CENTER:
    case AV_CHAN_FRONT_RIGHT_OF_CENTER:
        for (int n = 0; n < rdft_size; n++)
            y_out[n] = FMAX(y[n], F(0.0));
        break;
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_TOP_SIDE_LEFT:
    case AV_CHAN_TOP_SIDE_RIGHT:
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        for (int n = 0; n < rdft_size; n++)
            y_out[n] = F(1.0) - FMIN(FABS(y[n]*F(2.0)), F(1.0));
        break;
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_BACK_RIGHT:
    case AV_CHAN_BACK_LEFT:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_TOP_BACK_RIGHT:
        for (int n = 0; n < rdft_size; n++)
            y_out[n] = FMAX(-y[n], F(0.0));
        break;
    default:
        for (int n = 0; n < rdft_size; n++)
            y_out[n] = y[n];
        break;
    }

    switch (chan) {
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_TOP_BACK_RIGHT:
    case AV_CHAN_TOP_SIDE_LEFT:
    case AV_CHAN_TOP_SIDE_RIGHT:
        for (int n = 0; n < rdft_size; n++)
            z_out[n] = FMAX(z[n], F(0.0));
        break;
    case AV_CHAN_BOTTOM_FRONT_LEFT:
    case AV_CHAN_BOTTOM_FRONT_CENTER:
    case AV_CHAN_BOTTOM_FRONT_RIGHT:
        for (int n = 0; n < rdft_size; n++)
            z_out[n] = FMAX(-z[n], F(0.0));
        break;
    default:
        for (int n = 0; n < rdft_size; n++)
            z_out[n] = F(1.0) - FABS(z[n]);
        break;
    }

    fn(power_factors)(ctx, ch);
}

static void fn(bypass_transform)(AVFilterContext *ctx, int ch, int is_lfe)
{
    AudioSurroundContext *s = ctx->priv;
    const ctype *cnt = s->cnt;
    const ctype *lfe = s->lfe;
    const ctype *src = is_lfe ? lfe : cnt;
    ctype *dst = (ctype *)s->output->extended_data[ch];
    const int rdft_size = s->rdft_size;

    memcpy(dst, src, rdft_size * sizeof(*dst));
}

static void fn(do_transform)(AVFilterContext *ctx, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);
    const ftype *smooth_levels = s->smooth_levels;
    ftype *sfactor = (ftype *)s->sfactors->extended_data[ch];
    ftype *factor = (ftype *)s->factors->extended_data[ch];
    const ctype *odif = (const ctype *)s->output_dif->extended_data[ch];
    const ctype *osum = (const ctype *)s->output_sum->extended_data[ch];
    ctype *dst = (ctype *)s->output->extended_data[ch];
    const int rdft_size = s->rdft_size;

    if (chan == AV_CHAN_LOW_FREQUENCY ||
        chan == AV_CHAN_LOW_FREQUENCY_2) {
        memcpy(dst, osum, rdft_size * sizeof(*dst));
        return;
    }

    if (s->smooth_init) {
        const ftype *smooth = smooth_levels + ch * rdft_size;

        for (int n = 0; n < rdft_size; n++) {
            sfactor[n] = FMA(factor[n] - sfactor[n], smooth[n], sfactor[n]);
            sfactor[n] = isnormal(sfactor[n]) ? sfactor[n] : F(0.0);
        }
    } else {
        memcpy(sfactor, factor, rdft_size * sizeof(*sfactor));
    }
    factor = sfactor;

    for (int n = 0; n < rdft_size; n++) {
        const ctype dif = odif[n];
        const ctype sum = osum[n];
        const ftype a = factor[n];
        ctype out;

        out.re = a * (sum.re + dif.re);
        out.im = a * (sum.im + dif.im);

        out.re = isnormal(out.re) ? out.re : F(0.0);
        out.im = isnormal(out.im) ? out.im : F(0.0);

        dst[n] = out;
    }
}

static int fn(transform_xy)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSurroundContext *s = ctx->priv;
    const int rdft_size = s->rdft_size;
    const int start = (rdft_size * jobnr) / nb_jobs;
    const int end = (rdft_size * (jobnr+1)) / nb_jobs;
    const ftype angle = s->angle;
    const ftype focus_x = s->focus[0];
    const ftype focus_y = s->focus[FFMIN(1, s->nb_focus-1)];
    const ftype focus_z = s->focus[FFMIN(2, s->nb_focus-1)];
    const ftype shift_x = s->shift[0];
    const ftype shift_y = s->shift[FFMIN(1, s->nb_shift-1)];
    const ftype shift_z = s->shift[FFMIN(2, s->nb_shift-1)];
    const ftype depth_x = s->depth[0];
    const ftype depth_y = s->depth[FFMIN(1, s->nb_depth-1)];
    const ftype depth_z = s->depth[FFMIN(2, s->nb_depth-1)];
    ftype *x = s->x_pos;
    ftype *y = s->y_pos;
    ftype *z = s->z_pos;

    for (int n = start; n < end; n++) {
        fn(angle_transform)(&x[n], &y[n], angle);

        fn(shift_transform)(&x[n], shift_x);
        fn(shift_transform)(&y[n], shift_y);
        fn(shift_transform)(&z[n], shift_z);

        fn(depth_transform)(&x[n], depth_x);
        fn(depth_transform)(&y[n], depth_y);
        fn(depth_transform)(&z[n], depth_z);

        fn(focus_transform)(&x[n], focus_x);
        fn(focus_transform)(&y[n], focus_y);
        fn(focus_transform)(&z[n], focus_z);
    }

    return 0;
}

static void fn(stereo_copy)(AVFilterContext *ctx, int ch, int chan)
{
    AudioSurroundContext *s = ctx->priv;
    ctype *odif = (ctype *)s->output_dif->extended_data[ch];
    ctype *osum = (ctype *)s->output_sum->extended_data[ch];
    const ftype dif_factor = ch_dif[sc_map[chan]];
    const int rdft_size = s->rdft_size;
    const ctype *sum = s->sum;
    const ctype *dif = s->dif;

    if (chan == AV_CHAN_LOW_FREQUENCY ||
        chan == AV_CHAN_LOW_FREQUENCY_2) {
        memcpy(osum, s->lfe, rdft_size * sizeof(*osum));
        return;
    }

    memcpy(osum, sum, rdft_size * sizeof(*osum));
    for (int n = 0; n < rdft_size; n++) {
        odif[n].re = dif[n].re * dif_factor;
        odif[n].im = dif[n].im * dif_factor;
    }
}

static int fn(config_input)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    const ftype *window_func_lut;
    ftype overlap;

    s->win_size = 1 << av_ceil_log2((inlink->sample_rate + 19) / 20);
    s->rdft_size = s->win_size / 2 + 1;

    s->window_func_lut = av_calloc(s->win_size, sizeof(ftype));
    if (!s->window_func_lut)
        return AVERROR(ENOMEM);
    window_func_lut = s->window_func_lut;

    fun(generate_window_func)(s->window_func_lut, s->win_size, s->win_func, &overlap);
    if (s->overlap == F(1.0))
        s->overlap = overlap;

    s->hop_size = FFMAX(1, LRINT(s->win_size * (F(1.0) - s->overlap)));
    s->trim_size = s->win_size - s->hop_size;
    s->flush_size = s->win_size - s->hop_size;

    {
        ftype max = 0.f, *temp_lut = av_calloc(s->win_size, sizeof(*temp_lut));
        if (!temp_lut)
            return AVERROR(ENOMEM);

        for (int j = 0; j < s->win_size; j += s->hop_size) {
            for (int i = 0; i < s->win_size; i++)
                temp_lut[(i + j) % s->win_size] += window_func_lut[i];
        }

        for (int i = 0; i < s->win_size; i++)
            max = FMAX(temp_lut[i], max);
        av_freep(&temp_lut);

        s->win_gain = F(1.0) / max;
    }

    s->set_input_levels = fn(set_input_levels);
    s->set_output_levels = fn(set_output_levels);
    s->set_smooth_levels = fn(set_smooth_levels);
    s->ifft_channel = fn(ifft_channel);
    s->fft_channel = fn(fft_channel);
    s->calculate_factors = fn(calculate_factors);
    s->stereo_copy = fn(stereo_copy);
    s->do_transform = fn(do_transform);
    s->bypass_transform = fn(bypass_transform);
    s->transform_xy = fn(transform_xy);

    switch (s->in_ch_layout.u.mask) {
    case AV_CH_LAYOUT_STEREO:
        s->filter = fn(filter_stereo);
        s->upmix = stereo_upmix;
        break;
    case AV_CH_LAYOUT_2POINT1:
        s->filter = fn(filter_2_1);
        s->upmix = l2_1_upmix;
        break;
    case AV_CH_LAYOUT_SURROUND:
        s->filter = fn(filter_surround);
        s->upmix = surround_upmix;
        break;
    case AV_CH_LAYOUT_3POINT1:
        s->filter = fn(filter_3_1);
        s->upmix = l3_1_upmix;
        break;
    default:
        return AVERROR(EINVAL);
    }

    s->rdft = av_calloc(inlink->ch_layout.nb_channels, sizeof(*s->rdft));
    if (!s->rdft)
        return AVERROR(ENOMEM);
    s->nb_in_channels = inlink->ch_layout.nb_channels;

    for (int ch = 0; ch < inlink->ch_layout.nb_channels; ch++) {
        ftype scale = F(1.0);
        int ret;

        ret = av_tx_init(&s->rdft[ch], &s->tx_fn, TX_TYPE,
                         0, s->win_size, &scale, 0);
        if (ret < 0)
            return ret;
    }

    s->input_levels = av_malloc_array(s->nb_in_channels, sizeof(ftype));
    s->input_in = ff_get_audio_buffer(inlink, s->win_size);
    s->window = ff_get_audio_buffer(inlink, s->win_size);
    s->input = ff_get_audio_buffer(inlink, s->win_size + 2);
    if (!s->input_levels || !s->input_in || !s->window || !s->input)
        return AVERROR(ENOMEM);

    fn(set_input_levels)(ctx);

    s->lowcut = 1.f * s->lowcutf / (inlink->sample_rate * 0.5) * s->win_size;
    s->highcut = 1.f * s->highcutf / (inlink->sample_rate * 0.5) * s->win_size;

    return 0;
}
