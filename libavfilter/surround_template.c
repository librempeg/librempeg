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
#undef ATAN2
#undef FABS
#undef FEXP10
#undef FMA
#undef COS
#undef POW
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
#define ATAN2 atan2f
#define FABS fabsf
#define FEXP10 ff_exp10f
#define FMA fmaf
#define COS cosf
#define POW powf
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
#define ATAN2 atan2
#define FABS fabs
#define FEXP10 ff_exp10
#define FMA fma
#define COS cos
#define POW pow
#define SQRT sqrt
#define FMIN fmin
#define FMAX fmax
#define LRINT lrint
#define EPSILON DBL_EPSILON
#define CLIP av_clipd
#define SAMPLE_FORMAT dblp
#define TX_TYPE AV_TX_DOUBLE_RDFT
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define F(x) ((ftype)(x))
#define HYPOT(x, y) SQRT(((x)*(x))+((y)*(y)))

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

static ftype fn(get_angle)(const ctype cor)
{
    return ATAN2(cor.im, cor.re);
}

static void fn(stereo_position)(const ftype l, const ftype r,
                                const ctype cor,
                                ftype *x, ftype *y, ftype *z)
{
    if (r <= EPSILON && l <= EPSILON) {
        x[0] = F(0.0);
        y[0] = F(0.0);
        z[0] = F(0.0);
    } else {
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

static int fn(filter_stereo)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSurroundContext *s = ctx->priv;
    const int rdft_size = s->rdft_size;
    const int start = (rdft_size * jobnr) / nb_jobs;
    const int end = (rdft_size * (jobnr+1)) / nb_jobs;
    const ctype *srcl = (const ctype *)s->input->extended_data[0];
    const ctype *srcr = (const ctype *)s->input->extended_data[1];
    const int output_lfe = s->output_lfe && s->create_lfe;
    const int lfe_mode = s->lfe_mode;
    const ftype highcut = s->highcut;
    const ftype lowcut = s->lowcut;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;
    ctype *olfe = s->lfe;

    for (int n = start; n < end; n++) {
        ftype l_re = srcl[n].re, r_re = srcr[n].re;
        ftype l_im = srcl[n].im, r_im = srcr[n].im;
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ctype sum, lfe, cor;
        ftype x, y, z;

        cor.re = l_re * r_re + l_im * r_im;
        cor.im = r_re * l_im - r_im * l_re;

        sum.re = (l_re + r_re) * F(0.5);
        sum.im = (l_im + r_im) * F(0.5);

        sum.re = isnormal(sum.re) ? sum.re : F(0.0);
        sum.im = isnormal(sum.im) ? sum.im : F(0.0);

        fn(stereo_position)(l_mag, r_mag, cor, &x, &y, &z);
        fn(get_lfe)(output_lfe, n, lowcut, highcut, &lfe, sum, &sum, lfe_mode);

        xpos[n] = x;
        ypos[n] = y;
        zpos[n] = z;
        olfe[n] = lfe;
    }

    return 0;
}

static int fn(filter_2_1)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSurroundContext *s = ctx->priv;
    const int rdft_size = s->rdft_size;
    const int start = (rdft_size * jobnr) / nb_jobs;
    const int end = (rdft_size * (jobnr+1)) / nb_jobs;
    const ctype *srcl = (const ctype *)s->input->extended_data[0];
    const ctype *srcr = (const ctype *)s->input->extended_data[1];
    const ctype *srclfe = (const ctype *)s->input->extended_data[2];
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;
    ctype *olfe = s->lfe;

    for (int n = start; n < end; n++) {
        ftype l_re = srcl[n].re, r_re = srcr[n].re;
        ftype l_im = srcl[n].im, r_im = srcr[n].im;
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ftype x, y, z;
        ctype cor;

        cor.re = l_re * r_re + l_im * r_im;
        cor.im = r_re * l_im - r_im * l_re;

        fn(stereo_position)(l_mag, r_mag, cor, &x, &y, &z);

        xpos[n] = x;
        ypos[n] = y;
        zpos[n] = z;
        olfe[n] = srclfe[n];
    }

    return 0;
}

static int fn(filter_surround)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSurroundContext *s = ctx->priv;
    const int rdft_size = s->rdft_size;
    const int start = (rdft_size * jobnr) / nb_jobs;
    const int end = (rdft_size * (jobnr+1)) / nb_jobs;
    const ctype *srcl = (const ctype *)s->input->extended_data[0];
    const ctype *srcr = (const ctype *)s->input->extended_data[1];
    const ctype *srcc = (const ctype *)s->input->extended_data[2];
    const int output_lfe = s->output_lfe && s->create_lfe;
    const int lfe_mode = s->lfe_mode;
    const ftype highcut = s->highcut;
    const ftype lowcut = s->lowcut;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;
    ctype *ocnt = s->cnt;
    ctype *olfe = s->lfe;

    for (int n = start; n < end; n++) {
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

        sum.re = isnormal(sum.re) ? sum.re : F(0.0);
        sum.im = isnormal(sum.im) ? sum.im : F(0.0);
        dif.re = isnormal(dif.re) ? dif.re : F(0.0);
        dif.im = isnormal(dif.im) ? dif.im : F(0.0);

        cnt.re = c_re;
        cnt.im = c_im;

        fn(stereo_position)(l_mag, r_mag, cor, &x, &y, &z);
        fn(get_lfe)(output_lfe, n, lowcut, highcut, &lfe, cnt, &sum, lfe_mode);

        xpos[n] = x;
        ypos[n] = y;
        zpos[n] = z;
        ocnt[n] = cnt;
        olfe[n] = lfe;
    }

    return 0;
}

static int fn(filter_3_1)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSurroundContext *s = ctx->priv;
    const int rdft_size = s->rdft_size;
    const int start = (rdft_size * jobnr) / nb_jobs;
    const int end = (rdft_size * (jobnr+1)) / nb_jobs;
    const ctype *srcl = (const ctype *)s->input->extended_data[0];
    const ctype *srcr = (const ctype *)s->input->extended_data[1];
    const ctype *srcc = (const ctype *)s->input->extended_data[2];
    const ctype *srclfe = (const ctype *)s->input->extended_data[3];
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;
    ctype *ocnt = s->cnt;
    ctype *olfe = s->lfe;

    for (int n = start; n < end; n++) {
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
        ocnt[n] = srcc[n];
        olfe[n] = srclfe[n];
    }

    return 0;
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

    s->output_levels = av_malloc_array(s->nb_out_channels, sizeof(ftype));
    if (!s->output_levels)
        return AVERROR(ENOMEM);

    fn(set_output_levels)(ctx);

    s->output_out = ff_get_audio_buffer(outlink, s->win_size + 1);
    s->output = ff_get_audio_buffer(outlink, s->rdft_size * 2);
    s->overlap_buffer = ff_get_audio_buffer(outlink, s->win_size * 2);
    if (!s->overlap_buffer || !s->output || !s->output_out)
        return AVERROR(ENOMEM);

    s->x_pos = av_calloc(s->rdft_size, sizeof(ftype));
    s->y_pos = av_calloc(s->rdft_size, sizeof(ftype));
    s->z_pos = av_calloc(s->rdft_size, sizeof(ftype));
    s->lfe = av_calloc(s->rdft_size, sizeof(ctype));
    s->cnt = av_calloc(s->rdft_size, sizeof(ctype));
    if (!s->x_pos || !s->y_pos || !s->z_pos ||
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

static void fn(angle_transform)(ftype *x, ftype *y, ftype a, const int start, const int end)
{
    if (a == F(90.0))
        return;

    a /= F(90.0);
    a -= F(1.0);

    for (int n = start; n < end; n++)
        y[n] = CLIP(y[n] - FABS(x[n]) * (y[n]+F(1.0)) * a, F(-1.0), F(1.0));
}

static void fn(shift_transform)(ftype *y, const ftype shift, const int start, const int end)
{
    if (shift == F(0.0))
        return;

    for (int n = start; n < end; n++)
        y[n] = CLIP(y[n] + shift, F(-1.0), F(1.0));
}

static void fn(depth_transform)(ftype *y, const ftype depth, const int start, const int end)
{
    if (depth == F(0.0))
        return;

    for (int n = start; n < end; n++) {
        if (depth < F(0.0) && y[n] > F(0.0))
            continue;

        if (depth > F(0.0) && y[n] < F(0.0))
            continue;

        y[n] = CLIP(FMA(y[n], depth, y[n]), F(-1.0), F(1.0));
    }
}

static void fn(focus_transform)(ftype *x, ftype focus, const int start, const int end)
{
    if (focus == F(0.0))
        return;

    if (focus > F(0.0))
        focus = F(1.0) / (F(1.0) +  focus * F(10.0));
    if (focus < F(0.0))
        focus = F(1.0) * (F(1.0) + -focus * F(10.0));

    for (int n = start; n < end; n++)
        x[n] = CLIP(COPYSIGN(POW(FABS(x[n]), focus), x[n]), F(-1.0), F(1.0));
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

static ctype fn(transform)(const ctype l, const ctype r,
                           const ftype x, const ftype y,
                           const ftype z,
                           const ftype ch_x, const ftype ch_y,
                           const ftype ch_z,
                           const ftype sign0, const ftype sign1)
{
    const ftype xx = (FABS(ch_z - z) + FABS(ch_x - x) + FABS(ch_y - y)) * F(-0.5);
    const ftype xf = FEXP10(xx);
    ctype ret;

    ret.re = sign0 * xf * l.re + xf * r.re * sign1;
    ret.im = sign0 * xf * l.im + xf * r.im * sign1;

    return ret;
}

static void fn(do_transform)(AVFilterContext *ctx, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const int chan = av_channel_layout_channel_from_index(&s->out_ch_layout, ch);
    const int sc_chan = sc_map[chan];
    const ctype *srcl = (const ctype *)s->input->extended_data[0];
    const ctype *srcr = (const ctype *)s->input->extended_data[1];
    ctype *dst = (ctype *)s->output->extended_data[ch];
    const ftype ch_x = sc_ch_pos[sc_chan][0];
    const ftype ch_y = sc_ch_pos[sc_chan][1];
    const ftype ch_z = sc_ch_pos[sc_chan][2];
    const ftype sign0 = sc_ch_pos[sc_chan][3];
    const ftype sign1 = sc_ch_pos[sc_chan][4];
    const int rdft_size = s->rdft_size;
    const ftype *x = s->x_pos;
    const ftype *y = s->y_pos;
    const ftype *z = s->z_pos;

    for (int n = 0; n < rdft_size; n++)
        dst[n] = fn(transform)(srcl[n], srcr[n], x[n], y[n], z[n], ch_x, ch_y, ch_z, sign0, sign1);
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

    fn(angle_transform)(x, y, angle, start, end);

    fn(shift_transform)(x, shift_x, start, end);
    fn(shift_transform)(y, shift_y, start, end);
    fn(shift_transform)(z, shift_z, start, end);

    fn(depth_transform)(x, depth_x, start, end);
    fn(depth_transform)(y, depth_y, start, end);
    fn(depth_transform)(z, depth_z, start, end);

    fn(focus_transform)(x, focus_x, start, end);
    fn(focus_transform)(y, focus_y, start, end);
    fn(focus_transform)(z, focus_z, start, end);

    return 0;
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
    s->ifft_channel = fn(ifft_channel);
    s->fft_channel = fn(fft_channel);
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
