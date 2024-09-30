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

#undef MPI
#undef MPI2
#undef MPI4
#undef MLN10
#undef MSQRT1_2
#undef ftype
#undef ctype
#undef HYPOT
#undef ATAN2
#undef FABS
#undef SIN
#undef COS
#undef POW
#undef TAN
#undef SQRT
#undef FMIN
#undef FMAX
#undef EPSILON
#undef CLIP
#undef SAMPLE_FORMAT
#undef TX_TYPE

#if DEPTH == 32
#define MPI M_PIf
#define MPI2 M_PI_2f
#define MPI4 M_PI_4f
#define MLN10 M_LN10f
#define MSQRT1_2 M_SQRT1_2f
#define ftype float
#define ctype AVComplexFloat
#define HYPOT hypotf
#define ATAN2 atan2f
#define FABS fabsf
#define SIN sinf
#define COS cosf
#define POW powf
#define TAN tanf
#define SQRT sqrtf
#define FMIN fminf
#define FMAX fmaxf
#define EPSILON FLT_EPSILON
#define CLIP av_clipf
#define SAMPLE_FORMAT fltp
#define TX_TYPE AV_TX_FLOAT_RDFT
#else
#define MPI M_PI
#define MPI2 M_PI_2
#define MPI4 M_PI_4
#define MLN10 M_LN10
#define MSQRT1_2 M_SQRT1_2
#define ftype double
#define ctype AVComplexDouble
#define HYPOT hypot
#define ATAN2 atan2
#define FABS fabs
#define SIN sin
#define COS cos
#define POW pow
#define TAN tan
#define SQRT sqrt
#define FMIN fmin
#define FMAX fmax
#define EPSILON DBL_EPSILON
#define CLIP av_clipd
#define SAMPLE_FORMAT dblp
#define TX_TYPE AV_TX_DOUBLE_RDFT
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define F(x) ((ftype)(x))

static void fn(set_input_levels)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *input_levels = s->input_levels;

    for (int ch = 0;  ch < s->nb_in_channels; ch++) {
        const int fch = FFMIN3(ch, s->nb_f_i-1, SC_NB-1);

        input_levels[ch] = s->f_i[fch];
    }
}

static void fn(set_output_levels)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *output_levels = s->output_levels;

    for (int ch = 0;  ch < s->nb_out_channels; ch++) {
        const int fch = FFMIN3(ch, s->nb_f_o-1, SC_NB-1);

        output_levels[ch] = s->f_o[fch];
    }
}

static void fn(stereo_position)(const ftype l, const ftype r,
                                const ftype im, const ftype re,
                                ftype *x, ftype *y, ftype *z)
{
    const ftype im2 = im*im;
    const ftype re2 = re*re;
    const ftype l2 = l*l;
    const ftype r2 = r*r;
    const ftype h2 = SQRT(l2 + r2 + EPSILON) + EPSILON;
    const ftype h1 = SQRT(im2 + re2 + EPSILON) + EPSILON;
    const ftype h1h2 = h1*h2 + EPSILON;
    const ftype rel = re * l;
    const ftype rer = re * r;
    ftype x0 = MSQRT1_2 * (rer - rel) / h1h2;
    ftype y0 = MSQRT1_2 * (rer + rel) / h1h2;
    ftype z0 = im / h1;

    x0 = CLIP(x0, F(-1.0), F(1.0));
    y0 = CLIP(y0, F(-1.0), F(1.0));
    z0 = CLIP(z0, F(-1.0), F(1.0));

    *x = isnormal(x0) ? x0 : F(0.0);
    *y = isnormal(y0) ? y0 : F(0.0);
    *z = isnormal(z0) ? z0 : F(0.0);
}

static inline void fn(get_lfe)(int output_lfe, int n, ftype lowcut, ftype highcut,
                               ftype *lfe_mag, ftype c_mag, ftype *mag_total, int lfe_mode)
{
    if (output_lfe && n < highcut) {
        *lfe_mag    = n < lowcut ? F(1.0) : F(0.5)*(F(1.0)+COS(MPI*(lowcut-n)/(lowcut-highcut)));
        *lfe_mag   *= c_mag;
        if (lfe_mode)
            *mag_total -= *lfe_mag;
    } else {
        *lfe_mag = F(0.0);
    }
}

static void fn(filter_stereo)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    const ftype *srcl = (const ftype *)s->input->extended_data[0];
    const ftype *srcr = (const ftype *)s->input->extended_data[1];
    const int output_lfe = s->output_lfe && s->create_lfe;
    const int rdft_size = s->rdft_size;
    const int lfe_mode = s->lfe_mode;
    const ftype highcut = s->highcut;
    const ftype lowcut = s->lowcut;
    ftype *magtotal = s->mag_total;
    ftype *lfemag = s->lfe_mag;
    ftype *lphase = s->l_phase;
    ftype *rphase = s->r_phase;
    ftype *cphase = s->c_phase;
    ftype *cmag = s->c_mag;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;

    for (int n = 0; n < rdft_size; n++) {
        ftype l_re = srcl[2 * n], r_re = srcr[2 * n];
        ftype l_im = srcl[2 * n + 1], r_im = srcr[2 * n + 1];
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ftype l_phase = ATAN2(l_im, l_re);
        ftype r_phase = ATAN2(r_im, r_re);
        ftype re = l_re * r_re + l_im * r_im;
        ftype im = r_re * l_im - r_im * l_re;
        ftype mag_sum = l_mag + r_mag;
        ftype mag_total = mag_sum;
        ftype c_phase = (l_phase + r_phase) * F(0.5);
        ftype c_mag = mag_sum * F(0.5);
        ftype x, y, z;

        fn(stereo_position)(l_mag, r_mag, im, re, &x, &y, &z);
        fn(get_lfe)(output_lfe, n, lowcut, highcut, &lfemag[n], c_mag, &mag_total, lfe_mode);

        xpos[n]   = x;
        ypos[n]   = y;
        zpos[n]   = z;
        lphase[n] = l_phase;
        rphase[n] = r_phase;
        cmag[n]   = c_mag;
        cphase[n] = c_phase;
        magtotal[n] = mag_total;
    }
}

static void fn(filter_2_1)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    const ftype *srcl = (const ftype *)s->input->extended_data[0];
    const ftype *srcr = (const ftype *)s->input->extended_data[1];
    const ftype *srclfe = (const ftype *)s->input->extended_data[2];
    const int rdft_size = s->rdft_size;
    ftype *magtotal = s->mag_total;
    ftype *lfephase = s->lfe_phase;
    ftype *lfemag = s->lfe_mag;
    ftype *lphase = s->l_phase;
    ftype *rphase = s->r_phase;
    ftype *cphase = s->c_phase;
    ftype *cmag = s->c_mag;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;

    for (int n = 0; n < rdft_size; n++) {
        ftype l_re = srcl[2 * n], r_re = srcr[2 * n];
        ftype l_im = srcl[2 * n + 1], r_im = srcr[2 * n + 1];
        ftype lfe_re = srclfe[2 * n], lfe_im = srclfe[2 * n + 1];
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ftype lfe_mag = HYPOT(lfe_re, lfe_im);
        ftype lfe_phase = ATAN2(lfe_im, lfe_re);
        ftype l_phase = ATAN2(l_im, l_re);
        ftype r_phase = ATAN2(r_im, r_re);
        ftype re = l_re * r_re + l_im * r_im;
        ftype im = r_re * l_im - r_im * l_re;
        ftype mag_sum = l_mag + r_mag;
        ftype mag_total = mag_sum;
        ftype c_phase = (l_phase + r_phase) * F(0.5);
        ftype c_mag = mag_sum * F(0.5);
        ftype x, y, z;

        fn(stereo_position)(l_mag, r_mag, im, re, &x, &y, &z);

        xpos[n]   = x;
        ypos[n]   = y;
        zpos[n]   = z;
        lphase[n] = l_phase;
        rphase[n] = r_phase;
        cmag[n]   = c_mag;
        cphase[n] = c_phase;
        lfemag[n] = lfe_mag;
        lfephase[n] = lfe_phase;
        magtotal[n] = mag_total;
    }
}

static void fn(filter_surround)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    const ftype *srcl = (const ftype *)s->input->extended_data[0];
    const ftype *srcr = (const ftype *)s->input->extended_data[1];
    const ftype *srcc = (const ftype *)s->input->extended_data[2];
    const int output_lfe = s->output_lfe && s->create_lfe;
    const int rdft_size = s->rdft_size;
    const int lfe_mode = s->lfe_mode;
    const ftype highcut = s->highcut;
    const ftype lowcut = s->lowcut;
    ftype *magtotal = s->mag_total;
    ftype *lfemag = s->lfe_mag;
    ftype *lphase = s->l_phase;
    ftype *rphase = s->r_phase;
    ftype *cphase = s->c_phase;
    ftype *cmag = s->c_mag;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;

    for (int n = 0; n < rdft_size; n++) {
        ftype l_re = srcl[2 * n], r_re = srcr[2 * n];
        ftype l_im = srcl[2 * n + 1], r_im = srcr[2 * n + 1];
        ftype c_re = srcc[2 * n], c_im = srcc[2 * n + 1];
        ftype c_phase = ATAN2(c_im, c_re);
        ftype c_mag = HYPOT(c_re, c_im);
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ftype l_phase = ATAN2(l_im, l_re);
        ftype r_phase = ATAN2(r_im, r_re);
        ftype mag_total = c_mag + l_mag + r_mag;
        ftype re = l_re * r_re + l_im * r_im;
        ftype im = r_re * l_im - r_im * l_re;
        ftype x, y, z;

        fn(stereo_position)(l_mag, r_mag, im, re, &x, &y, &z);
        fn(get_lfe)(output_lfe, n, lowcut, highcut, &lfemag[n], c_mag, &mag_total, lfe_mode);

        xpos[n]   = x;
        ypos[n]   = y;
        zpos[n]   = z;
        lphase[n] = l_phase;
        rphase[n] = r_phase;
        cmag[n]   = c_mag;
        cphase[n] = c_phase;
        magtotal[n] = mag_total;
    }
}

static void fn(filter_3_1)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    const ftype *srcl = (const ftype *)s->input->extended_data[0];
    const ftype *srcr = (const ftype *)s->input->extended_data[1];
    const ftype *srcc = (const ftype *)s->input->extended_data[2];
    const ftype *srclfe = (const ftype *)s->input->extended_data[3];
    const int rdft_size = s->rdft_size;
    ftype *magtotal = s->mag_total;
    ftype *lfephase = s->lfe_phase;
    ftype *lfemag = s->lfe_mag;
    ftype *lphase = s->l_phase;
    ftype *rphase = s->r_phase;
    ftype *cphase = s->c_phase;
    ftype *cmag = s->c_mag;
    ftype *xpos = s->x_pos;
    ftype *ypos = s->y_pos;
    ftype *zpos = s->z_pos;

    for (int n = 0; n < rdft_size; n++) {
        ftype l_re = srcl[2 * n], r_re = srcr[2 * n];
        ftype l_im = srcl[2 * n + 1], r_im = srcr[2 * n + 1];
        ftype lfe_re = srclfe[2 * n], lfe_im = srclfe[2 * n + 1];
        ftype c_re = srcc[2 * n], c_im = srcc[2 * n + 1];
        ftype c_phase = ATAN2(c_im, c_re);
        ftype c_mag = HYPOT(c_re, c_im);
        ftype l_mag = HYPOT(l_re, l_im);
        ftype r_mag = HYPOT(r_re, r_im);
        ftype lfe_mag = HYPOT(lfe_re, lfe_im);
        ftype lfe_phase = ATAN2(lfe_im, lfe_re);
        ftype mag_total = c_mag + l_mag + r_mag;
        ftype l_phase = ATAN2(l_im, l_re);
        ftype r_phase = ATAN2(r_im, r_re);
        ftype re = l_re * r_re + l_im * r_im;
        ftype im = r_re * l_im - r_im * l_re;
        ftype x, y, z;

        fn(stereo_position)(l_mag, r_mag, im, re, &x, &y, &z);

        xpos[n]   = x;
        ypos[n]   = y;
        zpos[n]   = z;
        lphase[n] = l_phase;
        rphase[n] = r_phase;
        cmag[n]   = c_mag;
        cphase[n] = c_phase;
        lfemag[n] = lfe_mag;
        lfephase[n] = lfe_phase;
        magtotal[n] = mag_total;
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
        ftype iscale = F(1.0) / SQRT(s->win_size);
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

    s->rdft_size = s->win_size / 2 + 1;

    s->factors = ff_get_audio_buffer(outlink, s->rdft_size);
    s->sfactors = ff_get_audio_buffer(outlink, s->rdft_size);
    s->output_ph = ff_get_audio_buffer(outlink, s->rdft_size);
    s->output_mag = ff_get_audio_buffer(outlink, s->rdft_size);
    s->output_out = ff_get_audio_buffer(outlink, s->win_size + 1);
    s->output = ff_get_audio_buffer(outlink, s->rdft_size * 2);
    s->overlap_buffer = ff_get_audio_buffer(outlink, s->win_size * 2);
    s->x_out = ff_get_audio_buffer(outlink, s->rdft_size);
    s->y_out = ff_get_audio_buffer(outlink, s->rdft_size);
    s->z_out = ff_get_audio_buffer(outlink, s->rdft_size);
    if (!s->overlap_buffer || !s->factors || !s->sfactors ||
        !s->output || !s->output_out || !s->output_mag || !s->output_ph ||
        !s->x_out || !s->y_out || !s->z_out)
        return AVERROR(ENOMEM);

    for (int ch = 0; ch < outlink->ch_layout.nb_channels; ch++) {
        ftype *src = (ftype *)s->sfactors->extended_data[ch];

        for (int n = 0; n < s->rdft_size; n++)
            src[n] = F(1.0) / s->nb_out_channels;
    }

    s->x_pos = av_calloc(s->rdft_size, sizeof(ftype));
    s->y_pos = av_calloc(s->rdft_size, sizeof(ftype));
    s->z_pos = av_calloc(s->rdft_size, sizeof(ftype));
    s->l_phase = av_calloc(s->rdft_size, sizeof(ftype));
    s->r_phase = av_calloc(s->rdft_size, sizeof(ftype));
    s->c_mag   = av_calloc(s->rdft_size, sizeof(ftype));
    s->c_phase = av_calloc(s->rdft_size, sizeof(ftype));
    s->mag_total = av_calloc(s->rdft_size, sizeof(ftype));
    s->lfe_mag = av_calloc(s->rdft_size, sizeof(ftype));
    s->lfe_phase = av_calloc(s->rdft_size, sizeof(ftype));
    if (!s->x_pos || !s->y_pos || !s->z_pos ||
        !s->l_phase || !s->r_phase || !s->lfe_phase || !s->c_phase ||
        !s->mag_total || !s->lfe_mag || !s->c_mag)
        return AVERROR(ENOMEM);

    return 0;
}

static int fn(ifft_channel)(AVFilterContext *ctx, AVFrame *out, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    const float *window_func_lut = s->window_func_lut;
    const ftype *output_levels = s->output_levels;
    const ftype level_out = output_levels[ch] * s->win_gain;
    const int win_size = s->win_size;
    ftype *dst, *ptr;

    dst = (ftype *)s->output_out->extended_data[ch];
    ptr = (ftype *)s->overlap_buffer->extended_data[ch];
    s->itx_fn(s->irdft[ch], dst, (ftype *)s->output->extended_data[ch], sizeof(ctype));

    memmove(ptr, ptr + s->hop_size, win_size * sizeof(ftype));

    for (int n = 0; n < win_size; n++)
        ptr[n] += dst[n] * window_func_lut[n] * level_out;

    dst = (ftype *)out->extended_data[ch];
    memcpy(dst, ptr, s->hop_size * sizeof(ftype));

    return 0;
}

static int fn(fft_channel)(AVFilterContext *ctx, AVFrame *in, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *src = (ftype *)s->input_in->extended_data[ch];
    ftype *win = (ftype *)s->window->extended_data[ch];
    const float *window_func_lut = s->window_func_lut;
    const int offset = s->input_in->nb_samples - s->hop_size;
    const ftype *input_levels = s->input_levels;
    const ftype level_in = input_levels[ch];
    const int win_size = s->win_size;

    memmove(src, &src[s->hop_size], offset * sizeof(ftype));
    memcpy(&src[offset], in->extended_data[ch], in->nb_samples * sizeof(ftype));
    memset(&src[offset + in->nb_samples], 0, (s->hop_size - in->nb_samples) * sizeof(ftype));

    for (int n = 0; n < win_size; n++)
        win[n] = src[n] * window_func_lut[n] * level_in;

    s->tx_fn(s->rdft[ch], (ftype *)s->input->extended_data[ch], win, sizeof(ftype));

    return 0;
}

static ftype fn(sqrf)(ftype x)
{
    return x * x;
}

static ftype fn(r_distance)(ftype a)
{
    return FMIN(SQRT(F(1.0) + fn(sqrf)(TAN(a))), SQRT(F(1.0) + fn(sqrf)(F(1.0) / (TAN(a) + EPSILON))));
}

static void fn(angle_transform)(ftype *x, ftype *y, ftype angle)
{
    ftype reference, r, a;

    if (angle == F(90.0))
        return;

    reference = angle * MPI / F(180.0);
    r = HYPOT(*x, *y);
    a = ATAN2(*x, *y);

    r /= fn(r_distance)(a);

    if (FABS(a) <= MPI4)
        a *= reference / MPI2;
    else
        a = MPI + (F(-2.0) * MPI + reference) * (MPI - FABS(a)) * FFDIFFSIGN(a, F(0.0)) / (F(3.0) * MPI2);

    r *= fn(r_distance)(a);

    *x = CLIP(SIN(a) * r, F(-1.0), F(1.0));
    *y = CLIP(COS(a) * r, F(-1.0), F(1.0));
}

static void fn(shift_transform)(ftype *y, const ftype shift)
{
    if (shift == F(0.0))
        return;

    *y = CLIP(*y + shift, F(-1.0), F(1.0));
}

static void fn(depth_transform)(ftype *y, const ftype depth)
{
    if (depth == F(0.0))
        return;

    if (depth < F(0.0) && *y > F(0.0))
        return;

    if (depth > F(0.0) && *y < F(0.0))
        return;

    *y = CLIP(*y + *y * depth, F(-1.0), F(1.0));
}

static void fn(focus_transform)(ftype *x, ftype *y, ftype focus)
{
    ftype a, r, ra;

    if (focus == F(0.0))
        return;

    a = ATAN2(*x, *y);
    ra = fn(r_distance)(a);
    r = CLIP(HYPOT(*x, *y) / ra, F(0.0), F(1.0));
    r = focus > F(0.0) ? F(1.0) - POW(F(1.0) - r, F(1.0) + focus * F(20.0)) : POW(r, F(1.0) - focus * F(20.0));
    r *= ra;
    *x = CLIP(SIN(a) * r, F(-1.0), F(1.0));
    *y = CLIP(COS(a) * r, F(-1.0), F(1.0));
}

static void fn(calculate_factors)(AVFilterContext *ctx, int ch, int chan)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *factor = (ftype *)s->factors->extended_data[ch];
    ftype *x_out = (ftype *)s->x_out->extended_data[ch];
    ftype *y_out = (ftype *)s->y_out->extended_data[ch];
    ftype *z_out = (ftype *)s->z_out->extended_data[ch];
    const ftype f_x = s->f_x[FFMIN(sc_map[chan >= 0 ? chan : 0], s->nb_f_x-1)];
    const ftype f_y = s->f_y[FFMIN(sc_map[chan >= 0 ? chan : 0], s->nb_f_y-1)];
    const ftype f_z = s->f_z[FFMIN(sc_map[chan >= 0 ? chan : 0], s->nb_f_z-1)];
    const int rdft_size = s->rdft_size;
    const ftype *x = s->x_pos;
    const ftype *y = s->y_pos;
    const ftype *z = s->z_pos;

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
            x_out[n] = F(1.0) - FABS(x[n]);
        break;
    case AV_CHAN_BOTTOM_FRONT_LEFT:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_TOP_SIDE_LEFT:
    case AV_CHAN_BACK_LEFT:
        for (int n = 0; n < rdft_size; n++)
            x_out[n] = (F(1.0) + x[n]) * F(0.5);
        break;
    case AV_CHAN_BOTTOM_FRONT_RIGHT:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_BACK_RIGHT:
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_TOP_SIDE_RIGHT:
    case AV_CHAN_BACK_RIGHT:
        for (int n = 0; n < rdft_size; n++)
            x_out[n] = (F(1.0) - x[n]) * F(0.5);
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
        for (int n = 0; n < rdft_size; n++)
            y_out[n] = (F(1.0) + y[n]) * F(0.5);
        break;
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_TOP_SIDE_LEFT:
    case AV_CHAN_TOP_SIDE_RIGHT:
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        for (int n = 0; n < rdft_size; n++)
            y_out[n] = F(1.0) - FABS(y[n]);
        break;
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_BACK_RIGHT:
    case AV_CHAN_BACK_LEFT:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_TOP_BACK_RIGHT:
        for (int n = 0; n < rdft_size; n++)
            y_out[n] = (F(1.0) - y[n]) * F(0.5);
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
            z_out[n] = (F(1.0) + z[n]) * F(0.5);
        break;
    case AV_CHAN_BOTTOM_FRONT_LEFT:
    case AV_CHAN_BOTTOM_FRONT_CENTER:
    case AV_CHAN_BOTTOM_FRONT_RIGHT:
        for (int n = 0; n < rdft_size; n++)
            z_out[n] = (F(1.0) - z[n]) * F(0.5);
        break;
    default:
        for (int n = 0; n < rdft_size; n++)
            z_out[n] = F(1.0) - FABS(z[n]);
        break;
    }

    for (int n = 0; n < rdft_size; n++) {
        factor[n] = POW(x_out[n], f_x) *
                    POW(y_out[n], f_y) *
                    POW(z_out[n], f_z);
        factor[n] = isnormal(factor[n]) ? factor[n] : F(0.0);
    }
}

static void fn(do_transform)(AVFilterContext *ctx, int ch)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *sfactor = (ftype *)s->sfactors->extended_data[ch];
    ftype *factor = (ftype *)s->factors->extended_data[ch];
    ftype *omag = (ftype *)s->output_mag->extended_data[ch];
    ftype *oph = (ftype *)s->output_ph->extended_data[ch];
    ctype *dst = (ctype *)s->output->extended_data[ch];
    const int rdft_size = s->rdft_size;
    const ftype smooth = s->smooth[FFMIN(ch, s->nb_smooth-1)];

    if (smooth > F(0.0)) {
        for (int n = 0; n < rdft_size; n++) {
            sfactor[n] = smooth * factor[n] + (F(1.0) - smooth) * sfactor[n];
            sfactor[n] = isnormal(sfactor[n]) ? sfactor[n] : F(0.0);
        }

        factor = sfactor;
    }

    for (int n = 0; n < rdft_size; n++)
        omag[n] *= factor[n];

    for (int n = 0; n < rdft_size; n++) {
        const ftype mag = omag[n];
        const ftype ph = oph[n];

        dst[n].re = mag * COS(ph);
        dst[n].im = mag * SIN(ph);
    }
}

static int fn(transform_xy)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSurroundContext *s = ctx->priv;
    const int rdft_size = s->rdft_size;
    const int start = (rdft_size * jobnr) / nb_jobs;
    const int end = (rdft_size * (jobnr+1)) / nb_jobs;
    const ftype angle = s->angle;
    const ftype focus = s->focus;
    const ftype shift_x = s->shift[0];
    const ftype shift_y = s->shift[1];
    const ftype shift_z = s->shift[2];
    const ftype depth_x = s->depth[0];
    const ftype depth_y = s->depth[1];
    const ftype depth_z = s->depth[2];
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
        fn(focus_transform)(&x[n], &y[n], focus);
    }

    return 0;
}

static void fn(stereo_copy)(AVFilterContext *ctx, int ch, int chan)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *omag = (ftype *)s->output_mag->extended_data[ch];
    ftype *oph = (ftype *)s->output_ph->extended_data[ch];
    const ftype *mag_total = s->mag_total;
    const int rdft_size = s->rdft_size;
    const ftype *c_phase = s->c_phase;
    const ftype *l_phase = s->l_phase;
    const ftype *r_phase = s->r_phase;
    const ftype *lfe_mag = s->lfe_mag;
    const ftype *c_mag = s->c_mag;

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
        memcpy(omag, c_mag, rdft_size * sizeof(*omag));
        break;
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        memcpy(omag, lfe_mag, rdft_size * sizeof(*omag));
        break;
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_TOP_BACK_RIGHT:
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_BACK_LEFT:
    case AV_CHAN_BACK_RIGHT:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_TOP_SIDE_LEFT:
    case AV_CHAN_TOP_SIDE_RIGHT:
    case AV_CHAN_BOTTOM_FRONT_CENTER:
    case AV_CHAN_BOTTOM_FRONT_LEFT:
    case AV_CHAN_BOTTOM_FRONT_RIGHT:
        memcpy(omag, mag_total, rdft_size * sizeof(*omag));
        break;
    default:
        break;
    }

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_BOTTOM_FRONT_CENTER:
        memcpy(oph, c_phase, rdft_size * sizeof(*oph));
        break;
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_BACK_LEFT:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_SIDE_LEFT:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_BOTTOM_FRONT_LEFT:
        memcpy(oph, l_phase, rdft_size * sizeof(*oph));
        break;
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_BACK_RIGHT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_SIDE_RIGHT:
    case AV_CHAN_TOP_BACK_RIGHT:
    case AV_CHAN_BOTTOM_FRONT_RIGHT:
        memcpy(oph, r_phase, rdft_size * sizeof(*oph));
        break;
    default:
        break;
    }
}

static void fn(stereo_lfe_copy)(AVFilterContext *ctx, int ch, int chan)
{
    AudioSurroundContext *s = ctx->priv;
    ftype *omag = (ftype *)s->output_mag->extended_data[ch];
    ftype *oph = (ftype *)s->output_ph->extended_data[ch];
    const ftype *mag_total = s->mag_total;
    const ftype *lfe_phase = s->lfe_phase;
    const int rdft_size = s->rdft_size;
    const ftype *c_phase = s->c_phase;
    const ftype *l_phase = s->l_phase;
    const ftype *r_phase = s->r_phase;
    const ftype *lfe_mag = s->lfe_mag;
    const ftype *c_mag = s->c_mag;

    switch (chan) {
    case AV_CHAN_FRONT_CENTER:
        memcpy(omag, c_mag, rdft_size * sizeof(*omag));
        break;
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        memcpy(omag, lfe_mag, rdft_size * sizeof(*omag));
        break;
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_BACK_LEFT:
    case AV_CHAN_BACK_RIGHT:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_SIDE_LEFT:
    case AV_CHAN_TOP_SIDE_RIGHT:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_TOP_BACK_RIGHT:
    case AV_CHAN_BOTTOM_FRONT_CENTER:
    case AV_CHAN_BOTTOM_FRONT_LEFT:
    case AV_CHAN_BOTTOM_FRONT_RIGHT:
        memcpy(omag, mag_total, rdft_size * sizeof(*omag));
        break;
    default:
        break;
    }

    switch (chan) {
    case AV_CHAN_LOW_FREQUENCY:
    case AV_CHAN_LOW_FREQUENCY_2:
        memcpy(oph, lfe_phase, rdft_size * sizeof(*oph));
        break;
    case AV_CHAN_FRONT_CENTER:
    case AV_CHAN_BACK_CENTER:
    case AV_CHAN_TOP_FRONT_CENTER:
    case AV_CHAN_TOP_BACK_CENTER:
    case AV_CHAN_TOP_CENTER:
    case AV_CHAN_BOTTOM_FRONT_CENTER:
        memcpy(oph, c_phase, rdft_size * sizeof(*oph));
        break;
    case AV_CHAN_FRONT_LEFT:
    case AV_CHAN_BACK_LEFT:
    case AV_CHAN_SIDE_LEFT:
    case AV_CHAN_TOP_FRONT_LEFT:
    case AV_CHAN_TOP_SIDE_LEFT:
    case AV_CHAN_TOP_BACK_LEFT:
    case AV_CHAN_BOTTOM_FRONT_LEFT:
        memcpy(oph, l_phase, rdft_size * sizeof(*oph));
        break;
    case AV_CHAN_FRONT_RIGHT:
    case AV_CHAN_BACK_RIGHT:
    case AV_CHAN_SIDE_RIGHT:
    case AV_CHAN_TOP_FRONT_RIGHT:
    case AV_CHAN_TOP_SIDE_RIGHT:
    case AV_CHAN_TOP_BACK_RIGHT:
    case AV_CHAN_BOTTOM_FRONT_RIGHT:
        memcpy(oph, r_phase, rdft_size * sizeof(*oph));
        break;
    default:
        break;
    }
}

static int fn(config_input)(AVFilterContext *ctx)
{
    AudioSurroundContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];

    s->set_input_levels = fn(set_input_levels);
    s->set_output_levels = fn(set_output_levels);
    s->ifft_channel = fn(ifft_channel);
    s->fft_channel = fn(fft_channel);
    s->calculate_factors = fn(calculate_factors);
    s->stereo_copy = fn(stereo_copy);
    s->stereo_lfe_copy = fn(stereo_lfe_copy);
    s->do_transform = fn(do_transform);
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
        ftype scale = F(1.0) / SQRT(s->win_size);
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
