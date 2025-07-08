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

#undef FCOS
#undef FSQRT
#undef ftype
#undef ctype
#undef FEXP10
#undef FLOG10
#undef FMA
#undef FPI
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define FCOS cosf
#define FSQRT sqrtf
#define ftype float
#define ctype AVComplexFloat
#define FEXP10 ff_exp10f
#define FLOG10 log10f
#define FMA fmaf
#define FPI M_PIf
#define SAMPLE_FORMAT fltp
#else
#define FCOS cos
#define FSQRT sqrt
#define ftype double
#define ctype AVComplexDouble
#define FEXP10 ff_exp10
#define FLOG10 log10
#define FMA fma
#define FPI M_PI
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static void fn(generate_hann_window)(void *_window, int size)
{
    ftype *window = _window;

    for (int i = 0; i < size; i++) {
        ftype value = F(0.5) * (F(1.0) - FCOS(F(2.0) * FPI * i / size));

        window[i] = value;
    }
}

static void fn(apply_window)(AudioDRCContext *s, const ftype *in_frame,
                             ftype *out_frame, const int add_to_out_frame)
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

static void fn(get_energy)(AVFilterContext *ctx, const int len, ftype *energy,
                           const ftype *spectral)
{
    for (int n = 0; n < len; n++) {
        energy[n] = F(10.0) * FLOG10(fn(sqr)(spectral[2 * n]) + fn(sqr)(spectral[2 * n + 1]));
        if (!isnormal(energy[n]))
            energy[n] = F(-351.0);
    }
}

static void fn(get_threshold)(AVFilterContext *ctx,
                              const int len,
                              ftype *threshold,
                              const ftype *energy,
                              double *var_values,
                              const ftype fx, const int bypass)
{
    AudioDRCContext *s = ctx->priv;

    for (int n = 0; n < len; n++) {
        const ftype Xg = energy[n];

        var_values[VAR_P] = Xg;
        var_values[VAR_F] = n * fx;
        var_values[VAR_TH] = threshold[n];

        threshold[n] = av_expr_eval(s->threshold_expr, var_values, s);
    }
}

static void fn(get_target_gain)(AVFilterContext *ctx,
                                const int len,
                                ftype *threshold,
                                ftype *gain,
                                const ftype *energy,
                                double *var_values,
                                const ftype fx, const int bypass)
{
    AudioDRCContext *s = ctx->priv;

    if (bypass) {
        memcpy(gain, energy, sizeof(*gain) * len);
        return;
    }

    for (int n = 0; n < len; n++) {
        const ftype Xg = energy[n];

        var_values[VAR_P] = Xg;
        var_values[VAR_F] = n * fx;
        var_values[VAR_TH] = threshold[n];

        gain[n] = av_expr_eval(s->transfer_expr, var_values, s);
    }
}

static void fn(get_envelope)(AVFilterContext *ctx,
                             const int len,
                             ftype *envelope,
                             const ftype *energy,
                             const ftype *gain)
{
    AudioDRCContext *s = ctx->priv;
    const ftype release = s->release;
    const ftype attack = s->attack;

    for (int n = 0; n < len; n++) {
        const ftype Bg = gain[n] - energy[n];
        const ftype Vg = envelope[n];

        if (Bg > Vg) {
            envelope[n] = FMA(Vg - Bg, attack, Bg);
        } else if (Bg < Vg)  {
            envelope[n] = FMA(Vg - Bg, release, Bg);
        } else {
            envelope[n] = Vg;
        }
    }
}

static void fn(get_factors)(AVFilterContext *ctx,
                            const int len,
                            ftype *factors,
                            const ftype *envelope)
{
    for (int n = 0; n < len; n++)
        factors[n] = FSQRT(FEXP10(envelope[n] / F(10.0)));
}

static void fn(apply_factors)(AVFilterContext *ctx,
                              const int len,
                              ftype *spectrum,
                              const ftype *factors)
{
    for (int n = 0; n < len; n++) {
        spectrum[2*n+0] *= factors[n];
        spectrum[2*n+1] *= factors[n];
    }
}

static void fn(feed)(AVFilterContext *ctx, int ch,
                     const ftype *in_samples, ftype *out_samples,
                     ftype *in_frame, ftype *out_dist_frame,
                     ftype *windowed_frame, ftype *drc_frame,
                     ftype *spectrum_buf, ftype *energy, ftype *threshold,
                     ftype *target_gain, ftype *envelope,
                     ftype *factors)
{
    AudioDRCContext *s = ctx->priv;
    double var_values[VAR_VARS_NB];
    const int fft_size = s->fft_size;
    const int nb_coeffs = s->fft_size + 1;
    const int overlap = s->overlap;
    enum AVChannel channel = av_channel_layout_channel_from_index(&ctx->inputs[0]->ch_layout, ch);
    const int bypass = av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;
    const int offset = fft_size - overlap;

    memcpy(var_values, s->var_values, sizeof(var_values));

    var_values[VAR_CH] = ch;

    // shift in/out buffers
    memmove(in_frame, in_frame + overlap, offset * sizeof(*in_frame));
    memmove(out_dist_frame, out_dist_frame + overlap, offset * sizeof(*out_dist_frame));

    memcpy(in_frame + offset, in_samples, sizeof(*in_frame) * overlap);
    memset(out_dist_frame + offset, 0, sizeof(*out_dist_frame) * overlap);

    fn(apply_window)(s, in_frame, windowed_frame, 0);
    s->tx_fn(s->tx_ctx[ch], spectrum_buf, windowed_frame, sizeof(ftype));

    fn(get_energy)(ctx, nb_coeffs, energy, spectrum_buf);
    fn(get_threshold)(ctx, nb_coeffs, threshold, energy, var_values, s->fx, bypass);
    fn(get_target_gain)(ctx, nb_coeffs, threshold, target_gain, energy, var_values, s->fx, bypass);
    fn(get_envelope)(ctx, nb_coeffs, envelope, energy, target_gain);
    fn(get_factors)(ctx, nb_coeffs, factors, envelope);
    fn(apply_factors)(ctx, nb_coeffs, spectrum_buf, factors);

    s->itx_fn(s->itx_ctx[ch], drc_frame, spectrum_buf, sizeof(ctype));

    fn(apply_window)(s, drc_frame, out_dist_frame, 1);

    if (ff_filter_disabled(ctx))
        memcpy(out_samples, in_frame, sizeof(*out_samples) * overlap);
    else
        memcpy(out_samples, out_dist_frame, sizeof(*out_samples) * overlap);
}

static int fn(drc_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, int ch)
{
    AudioDRCContext *s = ctx->priv;
    ftype *in_buffer = (ftype *)s->in_buffer->extended_data[ch];
    const int overlap = s->overlap;

    for (int offset = 0; offset < out->nb_samples; offset += overlap) {
        const ftype *src = ((const ftype *)in->extended_data[ch])+offset;
        ftype *dst = ((ftype *)out->extended_data[ch])+offset;

        memcpy(in_buffer, src, sizeof(*in_buffer) * overlap);

        fn(feed)(ctx, ch, in_buffer, dst,
                 (ftype *)(s->in_frame->extended_data[ch]),
                 (ftype *)(s->out_dist_frame->extended_data[ch]),
                 (ftype *)(s->windowed_frame->extended_data[ch]),
                 (ftype *)(s->drc_frame->extended_data[ch]),
                 (ftype *)(s->spectrum_buf->extended_data[ch]),
                 (ftype *)(s->energy->extended_data[ch]),
                 (ftype *)(s->threshold->extended_data[ch]),
                 (ftype *)(s->target_gain->extended_data[ch]),
                 (ftype *)(s->envelope->extended_data[ch]),
                 (ftype *)(s->factors->extended_data[ch]));
    }

    return 0;
}

static int fn(drc_flush)(AVFilterContext *ctx, AVFrame *out, int ch)
{
    AudioDRCContext *s = ctx->priv;
    ftype *in_buffer = (ftype *)s->in_buffer->extended_data[ch];
    const int overlap = s->overlap;

    for (int offset = 0; offset < out->nb_samples; offset += overlap) {
        ftype *dst = ((ftype *)out->extended_data[ch])+offset;

        memset(in_buffer, 0, sizeof(*in_buffer) * overlap);

        fn(feed)(ctx, ch, in_buffer, dst,
                 (ftype *)(s->in_frame->extended_data[ch]),
                 (ftype *)(s->out_dist_frame->extended_data[ch]),
                 (ftype *)(s->windowed_frame->extended_data[ch]),
                 (ftype *)(s->drc_frame->extended_data[ch]),
                 (ftype *)(s->spectrum_buf->extended_data[ch]),
                 (ftype *)(s->energy->extended_data[ch]),
                 (ftype *)(s->threshold->extended_data[ch]),
                 (ftype *)(s->target_gain->extended_data[ch]),
                 (ftype *)(s->envelope->extended_data[ch]),
                 (ftype *)(s->factors->extended_data[ch]));
    }

    return 0;
}
