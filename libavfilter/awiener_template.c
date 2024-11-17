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

#undef FCOS
#undef ftype
#undef ctype
#undef FPI
#undef SAMPLE_FORMAT
#undef EPS
#if DEPTH == 32
#define FCOS cosf
#define ftype float
#define ctype AVComplexFloat
#define FPI M_PIf
#define SAMPLE_FORMAT fltp
#define EPS FLT_EPSILON
#else
#define FCOS cos
#define ftype double
#define ctype AVComplexDouble
#define FPI M_PI
#define SAMPLE_FORMAT dblp
#define EPS DBL_EPSILON
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

static void fn(apply_window)(AudioWienerContext *s, const ftype *in_frame,
                             ftype *out_frame, const int add_to_out_frame)
{
    const ftype *window = s->window;
    const int rdft_size = s->rdft_size;

    if (add_to_out_frame) {
        for (int i = 0; i < rdft_size; i++)
            out_frame[i] += in_frame[i] * window[i];
    } else {
        for (int i = 0; i < rdft_size; i++)
            out_frame[i] = in_frame[i] * window[i];
    }
}

static ftype fn(sqr)(const ftype x)
{
    return x * x;
}

static ftype fn(a_priori_gain)(const ftype snr)
{
    return snr / (snr + F(1.0));
}

static void fn(feed)(AVFilterContext *ctx, int ch,
                     const ftype *in_samples, ftype *out_samples,
                     ftype *in_frame, ftype *out_dist_frame,
                     ftype *windowed_frame, ftype *wiener_frame,
                     ctype *spectrum_buf, ftype *sbb)
{
    AudioWienerContext *s = ctx->priv;
    const ftype reduction = s->reduction * F(0.01);
    const ftype keep = F(1.0) - reduction;
    const int rdft_size = s->rdft_size;
    const int nb_coeffs = s->rdft_size/2 + 1;
    const int overlap = s->overlap;
    enum AVChannel channel = av_channel_layout_channel_from_index(&ctx->inputs[0]->ch_layout, ch);
    const int bypass = av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;
    const int offset = rdft_size - overlap;

    // shift in/out buffers
    memmove(in_frame, in_frame + overlap, offset * sizeof(*in_frame));
    memmove(out_dist_frame, out_dist_frame + overlap, offset * sizeof(*out_dist_frame));

    memcpy(in_frame + offset, in_samples, sizeof(*in_frame) * overlap);
    memset(out_dist_frame + offset, 0, sizeof(*out_dist_frame) * overlap);

    fn(apply_window)(s, in_frame, windowed_frame, 0);

    s->tx_fn(s->tx_ctx[ch], spectrum_buf, windowed_frame, sizeof(ftype));

    if (s->capture) {
        const ftype frame = F(s->noise_frame);
        const ftype scale = F(1.0) / (frame + F(1.0));

        for (int n = 0; n < nb_coeffs; n++)
            sbb[n] = frame * sbb[n] * scale + (fn(sqr)(spectrum_buf[n].re) + fn(sqr)(spectrum_buf[n].im)) * scale;
    }

    for (int n = 0; n < nb_coeffs; n++) {
        ftype p = fn(sqr)(spectrum_buf[n].re) + fn(sqr)(spectrum_buf[n].im);
        ftype snr_post = p / (sbb[n] + EPS);
        ftype g = fn(a_priori_gain)(snr_post);
        ctype new;

        new.re = spectrum_buf[n].re * g;
        new.im = spectrum_buf[n].im * g;

        spectrum_buf[n].re = new.re * reduction + keep * spectrum_buf[n].re;
        spectrum_buf[n].im = new.im * reduction + keep * spectrum_buf[n].im;
    }

    s->itx_fn(s->itx_ctx[ch], wiener_frame, spectrum_buf, sizeof(ctype));

    fn(apply_window)(s, wiener_frame, out_dist_frame, 1);

    if (ctx->is_disabled || bypass)
        memcpy(out_samples, in_frame, sizeof(*out_samples) * overlap);
    else
        memcpy(out_samples, out_dist_frame, sizeof(*out_samples) * overlap);
}

static int fn(wiener_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, int ch)
{
    AudioWienerContext *s = ctx->priv;
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
                 (ftype *)(s->wiener_frame->extended_data[ch]),
                 (ctype *)(s->spectrum_buf->extended_data[ch]),
                 (ftype *)(s->sbb->extended_data[ch]));
    }

    return 0;
}
