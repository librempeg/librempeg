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

#undef FEXP
#undef FLOG
#undef FCOS
#undef ftype
#undef ctype
#undef FPI
#undef SAMPLE_FORMAT
#undef TX_TYPE
#undef EPS
#if DEPTH == 32
#define FEXP expf
#define FLOG logf
#define FCOS cosf
#define ftype float
#define ctype AVComplexFloat
#define FPI M_PIf
#define SAMPLE_FORMAT fltp
#define TX_TYPE AV_TX_FLOAT_RDFT
#define EPS FLT_EPSILON
#else
#define FEXP exp
#define FLOG log
#define FCOS cos
#define ftype double
#define ctype AVComplexDouble
#define FPI M_PI
#define SAMPLE_FORMAT dblp
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define EPS FLT_EPSILON
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ftype *wiener_frame;
    ftype *sbb;
    ctype *ss[2];
    ftype *in_buffer;
    ftype *in_frame;
    ftype *out_dist_frame;
    ctype *spectrum_buf;
    ftype *windowed_frame;

    AVTXContext *tx_ctx, *itx_ctx;
    av_tx_fn tx_fn, itx_fn;

    int capture;
    int noise_frame;
} fn(StateContext);

static int fn(init)(AVFilterContext *ctx)
{
    AudioWienerContext *s = ctx->priv;
    ftype scale, iscale;
    fn(StateContext) *st;
    int ret;

    s->st = av_calloc(s->channels, sizeof(*st));
    if (!s->st)
        return AVERROR(ENOMEM);

    scale = F(1.0) / s->rdft_size;
    iscale = F(1.0 / 1.5);

    st = s->st;
    for (int ch = 0; ch < s->channels; ch++) {
        fn(StateContext) *stc = &st[ch];

        stc->capture = -1;

        stc->wiener_frame   = av_calloc(s->rdft_size, sizeof(*stc->wiener_frame));
        stc->sbb            = av_calloc(s->rdft_size + 2, sizeof(*stc->sbb));
        stc->ss[0]          = av_calloc(s->rdft_size/2+1, sizeof(*stc->ss[0]));
        stc->ss[1]          = av_calloc(s->rdft_size/2+1, sizeof(*stc->ss[1]));
        stc->in_buffer      = av_calloc(s->rdft_size, sizeof(*stc->in_buffer));
        stc->in_frame       = av_calloc(s->rdft_size, sizeof(*stc->in_frame));
        stc->out_dist_frame = av_calloc(s->rdft_size, sizeof(*stc->out_dist_frame));
        stc->spectrum_buf   = av_calloc(s->rdft_size/2+1, sizeof(*stc->spectrum_buf));
        stc->windowed_frame = av_calloc(s->rdft_size, sizeof(*stc->windowed_frame));
        if (!stc->in_buffer || !stc->in_frame || !stc->ss[0] || !stc->ss[1] ||
            !stc->out_dist_frame || !stc->windowed_frame ||
            !stc->wiener_frame || !stc->spectrum_buf || !stc->sbb)
            return AVERROR(ENOMEM);

        ret = av_tx_init(&stc->tx_ctx, &stc->tx_fn, TX_TYPE, 0, s->rdft_size, &scale, 0);
        if (ret < 0)
            return ret;

        ret = av_tx_init(&stc->itx_ctx, &stc->itx_fn, TX_TYPE, 1, s->rdft_size, &iscale, 0);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static void fn(uninit)(AVFilterContext *ctx)
{
    AudioWienerContext *s = ctx->priv;
    fn(StateContext) *st;

    st = s->st;
    for (int ch = 0; ch < s->channels && st; ch++) {
        fn(StateContext) *stc = &st[ch];

        av_freep(&stc->wiener_frame);
        av_freep(&stc->sbb);
        av_freep(&stc->ss[0]);
        av_freep(&stc->ss[1]);
        av_freep(&stc->in_buffer);
        av_freep(&stc->in_frame);
        av_freep(&stc->out_dist_frame);
        av_freep(&stc->spectrum_buf);
        av_freep(&stc->windowed_frame);

        av_tx_uninit(&stc->tx_ctx);
        av_tx_uninit(&stc->itx_ctx);
    }

    av_freep(&s->st);
}

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

static ftype fn(spectral_flatness)(const ctype *const spectral, int size)
{
    ftype num = F(0.0), den = F(0.0);

    for (int n = 0; n < size; n++) {
        const ftype v = fn(sqr)(spectral[n].re) + fn(sqr)(spectral[n].im);

        num += FLOG(v + EPS);
        den += v;
    }

    num /= size;
    den /= size;
    num = FEXP(num);
    den += EPS;

    return num / den;
}

static void fn(feed)(AVFilterContext *ctx, int ch,
                     const ftype *in_samples, ftype *out_samples,
                     ftype *in_frame, ftype *out_dist_frame,
                     ftype *windowed_frame, ftype *wiener_frame,
                     ctype *spectrum_buf, ctype *ss0, ctype *ss1, ftype *sbb)
{
    AudioWienerContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;
    fn(StateContext) *st = &stc[ch];
    const ftype reduction = s->reduction * F(0.01);
    const ftype keep = F(1.0) - reduction;
    const int rdft_size = s->rdft_size;
    const int nb_coeffs = s->rdft_size/2 + 1;
    const int overlap = s->overlap;
    enum AVChannel channel = av_channel_layout_channel_from_index(&ctx->inputs[0]->ch_layout, ch);
    const int bypass = av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;
    const int offset = rdft_size - overlap;
    int capture;

    // shift in/out buffers
    memmove(in_frame, in_frame + overlap, offset * sizeof(*in_frame));
    memmove(out_dist_frame, out_dist_frame + overlap, offset * sizeof(*out_dist_frame));

    memcpy(in_frame + offset, in_samples, sizeof(*in_frame) * overlap);
    memset(out_dist_frame + offset, 0, sizeof(*out_dist_frame) * overlap);

    fn(apply_window)(s, in_frame, windowed_frame, 0);

    stc->tx_fn(stc->tx_ctx, spectrum_buf, windowed_frame, sizeof(ftype));

    if (s->capture >= 0) {
        capture = s->capture;
    } else {
        const ftype flatness = fn(spectral_flatness)(spectrum_buf, nb_coeffs);

        capture = flatness >= F(0.985);
    }

    if (st->capture != capture) {
        st->capture = capture;
        st->noise_frame = 0;
    }

    if (st->capture > 0) {
        const ftype frame = (st->noise_frame > 0) ? F(0.5) : F(0.0);
        const ftype scale = F(1.0) / (frame + F(1.0));

        for (int n = 0; n < nb_coeffs; n++)
            sbb[n] = sbb[n] * (F(1.0) - scale) + (fn(sqr)(spectrum_buf[n].re) + fn(sqr)(spectrum_buf[n].im)) * scale;

        st->noise_frame++;
    }

    for (int n = 0; n < nb_coeffs; n++) {
        ftype snr_dd_prio, g_tsnr, snr_tsnr_prio, g_dd;
        ftype p = fn(sqr)(spectrum_buf[n].re) + fn(sqr)(spectrum_buf[n].im);
        ftype snr_post = p / (sbb[n] + EPS);
        ftype g = fn(a_priori_gain)(snr_post);
        const ftype beta = F(0.5);
        ctype new, s_dd;

        ss1[n].re = spectrum_buf[n].re * g;
        ss1[n].im = spectrum_buf[n].im * g;

        snr_dd_prio = beta*fn(sqr)(ss0[n].re) + fn(sqr)(ss0[n].im) / (sbb[n] + EPS) + (F(1.0) - beta)*((snr_post - F(1.0)) > F(1.0));
        g_dd = fn(a_priori_gain)(snr_dd_prio);
        s_dd.re = g_dd * spectrum_buf[n].re;
        s_dd.im = g_dd * spectrum_buf[n].im;

        snr_tsnr_prio = (s_dd.re*s_dd.re + s_dd.im*s_dd.im) / (sbb[n] + EPS);
        g_tsnr = fn(a_priori_gain)(snr_tsnr_prio);
        new.re = g_tsnr * spectrum_buf[n].re;
        new.im = g_tsnr * spectrum_buf[n].im;

        spectrum_buf[n].re = new.re * reduction + keep * spectrum_buf[n].re;
        spectrum_buf[n].im = new.im * reduction + keep * spectrum_buf[n].im;
    }

    stc->itx_fn(stc->itx_ctx, wiener_frame, spectrum_buf, sizeof(ctype));

    fn(apply_window)(s, wiener_frame, out_dist_frame, 1);

    if (ff_filter_disabled(ctx) || bypass)
        memcpy(out_samples, in_frame, sizeof(*out_samples) * overlap);
    else
        memcpy(out_samples, out_dist_frame, sizeof(*out_samples) * overlap);

    memcpy(ss0, ss1, sizeof(*ss0) * nb_coeffs);
}

static int fn(wiener_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, int ch)
{
    AudioWienerContext *s = ctx->priv;
    const int overlap = s->overlap;
    fn(StateContext) *st = s->st;
    fn(StateContext) *stc = &st[ch];
    ftype *in_buffer = stc->in_buffer;

    for (int offset = 0; offset < out->nb_samples; offset += overlap) {
        const ftype *src = ((const ftype *)in->extended_data[ch])+offset;
        const int nb_samples = FFMIN(out->nb_samples - offset, overlap);
        ftype *dst = ((ftype *)out->extended_data[ch])+offset;

        memcpy(in_buffer, src, sizeof(*in_buffer) * nb_samples);
        memset(in_buffer + nb_samples, 0, sizeof(*in_buffer) * (overlap - nb_samples));

        fn(feed)(ctx, ch, in_buffer, dst,
                 stc->in_frame,
                 stc->out_dist_frame,
                 stc->windowed_frame,
                 stc->wiener_frame,
                 stc->spectrum_buf,
                 stc->ss[0],
                 stc->ss[1],
                 stc->sbb);
    }

    return 0;
}

static int fn(wiener_flush_channel)(AVFilterContext *ctx, AVFrame *out, int ch)
{
    AudioWienerContext *s = ctx->priv;
    const int overlap = s->overlap;
    fn(StateContext) *st = s->st;
    fn(StateContext) *stc = &st[ch];
    ftype *in_buffer = stc->in_buffer;

    for (int offset = 0; offset < out->nb_samples; offset += overlap) {
        ftype *dst = ((ftype *)out->extended_data[ch])+offset;

        memset(in_buffer, 0, sizeof(*in_buffer) * overlap);

        fn(feed)(ctx, ch, in_buffer, dst,
                 stc->in_frame,
                 stc->out_dist_frame,
                 stc->windowed_frame,
                 stc->wiener_frame,
                 stc->spectrum_buf,
                 stc->ss[0],
                 stc->ss[1],
                 stc->sbb);
    }

    return 0;
}
