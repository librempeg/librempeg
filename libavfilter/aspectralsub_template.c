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
#undef FMAX
#undef FMIN
#undef ftype
#undef ctype
#undef FPI
#undef TX_TYPE
#undef SAMPLE_FORMAT
#undef EPS
#if DEPTH == 32
#define FCOS cosf
#define FMAX fmaxf
#define FMIN fminf
#define ftype float
#define ctype AVComplexFloat
#define FPI M_PIf
#define TX_TYPE AV_TX_FLOAT_RDFT
#define SAMPLE_FORMAT fltp
#define EPS FLT_EPSILON
#else
#define FCOS cos
#define FMAX fmax
#define FMIN fmin
#define ftype double
#define ctype AVComplexDouble
#define FPI M_PI
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define SAMPLE_FORMAT dblp
#define EPS FLT_EPSILON
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    int idx;

    ftype *r2c_in;
    ctype *r2c_out;

    ctype *c2r_in;
    ftype *c2r_out;

    ftype *input;
    ftype *output;
    ftype *p_n, *p_sn;
    ftype *history;

    AVTXContext *tx_ctx;
    av_tx_fn tx_fn;
    AVTXContext *itx_ctx;
    av_tx_fn itx_fn;
} fn(StateContext);

static int fn(init)(AVFilterContext *ctx)
{
    AudioSpectralSubtractionContext *s = ctx->priv;
    const int rdft_size = s->rdft_size;
    const int nb_coeffs = rdft_size/2 + 1;
    fn(StateContext) *stc;
    int ret;

    s->st = av_calloc(s->channels, sizeof(*stc));
    if (!s->st)
        return AVERROR(ENOMEM);

    stc = s->st;
    for (int ch = 0; ch < s->channels; ch++) {
        fn(StateContext) *st = &stc[ch];
        ftype scale, iscale;

        scale  = F(1.0) / s->rdft_size;
        iscale = F(1.0) / F(1.5);

        ret = av_tx_init(&st->tx_ctx, &st->tx_fn, TX_TYPE, 0, s->rdft_size, &scale, 0);
        if (ret < 0)
            return ret;

        ret = av_tx_init(&st->itx_ctx, &st->itx_fn, TX_TYPE, 1, s->rdft_size, &iscale, 0);
        if (ret < 0)
            return ret;

        st->r2c_in = av_calloc(rdft_size, sizeof(*st->r2c_in));
        if (!st->r2c_in)
            return AVERROR(ENOMEM);

        st->r2c_out = av_calloc(nb_coeffs, sizeof(*st->r2c_out));
        if (!st->r2c_out)
            return AVERROR(ENOMEM);

        st->c2r_in = av_calloc(nb_coeffs, sizeof(*st->c2r_in));
        if (!st->c2r_in)
            return AVERROR(ENOMEM);

        st->c2r_out = av_calloc(rdft_size, sizeof(*st->c2r_out));
        if (!st->c2r_out)
            return AVERROR(ENOMEM);

        st->input = av_calloc(rdft_size, sizeof(*st->input));
        if (!st->input)
            return AVERROR(ENOMEM);

        st->output = av_calloc(rdft_size, sizeof(*st->output));
        if (!st->output)
            return AVERROR(ENOMEM);

        st->p_n = av_calloc(nb_coeffs, sizeof(*st->p_n));
        if (!st->p_n)
            return AVERROR(ENOMEM);

        st->p_sn = av_calloc(nb_coeffs, sizeof(*st->p_sn));
        if (!st->p_sn)
            return AVERROR(ENOMEM);

        st->history = av_calloc(nb_coeffs * (s->history+1), sizeof(*st->history));
        if (!st->history)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static void fn(uninit)(AVFilterContext *ctx)
{
    AudioSpectralSubtractionContext *s = ctx->priv;
    fn(StateContext) *st;

    st = s->st;
    for (int ch = 0; ch < s->channels && st; ch++) {
        fn(StateContext) *stc = &st[ch];

        av_freep(&st->r2c_in);
        av_freep(&st->r2c_out);
        av_freep(&st->c2r_in);
        av_freep(&st->c2r_out);
        av_freep(&st->input);
        av_freep(&st->output);
        av_freep(&st->p_n);
        av_freep(&st->p_sn);
        av_freep(&st->history);

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

static void fn(apply_window)(AudioSpectralSubtractionContext *s, const ftype *in_frame,
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

static void fn(feed)(AVFilterContext *ctx, int ch,
                     const ftype *src, ftype *dst,
                     const int nb_samples)
{
    AudioSpectralSubtractionContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;
    fn(StateContext) *st = &stc[ch];
    const ftype gmin = s->reduction;
    const ftype beta = s->beta;
    const int rdft_size = s->rdft_size;
    const int nb_coeffs = rdft_size/2 + 1;
    const int overlap = s->overlap;
    enum AVChannel channel = av_channel_layout_channel_from_index(&ctx->inputs[0]->ch_layout, ch);
    const int bypass = av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;
    const int offset = rdft_size - overlap;
    const int stride = s->history + 1;
    ftype *p_sn = st->p_sn;
    ftype *p_n = st->p_n;
    int idx = st->idx;

    // shift in/out buffers
    memmove(st->input, st->input + overlap, offset * sizeof(*st->input));
    memmove(st->output, st->output + overlap, offset * sizeof(*st->output));

    if (src)
        memcpy(st->input + offset, src, sizeof(*st->input) * overlap);
    else
        memset(st->input + offset, 0, sizeof(*st->input) * overlap);

    memset(st->output + offset, 0, sizeof(*st->output) * overlap);

    fn(apply_window)(s, st->input, st->r2c_in, 0);

    st->tx_fn(st->tx_ctx, st->r2c_out, st->r2c_in, sizeof(ftype));

    {
        ctype *in = st->r2c_out;
        ctype *out = st->c2r_in;

        for (int n = 0; n < nb_coeffs; n++) {
            ftype *h = st->history + n * stride;
            ftype p_min = FLT_MAX;

            p_sn[n] = fn(sqr)(in[n].re) + fn(sqr)(in[n].im);
            h[idx] = p_sn[n];

            for (int i = 0; i < stride; i++)
                p_min = FMIN(p_min, h[i]);

            p_n[n] = p_min;
        }

        for (int n = 0; n < nb_coeffs; n++) {
            ftype g = FMAX(FFMAX(p_sn[n]-beta*p_n[n], F(0.0))/(p_sn[n] + EPS), gmin);

            out[n].re = in[n].re * g;
            out[n].im = in[n].im * g;
        }
    }

    st->itx_fn(st->itx_ctx, st->c2r_out, st->c2r_in, sizeof(ctype));

    fn(apply_window)(s, st->c2r_out, st->output, 1);

    if (ff_filter_disabled(ctx) || bypass)
        if (src)
            memcpy(dst, src, sizeof(*dst) * overlap);
        else
            memset(dst, 0, sizeof(*dst) * overlap);
    else
        memcpy(dst, st->output, sizeof(*dst) * overlap);

    st->idx++;
    if (st->idx >= stride)
        st->idx = 0;
}

static int fn(spectral_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, int ch)
{
    AudioSpectralSubtractionContext *s = ctx->priv;
    const int overlap = s->overlap;

    for (int offset = 0; offset < out->nb_samples; offset += overlap) {
        const ftype *src = ((const ftype *)in->extended_data[ch])+offset;
        ftype *dst = ((ftype *)out->extended_data[ch])+offset;

        fn(feed)(ctx, ch, src, dst, FFMIN(in->nb_samples - offset, overlap));
    }

    return 0;
}

static int fn(spectral_flush_channel)(AVFilterContext *ctx, AVFrame *out, int ch)
{
    AudioSpectralSubtractionContext *s = ctx->priv;
    const int overlap = s->overlap;

    for (int offset = 0; offset < out->nb_samples; offset += overlap) {
        ftype *dst = ((ftype *)out->extended_data[ch])+offset;

        fn(feed)(ctx, ch, NULL, dst, FFMIN(out->nb_samples - offset, overlap));
    }

    return 0;
}
