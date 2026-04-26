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

#undef TX_TYPE
#undef FABS
#undef FMAX
#undef FMIN
#undef FSIN
#undef ftype
#undef ctype
#undef FPI
#undef SAMPLE_FORMAT
#undef EPS
#if DEPTH == 32
#define FABS fabsf
#define FMAX fmaxf
#define FMIN fminf
#define FSIN sinf
#define ftype float
#define ctype AVComplexFloat
#define FPI M_PIf
#define SAMPLE_FORMAT fltp
#define EPS FLT_EPSILON
#define TX_TYPE AV_TX_FLOAT_MDCT
#else
#define FABS fabs
#define FMAX fmax
#define FMIN fmin
#define FSIN sin
#define ftype double
#define ctype AVComplexDouble
#define FPI M_PI
#define SAMPLE_FORMAT dblp
#define EPS FLT_EPSILON
#define TX_TYPE AV_TX_DOUBLE_MDCT
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ftype *prev_frame;
    ftype *lossy_frame;
    ftype *in_buffer;
    ftype *in_frame;
    ftype *out_dist_frame;
    ftype *spectrum_buf;
    ftype *windowed_frame;

    AVTXContext *tx_ctx;
    av_tx_fn tx_fn;
    AVTXContext *itx_ctx;
    av_tx_fn itx_fn;
} fn(StateContext);

static int fn(init)(AVFilterContext *ctx)
{
    AudioLossyContext *s = ctx->priv;
    fn(StateContext) *state;
    ftype iscale = F(-1.0) / (s->mdct_size / 2);
    ftype scale = F(1.0);

    s->state = av_calloc(s->channels, sizeof(*state));
    if (!s->state)
        return AVERROR(ENOMEM);
    state = s->state;

    for (int ch = 0; ch < s->channels; ch++) {
        fn(StateContext) *stc = &state[ch];
        int ret;

        stc->lossy_frame    = av_calloc(s->mdct_size * 2, sizeof(*stc->lossy_frame));
        stc->prev_frame     = av_calloc(s->mdct_size, sizeof(*stc->prev_frame));
        stc->in_buffer      = av_calloc(s->mdct_size, sizeof(*stc->in_buffer));
        stc->in_frame       = av_calloc(s->mdct_size * 2, sizeof(*stc->in_frame));
        stc->out_dist_frame = av_calloc(s->mdct_size, sizeof(*stc->out_dist_frame));
        stc->spectrum_buf   = av_calloc(s->mdct_size, sizeof(*stc->spectrum_buf));
        stc->windowed_frame = av_calloc(s->mdct_size * 2, sizeof(*stc->windowed_frame));
        if (!stc->in_buffer || !stc->in_frame || !stc->prev_frame ||
            !stc->out_dist_frame || !stc->windowed_frame ||
            !stc->lossy_frame || !stc->spectrum_buf)
            return AVERROR(ENOMEM);

        ret = av_tx_init(&stc->tx_ctx, &stc->tx_fn, TX_TYPE, 0, s->mdct_size, &scale, 0);
        if (ret < 0)
            return ret;

        ret = av_tx_init(&stc->itx_ctx, &stc->itx_fn, TX_TYPE, 1, s->mdct_size, &iscale, AV_TX_FULL_IMDCT);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static void fn(uninit)(AVFilterContext *ctx)
{
    AudioLossyContext *s = ctx->priv;

    if (s->state) {
        fn(StateContext) *state = s->state;

        for (int ch = 0; ch < s->channels; ch++) {
            fn(StateContext) *stc = &state[ch];

            av_freep(&stc->prev_frame);
            av_freep(&stc->lossy_frame);
            av_freep(&stc->in_buffer);
            av_freep(&stc->in_frame);
            av_freep(&stc->out_dist_frame);
            av_freep(&stc->spectrum_buf);
            av_freep(&stc->windowed_frame);

            av_tx_uninit(&stc->tx_ctx);
            av_tx_uninit(&stc->itx_ctx);
        }
    }

    av_freep(&s->state);
}

static void fn(generate_window)(void *_window, const int size)
{
    const int N = size;
    ftype *window = _window;

    for (int n = 0; n < N; n++)
        window[n] = FSIN(FPI*(n+F(0.5))/N);
}

static void fn(apply_window)(AudioLossyContext *s, const ftype *in_frame,
                             ftype *out_frame)
{
    const ftype *window = s->window;
    const int N = s->mdct_size * 2;

    for (int n = 0; n < N; n++)
        out_frame[n] = in_frame[n] * window[n];
}

static void fn(overlap_add)(AudioLossyContext *s, const ftype *in_frame,
                            const ftype *prev_frame, ftype *out_frame)
{
    const int N = s->overlap;

    for (int n = 0; n < N; n++)
        out_frame[n] = in_frame[n] + prev_frame[n];
}

static void fn(feed)(AVFilterContext *ctx, int ch,
                     const ftype *in_samples, ftype *out_samples,
                     ftype *in_frame, ftype *out_dist_frame,
                     ftype *windowed_frame, ftype *lossy_frame,
                     ftype *prev_frame, ftype *spectrum_buf)
{
    AudioLossyContext *s = ctx->priv;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    const ftype reduction = s->reduction * F(0.01);
    const int overlap = s->overlap;
    enum AVChannel channel = av_channel_layout_channel_from_index(&ctx->inputs[0]->ch_layout, ch);
    const int bypass = av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;
    const int size = s->mdct_size;

    memmove(in_frame, in_frame + size, sizeof(*in_frame) * size);
    memcpy(in_frame + size, in_samples, sizeof(*in_frame) * size);

    fn(apply_window)(s, in_frame, windowed_frame);

    stc->tx_fn(stc->tx_ctx, spectrum_buf, windowed_frame, sizeof(ftype));

    {
        ftype maxf = F(0.0);
        ftype minf = INT_MAX;

        for (int n = 0; n < size; n++) {
            maxf = FMAX(FABS(spectrum_buf[n]), maxf);
            minf = FMIN(FABS(spectrum_buf[n]), minf);
        }

        for (int n = 0; n < size; n++) {
            if (FABS(spectrum_buf[n]) < minf + (maxf-minf) * reduction)
                spectrum_buf[n] = F(0.0);
        }
    }

    stc->itx_fn(stc->itx_ctx, lossy_frame, spectrum_buf, sizeof(ftype));

    fn(apply_window)(s, lossy_frame, lossy_frame);
    fn(overlap_add)(s, lossy_frame, prev_frame, out_dist_frame);

    memcpy(prev_frame, lossy_frame + overlap, overlap * sizeof(*prev_frame));

    if (ff_filter_disabled(ctx) || bypass)
        memcpy(out_samples, in_frame, sizeof(*out_samples) * overlap);
    else
        memcpy(out_samples, out_dist_frame, sizeof(*out_samples) * overlap);
}

static int fn(lossy_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, int ch)
{
    AudioLossyContext *s = ctx->priv;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    ftype *in_buffer = stc->in_buffer;
    const int overlap = s->overlap;

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
                 stc->lossy_frame,
                 stc->prev_frame,
                 stc->spectrum_buf);
    }

    return 0;
}

static int fn(lossy_flush_channel)(AVFilterContext *ctx, AVFrame *out, int ch)
{
    AudioLossyContext *s = ctx->priv;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    ftype *in_buffer = stc->in_buffer;
    const int overlap = s->overlap;

    for (int offset = 0; offset < out->nb_samples; offset += overlap) {
        ftype *dst = ((ftype *)out->extended_data[ch])+offset;

        memset(in_buffer, 0, sizeof(*in_buffer) * overlap);

        fn(feed)(ctx, ch, in_buffer, dst,
                 stc->in_frame,
                 stc->out_dist_frame,
                 stc->windowed_frame,
                 stc->lossy_frame,
                 stc->prev_frame,
                 stc->spectrum_buf);
    }

    return 0;
}
