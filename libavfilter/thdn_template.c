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

#undef FSQRT
#undef ftype
#undef ctype
#undef TX_TYPE
#undef EPSILON
#undef SAMPLE_FORMAT
#if DEPTH == 64
#define FSQRT sqrt
#define ctype AVComplexDouble
#define ftype double
#define SAMPLE_FORMAT dblp
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define EPSILON DBL_EPSILON
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define SQR(x) ((x)*(x))
#define SQRA(x, y) (SQR(x) + SQR(y))

typedef struct fn(StateContext) {
    AVTXContext *tx_ctx;
    av_tx_fn tx_fn;

    ftype thdn;
    int pos;

    ftype *input;
    ctype *output;
} fn(StateContext);

static int fn(init)(AVFilterContext *ctx)
{
    THDNContext *s = ctx->priv;
    fn(StateContext) *state;

    if (!s->state)
        s->state = av_calloc(s->channels, sizeof(*state));
    if (!s->state)
        return AVERROR(ENOMEM);
    state = s->state;

    s->window = av_calloc(s->rdft_size, sizeof(ftype));
    if (!s->window)
        return AVERROR(ENOMEM);

    {
        const int N = s->rdft_size;
        const ftype beta = F(20.0 * M_PI);
        const ftype wscale = av_bessel_i0(beta);
        const ftype x = F(2.0) / N;
        ftype *win = s->window;

        for (int n = 0; n < N; n++)
            win[n] = av_bessel_i0(beta * FSQRT(F(1.0) - SQR(n * x - F(1.0)))) / wscale;
    }

    for (int ch = 0; ch < s->channels; ch++) {
        fn(StateContext) *stc = &state[ch];
        ftype scale = F(1.0);
        int ret;

        ret = av_tx_init(&stc->tx_ctx, &stc->tx_fn, TX_TYPE, 0, s->rdft_size, &scale, 0);
        if (ret < 0)
            return ret;

        stc->input = av_calloc(s->rdft_size, sizeof(*stc->input));
        if (!stc->input)
            return AVERROR(ENOMEM);

        stc->output = av_calloc(s->rdft_size/2+1, sizeof(*stc->output));
        if (!stc->output)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static void fn(uninit)(AVFilterContext *ctx)
{
    THDNContext *s = ctx->priv;
    fn(StateContext) *state = s->state;

    for (int ch = 0; ch < s->channels && state; ch++) {
        fn(StateContext) *stc = &state[ch];

        av_freep(&stc->input);
        av_freep(&stc->output);
        av_tx_uninit(&stc->tx_ctx);

        av_log(ctx, AV_LOG_INFO, "THD+N ch%d: %.8f%% (%g dB)\n", ch, stc->thdn * 100.0, F(20.0) * log10(stc->thdn));
    }

    av_freep(&s->window);
    av_freep(&s->state);
}

static int fn(thdn_channel)(AVFilterContext *ctx, AVFrame *in, const int ch)
{
    THDNContext *s = ctx->priv;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    const ftype *src = (const ftype *)in->extended_data[ch];
    const int nb_samples = FFMIN(s->rdft_size - stc->pos, in->nb_samples);

    if (nb_samples > 0) {
        memcpy(stc->input + stc->pos, in->extended_data[ch], nb_samples * sizeof(*src));
        stc->pos += nb_samples;
    }

    if (stc->pos == s->rdft_size) {
        const ftype *win = s->window;
        ftype *input = stc->input;

        for (int n = 0; n < s->rdft_size; n++)
            input[n] *= win[n];

        stc->tx_fn(stc->tx_ctx, stc->output,  stc->input,  sizeof(ftype));

        ctype *out = stc->output;
        const int N = s->rdft_size/2 + 1;
        ftype te = EPSILON;
        ftype fe = F(0.0);
        ftype fs = F(0.0);
        int index = 0;

        for (int n = 0; n < N; n++) {
            ftype v = SQRA(out[n].re, out[n].im);

            if (v > fe) {
                fe = v;
                index = n;
            }
            te += v;
        }
        te += EPSILON;

        for (int n = index; n >= 0; n--) {
            ftype v = SQRA(out[n].re, out[n].im);

            if (v > fe)
                break;

            fs += v;
            fe = v;
        }

        fe = SQRA(out[index].re, out[index].im);
        for (int n = index+1; n < N; n++) {
            ftype v = SQRA(out[n].re, out[n].im);

            if (v > fe)
                break;

            fs += v;
            fe = v;
        }

        fs = te - fs;
        fs = FSQRT(fs / N);
        te = FSQRT(te / N);

        stc->thdn = fmax(fs, 0.0) / te;
        stc->pos++;
    }

    return 0;
}
