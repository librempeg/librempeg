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

#undef ctype
#undef ftype
#undef SAMPLE_FORMAT
#undef TX_TYPE
#undef EPS
#if DEPTH == 32
#define ctype AVComplexFloat
#define SAMPLE_FORMAT fltp
#define TX_TYPE AV_TX_FLOAT_RDFT
#define ftype float
#define EPS FLT_EPSILON
#else
#define ctype AVComplexDouble
#define SAMPLE_FORMAT dblp
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define ftype double
#define EPS DBL_EPSILON
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(ChannelState) {
    ftype *rdft_in[2];
    ctype *rdft_complex[2];
    ftype *rdft_out;

    AVTXContext *tx_ctx, *itx_ctx;
    av_tx_fn tx_fn, itx_fn;
} fn(ChannelState);

static int fn(acorr_init)(AVFilterContext *ctx)
{
    const int channels = ctx->inputs[0]->ch_layout.nb_channels;
    AudioCorrContext *s = ctx->priv;
    const ftype iscale = F(1.0) / s->size;
    const ftype scale = F(1.0);
    fn(ChannelState) *state;
    int ret;

    s->state = av_calloc(channels, sizeof(*state));
    if (!s->state)
        return AVERROR(ENOMEM);
    s->channels = channels;
    state = s->state;

    for (int ch = 0; ch < channels; ch++) {
        fn(ChannelState) *stc = &state[ch];

        ret = av_tx_init(&stc->tx_ctx, &stc->tx_fn, TX_TYPE, 0, s->size*2, &scale, 0);
        if (ret < 0)
            return ret;

        ret = av_tx_init(&stc->itx_ctx, &stc->itx_fn, TX_TYPE, 1, s->size*2, &iscale, 0);
        if (ret < 0)
            return ret;

        stc->rdft_in[0] = av_calloc(s->size*2, sizeof(*stc->rdft_in[0]));
        if (!stc->rdft_in[0])
            return AVERROR(ENOMEM);

        stc->rdft_in[1] = av_calloc(s->size*2, sizeof(*stc->rdft_in[1]));
        if (!stc->rdft_in[1])
            return AVERROR(ENOMEM);

        stc->rdft_complex[0] = av_calloc(s->size+1, sizeof(*stc->rdft_complex[0]));
        if (!stc->rdft_complex[0])
            return AVERROR(ENOMEM);

        stc->rdft_complex[1] = av_calloc(s->size+1, sizeof(*stc->rdft_complex[1]));
        if (!stc->rdft_complex[1])
            return AVERROR(ENOMEM);

        stc->rdft_out = av_calloc(s->size*2, sizeof(*stc->rdft_out));
        if (!stc->rdft_out)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static void fn(acorr_uninit)(AVFilterContext *ctx)
{
    AudioCorrContext *s = ctx->priv;

    if (s->state) {
        fn(ChannelState) *state = s->state;

        for (int ch = 0; ch < s->channels; ch++) {
            fn(ChannelState) *stc = &state[ch];

            av_tx_uninit(&stc->tx_ctx);
            av_tx_uninit(&stc->itx_ctx);

            av_freep(&stc->rdft_in[0]);
            av_freep(&stc->rdft_in[1]);
            av_freep(&stc->rdft_complex[0]);
            av_freep(&stc->rdft_complex[1]);
            av_freep(&stc->rdft_out);
        }
    }

    av_freep(&s->state);
}

static void fn(acorr)(AVFilterContext *ctx, AVFrame *out, const int ch)
{
    AudioCorrContext *s = ctx->priv;
    const int nb_samples = FFMIN(s->in[0]->nb_samples, s->in[1]->nb_samples);
    const ftype *inx = (const ftype *)s->in[0]->extended_data[ch];
    const ftype *iny = (const ftype *)s->in[1]->extended_data[ch];
    fn(ChannelState) *ch_state = s->state;
    fn(ChannelState) *state = &ch_state[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    ctype *cplx_x = state->rdft_complex[0];
    ctype *cplx_y = state->rdft_complex[1];
    ftype *rdft_x = state->rdft_in[0];
    ftype *rdft_y = state->rdft_in[1];
    ftype *rdft_o = state->rdft_out;
    const int N = s->size+1;
    const int S = s->size*2;
    int index = 0;
    ftype max;

    memcpy(rdft_x, inx, sizeof(*rdft_x) * nb_samples);
    memcpy(rdft_y, iny, sizeof(*rdft_y) * nb_samples);
    memset(rdft_x + nb_samples, 0, sizeof(*rdft_x) * (S-nb_samples));
    memset(rdft_y + nb_samples, 0, sizeof(*rdft_y) * (S-nb_samples));

    state->tx_fn(state->tx_ctx, cplx_x, rdft_x, sizeof(ftype));
    state->tx_fn(state->tx_ctx, cplx_y, rdft_y, sizeof(ftype));

    for (int n = 0; n < N; n++) {
        const float xre = cplx_x[n].re;
        const float xim = cplx_x[n].im;
        const float yre = cplx_y[n].re;
        const float yim = cplx_y[n].im;

        cplx_x[n].re = xre * yre + xim * yim;
        cplx_x[n].im = xre * yim - xim * yre;
    }

    state->itx_fn(state->itx_ctx, rdft_o, cplx_x, sizeof(ctype));

    max = rdft_o[0];
    for (int n = 1; n < S; n++) {
        if (rdft_o[n] > max) {
            max = rdft_o[n];
            index = n;
        }
    }

    av_log(ctx, AV_LOG_DEBUG, "ch%d: max: %d/%d\n", ch, index >= s->size ? S-index : index, s->size);

    memcpy(dst, rdft_o, sizeof(*dst) * nb_samples);
}
