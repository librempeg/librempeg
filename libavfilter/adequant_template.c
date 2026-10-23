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

#undef CLIP
#undef FMAX
#undef SQRT
#undef ftype
#undef ctype
#undef MPI
#undef SAMPLE_FORMAT
#undef TX_TYPE
#if DEPTH == 32
#define MPI M_PIf
#define CLIP av_clipf
#define FMAX fmaxf
#define SQRT sqrtf
#define ftype float
#define ctype AVComplexFloat
#define SAMPLE_FORMAT fltp
#define TX_TYPE AV_TX_FLOAT_RDFT
#else
#define MPI M_PI
#define CLIP av_clipd
#define FMAX fmax
#define SQRT sqrt
#define ftype double
#define ctype AVComplexDouble
#define SAMPLE_FORMAT dblp
#define TX_TYPE AV_TX_DOUBLE_RDFT
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    int32_t *in_frame;
    ftype *dequant_frame;

    ftype  *lo;
    ftype  *hi;
    ftype  *x;
    ftype  *y;
    ftype  *fr;
    ctype  *spec;

    ftype  sig;

    AVTXContext *tx, *itx;
    av_tx_fn     tx_fn, itx_fn;
} fn(StateContext);

static void fn(adequant_uninit)(AVFilterContext *ctx)
{
    ADequantContext *s = ctx->priv;
    fn(StateContext) *stc = s->state;

    if (stc) {
        for (int ch = 0; ch < s->nb_channels; ch++) {
            fn(StateContext) *sc = &stc[ch];

            av_freep(&sc->in_frame);
            av_freep(&sc->dequant_frame);
            av_freep(&sc->lo);
            av_freep(&sc->hi);
            av_freep(&sc->x);
            av_freep(&sc->y);
            av_freep(&sc->fr);
            av_freep(&sc->spec);

            av_tx_uninit(&sc->tx);
            av_tx_uninit(&sc->itx);
        }

        av_freep(&s->state);
    }

    av_freep(&s->window);
}

static int fn(adequant_init)(AVFilterContext *ctx)
{
    ADequantContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    fn(StateContext) *stc;
    const int N = s->block;
    ftype scale = 1.0 / N;
    ftype *window;
    int ret;

    s->window = av_calloc(N, sizeof(ftype));
    if (!s->window)
        return AVERROR(ENOMEM);
    window = s->window;

    s->sq = 0.0;
    for (int n = 0; n < N; n++) {
        window[n] = sin(MPI*(n+0.5)/N);
        s->sq += window[n] * window[n];
    }

    s->state = av_calloc(inlink->ch_layout.nb_channels, sizeof(*stc));
    if (!s->state)
        return AVERROR(ENOMEM);
    stc = s->state;
    s->nb_channels = inlink->ch_layout.nb_channels;

    for (int ch = 0; ch < s->nb_channels; ch++) {
        fn(StateContext) *sc = &stc[ch];

        sc->in_frame = av_calloc(N * 2, sizeof(*sc->in_frame));
        sc->dequant_frame = av_calloc(N * 2, sizeof(*sc->dequant_frame));
        if (!sc->in_frame || !sc->dequant_frame)
            return AVERROR(ENOMEM);

        sc->lo   = av_calloc(N, sizeof(*sc->lo));
        sc->hi   = av_calloc(N, sizeof(*sc->hi));
        sc->x    = av_calloc(N, sizeof(*sc->x));
        sc->y    = av_calloc(N/2+1, sizeof(*sc->y));
        sc->fr   = av_calloc(N, sizeof(*sc->fr));
        sc->spec = av_calloc(N/2+1, sizeof(*sc->spec));
        if (!sc->lo || !sc->hi || !sc->x || !sc->y || !sc->fr || !sc->spec)
            return AVERROR(ENOMEM);

        ret = av_tx_init(&sc->tx, &sc->tx_fn, TX_TYPE, 0, N, NULL, 0);
        if (ret < 0)
            return ret;

        ret = av_tx_init(&sc->itx, &sc->itx_fn, TX_TYPE, 1, N, &scale, 0);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int fn(detect_block)(const int32_t *v, const int N, int native_k, ftype tol)
{
    int hist[33] = { 0 };
    int nz = 0, cnt = 0;

    for (int n = 0; n < N; n++) {
        if (v[n]) {
            hist[ff_ctz(v[n])]++;
            nz++;
        }
    }

    if (nz < MIN_NONZERO)
        return -1;

    for (int k = 31; k > native_k; k--) {
        cnt += hist[k];
        if (cnt >= tol * nz)
            return k;
    }

    return native_k;
}

static void fn(reconstruct)(ADequantContext *s, const int ch, const int N)
{
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    const ftype *win = s->window;
    const ftype *lo = stc->lo;
    const ftype *hi = stc->hi;
    ctype *spec = stc->spec;
    const int N_2 = N / 2;
    ftype *fr = stc->fr;
    ftype *x = stc->x;

    for (int n = 0; n < N; n++)
        fr[n] = x[n] * win[n];

    stc->tx_fn(stc->tx, spec, fr, sizeof(ftype));

    ftype npow = s->alpha * stc->sig * stc->sig * s->sq;

    for (int k = 0; k <= N_2; k++) {
        ftype re = spec[k].re, im = spec[k].im;
        ftype P = re * re + im * im;
        ftype g = FMAX(P - npow, F(0.0)) / (P + 1e-30);

        spec[k].re = re * g;
        spec[k].im = im * g;
    }

    stc->itx_fn(stc->itx, fr, spec, sizeof(ctype));

    for (int n = 0; n < N; n++)
        x[n] = fr[n] / win[n];

    for (int n = 0; n < N; n++)
        x[n] = CLIP(x[n], lo[n], hi[n]);
}

static void fn(dequantize_window)(AVFilterContext *ctx, const int ch, int N,
                                  ftype *dst, const int32_t *src)
{
    const int is_disabled = ff_filter_disabled(ctx);
    ADequantContext *s = ctx->priv;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    const int native_k = s->native;
    const int mode = s->mode;
    ftype *lo = stc->lo;
    ftype *hi = stc->hi;
    ftype *x = stc->x;
    int any = 0;
    int k = s->bits ? 32 - s->bits : 32 - fn(detect_block)(src, N, 32 - native_k, s->tol);

    if (k < native_k) {
        ftype delta = ldexp(1.0, -k+1);

        if (mode == MODE_ROUND) {
            const ftype hdelta = 0.5 * delta;

            for (int n = 0; n < N; n++) {
                ftype x0 = src[n] * (1.0 / 2147483648.0);

                lo[n] = x0 - hdelta;
                hi[n] = x0 + hdelta;
                x[n] = x0;
            }
        } else {
            for (int n = 0; n < N; n++) {
                ftype x0 = src[n] * (1.0 / 2147483648.0);

                lo[n] = x0;
                hi[n] = x0 + delta;
                x[n] = x0;
            }
        }

        stc->sig = delta * INV_SQRT12;
        any |= 1;
    } else {
        for (int n = 0; n < N; n++) {
            ftype x0 = src[n] * (1.0 / 2147483648.0);
            lo[n] = hi[n] = x[n] = x0;
        }

        stc->sig = 0.0;
    }

    if (any)
        fn(reconstruct)(s, ch, N);

    if (!is_disabled) {
        for (int n = 0; n < N; n++)
            dst[n] = x[n];
    }
}

static void fn(get_output)(AVFilterContext *ctx, const int N, int32_t *dst, const ftype *src)
{
    ADequantContext *s = ctx->priv;
    const int outbits = s->outbits;
    const ftype scale = ldexp(1.0, outbits - 1);
    const int64_t qmax = ((int64_t)1 << (outbits - 1)) - 1;
    const int64_t qmin = -((int64_t)1 << (outbits - 1));

    for (int n = 0; n < N; n++) {
        int64_t q = lrint(src[n] * scale);
        q = av_clip64(q, qmin, qmax);
        dst[n] = (int32_t)(q * ((int64_t)1 << (32 - outbits)));
    }
}

static int fn(adequant_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch)
{
    const int is_disabled = ff_filter_disabled(ctx);
    const int32_t *in_samples = (const int32_t *)in->extended_data[ch];
    int32_t *out_samples = (int32_t *)out->extended_data[ch];
    ADequantContext *s = ctx->priv;
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    const int N = s->rdft_size;
    const int overlap = s->overlap;
    const int nb_samples = FFMIN(in->nb_samples, overlap);
    const int offset = N - overlap;

    memmove(stc->in_frame, stc->in_frame + overlap, offset * sizeof(*stc->in_frame));
    memcpy(stc->in_frame + offset, in_samples, sizeof(*stc->in_frame) * nb_samples);

    fn(dequantize_window)(ctx, ch, N, stc->dequant_frame, stc->in_frame);

    if (is_disabled) {
        const int nb_samples = FFMIN(out->nb_samples, overlap);

        for (int n = 0; n < nb_samples; n++)
            out_samples[n] = in_samples[n];
    } else {
        fn(get_output)(ctx, out->nb_samples, out_samples, stc->dequant_frame + overlap/2);
    }

    return 0;
}
