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

#undef AVFILTER_WINDOW_FUNC_H
#include "window_func.h"

#undef CLIP
#undef FABS
#undef FSQRT
#undef FCBRT
#undef FPOW
#undef FLOG10
#undef FCOS
#undef FSIN
#undef ftype
#undef ctype
#undef FPI
#undef HYPOT
#undef ATAN2
#undef SAMPLE_FORMAT
#undef TX_TYPE
#undef CTX_TYPE
#undef EPS
#if DEPTH == 32
#define CLIP av_clipf
#define FABS fabsf
#define FSQRT sqrtf
#define FCBRT cbrtf
#define FPOW powf
#define FLOG10 log10f
#define FCOS cosf
#define FSIN sinf
#define ftype float
#define ctype AVComplexFloat
#define FPI M_PIf
#define HYPOT hypotf
#define ATAN2 atan2f
#define SAMPLE_FORMAT fltp
#define TX_TYPE AV_TX_FLOAT_RDFT
#define CTX_TYPE AV_TX_FLOAT_FFT
#define EPS FLT_EPSILON
#else
#define CLIP av_clipd
#define FABS fabs
#define FSQRT sqrt
#define FCBRT cbrt
#define FPOW pow
#define FLOG10 log10
#define FCOS cos
#define FSIN sin
#define ftype double
#define ctype AVComplexDouble
#define FPI M_PI
#define HYPOT hypot
#define ATAN2 atan2
#define SAMPLE_FORMAT dblp
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define CTX_TYPE AV_TX_DOUBLE_FFT
#define EPS DBL_EPSILON
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ftype **rdft_in;            ///< input RDFT coeffs
    ctype **fft_in;    ///< input FFT coeffs
    ctype **fft_data;  ///< bins holder for each (displayed) channels
    ctype **fft_scratch;///< scratch buffers
    ftype *window_func_lut;     ///< Window function LUT
    ftype **magnitudes;
    ftype **phases;

    AVTXContext **rdft;         ///< Real Discrete Fourier Transform context
    AVTXContext **fft;          ///< Fast Fourier Transform context
    AVTXContext **ifft;         ///< Inverse Fast Fourier Transform context
    av_tx_fn rtx_fn;
    av_tx_fn tx_fn;
    av_tx_fn itx_fn;
} fn(StateContext);

static void fn(uninit)(AVFilterContext *ctx)
{
    ASpectrogramContext *s = ctx->priv;
    fn(StateContext) *st;

    st = s->state;
    if (!st)
        return;

    if (st->rdft) {
        for (int i = 0; i < s->nb_display_channels; i++)
            av_tx_uninit(&st->rdft[i]);
    }
    av_freep(&st->rdft);
    if (st->fft) {
        for (int i = 0; i < s->nb_display_channels; i++)
            av_tx_uninit(&st->fft[i]);
    }
    av_freep(&st->fft);
    if (st->ifft) {
        for (int i = 0; i < s->nb_display_channels; i++)
            av_tx_uninit(&st->ifft[i]);
    }
    av_freep(&st->ifft);
    if (st->fft_data) {
        for (int i = 0; i < s->nb_display_channels; i++)
            av_freep(&st->fft_data[i]);
    }
    av_freep(&st->fft_data);
    if (st->rdft_in) {
        for (int i = 0; i < s->nb_display_channels; i++)
            av_freep(&st->rdft_in[i]);
    }
    av_freep(&st->rdft_in);
    if (st->fft_in) {
        for (int i = 0; i < s->nb_display_channels; i++)
            av_freep(&st->fft_in[i]);
    }
    av_freep(&st->fft_in);
    if (st->fft_scratch) {
        for (int i = 0; i < s->nb_display_channels; i++)
            av_freep(&st->fft_scratch[i]);
    }
    av_freep(&st->fft_scratch);
    av_freep(&st->window_func_lut);
    if (st->magnitudes) {
        for (int i = 0; i < s->nb_display_channels; i++)
            av_freep(&st->magnitudes[i]);
    }
    av_freep(&st->magnitudes);

    if (st->phases) {
        for (int i = 0; i < s->nb_display_channels; i++)
            av_freep(&st->phases[i]);
    }
    av_freep(&st->phases);

    av_freep(&s->state);
}

static int fn(init)(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    ASpectrogramContext *s = ctx->priv;
    const int fft_size = s->fft_size;
    fn(StateContext) *st;
    ftype overlap;
    int ret;

    s->state = av_calloc(1, sizeof(*st));
    if (!s->state)
        return AVERROR(ENOMEM);

    st = s->state;
    if (!st->rdft) {
        st->rdft = av_calloc(inlink->ch_layout.nb_channels, sizeof(*st->rdft));
        if (!st->rdft)
            return AVERROR(ENOMEM);
    }

    if (!st->fft) {
        st->fft = av_calloc(inlink->ch_layout.nb_channels, sizeof(*st->fft));
        if (!st->fft)
            return AVERROR(ENOMEM);
    }

    if (s->stop) {
        if (!st->ifft) {
            st->ifft = av_calloc(inlink->ch_layout.nb_channels, sizeof(*st->ifft));
            if (!st->ifft)
                return AVERROR(ENOMEM);
        }
    }

    for (int i = 0; i < s->nb_display_channels; i++) {
        ftype scale = F(1.0);

        if (s->stop) {
            ret = av_tx_init(&st->fft[i], &st->tx_fn, CTX_TYPE, 0, fft_size << (!!s->stop), &scale, 0);
            if (ret < 0)
                return ret;
            ret = av_tx_init(&st->ifft[i], &st->itx_fn, CTX_TYPE, 1, fft_size << (!!s->stop), &scale, 0);
        } else {
            ret = av_tx_init(&st->rdft[i], &st->rtx_fn, TX_TYPE, 0, fft_size, &scale, 0);
        }
        if (ret < 0)
            return ret;
    }

    st->magnitudes = av_calloc(s->nb_display_channels, sizeof(*st->magnitudes));
    if (!st->magnitudes)
        return AVERROR(ENOMEM);
    for (int i = 0; i < s->nb_display_channels; i++) {
        st->magnitudes[i] = av_calloc(s->orientation == VERTICAL ? s->h : s->w, sizeof(**st->magnitudes));
        if (!st->magnitudes[i])
            return AVERROR(ENOMEM);
    }

    st->phases = av_calloc(s->nb_display_channels, sizeof(*st->phases));
    if (!st->phases)
        return AVERROR(ENOMEM);
    for (int i = 0; i < s->nb_display_channels; i++) {
        st->phases[i] = av_calloc(s->orientation == VERTICAL ? s->h : s->w, sizeof(**st->phases));
        if (!st->phases[i])
            return AVERROR(ENOMEM);
    }

    st->rdft_in = av_calloc(s->nb_display_channels, sizeof(*st->rdft_in));
    if (!st->rdft_in)
        return AVERROR(ENOMEM);
    st->fft_in = av_calloc(s->nb_display_channels, sizeof(*st->fft_in));
    if (!st->fft_in)
        return AVERROR(ENOMEM);
    st->fft_data = av_calloc(s->nb_display_channels, sizeof(*st->fft_data));
    if (!st->fft_data)
        return AVERROR(ENOMEM);
    st->fft_scratch = av_calloc(s->nb_display_channels, sizeof(*st->fft_scratch));
    if (!st->fft_scratch)
        return AVERROR(ENOMEM);
    for (int i = 0; i < s->nb_display_channels; i++) {
        st->rdft_in[i] = av_calloc(s->buf_size, sizeof(**st->rdft_in));
        if (!st->rdft_in[i])
            return AVERROR(ENOMEM);

        st->fft_in[i] = av_calloc(s->buf_size, sizeof(**st->fft_in));
        if (!st->fft_in[i])
            return AVERROR(ENOMEM);

        st->fft_data[i] = av_calloc(s->buf_size, sizeof(**st->fft_data));
        if (!st->fft_data[i])
            return AVERROR(ENOMEM);

        st->fft_scratch[i] = av_calloc(s->buf_size, sizeof(**st->fft_scratch));
        if (!st->fft_scratch[i])
            return AVERROR(ENOMEM);
    }

    /* pre-calc windowing function */
    st->window_func_lut = av_realloc_f(st->window_func_lut, s->win_size, sizeof(*st->window_func_lut));
    if (!st->window_func_lut)
        return AVERROR(ENOMEM);
    fun(generate_window_func)(st->window_func_lut, s->win_size, s->win_func, &overlap);
    if (s->overlap == 1)
        s->overlap = overlap;

    s->win_scale = 0;
    for (int i = 0; i < s->win_size; i++)
        s->win_scale += st->window_func_lut[i] * st->window_func_lut[i];
    s->win_scale = F(1.0) / FSQRT(s->win_scale);

    return 0;
}

static int fn(run_channel_fft)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ASpectrogramContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    fn(StateContext) *st = s->state;
    const ftype *window_func_lut = st->window_func_lut;
    AVFrame *fin = arg;
    const int ch = jobnr;

    /* fill FFT input with the number of samples available */
    const ftype *p = (ftype *)fin->extended_data[ch];
    ftype *in_frame = (ftype *)s->in_frame->extended_data[ch];

    memmove(in_frame, in_frame + s->hop_size, (s->fft_size - s->hop_size) * sizeof(ftype));
    memcpy(in_frame + s->fft_size - s->hop_size, p, fin->nb_samples * sizeof(ftype));

    for (int i = fin->nb_samples; i < s->hop_size; i++)
        in_frame[i + s->fft_size - s->hop_size] = F(0.0);

    if (s->stop) {
        ftype theta, phi, psi, a, b, S, c;
        ctype *f = st->fft_in[ch];
        ctype *g = st->fft_data[ch];
        ctype *h = st->fft_scratch[ch];
        int L = s->buf_size;
        int N = s->win_size;
        int M = s->win_size / 2;

        for (int n = 0; n < s->win_size; n++) {
            st->fft_data[ch][n].re = in_frame[n] * window_func_lut[n];
            st->fft_data[ch][n].im = 0;
        }

        phi = F(2.0) * M_PI * (s->stop - s->start) / (ftype)inlink->sample_rate / (M - 1);
        theta = F(2.0) * M_PI * s->start / (ftype)inlink->sample_rate;

        for (int n = 0; n < M; n++) {
            h[n].re = FCOS(n * n / 2.f * phi);
            h[n].im = FSIN(n * n / 2.f * phi);
        }

        for (int n = M; n < L; n++) {
            h[n].re = F(0.0);
            h[n].im = F(0.0);
        }

        for (int n = L - N; n < L; n++) {
            h[n].re = FCOS((L - n) * (L - n) / 2.f * phi);
            h[n].im = FSIN((L - n) * (L - n) / 2.f * phi);
        }

        for (int n = N; n < L; n++) {
            g[n].re = F(0.0);
            g[n].im = F(0.0);
        }

        for (int n = 0; n < N; n++) {
            psi = n * theta + n * n / 2.f * phi;
            c =  FCOS(psi);
            S = -FSIN(psi);
            a = c * g[n].re - S * g[n].im;
            b = S * g[n].re + c * g[n].im;
            g[n].re = a;
            g[n].im = b;
        }

        memcpy(f, h, s->buf_size * sizeof(*f));
        st->tx_fn(st->fft[ch], h, f, sizeof(ctype));

        memcpy(f, g, s->buf_size * sizeof(*f));
        st->tx_fn(st->fft[ch], g, f, sizeof(ctype));

        for (int n = 0; n < L; n++) {
            c = g[n].re;
            S = g[n].im;
            a = c * h[n].re - S * h[n].im;
            b = S * h[n].re + c * h[n].im;

            g[n].re = a / L;
            g[n].im = b / L;
        }

        memcpy(f, g, s->buf_size * sizeof(*f));
        st->itx_fn(st->ifft[ch], g, f, sizeof(ctype));

        for (int k = 0; k < M; k++) {
            psi = k * k / F(2.0) * phi;
            c =  FCOS(psi);
            S = -FSIN(psi);
            a = c * g[k].re - S * g[k].im;
            b = S * g[k].re + c * g[k].im;
            st->fft_data[ch][k].re = a;
            st->fft_data[ch][k].im = b;
        }
    } else {
        const int win_size = s->win_size;
        ftype *dst = st->rdft_in[ch];

        for (int n = 0; n < win_size; n++)
            dst[n] = in_frame[n] * window_func_lut[n];

        st->rtx_fn(st->rdft[ch], st->fft_data[ch], st->rdft_in[ch], sizeof(*dst));
    }

    return 0;
}

static ftype fn(get_scale)(AVFilterContext *ctx, int scale, ftype a)
{
    ASpectrogramContext *s = ctx->priv;
    const ftype dmin = s->dmin;
    const ftype dmax = s->dmax;

    a = CLIP(a, dmin, dmax);
    if (scale != LOG)
        a = (a - dmin) / (dmax - dmin);

    switch (scale) {
    case LINEAR:
        break;
    case SQRT:
        a = FSQRT(a);
        break;
    case CBRT:
        a = FCBRT(a);
        break;
    case FOURTHRT:
        a = FSQRT(FSQRT(a));
        break;
    case FIFTHRT:
        a = FPOW(a, F(0.2));
        break;
    case LOG:
        a = (s->drange - s->limit + FLOG10(a) * F(20.0)) / s->drange;
        break;
    default:
        av_assert0(0);
    }

    return a;
}

static float fn(get_value)(AVFilterContext *ctx, int ch, int y)
{
    ASpectrogramContext *s = ctx->priv;
    fn(StateContext) *st = s->state;
    ftype *magnitudes = st->magnitudes[ch];
    ftype *phases = st->phases[ch];
    ftype a;

    switch (s->data) {
    case D_MAGNITUDE:
        /* get magnitude */
        a = magnitudes[y];
        break;
    case D_UPHASE:
    case D_PHASE:
        /* get phase */
        a = phases[y];
        break;
    default:
        av_assert0(0);
    }

    return CLIP(fn(get_scale)(ctx, s->scale, a), F(0.0), F(1.0));
}

static int fn(plot_channel_lin)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ASpectrogramContext *s = ctx->priv;
    const int h = s->orientation == VERTICAL ? s->channel_height : s->channel_width;
    const int ch = jobnr;
    float yf, uf, vf;
    int y;

    /* decide color range */
    color_range(s, ch, &yf, &uf, &vf);

    /* draw the channel */
    for (y = 0; y < h; y++) {
        int row = (s->mode == COMBINED) ? y : ch * h + y;
        float *out = &s->color_buffer[ch][4 * row];
        float a = fn(get_value)(ctx, ch, y);

        pick_color(s, yf, uf, vf, a, out);
    }

    return 0;
}

static int fn(plot_channel_log)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ASpectrogramContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    const int h = s->orientation == VERTICAL ? s->channel_height : s->channel_width;
    const int ch = jobnr;
    float yf, uf, vf;

    /* decide color range */
    color_range(s, ch, &yf, &uf, &vf);

    /* draw the channel */
    for (int yy = 0; yy < h; yy++) {
        float range = s->stop ? s->stop - s->start : inlink->sample_rate / 2;
        float pos = bin_pos(yy, h, s->start, s->start + range);
        float delta = pos - floorf(pos);
        float a0, a1;

        a0 = fn(get_value)(ctx, ch, av_clip(pos, 0, h-1));
        a1 = fn(get_value)(ctx, ch, av_clip(pos+1, 0, h-1));
        {
            int row = (s->mode == COMBINED) ? yy : ch * h + yy;
            float *out = &s->color_buffer[ch][4 * row];

            pick_color(s, yf, uf, vf, delta * a1 + (1.f - delta) * a0, out);
        }
    }

    return 0;
}

#define RE(y) fft_data[y].re
#define IM(y) fft_data[y].im
#define MAGNITUDE(y) HYPOT(RE(y), IM(y))
#define PHASE(y) ATAN2(IM(y), RE(y))

static int fn(calc_channel_magnitudes)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ASpectrogramContext *s = ctx->priv;
    const double w = s->win_scale * (s->scale == LOG ? s->win_scale : 1);
    const int apodization = s->win_func == WFUNC_RECT;
    int h = s->orientation == VERTICAL ? s->h : s->w;
    fn(StateContext) *st = s->state;
    const ftype f = s->gain * w;
    const int ch = jobnr;
    const ctype *fft_data = st->fft_data[ch];
    ftype *magnitudes = st->magnitudes[ch];

    if (apodization) {
        ctype num, den, ret;
        ftype beta;

        for (int y = 1; y < h-1; y++) {

            num.re = F(2.0) * RE(y);
            num.im = F(2.0) * IM(y);

            den.re = RE(y-1) + RE(y+1);
            den.im = IM(y-1) + IM(y+1);

            beta = F(0.5) * CLIP((num.re * den.re + num.im * den.im) / (den.re * den.re + den.im * den.im + EPS), F(0.0), F(1.0));

            ret.re = RE(y) - beta * den.re;
            ret.im = IM(y) - beta * den.im;

            magnitudes[y] = HYPOT(ret.re, ret.im) * f;
        }
    } else {
        for (int y = 0; y < h; y++)
            magnitudes[y] = MAGNITUDE(y) * f;
    }

    return 0;
}

static int fn(calc_channel_phases)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ASpectrogramContext *s = ctx->priv;
    const int h = s->orientation == VERTICAL ? s->h : s->w;
    fn(StateContext) *st = s->state;
    const int ch = jobnr;
    const ctype *fft_data = st->fft_data[ch];
    ftype *phases = st->phases[ch];

    for (int y = 0; y < h; y++)
        phases[y] = (PHASE(y) / M_PI + 1) / 2;

    return 0;
}

static void fn(unwrap)(ftype *x, int N, ftype tol, ftype *mi, ftype *ma)
{
    const ftype rng = 2.f * M_PI;
    ftype prev_p = 0.f;
    ftype max = -FLT_MAX;
    ftype min = FLT_MAX;

    for (int i = 0; i < N; i++) {
        const ftype d = x[FFMIN(i + 1, N)] - x[i];
        const ftype p = ceilf(FABS(d) / rng) * rng * (((d < tol) > 0.f) - ((d > -tol) > 0.f));

        x[i] += p + prev_p;
        prev_p += p;
        max = fmaxf(x[i], max);
        min = fminf(x[i], min);
    }

    *mi = min;
    *ma = max;
}

static int fn(calc_channel_uphases)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ASpectrogramContext *s = ctx->priv;
    const int h = s->orientation == VERTICAL ? s->h : s->w;
    fn(StateContext) *st = s->state;
    const int ch = jobnr;
    const ctype *fft_data = st->fft_data[ch];
    ftype *phases = st->phases[ch];
    ftype min, max, scale;

    for (int y = 0; y < h; y++)
        phases[y] = PHASE(y);
    fn(unwrap)(phases, h, M_PI, &min, &max);
    scale = 1.f / (max - min + FLT_MIN);
    for (int y = 0; y < h; y++)
        phases[y] = FABS((phases[y] - min) * scale);

    return 0;
}

static void fn(acalc_magnitudes)(ASpectrogramContext *s)
{
    const ftype w = s->win_scale * (s->scale == LOG ? s->win_scale : 1);
    const int h = s->orientation == VERTICAL ? s->h : s->w;
    fn(StateContext) *st = s->state;
    const ftype f = s->gain * w;

    for (int ch = 0; ch < s->nb_display_channels; ch++) {
        const ctype *fft_data = st->fft_data[ch];
        ftype *magnitudes = st->magnitudes[ch];

        for (int y = 0; y < h; y++)
            magnitudes[y] += MAGNITUDE(y) * f;
    }
}

static void fn(scale_magnitudes)(ASpectrogramContext *s, double scale)
{
    const int h = s->orientation == VERTICAL ? s->h : s->w;
    fn(StateContext) *st = s->state;

    for (int ch = 0; ch < s->nb_display_channels; ch++) {
        ftype *magnitudes = st->magnitudes[ch];

        for (int y = 0; y < h; y++)
            magnitudes[y] *= scale;
    }
}

static void fn(clear_magnitudes)(AVFilterContext *ctx)
{
    ASpectrogramContext *s = ctx->priv;
    const int h = s->orientation == VERTICAL ? s->h : s->w;
    fn(StateContext) *st = s->state;

    for (int ch = 0; ch < s->nb_display_channels; ch++)
        memset(st->magnitudes[ch], 0, h * sizeof(ftype));
}
