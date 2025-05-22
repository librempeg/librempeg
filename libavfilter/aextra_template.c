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

#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(ExtraContext) {
    int cache_pos;
    ftype *cache;
    double lpc[64];
    double ac[64+1];
} fn(ExtraContext);

static void fn(uninit_aextra)(AVFilterContext *ctx)
{
    AudioExtraContext *s = ctx->priv;
    fn(ExtraContext) *stc = s->st;

    for (int ch = 0; ch < s->nb_channels && stc; ch++) {
        fn(ExtraContext) *st = &stc[ch];

        av_freep(&st->cache);
    }

    av_freep(&s->st);
}

static int fn(init_aextra)(AVFilterContext *ctx)
{
    AudioExtraContext *s = ctx->priv;
    fn(ExtraContext) *stc;

    s->st = av_calloc(s->nb_channels, sizeof(*stc));
    if (!s->st)
        return AVERROR(ENOMEM);
    stc = s->st;

    for (int ch = 0; ch < s->nb_channels; ch++) {
        fn(ExtraContext) *st = &stc[ch];

        st->cache = av_calloc(s->nb_samples, sizeof(*st->cache));
        if (!st->cache)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static void fn(apply_window)(ftype *data, const int N)
{
    const ftype n_2 = F(N+1)/F(2.0);

    for (int n = 0; n < N; n++) {
        ftype k = (F(n+1.0) - n_2) / n_2;

        data[n] *= F(1.0) - k * k;
    }
}

static void fn(autocorr)(const ftype *data, const int N, double *ac, const int m)
{
    for (int j = 0; j <= m; j++) {
        double d = 0.0;

        for (int n = j; n < N; n++)
            d += (double)data[n] * (double)data[n-j];

        ac[j] = d;
    }
}

static int fn(do_lpc)(const double *ac, double *lpc, const int lpc_order)
{
    double r, error, epsilon;
    int max_order = lpc_order;

    error = ac[0] * (1.+1e-10);
    epsilon = 1e-9 * ac[0] + 1e-10;

    for (int i = 0; i < lpc_order; i++) {
        if (error < epsilon) {
            memset(&lpc[i], 0, (lpc_order - i) * sizeof(lpc[0]));
            max_order = i;
            break;
        }

        r = -ac[i+1];
        for (int j = 0; j < i; j++)
            r -= lpc[j] * ac[i-j];
        r /= error;

        lpc[i] = r;
        for (int j = 0; j < i/2; j++) {
            const double tmp = lpc[j];

            lpc[j    ] += r * lpc[i-1-j];
            lpc[i-1-j] += r * tmp;
        }

        if (i & 1)
            lpc[i/2] += lpc[i/2]*r;

        error *= 1.0 - r*r;
    }

    {
        const double g = F(0.999);
        double damp = g;

        for (int j = 0; j < max_order; j++) {
            lpc[j] *= damp;
            damp *= g;
        }
    }

    if (max_order == 0) {
        max_order = 1;
        lpc[0] = F(-1.0);
    }

    return max_order;
}

static void fn(extrapolate)(ftype *data0, const size_t N,
                            const int extra, const double *lpc,
                            const int O, const int dir)
{
    if (dir) {
        ftype *data = data0 - 1 + O;

        for (int n = 0; n < extra; n++) {
            ftype sum = F(0.0);

            for (int j = 0; j < O; j++)
                sum -= data[-n-j] * lpc[O-1-j];

            data[-O-n] = sum;
        }
    } else {
        ftype *data = data0 + N - O;

        for (int n = 0; n < extra; n++) {
            ftype sum = F(0.0);

            for (int j = 0; j < O; j++)
                sum -= data[n+j] * lpc[O-1-j];

            data[O+n] = sum;
        }
    }
}

static void fn(aextra_channel)(AVFilterContext *ctx, void *state, const int nb_samples,
                               const uint8_t *ssrc, uint8_t *ddst, const int ch)
{
    AudioExtraContext *s = ctx->priv;
    fn(ExtraContext) *stc = state;
    fn(ExtraContext) *st = &stc[ch];
    const ftype *restrict src = (const ftype *restrict)ssrc;
    ftype *restrict dst = (ftype *restrict)ddst;
    int pre = s->pre;
    const int extra_samples = s->nb_samples;
    const int offset = pre ? 0 : extra_samples;
    const int lpc_order = s->order;
    ftype *cache = st->cache;
    double *lpc = st->lpc;
    double *ac = st->ac;
    int order, pos = st->cache_pos;

    for (int n = 0; !s->eof && n < nb_samples; n++) {
        const ftype in = src[n];

        dst[n + offset] = in;
        cache[pos] = in;
        pos++;
        if (pos >= extra_samples)
            pos = 0;
    }

    if (!pre) {
        memset(dst, 0, extra_samples * sizeof(ftype));
        fn(apply_window)(dst+offset, nb_samples);
        fn(autocorr)(dst+offset, nb_samples, ac, lpc_order);
        order = fn(do_lpc)(ac, lpc, lpc_order);
        memcpy(dst+offset, src, nb_samples * sizeof(ftype));
        fn(extrapolate)(dst+offset, nb_samples, extra_samples, lpc, order, 1);
    } else if (!s->post && s->eof) {
        for (int i = 0; i < nb_samples; i++) {
            dst[i] = cache[pos++];
            if (pos >= nb_samples)
                pos = 0;
        }
        fn(apply_window)(dst, nb_samples);
        fn(autocorr)(dst, nb_samples, ac, lpc_order);
        order = fn(do_lpc)(ac, lpc, lpc_order);
        pos = st->cache_pos;
        for (int i = 0; i < nb_samples; i++) {
            dst[i] = cache[pos++];
            if (pos >= nb_samples)
                pos = 0;
        }
        fn(extrapolate)(dst, nb_samples, extra_samples, lpc, order, 0);
        memmove(dst, dst+nb_samples, extra_samples * sizeof(ftype));
    }

    st->cache_pos = pos;
}
