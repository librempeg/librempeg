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

typedef struct fn(StateContext) {
    ftype u, v;
    ftype k1, k2;
    ftype amplitude;
    ftype offset;
    ftype old_freq;
} fn(StateContext);

static int fn(init_state)(AVFilterContext *ctx)
{
    TremoloContext *s = ctx->priv;
    const double offset = 1. - s->depth * 0.5;
    fn(StateContext) *stc;

    s->st = av_calloc(s->nb_channels, sizeof(*stc));
    if (!s->st)
        return AVERROR(ENOMEM);

    stc = s->st;
    for (int ch = 0; ch < s->nb_channels; ch++) {
        fn(StateContext) *st = &stc[ch];
        const double freq = s->freq / ctx->inputs[0]->sample_rate;
        double w0 = fmin(freq, 0.49999) * 2.0*M_PI;

        st->offset = offset;
        st->amplitude = s->depth * 0.5;
        st->k1 = tan(w0*0.5);
        st->k2 = sin(w0);
        if (st->u == st->v && st->u == F(0.0)) {
            st->u = F(1.0);
            st->v = F(0.0);
        }
        st->old_freq = s->freq;
    }

    return 0;
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    TremoloContext *s = ctx->priv;
    AVFrame *in = s->in;
    AVFrame *out = arg;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = in->nb_samples;
    fn(StateContext) *stc = s->st;

    for (int ch = start; ch < end; ch++) {
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        fn(StateContext) *st = &stc[ch];
        const ftype offset = st->offset;
        const ftype a = st->amplitude;
        ftype u = st->u, v = st->v, w;
        const ftype k1 = st->k1;
        const ftype k2 = st->k2;

        for (int n = 0; n < nb_samples; n++) {
            dst[n] = src[n] * (u * a + offset);
            w = u - k1 * v;
            v += k2 * w;
            u = w - k1 * v;
        }

        st->u = u;
        st->v = v;
    }

    return 0;
}
