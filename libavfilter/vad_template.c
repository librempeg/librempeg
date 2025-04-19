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

#undef EPS
#undef FABS
#undef FEXP2
#undef FLOG2
#undef FMIN
#undef FTAN
#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define EPS FLT_EPSILON
#define FABS fabsf
#define FEXP2 exp2f
#define FLOG2 log2f
#define FMIN fminf
#define FTAN tanf
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define EPS FLT_EPSILON
#define FABS fabs
#define FEXP2 exp2
#define FLOG2 log2
#define FMIN fmin
#define FTAN tan
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define LIN2LOG(x) (F(6.0206) * FLOG2((x) + EPS))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ftype a[3], m[3], b0[2], b1[2];
} fn(StateContext);

static int fn(init_state)(AVFilterContext *ctx)
{
    VADContext *s = ctx->priv;
    fn(StateContext) *stc;

    s->st = av_calloc(s->nb_channels, sizeof(*stc));
    if (!s->st)
        return AVERROR(ENOMEM);

    stc = s->st;
    for (int ch = 0; ch < s->nb_channels; ch++) {
        const int sample_rate = ctx->inputs[0]->sample_rate;
        fn(StateContext) *st = &stc[ch];
        const ftype g = FTAN(F(M_PI) * FMIN(s->freq / sample_rate, F(0.4999999)));
        const ftype qfactor = s->qf;
        ftype ka = F(1.0) / qfactor;
        ftype *a = st->a;
        ftype *m = st->m;

        a[0] = F(1.0) / (F(1.0) + g * (g + ka));
        a[1] = g * a[0];
        a[2] = g * a[1];

        m[0] = F(0.0);
        m[1] = F(0.0);
        m[2] = F(1.0);
    }

    return 0;
}

static ftype fn(svf_low)(ftype in, const ftype m[3],
                         const ftype a[3], ftype b[2])
{
    const ftype v0 = in;
    const ftype v3 = v0 - b[1];
    const ftype v1 = a[0] * b[0] + a[1] * v3;
    const ftype v2 = b[1] + a[1] * b[0] + a[2] * v3;

    b[0] = F(2.0) * v1 - b[0];
    b[1] = F(2.0) * v2 - b[1];

    return m[2] * v2;
}

static void fn(update_state)(ftype *b)
{
    b[0] = isnormal(b[0]) ? b[0] : F(0.0);
    b[1] = isnormal(b[1]) ? b[1] : F(0.0);
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    VADContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int is_disabled = ctx->is_disabled;
    const int nb_samples = in->nb_samples;
    fn(StateContext) *stc = s->st;

    for (int ch = start; ch < end; ch++) {
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        fn(StateContext) *st = &stc[ch];
        const ftype *a = st->a;
        const ftype *m = st->m;
        ftype *astate = st->b0;
        ftype *lstate = st->b1;

        for (int n = 0; n < nb_samples; n++) {
            const ftype detect = FABS(src[n]) + EPS;
            ftype avg = fn(svf_low)(detect, m, a, astate);
            ftype log_avg = fn(svf_low)(FLOG2(detect), m, a, lstate);

            dst[n] = is_disabled ? src[n] : (avg > EPS) ? LIN2LOG(FEXP2(log_avg) / (avg + EPS)) : -FLT_MAX;
        }

        fn(update_state)(astate);
        fn(update_state)(lstate);
    }

    return 0;
}
