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

#undef FTAN
#undef FSQRT
#undef ftype
#undef FPI
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define FTAN tanf
#define FSQRT sqrtf
#define ftype float
#define FPI M_PIf
#define SAMPLE_FORMAT flt
#else
#define FTAN tan
#define FSQRT sqrt
#define ftype double
#define FPI M_PI
#define SAMPLE_FORMAT dbl
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ftype a0, a1, a2;
    ftype b0, b1, b2;

    ftype w1, w2;
    ftype rw1, rw2;

    ftype *mid;
    ftype *side[3];
} fn(StateContext);

static int fn(init_state)(AVFilterContext *ctx)
{
    CrossfeedContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    const ftype sgn = s->is_sideboost ? F(0.5) : F(-1.0);
    ftype A = ff_exp10(s->strength * sgn * F(30.0) / F(40.0));
    fn(StateContext) *stc;
    ftype g, k, Q;

    if (!s->st)
        s->st = av_calloc(1, sizeof(*stc));
    if (!s->st)
        return AVERROR(ENOMEM);
    stc = s->st;

    Q = F(1.0) / FSQRT((A + F(1.0) / A) * (F(1.0) / s->slope - F(1.0)) + F(2.0));
    g = FTAN(FPI * (F(1.0) - s->range) * F(2100.0) / inlink->sample_rate) / FSQRT(A);
    k = F(1.0) / Q;

    stc->a0 = F(1.0) / (F(1.0) + g * (g + k));
    stc->a1 = g * stc->a0;
    stc->a2 = g * stc->a1;

    if (s->is_sideboost) {
        stc->b0 = A * A;
        stc->b1 = k * (F(1.0) - A) * A;
        stc->b2 = F(1.0) - A * A;
    } else {
        stc->b0 = F(1.0);
        stc->b1 = k * (A - F(1.0));
        stc->b2 = A * A - F(1.0);
    }

    if (s->block_samples == 0 && s->block_size > 0) {
        s->pts = AV_NOPTS_VALUE;
        s->block_samples = s->block_size;
        if (!stc->mid)
            stc->mid = av_calloc(s->block_samples * 2, sizeof(*stc->mid));
        for (int i = 0; i < 3; i++) {
            if (!stc->side[i])
                stc->side[i] = av_calloc(s->block_samples * 2, sizeof(*stc->side[0]));
            if (!stc->side[i])
                return AVERROR(ENOMEM);
        }
    }

    return 0;
}

static void fn(uninit_state)(AVFilterContext *ctx)
{
    CrossfeedContext *s = ctx->priv;
    fn(StateContext) *stc = s->st;

    if (!stc)
        return;

    for (int i = 0; i < 3; i++)
        av_freep(&stc->side[i]);
    av_freep(&stc->mid);
    av_freep(&s->st);
}

static void fn(reverse_samples)(ftype *dst, const ftype *src,
                                const int nb_samples)
{
    for (int i = 0, j = nb_samples - 1; i < nb_samples; i++, j--)
        dst[i] = src[j];
}

static void fn(filter_samples)(ftype *dst, const ftype *src,
                               const int nb_samples,
                               const ftype m0, const ftype m1, const ftype m2,
                               const ftype a0, const ftype a1, const ftype a2,
                               ftype *sw1, ftype *sw2)
{
    ftype w1 = *sw1;
    ftype w2 = *sw2;

    for (int n = 0; n < nb_samples; n++) {
        const ftype in = src[n];
        const ftype v0 = in;
        const ftype v3 = v0 - w2;
        const ftype v1 = a0 * w1 + a1 * v3;
        const ftype v2 = w2 + a1 * w1 + a2 * v3;

        w1 = F(2.0) * v1 - w1;
        w2 = F(2.0) * v2 - w2;

        dst[n] = m0 * v0 + m1 * v1 + m2 * v2;
    }

    *sw1 = w1;
    *sw2 = w2;
}

static int fn(xfeed_frame)(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    CrossfeedContext *s = ctx->priv;
    const ftype *src = (const ftype *)in->data[0];
    const int is_disabled = ff_filter_disabled(ctx);
    fn(StateContext) *stc = s->st;
    const ftype level_in = s->level_in * F(0.5);
    const ftype level_out = s->level_out;
    const ftype b0 = stc->b0;
    const ftype b1 = stc->b1;
    const ftype b2 = stc->b2;
    const ftype a0 = stc->a0;
    const ftype a1 = stc->a1;
    const ftype a2 = stc->a2;
    AVFrame *out;
    int drop = 0;
    ftype *dst;

    if (av_frame_is_writable(in) && s->block_samples == 0) {
        out = in;
    } else {
        out = ff_get_audio_buffer(outlink, s->block_samples > 0 ? s->block_samples : in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }
    dst = (ftype *)out->data[0];

    if (s->block_samples > 0 && s->pts == AV_NOPTS_VALUE)
        drop = 1;

    if (s->block_samples == 0) {
        const int nb_samples = out->nb_samples;
        ftype w1 = stc->w1;
        ftype w2 = stc->w2;

        for (int n = 0; n < nb_samples; n++, src += 2, dst += 2) {
            const ftype mid = (src[0] + src[1]) * level_in;
            const ftype side = (src[0] - src[1]) * level_in;
            const ftype in = side;
            const ftype v0 = in;
            const ftype v3 = v0 - w2;
            const ftype v1 = a0 * w1 + a1 * v3;
            const ftype v2 = w2 + a1 * w1 + a2 * v3;
            ftype oside;

            w1 = F(2.0) * v1 - w1;
            w2 = F(2.0) * v2 - w2;

            oside = b0 * v0 + b1 * v1 + b2 * v2;

            if (is_disabled) {
                dst[0] = src[0];
                dst[1] = src[1];
            } else {
                dst[0] = (mid + oside) * level_out;
                dst[1] = (mid - oside) * level_out;
            }
        }

        stc->w1 = isnormal(w1) ? w1 : F(0.0);
        stc->w2 = isnormal(w2) ? w2 : F(0.0);
    } else {
        const int block_samples = s->block_samples;
        const int nb_samples = in->nb_samples;
        ftype *mdst = stc->mid + s->block_samples;
        ftype *sdst = stc->side[0] + s->block_samples;
        ftype w1 = stc->w1;
        ftype w2 = stc->w2;

        for (int n = 0; n < nb_samples; n++, src += 2) {
            mdst[n] = (src[0] + src[1]) * level_in;
            sdst[n] = (src[0] - src[1]) * level_in;
        }

        for (int n = nb_samples; n < block_samples; n++) {
            mdst[n] = F(0.0);
            sdst[n] = F(0.0);
        }

        fn(filter_samples)(sdst, sdst, nb_samples,
                           b0, b1, b2, a0, a1, a2, &w1, &w2);

        stc->w1 = isnormal(w1) ? w1 : F(0.0);
        stc->w2 = isnormal(w2) ? w2 : F(0.0);

        fn(reverse_samples)(stc->side[1], stc->side[0], block_samples * 2);
        fn(filter_samples)(stc->side[1], stc->side[1], block_samples * 2,
                           b0, b1, b2, a0, a1, a2, &stc->rw1, &stc->rw2);
        stc->rw1 = isnormal(stc->rw1) ? stc->rw1 : F(0.0);
        stc->rw2 = isnormal(stc->rw2) ? stc->rw2 : F(0.0);
        fn(reverse_samples)(stc->side[2], stc->side[1], block_samples * 2);

        src = (const ftype *)in->data[0];
        mdst = stc->mid;
        sdst = stc->side[2];
        for (int n = 0; n < block_samples; n++, src += 2, dst += 2) {
            if (is_disabled) {
                dst[0] = src[0];
                dst[1] = src[1];
            } else {
                dst[0] = (mdst[n] + sdst[n]) * level_out;
                dst[1] = (mdst[n] - sdst[n]) * level_out;
            }
        }

        memcpy(stc->mid, stc->mid + block_samples,
               block_samples * sizeof(*stc->mid));
        memcpy(stc->side[0], stc->side[0] + block_samples,
               block_samples * sizeof(*stc->side[0]));
    }

    if (s->block_samples > 0) {
        int nb_samples = in->nb_samples;
        int64_t pts = in->pts;

        out->pts = s->pts;
        out->nb_samples = s->nb_samples;
        s->pts = pts;
        s->nb_samples = nb_samples;
    }

    if (out != in)
        ff_graph_frame_free(ctx, &in);
    if (!drop) {
        return ff_filter_frame(outlink, out);
    } else {
        ff_graph_frame_free(ctx, &out);
        ff_filter_set_ready(ctx, 10);
        return 0;
    }
}
