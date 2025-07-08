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

#undef stype
#undef ftype
#undef SAMPLE_FORMAT
#undef FIXED
#undef MULT
#if DEPTH == 16
#define stype int16_t
#define ftype int32_t
#define SAMPLE_FORMAT s16p
#define FIXED(x) (lrint((x) * (1 << (DEPTH-1))))
#define MULT(x, y) (((x) * (y)) >> (DEPTH-1))
#elif DEPTH == 31
#define stype int32_t
#define ftype int64_t
#define SAMPLE_FORMAT s32p
#define FIXED(x) (lrint((x) * (1U << (DEPTH))))
#define MULT(x, y) (((x) * (y)) >> (DEPTH))
#elif DEPTH == 32
#define stype float
#define ftype double
#define SAMPLE_FORMAT fltp
#define FIXED(x) (x)
#define MULT(x, y) ((x) * (y))
#else
#define stype double
#define ftype double
#define SAMPLE_FORMAT dblp
#define FIXED(x) (x)
#define MULT(x, y) ((x) * (y))
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
} fn(StateContext);

static av_cold int fn(init_state)(AVFilterContext *ctx)
{
    AQuadOscContext *s = ctx->priv;
    fn(StateContext) *st;

    if (!s->st)
        s->st = av_calloc(1, sizeof(*st));
    if (!s->st)
        return AVERROR(ENOMEM);
    st = s->st;

    if (s->frequency != s->old_frequency) {
        double w0 = fmin(s->frequency, 0.49999) * 2.0*M_PI;

        st->k1 = FIXED(tan(w0*0.5));
        st->k2 = FIXED(sin(w0));
        s->old_frequency = s->frequency;
    }

    if (s->phase != s->old_phase) {
        double p0 = M_PI*s->phase;

        st->u = FIXED(cos(p0));
        st->v = FIXED(sin(p0));
        s->old_phase = s->phase;
    }

    return 0;
}

static void fn(output_samples)(AVFilterContext *ctx, void *stc, AVFrame *frame)
{
    AQuadOscContext *s = ctx->priv;
    fn(StateContext) *st = stc;
    stype *real = (stype *)frame->extended_data[0];
    stype *imag = (stype *)frame->extended_data[1];
    const int nb_samples = frame->nb_samples;
    const ftype offset = FIXED(s->offset);
    const ftype a = FIXED(s->amplitude);
    const ftype k1 = st->k1;
    const ftype k2 = st->k2;
    ftype u = st->u;
    ftype v = st->v;
    ftype w;

    for (int i = 0; i < nb_samples; i++) {
        real[i] = MULT(u, a) + offset;
        imag[i] = MULT(v, a) + offset;
        w = u - MULT(k1, v);
        v += MULT(k2, w);
        u = w - MULT(k1, v);
    }

    st->u = u;
    st->v = v;
}
