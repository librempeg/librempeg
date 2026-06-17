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

#undef ftype
#undef stype
#undef LRINT
#undef SAMPLE_FORMAT
#undef FIXED
#undef MULT
#if DEPTH == 16
#define LRINT lrint
#define ftype int16_t
#define stype int32_t
#define SAMPLE_FORMAT s16
#define FIXED(x) (LRINT((x) * (1ULL << (DEPTH-1))))
#define MULT(x, y) (((x) * (y)) >> (DEPTH-1))
#elif DEPTH == 32
#define LRINT llrint
#define ftype int32_t
#define stype int64_t
#define SAMPLE_FORMAT s32
#define FIXED(x) (LRINT((x) * (1ULL << (DEPTH-1))))
#define MULT(x, y) (((x) * (y)) >> (DEPTH-1))
#elif DEPTH == 33
#define LRINT
#define ftype float
#define stype float
#define SAMPLE_FORMAT flt
#define FIXED(x) (x)
#define MULT(x, y) ((x) * (y))
#elif DEPTH == 64
#define LRINT
#define ftype double
#define stype double
#define SAMPLE_FORMAT dbl
#define FIXED(x) (x)
#define MULT(x, y) ((x) * (y))
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(OscContext) {
    stype u, v;
    stype k1, k2;
} fn(OscContext);

static av_cold int fn(init_state)(AVFilterContext *ctx)
{
    SineContext *s = ctx->priv;
    double w0 = fmin(s->frequency / (double)s->sample_rate, 0.49999) * 2.0*M_PI;
    int ret;

    s->beep_osc = av_calloc(1, sizeof(fn(OscContext)));
    if (!s->beep_osc)
        return AVERROR(ENOMEM);

    s->osc = av_calloc(1, sizeof(fn(OscContext)));
    if (!s->osc)
        return AVERROR(ENOMEM);

    fn(OscContext) *osc = s->osc;
    osc->k1 = FIXED(tan(w0*0.5));
    osc->k2 = FIXED(sin(w0));
    osc->u  = FIXED(cos(0));
    osc->v  = FIXED(sin(0));

    if (s->beep_factor) {
        fn(OscContext) *beep_osc = s->beep_osc;

        w0 = fmin(s->frequency * s->beep_factor / (double)s->sample_rate, 0.49999) * 2.0*M_PI;
        beep_osc->k1 = FIXED(tan(w0*0.5));
        beep_osc->k2 = FIXED(sin(w0));
        beep_osc->u  = FIXED(cos(0));
        beep_osc->v  = FIXED(sin(0));

        s->beep_period = s->sample_rate;
        s->beep_length = s->beep_period / 25;
    }

    ret = av_expr_parse(&s->samples_per_frame_expr,
                        s->samples_per_frame, var_names,
                        NULL, NULL, NULL, NULL, 0, s);
    if (ret < 0)
        return ret;

    return 0;
}

static void fn(output_samples)(AVFilterContext *ctx, AVFrame *frame)
{
    const int nb_samples = frame->nb_samples;
    SineContext *s = ctx->priv;
    ftype *samples;

    samples = (ftype *)frame->data[0];

    {
        fn(OscContext) *osc = s->osc;

        const stype k1 = osc->k1;
        const stype k2 = osc->k2;
        stype u = osc->u;
        stype v = osc->v;
        stype w;

        for (int i = 0; i < nb_samples; i++) {
#if DEPTH == 16 || DEPTH == 32
            samples[i] = u >> 3;
#else
            samples[i] = u * F(0.125);
#endif
            w = u - MULT(k1, v);
            v += MULT(k2, w);
            u = w - MULT(k1, v);
        }

        osc->u = u;
        osc->v = v;

        if (s->beep_length > 0) {
            fn(OscContext) *beep_osc = s->beep_osc;
            const stype k1 = beep_osc->k1;
            const stype k2 = beep_osc->k2;
            stype u = beep_osc->u;
            stype v = beep_osc->v;

            for (int i = 0; i < nb_samples; i++) {
                if (s->beep_index < s->beep_length) {
#if DEPTH == 16 || DEPTH == 32
                    samples[i] += u >> 2;
#else
                    samples[i] += u * F(0.25);
#endif
                    w = u - MULT(k1, v);
                    v += MULT(k2, w);
                    u = w - MULT(k1, v);
                }

                if (++s->beep_index == s->beep_period)
                    s->beep_index = 0;
            }

            beep_osc->u = u;
            beep_osc->v = v;
        }
    }
}
