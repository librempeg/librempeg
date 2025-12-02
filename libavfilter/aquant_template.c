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
#undef ROUND
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ROUND roundf
#define SAMPLE_FORMAT float
#define ftype float
#else
#define ROUND round
#define SAMPLE_FORMAT double
#define ftype double
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ftype a[3];
    ftype b[3];
    ftype c[4];
    ftype quant_err;
} fn(StateContext);

static int fn(init_state)(AVFilterContext *ctx)
{
    AudioQuantContext *s = ctx->priv;
    fn(StateContext) *state;

    s->state = av_calloc(s->channels, sizeof(*state));
    if (!s->state)
        return AVERROR(ENOMEM);

    state = s->state;
    for (int ch = 0; ch < s->channels; ch++) {
        fn(StateContext) *stc = &state[ch];
        const ftype w0 = M_PI * 0.1;
        const ftype cos_w0 = cos(w0);
        const ftype sin_w0 = sin(w0);
        ftype a[3], b[3], bb[3];
        ftype alpha;

        alpha = sin_w0 / 0.02;

        a[0] =  1 + alpha;
        a[1] = -2 * cos_w0;
        a[2] =  1 - alpha;
        b[0] =  1;
        b[1] = -2 * cos_w0;
        b[2] =  1;

        a[1] /= a[0];
        a[2] /= a[0];
        b[0] /= a[0];
        b[1] /= a[0];
        b[2] /= a[0];
        a[0] /= a[0];

        bb[0] = b[0]/b[0];
        bb[1] = b[1]/b[0];
        bb[2] = b[2]/b[0];

        bb[1] -= a[1];
        bb[2] -= a[2];

        stc->a[0] = a[0];
        stc->a[1] = a[1];
        stc->a[2] = a[2];
        stc->b[0] = bb[0];
        stc->b[1] = bb[1];
        stc->b[2] = bb[2];
    }

    return 0;
}

static int fn(shaper_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    const int is_disabled = ff_filter_disabled(ctx);
    AudioQuantContext *s = ctx->priv;
    AVFrame *in = s->in;
    AVFrame *out = arg;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = in->nb_samples;
    const ftype factor = 1U << (s->bits-1);
    const ftype scale = F(1.0) / factor;

    for (int c = start; c < end; c++) {
        const ftype *input = (const ftype *)in->extended_data[c];
        ftype *output = (ftype *)out->extended_data[c];

        if (is_disabled) {
            for (int n = 0; n < nb_samples; n++)
                output[n] = input[n];
        } else {
            fn(StateContext) *state = s->state;
            fn(StateContext) *stc = &state[c];
            ftype *fcache = stc->c;
            const ftype *a = stc->a;
            const ftype *b = stc->b;
            const ftype a1 = -a[1];
            const ftype a2 = -a[2];
            const ftype b1 = b[1];
            const ftype b2 = b[2];
            ftype w1 = fcache[0];
            ftype w2 = fcache[1];
            ftype quant_err = stc->quant_err;

            for (int n = 0; n < nb_samples; n++) {
                ftype filter_out, quant_out, quant_in, in = input[n];

                filter_out = quant_err + w1;

                w1 = b1 * quant_err + w2 + a1 * filter_out;
                w2 = b2 * quant_err + a2 * filter_out;

                quant_in = in + filter_out;
                quant_out = ROUND(quant_in * factor) * scale;
                quant_err = quant_out - quant_in;
                output[n] = quant_out;
            }

            stc->quant_err = isnormal(quant_err) ? quant_err : F(0.0);

            fcache[0] = isnormal(w1) ? w1 : F(0.0);
            fcache[1] = isnormal(w2) ? w2 : F(0.0);
        }
    }

    return 0;
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    const int is_disabled = ff_filter_disabled(ctx);
    AudioQuantContext *s = ctx->priv;
    AVFrame *in = s->in;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = out->nb_samples;
    const ftype factor = 1U << (s->bits-1);
    const ftype scale = F(1.0) / factor;

    for (int c = start; c < end; c++) {
        const ftype *input = (const ftype *)in->extended_data[c];
        ftype *output = (ftype *)out->extended_data[c];

        if (is_disabled) {
            for (int n = 0; n < nb_samples; n++)
                output[n] = input[n];

            continue;
        }

        for (int n = 0; n < nb_samples; n++)
            output[n] = ROUND(input[n] * factor) * scale;
    }

    return 0;
}
