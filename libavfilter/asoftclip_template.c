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
#undef CLIP
#undef FSIN
#undef FERF
#undef TANH
#undef ATAN
#undef FPOW
#undef SQRT
#undef FABS
#undef FEXP
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ftype float
#define CLIP av_clipf
#define FSIN sinf
#define FERF erff
#define TANH tanhf
#define ATAN atanf
#define FPOW powf
#define SQRT sqrtf
#define FABS fabsf
#define FEXP expf
#define SAMPLE_FORMAT fltp
#else
#define ftype double
#define CLIP av_clipd
#define FSIN sin
#define FERF erf
#define TANH tanh
#define ATAN atan
#define FPOW pow
#define SQRT sqrt
#define FABS fabs
#define FEXP exp
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static ftype fn(run_lowpass)(const Lowpass *const s,
                             ftype src, ftype *w)
{
    ftype dst;

    dst = src * s->fb0 + w[0];
    w[0] = s->fb1 * src + w[1] - s->fa1 * dst;
    w[1] = s->fb2 * src - s->fa2 * dst;

    return dst;
}

static void fn(softclip)(ASoftClipContext *s,
                         ftype *dst, const ftype *src,
                         const int nb_samples, const int ch)
{
    const int oversample = s->oversample;
    const int nb_osamples = nb_samples * oversample;
    const ftype scale = oversample > 1 ? oversample * F(0.5) : F(1.0);
    ftype threshold = s->threshold;
    ftype gain = s->output * threshold;
    ftype factor = F(1.0) / threshold;
    ftype param = s->param;
    ftype *w = (ftype *)(s->frame[0]->extended_data[ch]) + 2 * (oversample - 1);

    for (int n = 0; n < nb_samples; n++) {
        dst[oversample * n] = src[n];

        for (int m = 1; m < oversample; m++)
            dst[oversample * n + m] = F(0.0);
    }

    for (int n = 0; n < nb_osamples && oversample > 1; n++)
        dst[n] = fn(run_lowpass)(&s->lowpass[oversample - 1], dst[n], w);

    switch (s->type) {
    case ASC_HARD:
        for (int n = 0; n < nb_osamples; n++) {
            dst[n] = CLIP(dst[n] * factor, F(-1.0), F(1.0));
            dst[n] *= gain;
        }
        break;
    case ASC_TANH:
        for (int n = 0; n < nb_osamples; n++) {
            dst[n] = TANH(dst[n] * factor * param);
            dst[n] *= gain;
        }
        break;
    case ASC_ATAN:
        for (int n = 0; n < nb_osamples; n++) {
            dst[n] = F(2.0 / M_PI) * ATAN(dst[n] * factor * param);
            dst[n] *= gain;
        }
        break;
    case ASC_CUBIC:
        for (int n = 0; n < nb_osamples; n++) {
            ftype sample = dst[n] * factor;

            if (FFABS(sample) >= F(1.5))
                dst[n] = FFSIGN(sample);
            else
                dst[n] = sample - F(0.1481) * FPOW(sample, F(3.0));
            dst[n] *= gain;
        }
        break;
    case ASC_EXP:
        for (int n = 0; n < nb_osamples; n++) {
            dst[n] = F(2.0) / (F(1.0) + FEXP((-2.0) * dst[n] * factor)) - F(1.0);
            dst[n] *= gain;
        }
        break;
    case ASC_ALG:
        for (int n = 0; n < nb_osamples; n++) {
            ftype sample = dst[n] * factor;

            dst[n] = sample / (SQRT(param + sample * sample));
            dst[n] *= gain;
        }
        break;
    case ASC_QUINTIC:
        for (int n = 0; n < nb_osamples; n++) {
            ftype sample = dst[n] * factor;

            if (FABS(sample) >= F(1.25))
                dst[n] = FFSIGN(sample);
            else
                dst[n] = sample - F(0.08192) * FPOW(sample, F(5.0));
            dst[n] *= gain;
        }
        break;
    case ASC_SIN:
        for (int n = 0; n < nb_osamples; n++) {
            ftype sample = dst[n] * factor;

            if (FFABS(sample) >= F(M_PI_2))
                dst[n] = FFSIGN(sample);
            else
                dst[n] = FSIN(sample);
            dst[n] *= gain;
        }
        break;
    case ASC_ERF:
        for (int n = 0; n < nb_osamples; n++) {
            dst[n] = FERF(dst[n] * factor);
            dst[n] *= gain;
        }
        break;
    default:
        av_assert0(0);
    }

    w = (ftype *)(s->frame[1]->extended_data[ch]) + 2 * (oversample - 1);
    for (int n = 0; n < nb_osamples && oversample > 1; n++)
        dst[n] = fn(run_lowpass)(&s->lowpass[oversample - 1], dst[n], w);

    for (int n = 0; n < nb_samples; n++)
        dst[n] = dst[n * oversample] * scale;
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ASoftClipContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = in->nb_samples;

    for (int ch = start; ch < end; ch++) {
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];

        fn(softclip)(s, dst, src, nb_samples, ch);
    }

    return 0;
}
