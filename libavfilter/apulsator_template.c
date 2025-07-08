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
#undef FABS
#undef FMOD
#undef SIN
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define SAMPLE_FORMAT fltp
#define FABS fabsf
#define FMOD fmodf
#define SIN sinf
#define ftype float
#else
#define SAMPLE_FORMAT dblp
#define FABS fabs
#define FMOD fmod
#define SIN sin
#define ftype double
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static ftype fn(lfo_get_value)(const int mode, ftype amount,
                               ftype phase, ftype width, ftype offset)
{
    ftype phs = phase / width + offset;
    ftype val;

    if (phs > F(1.0))
        phs = FMOD(phs, F(1.0));

    switch (mode) {
    case SINE:
        val = SIN(phs * F(2.0 * M_PI));
        break;
    case TRIANGLE:
        if (phs > F(0.75))
            val = (phs - F(0.75)) * F(4.0) - F(1.0);
        else if (phs > F(0.25))
            val = F(-4.0) * phs + F(2.0);
        else
            val = phs * F(4.0);
        break;
    case SQUARE:
        val = phs < F(0.5) ? -F(1.0) : F(1.0);
        break;
    case SAWUP:
        val = phs * F(2.0) - F(1.0);
        break;
    case SAWDOWN:
        val = F(1.0) - phs * F(2.0);
        break;
    default:
        av_assert0(0);
    }

    return val * amount;
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioPulsatorContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int nb_channels = in->ch_layout.nb_channels;
    const int start = (nb_channels * jobnr) / nb_jobs;
    const int end = (nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        const ftype *src = (const ftype *)in->extended_data[ch];
        const ftype level_out = s->level_out[FFMIN(ch, s->nb_level_out-1)];
        const ftype level_in = s->level_in[FFMIN(ch, s->nb_level_in-1)];
        const ftype amount = s->amount[FFMIN(ch, s->nb_amount-1)];
        const int timing = s->timing[FFMIN(ch, s->nb_timing-1)];
        const ftype offset = s->offset[FFMIN(ch, s->nb_offset-1)];
        const ftype width = s->width[FFMIN(ch, s->nb_width-1)];
        const ftype hertz = s->hertz[FFMIN(ch, s->nb_hertz-1)];
        const ftype bpm = s->bpm[FFMIN(ch, s->nb_bpm-1)];
        const int mode = s->mode[FFMIN(ch, s->nb_mode-1)];
        const ftype ms = s->ms[FFMIN(ch, s->nb_ms-1)];
        ftype *dst = (ftype *)out->extended_data[ch];
        const ftype fs = F(1.0) / in->sample_rate;
        const int nb_samples = in->nb_samples;
        ftype phase = s->phase[ch];
        ftype freq;

        switch (timing) {
        case UNIT_BPM:
            freq = bpm / F(60.0);
            break;
        case UNIT_MS:
            freq = F(1.0) / (ms / F(1000.0));
            break;
        case UNIT_HZ:
            freq = hertz;
            break;
        default:
            av_assert0(0);
        }

        for (int n = 0; n < nb_samples; n++) {
            ftype in = src[n] * level_in;
            ftype proc = in;
            ftype out;

            proc *= fn(lfo_get_value)(mode, amount, phase, width,
                                      offset) * F(0.5) + amount / F(2.0);

            out = proc + in * (F(1.0) - amount);
            out *= level_out;

            dst[n] = out;

            phase = FABS(phase + freq * fs);
            if (phase >= F(1.0))
                phase = FMOD(phase, F(1.0));
        }

        s->phase[ch] = phase;
    }

    return 0;
}
