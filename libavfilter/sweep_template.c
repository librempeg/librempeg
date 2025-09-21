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
#undef FSIN
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ftype float
#define FSIN sinf
#define SAMPLE_FORMAT fltp
#else
#define FSIN sin
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static av_cold int fn(init_state)(AVFilterContext *ctx)
{
    SweepContext *s = ctx->priv;

    s->frequency_range = s->stop_frequency - s->start_frequency;
    s->direction = (s->start_frequency < s->stop_frequency) ? 1.0 : -1.0;
    s->samples = s->period * s->sample_rate;
    s->increment = 1.0 / s->samples;

    return 0;
}

static void fn(output_samples)(AVFilterContext *ctx, AVFrame *frame)
{
    SweepContext *s = ctx->priv;
    const ftype frequency_range = s->frequency_range;
    const ftype start_frequency = s->start_frequency;
    const ftype stop_frequency = s->stop_frequency;
    ftype *dst = (ftype *)frame->extended_data[0];
    const int nb_samples = frame->nb_samples;
    const ftype amplitude = s->amplitude;
    const ftype increment = s->increment;
    ftype direction = s->direction;
    ftype position = s->position;
    const int type = s->type;
    ftype phase = s->phase;
    ftype current_frequency;

    for (int i = 0; i < nb_samples; i++) {
        position += increment * direction;
        if (position >= F(1.0)) {
            position = F(1.0);
            direction = F(-1.0);
        } else if (position <= F(0.0)) {
            position = F(0.0);
            direction = F(1.0);
        }

        dst[i] = FSIN(phase) * amplitude;
        switch (type) {
        case 0:
            current_frequency = start_frequency + position * frequency_range;
            break;
        case 1:
            current_frequency = start_frequency * pow(stop_frequency/start_frequency, position);
            break;
        }
        phase += F(2.0) * F(M_PI) * current_frequency;

        if (phase > F(2.0) * F(M_PI))
            phase -= F(2.0) * F(M_PI);
        else if (phase < F(0.0))
            phase += F(2.0) * F(M_PI);
    }

    s->phase = phase;
    s->position = position;
    s->direction = direction;
}
