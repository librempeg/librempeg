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

#undef FABS
#undef FEXP
#undef FLOG
#undef FMAX
#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define FABS fabsf
#define FEXP expf
#define FLOG logf
#define FMAX fmaxf
#define ftype float
#define SAMPLE_FORMAT flt
#else
#define FABS fabs
#define FEXP exp
#define FLOG log
#define FMAX fmax
#define ftype double
#define SAMPLE_FORMAT dbl
#endif

#include "hermite.h"

#define F(x) ((ftype)(x))

#define LIN2LOG(x) (FLOG((x)))
#define LOG2LIN(x) (FEXP((x)))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static ftype fn(output_gain)(ftype lin_slope, ftype ratio, ftype thres,
                             ftype knee, ftype knee_start, ftype knee_stop,
                             ftype range, int direction)
{
    ftype slope = FLOG(lin_slope);
    ftype gain;
    ftype delta;

    gain = (slope - thres) * ratio + thres;
    delta = ratio;

    if (direction) {
        if (knee > F(1.0) && slope < knee_stop)
            gain = hermite_interpolation(slope, knee_stop, knee_start, ((knee_stop - thres) * ratio  + thres), knee_start, delta, F(1.0));
    } else {
        if (knee > F(1.0) && slope > knee_start)
            gain = hermite_interpolation(slope, knee_start, knee_stop, ((knee_start - thres) * ratio  + thres), knee_stop, delta, F(1.0));
    }
    return FMAX(range, FEXP(gain - slope));
}

static void fn(gate)(AVFilterContext *ctx, AVFrame *out, const int nb_samples,
                     AVFilterLink *inlink, AVFilterLink *sclink)
{
    AudioGateContext *s = ctx->priv;
    AVFrame *sc = s->sc ? s->sc : s->in;
    AVFrame *in = s->in;
    const ftype *scsrc = (const ftype *)sc->data[0];
    const ftype *src = (const ftype *)in->data[0];
    ftype *dst = (ftype *)out->data[0];
    const int sc_nb_channels = sclink->ch_layout.nb_channels;
    const int nb_channels = inlink->ch_layout.nb_channels;
    const ftype lin_knee_start = s->lin_knee_start;
    const ftype lin_knee_stop = s->lin_knee_stop;
    const ftype makeup = s->makeup;
    const ftype attack_coeff = s->attack_coeff;
    const ftype release_coeff = s->release_coeff;
    const int is_disabled = ff_filter_disabled(ctx);
    const int detection = s->detection;
    ftype *lin_slopep = s->lin_slope;
    const ftype ratio = s->ratio;
    const ftype range = s->range;
    const ftype knee_start = s->knee_start;
    const ftype knee_stop = s->knee_stop;
    const ftype level_in = s->level_in;
    const ftype level_sc = s->level_sc;
    const ftype thres = s->thres;
    const ftype knee = s->knee;
    const int link = s->link;
    const int direction = s->direction;

    for (int n = 0; n < nb_samples; n++, src += nb_channels, dst += nb_channels, scsrc += sc_nb_channels) {
        ftype abs_sample, factor, lin_slope;
        ftype gain = F(1.0);
        int detected;

        switch (link) {
        case LINKMODE_MAX:
        case LINKMODE_AVG:
            lin_slope = lin_slopep[0];
            abs_sample = FABS(scsrc[0] * level_sc);
            if (link == LINKMODE_MAX) {
                for (int c = 1; c < sc_nb_channels; c++)
                    abs_sample = FMAX(FABS(scsrc[c] * level_sc), abs_sample);
            } else {
                for (int c = 1; c < sc_nb_channels; c++)
                    abs_sample += FABS(scsrc[c] * level_sc);
                abs_sample /= sc_nb_channels;
            }

            if (detection)
                abs_sample *= abs_sample;
            lin_slope += (abs_sample - lin_slope) * (abs_sample > lin_slope ? attack_coeff : release_coeff);

            if (direction)
                detected = lin_slope > lin_knee_start;
            else
                detected = lin_slope < lin_knee_stop;

            if (lin_slope > F(0.0) && detected)
                gain = fn(output_gain)(lin_slope, ratio, thres,
                                       knee, knee_start, knee_stop,
                                       range, direction);

            factor = is_disabled ? F(1.0) : level_in * gain * makeup;
            for (int c = 0; c < nb_channels; c++)
                dst[c] = src[c] * factor;
            lin_slopep[0] = lin_slope;
            break;
        case LINKMODE_NONE:
            for (int c = 0; c < sc_nb_channels; c++) {
                ftype lin_slope = lin_slopep[c];
                ftype gain = F(1.0);

                abs_sample = FABS(scsrc[c] * level_sc);
                if (detection)
                    abs_sample *= abs_sample;
                lin_slope += (abs_sample - lin_slope) * (abs_sample > lin_slope ? attack_coeff : release_coeff);

                if (direction)
                    detected = lin_slope > lin_knee_start;
                else
                    detected = lin_slope < lin_knee_stop;

                if (lin_slope > F(0.0) && detected)
                    gain = fn(output_gain)(lin_slope, ratio, thres,
                                           knee, knee_start, knee_stop,
                                           range, direction);

                factor = is_disabled ? F(1.0) : level_in * gain * makeup;
                dst[c] = src[c] * factor;

                lin_slopep[c] = lin_slope;
            }
            break;
        }
    }
}
