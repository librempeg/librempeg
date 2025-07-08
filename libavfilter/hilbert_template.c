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
#undef FCOS
#undef FSIN
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ftype float
#define FCOS cosf
#define FSIN sinf
#define SAMPLE_FORMAT flt
#else
#define ftype double
#define FCOS cos
#define FSIN sin
#define SAMPLE_FORMAT dbl
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define F(x) ((ftype)(x))

#undef AVFILTER_WINDOW_FUNC_H
#include "window_func.h"

static int fn(generate)(AVFilterContext *ctx)
{
    HilbertContext *s = ctx->priv;
    const ftype angle = s->angle;
    const ftype factor = FSIN(F(M_PI)*angle/F(180.0));
    const ftype first = FCOS(F(M_PI)*angle/F(180.0));
    const int nb_taps = s->nb_taps;
    ftype overlap, *taps;

    s->taps = av_malloc_array(nb_taps, sizeof(*taps));
    if (!s->taps)
        return AVERROR(ENOMEM);
    taps = s->taps;

    fun(generate_window_func)(taps, nb_taps, s->win_func, &overlap);

    for (int i = 0; i < nb_taps; i++) {
        int k = -(nb_taps / 2) + i;

        if (k & 1) {
            ftype pk = F(M_PI) * k;

            taps[i] *= (F(1.0) - FCOS(pk)) / pk;
            taps[i] *= factor;
        } else {
            taps[i] = F(0.0);
            if (k == 0)
                taps[i] += first;
        }
    }

    return 0;
}
