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

#include "avfilter.h"
#include "audio.h"

#define fnb3(a,b,c) a##_##b##_##c
#define fnb2(a,b,c) fnb3(a,b,c)
#define fnb(a)      fnb2(a, SAMPLE_FORMAT, NBR)

static int fnb(src)(AVFilterContext *ctx, AVFrame *in, AVFrame *out,
                    const int ch)
{
    AudioIIRSRCContext *s = ctx->priv;
    const itype *src = ((const itype *)in->extended_data[ch]);
    itype *dst = ((itype *)out->extended_data[ch]);
    fn(StateContext) *state = s->state;
    fn(StateContext) *stc = &state[ch];
    const int nb_samples = in->nb_samples;
    const int nbr_coefs = NBR;
    const int nb_stages = NBR;
    const ftype *coefs_arr = stc->coefs_arr;
    ftype *stage_arr = stc->stage_arr;

    for (int n = 0; n < nb_samples; n++) {
        const ftype x = src[n];
        ftype even = x;
        ftype odd  = x;
        int i;

        for (i = nb_stages; i >= 2; i -= 2)
            fn(process_sample_pos)(i, nbr_coefs, &even, &odd, stage_arr, coefs_arr);

        if (i == 1)
            fn(process_sample_pos1)(nbr_coefs, &even, &odd, stage_arr, coefs_arr);

        if (i == 0)
            fn(process_sample_pos0)(nbr_coefs, &even, &odd, stage_arr);

        dst[n*2+0] = even;
        dst[n*2+1] = odd;
    }

    return 0;
}
