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

#undef ftype
#undef SAMPLE_FORMAT
#undef VECTOR_MUL
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define ftype float
#define VECTOR_MUL s->fdsp->vector_fmul
#else
#define SAMPLE_FORMAT double
#define ftype double
#define VECTOR_MUL s->fdsp->vector_dmul
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    AVFrame *w = td->w;
    const int nb_channels = out->ch_layout.nb_channels;
    const int nb_samples = FFALIGN(in->nb_samples, 16);
    const int start = (nb_channels * jobnr) / nb_jobs;
    const int end = (nb_channels * (jobnr+1)) / nb_jobs;
    AudioSpaceContext *s = ctx->priv;

    for (int ch = start; ch < end; ch++) {
        VECTOR_MUL((ftype *)out->extended_data[ch],
                   (const ftype *)in->extended_data[0],
                   (const ftype *)w->extended_data[ch],
                   nb_samples);
    }

    return 0;
}
