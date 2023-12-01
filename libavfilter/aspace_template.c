/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "avfilter.h"
#include "internal.h"
#include "audio.h"

#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define ftype float
#else
#define SAMPLE_FORMAT double
#define ftype double
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static void fn(process)(AVFilterContext *ctx,
                        AVFrame *in, AVFrame *out, AVFrame *w)
{
    const int nb_channels = out->ch_layout.nb_channels;
    const int nb_samples = FFALIGN(in->nb_samples, 16);
    AudioSpaceContext *s = ctx->priv;

    for (int i = 0; i < nb_channels; i++) {
#if DEPTH == 32
        s->fdsp->vector_fmul((ftype *)out->extended_data[i],
                             (const ftype *)in->extended_data[0],
                             (const ftype *)w->extended_data[i],
                             nb_samples);
#else
        s->fdsp->vector_dmul((ftype *)out->extended_data[i],
                             (const ftype *)in->extended_data[0],
                             (const ftype *)w->extended_data[i],
                             nb_samples);
#endif
    }
}
