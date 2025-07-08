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
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ftype float
#define CLIP av_clipf
#define SAMPLE_FORMAT flt
#else
#define ftype double
#define CLIP av_clipd
#define SAMPLE_FORMAT dbl
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static void fn(extrastereo)(AVFilterContext *ctx, AVFrame *out, const AVFrame *in, const int clip)
{
    ExtraStereoContext *s = ctx->priv;
    const ftype *src = (const ftype *)in->data[0];
    const int nb_samples = in->nb_samples;
    ftype *dst = (ftype *)out->data[0];
    const ftype mult = s->mult;

    for (int n = 0; n < nb_samples; n++) {
        ftype average, left, right;

        left    = src[n * 2    ];
        right   = src[n * 2 + 1];
        average = (left + right) * F(0.5);
        left    = average + mult * (left  - average);
        right   = average + mult * (right - average);

        if (clip) {
            left  = CLIP(left,  F(-1.0), F(1.0));
            right = CLIP(right, F(-1.0), F(1.0));
        }

        dst[n * 2    ] = left;
        dst[n * 2 + 1] = right;
    }
}
