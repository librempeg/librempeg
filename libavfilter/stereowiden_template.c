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

#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ftype float
#define SAMPLE_FORMAT flt
#else
#define ftype double
#define SAMPLE_FORMAT dbl
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static void fn(stereowiden)(AVFilterContext *ctx, AVFrame *out, const AVFrame *in)
{
    StereoWidenContext *s = ctx->priv;
    const ftype *src = (const ftype *)in->data[0];
    const int is_disabled = ff_filter_disabled(ctx);
    const ftype crossfeed = s->crossfeed;
    const ftype feedback = s->feedback;
    const ftype drymix = s->drymix;
    const int nb_samples = in->nb_samples;
    ftype *dst = (ftype *)out->data[0];
    const int length = s->length;
    ftype *buffer = s->buffer;
    int idx = s->idx;

    for (int n = 0; n < nb_samples; n++, src += 2, dst += 2, idx += 2) {
        const ftype left = src[0], right = src[1];

        if (idx >= length)
            idx = 0;

        if (is_disabled) {
            dst[0] = left;
            dst[1] = right;
        } else {
            dst[0] = drymix * left - crossfeed * right - feedback * buffer[idx+1];
            dst[1] = drymix * right - crossfeed * left - feedback * buffer[idx+0];
        }

        buffer[idx+0] = left;
        buffer[idx+1] = right;
    }

    s->idx = idx;
}
