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
#define SAMPLE_FORMAT fltp
#else
#define ftype double
#define SAMPLE_FORMAT dblp
#endif
#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static void fn(dc_denorm)(AVFilterContext *ctx, void *dstp,
                          const void *srcp, int nb_samples)
{
    ADenormContext *s = ctx->priv;
    const ftype *src = (const ftype *)srcp;
    ftype *dst = (ftype *)dstp;
    const ftype dc = s->level;

    for (int n = 0; n < nb_samples; n++) {
        dst[n] = src[n] + dc;
    }
}

static void fn(ac_denorm)(AVFilterContext *ctx, void *dstp,
                          const void *srcp, int nb_samples)
{
    ADenormContext *s = ctx->priv;
    const ftype *src = (const ftype *)srcp;
    ftype *dst = (ftype *)dstp;
    const ftype dc = s->level;
    const int64_t N = s->in_samples;

    for (int n = 0; n < nb_samples; n++) {
        dst[n] = src[n] + dc * (((N + n) & 1) ? F(-1.0) : F(1.0));
    }
}

static void fn(sq_denorm)(AVFilterContext *ctx, void *dstp,
                          const void *srcp, int nb_samples)
{
    ADenormContext *s = ctx->priv;
    const ftype *src = (const ftype *)srcp;
    ftype *dst = (ftype *)dstp;
    const ftype dc = s->level;
    const int64_t N = s->in_samples;

    for (int n = 0; n < nb_samples; n++) {
        dst[n] = src[n] + dc * ((((N + n) >> 8) & 1) ? F(-1.0) : F(1.0));
    }
}

static void fn(ps_denorm)(AVFilterContext *ctx, void *dstp,
                          const void *srcp, int nb_samples)
{
    ADenormContext *s = ctx->priv;
    const ftype *src = (const ftype *)srcp;
    ftype *dst = (ftype *)dstp;
    const ftype dc = s->level;
    const int64_t N = s->in_samples;

    for (int n = 0; n < nb_samples; n++) {
        dst[n] = src[n] + dc * (((N + n) & 255) ? F(0.0) : F(1.0));
    }
}
