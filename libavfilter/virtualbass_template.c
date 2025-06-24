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

#undef FSIN
#undef ATAN
#undef SQRT
#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define FSIN sinf
#define ATAN atanf
#define SQRT sqrtf
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define FSIN sin
#define ATAN atan
#define SQRT sqrt
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define SQR(x) ((x) * (x))

static ftype fn(vb_fun)(ftype x)
{
    ftype y = F(2.5) * ATAN(F(0.9) * x) + F(2.5) * SQRT(F(1.0) - SQR(F(0.9) * x)) - F(2.5);

    return y < F(0.0) ? FSIN(y) : y;
}

static void fn(vb_stereo)(AVFilterContext *ctx, AVFrame *out, const AVFrame *in)
{
    AudioVirtualBassContext *s = ctx->priv;
    const ftype *lsrc = (const ftype *)in->extended_data[0];
    const ftype *rsrc = (const ftype *)in->extended_data[1];
    ftype *ldst = (ftype *)out->extended_data[0];
    ftype *rdst = (ftype *)out->extended_data[1];
    ftype *lfe = (ftype *)out->extended_data[2];
    const int nb_samples = in->nb_samples;
    const ftype st = M_PI / s->strength;
    const ftype a0 = s->a[0];
    const ftype a1 = s->a[1];
    const ftype a2 = s->a[2];
    const ftype m2 = s->m[2];
    ftype b0 = s->cf[0];
    ftype b1 = s->cf[1];

    memcpy(ldst, lsrc, nb_samples * sizeof(*ldst));
    memcpy(rdst, rsrc, nb_samples * sizeof(*rdst));

    for (int n = 0; n < nb_samples; n++) {
        const ftype center = (lsrc[n] + rsrc[n]) * F(0.5);
        const ftype v0 = center;
        const ftype v3 = v0 - b1;
        const ftype v1 = a0 * b0 + a1 * v3;
        const ftype v2 = b1 + a1 * b0 + a2 * v3;
        ftype b, vb;

        b0 = F(2.0) * v1 - b0;
        b1 = F(2.0) * v2 - b1;

        b = m2 * v2;
        vb = FSIN(fn(vb_fun)(b) * st);

        lfe[n] = vb;
    }

    s->cf[0] = isnormal(b0) ? b0 : F(0.0);
    s->cf[1] = isnormal(b1) ? b1 : F(0.0);
}
