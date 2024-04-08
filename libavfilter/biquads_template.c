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

#undef min
#undef max
#undef stype
#undef ftype
#undef SAMPLE_FORMAT
#undef need_clipping
#if DEPTH == 16
#define need_clipping 1
#define ftype float
#define stype int16_t
#define min INT16_MIN
#define max INT16_MAX
#define SAMPLE_FORMAT s16
#elif DEPTH == 31
#define need_clipping 1
#define ftype double
#define stype int32_t
#define min INT32_MIN
#define max INT32_MAX
#define SAMPLE_FORMAT s32
#elif DEPTH == 32
#define need_clipping 0
#define ftype float
#define stype float
#define min -1.f
#define max  1.f
#define SAMPLE_FORMAT flt
#else
#define need_clipping 0
#define ftype double
#define stype double
#define min -1.0
#define max  1.0
#define SAMPLE_FORMAT dbl
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define ft3(a,b)   a##_##b
#define ft2(a,b)   ft3(a,b)
#define ft(a)      ft2(a, ftype)

static void fn(biquad_di)(BiquadsContext *s,
                          const void *input, void *output, int len,
                          void *cache, int *clippings, int disabled)
{
    const stype *ibuf = input;
    stype *obuf = output;
    ftype *fcache = cache;
    ftype i1 = fcache[0], i2 = fcache[1], o1 = fcache[2], o2 = fcache[3];
    const ftype *a = ft(s->a);
    const ftype *b = ft(s->b);
    const ftype a1 = -a[1];
    const ftype a2 = -a[2];
    const ftype b0 = b[0];
    const ftype b1 = b[1];
    const ftype b2 = b[2];
    const ftype wet = s->mix;
    const ftype dry = F(1.0) - wet;
    ftype out;
    int i;

    for (i = 0; i+1 < len; i++) {
        o2 = i2 * b2 + i1 * b1 + ibuf[i] * b0 + o2 * a2 + o1 * a1;
        i2 = ibuf[i];
        out = o2 * wet + i2 * dry;
        if (disabled) {
            obuf[i] = i2;
        } else if (need_clipping && out < min) {
            (*clippings)++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            (*clippings)++;
            obuf[i] = max;
        } else {
            obuf[i] = out;
        }
        i++;
        o1 = i1 * b2 + i2 * b1 + ibuf[i] * b0 + o1 * a2 + o2 * a1;
        i1 = ibuf[i];
        out = o1 * wet + i1 * dry;
        if (disabled) {
            obuf[i] = i1;
        } else if (need_clipping && out < min) {
            (*clippings)++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            (*clippings)++;
            obuf[i] = max;
        } else {
            obuf[i] = out;
        }
    }
    if (i < len) {
        ftype o0 = ibuf[i] * b0 + i1 * b1 + i2 * b2 + o1 * a1 + o2 * a2;
        i2 = i1;
        i1 = ibuf[i];
        o2 = o1;
        o1 = o0;
        out = o0 * wet + i1 * dry;
        if (disabled) {
            obuf[i] = i1;
        } else if (need_clipping && out < min) {
            (*clippings)++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            (*clippings)++;
            obuf[i] = max;
        } else {
            obuf[i] = out;
        }
    }
    fcache[0] = i1;
    fcache[1] = i2;
    fcache[2] = o1;
    fcache[3] = o2;
}

static void fn(biquad_dii)(BiquadsContext *s,
                           const void *input, void *output, int len,
                           void *cache, int *clippings, int disabled)
{
    const stype *ibuf = input;
    stype *obuf = output;
    ftype *fcache = cache;
    const ftype *a = ft(s->a);
    const ftype *b = ft(s->b);
    const ftype a1 = -a[1];
    const ftype a2 = -a[2];
    const ftype b0 = b[0];
    const ftype b1 = b[1];
    const ftype b2 = b[2];
    ftype w1 = fcache[0];
    ftype w2 = fcache[1];
    const ftype wet = s->mix;
    const ftype dry = F(1.0) - wet;
    ftype in, out, w0;

    for (int i = 0; i < len; i++) {
        in = ibuf[i];
        w0 = in + a1 * w1 + a2 * w2;
        out = b0 * w0 + b1 * w1 + b2 * w2;
        w2 = w1;
        w1 = w0;
        out = out * wet + in * dry;
        if (disabled) {
            obuf[i] = in;
        } else if (need_clipping && out < min) {
            (*clippings)++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            (*clippings)++;
            obuf[i] = max;
        } else {
            obuf[i] = out;
        }
    }
    fcache[0] = w1;
    fcache[1] = w2;
}

static void fn(biquad_tdi)(BiquadsContext *s,
                           const void *input, void *output, int len,
                           void *cache, int *clippings, int disabled)
{
    const stype *ibuf = input;
    stype *obuf = output;
    ftype *fcache = cache;
    const ftype *a = ft(s->a);
    const ftype *b = ft(s->b);
    const ftype a1 = -a[1];
    const ftype a2 = -a[2];
    const ftype b0 = b[0];
    const ftype b1 = b[1];
    const ftype b2 = b[2];
    ftype s1 = fcache[0];
    ftype s2 = fcache[1];
    ftype s3 = fcache[2];
    ftype s4 = fcache[3];
    const ftype wet = s->mix;
    const ftype dry = F(1.0) - wet;
    ftype in, out;

    for (int i = 0; i < len; i++) {
        ftype t1, t2, t3, t4;
        in = ibuf[i] + s1;
        t1 = in * a1 + s2;
        t2 = in * a2;
        t3 = in * b1 + s4;
        t4 = in * b2;
        out = b0 * in + s3;
        out = out * wet + in * dry;
        s1 = t1; s2 = t2; s3 = t3; s4 = t4;
        if (disabled) {
            obuf[i] = in;
        } else if (need_clipping && out < min) {
            (*clippings)++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            (*clippings)++;
            obuf[i] = max;
        } else {
            obuf[i] = out;
        }
    }

    fcache[0] = s1;
    fcache[1] = s2;
    fcache[2] = s3;
    fcache[3] = s4;
}

static void fn(biquad_tdii)(BiquadsContext *s,
                            const void *input, void *output, int len,
                            void *cache, int *clippings, int disabled)
{
    const stype *ibuf = input;
    stype *obuf = output;
    ftype *fcache = cache;
    const ftype *a = ft(s->a);
    const ftype *b = ft(s->b);
    const ftype a1 = -a[1];
    const ftype a2 = -a[2];
    const ftype b0 = b[0];
    const ftype b1 = b[1];
    const ftype b2 = b[2];
    ftype w1 = fcache[0];
    ftype w2 = fcache[1];
    const ftype wet = s->mix;
    const ftype dry = F(1.0) - wet;
    ftype in, out;

    for (int i = 0; i < len; i++) {
        in = ibuf[i];
        out = b0 * in + w1;
        w1 = b1 * in + w2 + a1 * out;
        w2 = b2 * in + a2 * out;
        out = out * wet + in * dry;
        if (disabled) {
            obuf[i] = in;
        } else if (need_clipping && out < min) {
            (*clippings)++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            (*clippings)++;
            obuf[i] = max;
        } else {
            obuf[i] = out;
        }
    }
    fcache[0] = w1;
    fcache[1] = w2;
}

static void fn(biquad_latt)(BiquadsContext *s,
                           const void *input, void *output, int len,
                           void *cache, int *clippings, int disabled)
{
    const stype *ibuf = input;
    stype *obuf = output;
    ftype *fcache = cache;
    const ftype *a = ft(s->a);
    const ftype *b = ft(s->b);
    const ftype k0 = a[1];
    const ftype k1 = a[2];
    const ftype v0 = b[0];
    const ftype v1 = b[1];
    const ftype v2 = b[2];
    ftype s0 = fcache[0];
    ftype s1 = fcache[1];
    const ftype wet = s->mix;
    const ftype dry = F(1.0) - wet;
    ftype in, out;
    ftype t0, t1;

    for (int i = 0; i < len; i++) {
        out  = F(0.0);
        in   = ibuf[i];
        t0   = in - k1 * s0;
        t1   = t0 * k1 + s0;
        out += t1 * v2;

        t0    = t0 - k0 * s1;
        t1    = t0 * k0 + s1;
        out  += t1 * v1;

        out  += t0 * v0;
        s0    = t1;
        s1    = t0;

        out = out * wet + in * dry;
        if (disabled) {
            obuf[i] = in;
        } else if (need_clipping && out < min) {
            (*clippings)++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            (*clippings)++;
            obuf[i] = max;
        } else {
            obuf[i] = out;
        }
    }
    fcache[0] = s0;
    fcache[1] = s1;
}

static void fn(biquad_svf)(BiquadsContext *s,
                           const void *input, void *output, int len,
                           void *cache, int *clippings, int disabled)
{
    const stype *ibuf = input;
    stype *obuf = output;
    ftype *fcache = cache;
    const ftype *a = ft(s->a);
    const ftype *b = ft(s->b);
    const ftype a1 = a[1];
    const ftype a2 = a[2];
    const ftype b0 = b[0];
    const ftype b1 = b[1];
    const ftype b2 = b[2];
    ftype s0 = fcache[0];
    ftype s1 = fcache[1];
    const ftype wet = s->mix;
    const ftype dry = F(1.0) - wet;
    ftype in, out;
    ftype t0, t1;

    for (int i = 0; i < len; i++) {
        in  = ibuf[i];
        out = b2 * in + s0;
        t0  = b0 * in + a1 * s0 + s1;
        t1  = b1 * in + a2 * s0;
        s0  = t0;
        s1  = t1;

        out = out * wet + in * dry;
        if (disabled) {
            obuf[i] = in;
        } else if (need_clipping && out < min) {
            (*clippings)++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            (*clippings)++;
            obuf[i] = max;
        } else {
            obuf[i] = out;
        }
    }
    fcache[0] = s0;
    fcache[1] = s1;
}

static void fn(biquad_zdf)(BiquadsContext *s,
                           const void *input, void *output, int len,
                           void *cache, int *clippings, int disabled)
{
    const stype *ibuf = input;
    stype *obuf = output;
    ftype *fcache = cache;
    const ftype *a = ft(s->a);
    const ftype *b = ft(s->b);
    const ftype m0 = b[0];
    const ftype m1 = b[1];
    const ftype m2 = b[2];
    const ftype a0 = a[0];
    const ftype a1 = a[1];
    const ftype a2 = a[2];
    ftype b0 = fcache[0];
    ftype b1 = fcache[1];
    const ftype wet = s->mix;
    const ftype dry = F(1.0) - wet;
    ftype out;

    for (int i = 0; i < len; i++) {
        const ftype in = ibuf[i];
        const ftype v0 = in;
        const ftype v3 = v0 - b1;
        const ftype v1 = a0 * b0 + a1 * v3;
        const ftype v2 = b1 + a1 * b0 + a2 * v3;

        b0 = F(2.0) * v1 - b0;
        b1 = F(2.0) * v2 - b1;

        out = m0 * v0 + m1 * v1 + m2 * v2;
        out = out * wet + in * dry;
        if (disabled) {
            obuf[i] = in;
        } else if (need_clipping && out < min) {
            (*clippings)++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            (*clippings)++;
            obuf[i] = max;
        } else {
            obuf[i] = out;
        }
    }
    fcache[0] = b0;
    fcache[1] = b1;
}
