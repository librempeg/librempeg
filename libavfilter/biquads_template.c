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

#undef imin
#undef imax
#undef min
#undef max
#undef atype
#undef itype
#undef stype
#undef ftype
#undef ISNORMAL
#undef IDEPTH
#undef FRACT
#undef RSHIFT
#undef SAMPLE_FORMAT
#undef LRINT
#undef need_clipping
#undef uhalf
#if DEPTH == 8
#define IDEPTH 6
#define need_clipping 1
#define FRACT(x) ((x) & ((1L << IDEPTH)-1))
#define RSHIFT(x) ((x) >> IDEPTH)
#define LRINT lrintf
#define atype int16_t
#define itype int8_t
#define ftype float
#define stype uint8_t
#define imin -(1LL << (IDEPTH*2+1))
#define imax ((1LL << (IDEPTH*2+1))-1)
#define min INT8_MIN
#define max INT8_MAX
#define uhalf 0x80
#define ISNORMAL(x) ((x)==(x))
#define SAMPLE_FORMAT u8p
#elif DEPTH == 16
#define IDEPTH 14
#define FRACT(x) ((x) & ((1L << IDEPTH)-1))
#define RSHIFT(x) ((x) >> IDEPTH)
#define LRINT lrintf
#define need_clipping 1
#define atype int32_t
#define itype int16_t
#define ftype float
#define stype int16_t
#define imin -(1LL << (IDEPTH*2+1))
#define imax ((1LL << (IDEPTH*2+1))-1)
#define min INT16_MIN
#define max INT16_MAX
#define uhalf 0
#define ISNORMAL(x) ((x)==(x))
#define SAMPLE_FORMAT s16p
#elif DEPTH == 31
#define IDEPTH 30
#define FRACT(x) ((x) & ((1LL << IDEPTH)-1))
#define RSHIFT(x) ((x) >> IDEPTH)
#define LRINT lrint
#define need_clipping 1
#define atype int64_t
#define itype int32_t
#define ftype double
#define stype int32_t
#define imin -(1LL << (IDEPTH*2+1))
#define imax ((1LL << (IDEPTH*2+1))-1)
#define min INT32_MIN
#define max INT32_MAX
#define uhalf 0
#define ISNORMAL(x) ((x)==(x))
#define SAMPLE_FORMAT s32p
#elif DEPTH == 32
#define IDEPTH 0
#define FRACT(x) ((x)!=(x))
#define RSHIFT(x) (x)
#define LRINT(x) (x)
#define need_clipping 0
#define atype float
#define itype float
#define ftype float
#define stype float
#define imin -1.f
#define imax  1.f
#define min -1.f
#define max  1.f
#define uhalf 0
#define ISNORMAL isnormal
#define SAMPLE_FORMAT fltp
#else
#define IDEPTH 0
#define FRACT(x) ((x)!=(x))
#define RSHIFT(x) (x)
#define LRINT(x) (x)
#define need_clipping 0
#define atype double
#define itype double
#define ftype double
#define stype double
#define imin -1.0
#define imax  1.0
#define min -1.0
#define max  1.0
#define uhalf 0
#define ISNORMAL isnormal
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(BiquadContext) {
    ftype a[3];
    ftype b[3];
    ftype c[4];
#if DEPTH == 8 || DEPTH == 16 || DEPTH == 31
    itype ia[3];
    itype ib[3];
    itype ic[4];
#endif
    ftype mix;
    itype fraction;
    unsigned clip;
} fn(BiquadContext);

static int fn(init_biquad)(AVFilterContext *ctx, void **st,
                          const int nb_channels,
                          const int block_samples, const int reset,
                          const double a[3], const double b[3],
                          const double mix)
{
    fn(BiquadContext) *stc;

    if (!st[0])
        st[0] = av_calloc(nb_channels * (1+(block_samples>0)), sizeof(*stc));
    if (!st[0])
        return AVERROR(ENOMEM);

    stc = st[0];
    for (int ch = 0; ch < nb_channels * (1+(block_samples>0)); ch++) {
        fn(BiquadContext) *st = &stc[ch];

        if (reset)
            memset(st->c, 0, sizeof(st->c));

        st->mix = mix;
        st->a[0] = a[0];
        st->a[1] = a[1];
        st->a[2] = a[2];
        st->b[0] = b[0];
        st->b[1] = b[1];
        st->b[2] = b[2];

#if DEPTH == 8 || DEPTH == 16 || DEPTH == 31
        st->ia[0] = LRINT(a[0] * (1LL << IDEPTH));
        st->ia[1] = LRINT(a[1] * (1LL << IDEPTH));
        st->ia[2] = LRINT(a[2] * (1LL << IDEPTH));
        st->ib[0] = LRINT(b[0] * (1LL << IDEPTH));
        st->ib[1] = LRINT(b[1] * (1LL << IDEPTH));
        st->ib[2] = LRINT(b[2] * (1LL << IDEPTH));
#endif
    }

    return 0;
}

#if CLIP_RESET
static void fn(clip_reset)(AVFilterContext *ctx,
                           void *st, const int nb_channels)
{
    fn(BiquadContext) *state = st;

    for (int ch = 0; ch < nb_channels; ch++) {
        fn(BiquadContext) *stc = &state[ch];

        if (stc->clip > 0) {
            av_log(ctx, AV_LOG_WARNING, "Channel %d clipping %d times. Please reduce gain.\n",
                   ch, stc->clip);
            stc->clip = 0;
        }
    }
}
#endif

#if BIQUAD_DI
static void fn(biquad_di)(void *st,
                          const void *input, void *output, const int len,
                          const int ch, const int disabled)
{
    const stype *restrict ibuf = input;
    stype *restrict obuf = output;
    fn(BiquadContext) *state = st;
    fn(BiquadContext) *stc = &state[ch];
#if DEPTH == 8 || DEPTH == 16 || DEPTH == 31
    itype *fcache = stc->ic;
#else
    itype *fcache = stc->c;
#endif
    itype i1 = fcache[0], i2 = fcache[1], o1 = fcache[2], o2 = fcache[3];
    itype fraction = stc->fraction;
#if DEPTH == 8 || DEPTH == 16 || DEPTH == 31
    const itype *a = stc->ia;
    const itype *b = stc->ib;
#else
    const itype *a = stc->a;
    const itype *b = stc->b;
#endif
    const itype a1 = -a[1];
    const itype a2 = -a[2];
    const itype b0 = b[0];
    const itype b1 = b[1];
    const itype b2 = b[2];
    const ftype wet = stc->mix;
    const ftype dry = F(1.0) - wet;

    for (int i = 0; i < len; i++) {
        itype out, in = ibuf[i] - uhalf;
        atype o0;

        o0  = fraction;
        o0 += (atype)in * b0;
        o0 += (atype)i1 * b1;
        o0 += (atype)i2 * b2;
        o0 += (atype)o1 * a1;
        o0 += (atype)o2 * a2;

        if (need_clipping && o0 > imax) {
            stc->clip++;
            o0 = imax;
        } else if (need_clipping && o0 < imin) {
            stc->clip++;
            o0 = imin;
        }

        fraction = FRACT(o0);
        out = RSHIFT(o0);
        i2 = i1;
        i1 = in;
        o2 = o1;
        o1 = out;

        out = out * wet + in * dry;
        if (disabled) {
            obuf[i] = in + uhalf;
        } else {
            obuf[i] = out + uhalf;
        }
    }
    fcache[0] = ISNORMAL(i1) ? i1 : F(0.0);
    fcache[1] = ISNORMAL(i2) ? i2 : F(0.0);
    fcache[2] = ISNORMAL(o1) ? o1 : F(0.0);
    fcache[3] = ISNORMAL(o2) ? o2 : F(0.0);

    stc->fraction = fraction;
}
#endif

#if BIQUAD_DII
static void fn(biquad_dii)(void *st,
                           const void *input, void *output, const int len,
                           const int ch, const int disabled)
{
    const stype *restrict ibuf = input;
    stype *restrict obuf = output;
    fn(BiquadContext) *state = st;
    fn(BiquadContext) *stc = &state[ch];
    ftype *fcache = stc->c;
    const ftype *a = stc->a;
    const ftype *b = stc->b;
    const ftype a1 = -a[1];
    const ftype a2 = -a[2];
    const ftype b0 = b[0];
    const ftype b1 = b[1];
    const ftype b2 = b[2];
    ftype w1 = fcache[0];
    ftype w2 = fcache[1];
    const ftype wet = stc->mix;
    const ftype dry = F(1.0) - wet;

    for (int i = 0; i < len; i++) {
        ftype in = ibuf[i] - uhalf;
        ftype w0 = in + a1 * w1 + a2 * w2;
        ftype out = b0 * w0 + b1 * w1 + b2 * w2;
        w2 = w1;
        w1 = w0;
        out = out * wet + in * dry;
        if (disabled) {
            obuf[i] = in + uhalf;
        } else if (need_clipping && out < min) {
            stc->clip++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            stc->clip++;
            obuf[i] = max;
        } else {
            obuf[i] = out + uhalf;
        }
    }
    fcache[0] = isnormal(w1) ? w1 : F(0.0);
    fcache[1] = isnormal(w2) ? w2 : F(0.0);
}
#endif

#if BIQUAD_TDI
static void fn(biquad_tdi)(void *st,
                           const void *input, void *output, const int len,
                           const int ch, const int disabled)
{
    const stype *restrict ibuf = input;
    stype *restrict obuf = output;
    fn(BiquadContext) *state = st;
    fn(BiquadContext) *stc = &state[ch];
    ftype *fcache = stc->c;
    const ftype *a = (const ftype *)stc->a;
    const ftype *b = (const ftype *)stc->b;
    const ftype a1 = -a[1];
    const ftype a2 = -a[2];
    const ftype b0 = b[0];
    const ftype b1 = b[1];
    const ftype b2 = b[2];
    ftype s1 = fcache[0];
    ftype s2 = fcache[1];
    ftype s3 = fcache[2];
    ftype s4 = fcache[3];
    const ftype wet = stc->mix;
    const ftype dry = F(1.0) - wet;
    ftype out;

    for (int i = 0; i < len; i++) {
        ftype t0, t1, t2, t3, t4;
        ftype in = ibuf[i] - uhalf;
        t0 = in + s1;
        t1 = t0 * a1 + s2;
        t2 = t0 * a2;
        t3 = t0 * b1 + s4;
        t4 = t0 * b2;
        out = b0 * t0 + s3;
        out = out * wet + in * dry;
        s1 = t1; s2 = t2; s3 = t3; s4 = t4;
        if (disabled) {
            obuf[i] = in + uhalf;
        } else if (need_clipping && out < min) {
            stc->clip++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            stc->clip++;
            obuf[i] = max;
        } else {
            obuf[i] = out + uhalf;
        }
    }

    fcache[0] = isnormal(s1) ? s1 : F(0.0);
    fcache[1] = isnormal(s2) ? s2 : F(0.0);
    fcache[2] = isnormal(s3) ? s3 : F(0.0);
    fcache[3] = isnormal(s4) ? s4 : F(0.0);
}
#endif

#if BIQUAD_TDII
static void fn(biquad_tdii)(void *st,
                            const void *input, void *output, const int len,
                            const int ch, const int disabled)
{
    const stype *restrict ibuf = input;
    stype *restrict obuf = output;
    fn(BiquadContext) *state = st;
    fn(BiquadContext) *stc = &state[ch];
    ftype *fcache = stc->c;
    const ftype *a = stc->a;
    const ftype *b = stc->b;
    const ftype a1 = -a[1];
    const ftype a2 = -a[2];
    const ftype b0 = b[0];
    const ftype b1 = b[1];
    const ftype b2 = b[2];
    ftype w1 = fcache[0];
    ftype w2 = fcache[1];
    const ftype wet = stc->mix;
    const ftype dry = F(1.0) - wet;

    for (int i = 0; i < len; i++) {
        ftype in = ibuf[i] - uhalf;
        ftype out = b0 * in + w1;
        w1 = b1 * in + w2 + a1 * out;
        w2 = b2 * in + a2 * out;
        out = out * wet + in * dry;
        if (disabled) {
            obuf[i] = in + uhalf;
        } else if (need_clipping && out < min) {
            stc->clip++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            stc->clip++;
            obuf[i] = max;
        } else {
            obuf[i] = out + uhalf;
        }
    }
    fcache[0] = isnormal(w1) ? w1 : F(0.0);
    fcache[1] = isnormal(w2) ? w2 : F(0.0);
}
#endif

#if BIQUAD_LATT
static void fn(biquad_latt)(void *st,
                           const void *input, void *output, const int len,
                           const int ch, const int disabled)
{
    const stype *restrict ibuf = input;
    stype *restrict obuf = output;
    fn(BiquadContext) *state = st;
    fn(BiquadContext) *stc = &state[ch];
    ftype *fcache = stc->c;
    const ftype *a = (const ftype *)stc->a;
    const ftype *b = (const ftype *)stc->b;
    const ftype k0 = a[1];
    const ftype k1 = a[2];
    const ftype v0 = b[0];
    const ftype v1 = b[1];
    const ftype v2 = b[2];
    ftype s0 = fcache[0];
    ftype s1 = fcache[1];
    const ftype wet = stc->mix;
    const ftype dry = F(1.0) - wet;
    ftype t0, t1;

    for (int i = 0; i < len; i++) {
        ftype out = F(0.0);
        ftype in = ibuf[i] - uhalf;
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
            obuf[i] = in + uhalf;
        } else if (need_clipping && out < min) {
            stc->clip++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            stc->clip++;
            obuf[i] = max;
        } else {
            obuf[i] = out + uhalf;
        }
    }
    fcache[0] = isnormal(s0) ? s0 : F(0.0);
    fcache[1] = isnormal(s1) ? s1 : F(0.0);
}
#endif

#if BIQUAD_SVF
static void fn(biquad_svf)(void *st,
                           const void *input, void *output, const int len,
                           const int ch, const int disabled)
{
    const stype *restrict ibuf = input;
    stype *restrict obuf = output;
    fn(BiquadContext) *state = st;
    fn(BiquadContext) *stc = &state[ch];
    ftype *fcache = stc->c;
    const ftype *a = stc->a;
    const ftype *b = stc->b;
    const ftype a1 = a[1];
    const ftype a2 = a[2];
    const ftype b0 = b[0];
    const ftype b1 = b[1];
    const ftype b2 = b[2];
    ftype s0 = fcache[0];
    ftype s1 = fcache[1];
    const ftype wet = stc->mix;
    const ftype dry = F(1.0) - wet;
    ftype t0, t1;

    for (int i = 0; i < len; i++) {
        ftype in = ibuf[i] - uhalf;
        ftype out = b2 * in + s0;
        t0  = b0 * in + a1 * s0 + s1;
        t1  = b1 * in + a2 * s0;
        s0  = t0;
        s1  = t1;

        out = out * wet + in * dry;
        if (disabled) {
            obuf[i] = in + uhalf;
        } else if (need_clipping && out < min) {
            stc->clip++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            stc->clip++;
            obuf[i] = max;
        } else {
            obuf[i] = out + uhalf;
        }
    }
    fcache[0] = isnormal(s0) ? s0 : F(0.0);
    fcache[1] = isnormal(s1) ? s1 : F(0.0);
}
#endif

#if BIQUAD_WDF
static void fn(biquad_wdf)(void *st,
                           const void *input, void *output, const int len,
                           const int ch, const int disabled)
{
    const stype *restrict ibuf = input;
    stype *restrict obuf = output;
    fn(BiquadContext) *state = st;
    fn(BiquadContext) *stc = &state[ch];
    ftype *fcache = stc->c;
    const ftype *a = (const ftype *)stc->a;
    const ftype *b = (const ftype *)stc->b;
    const ftype b0 = b[0];
    const ftype b1 = -b[1];
    const ftype b2 = -b[2];
    const ftype a1 = a[1];
    const ftype a2 = a[2];
    ftype w0 = fcache[0];
    ftype w1 = fcache[1];
    const ftype wet = stc->mix;
    const ftype dry = F(1.0) - wet;

    for (int i = 0; i < len; i++) {
        const ftype in = ibuf[i] - uhalf;
        ftype out, v0, v1;

        out = in * b0 + w0 + w1;

        v0 = in * b1 + out * a1 + w1;
        v1 = out * a2 + in * b2 - w0;

        w0 = v0;
        w1 = v1;

        out = out * wet + in * dry;
        if (disabled) {
            obuf[i] = in + uhalf;
        } else if (need_clipping && out < min) {
            stc->clip++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            stc->clip++;
            obuf[i] = max;
        } else {
            obuf[i] = out + uhalf;
        }
    }
    fcache[0] = isnormal(w0) ? w0 : F(0.0);
    fcache[1] = isnormal(w1) ? w1 : F(0.0);
}
#endif

#if BIQUAD_ZDF
static void fn(biquad_zdf)(void *st,
                           const void *input, void *output, const int len,
                           const int ch, const int disabled)
{
    const stype *restrict ibuf = input;
    stype *restrict obuf = output;
    fn(BiquadContext) *state = st;
    fn(BiquadContext) *stc = &state[ch];
    const ftype *a = stc->a;
    const ftype *b = stc->b;
    const ftype m0 = b[0];
    const ftype m1 = b[1];
    const ftype m2 = b[2];
    const ftype a0 = a[0];
    const ftype a1 = a[1];
    const ftype a2 = a[2];
    ftype b0 = stc->c[0];
    ftype b1 = stc->c[1];
    const ftype wet = stc->mix;
    const ftype dry = F(1.0) - wet;

    for (int i = 0; i < len; i++) {
        const ftype in = ibuf[i] - uhalf;
        const ftype v0 = in;
        const ftype v3 = v0 - b1;
        const ftype v1 = a0 * b0 + a1 * v3;
        const ftype v2 = b1 + a1 * b0 + a2 * v3;
        ftype out;

        b0 = F(2.0) * v1 - b0;
        b1 = F(2.0) * v2 - b1;

        out = m0 * v0 + m1 * v1 + m2 * v2;
        out = out * wet + in * dry;
        if (disabled) {
            obuf[i] = in + uhalf;
        } else if (need_clipping && out < min) {
            stc->clip++;
            obuf[i] = min;
        } else if (need_clipping && out > max) {
            stc->clip++;
            obuf[i] = max;
        } else {
            obuf[i] = out + uhalf;
        }
    }
    stc->c[0] = isnormal(b0) ? b0 : F(0.0);
    stc->c[1] = isnormal(b1) ? b1 : F(0.0);
}
#endif
