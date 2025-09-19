/*
 * Copyright (c) Lynne
 *
 * Power of two FFT:
 * Copyright (c) Lynne
 * Copyright (c) 2008 Loren Merritt
 * Copyright (c) 2002 Fabrice Bellard
 * Partly based on libdjbfft by D. J. Bernstein
 *
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

#include "libavutil/cpu.h"
#include "libavutil/mem.h"

#define TABLE_DEF(name, size) \
    DECLARE_ALIGNED(32, TXSample, TX_TAB(ff_tx_tab_ ##name))[size]

#define SR_POW2_TABLES \
    SR_TABLE(8)        \
    SR_TABLE(16)       \
    SR_TABLE(32)       \
    SR_TABLE(64)       \
    SR_TABLE(128)      \
    SR_TABLE(256)      \
    SR_TABLE(512)      \
    SR_TABLE(1024)     \
    SR_TABLE(2048)     \
    SR_TABLE(4096)     \
    SR_TABLE(8192)     \
    SR_TABLE(16384)    \
    SR_TABLE(32768)    \
    SR_TABLE(65536)    \
    SR_TABLE(131072)   \
    SR_TABLE(262144)   \
    SR_TABLE(524288)   \
    SR_TABLE(1048576)   \
    SR_TABLE(2097152)   \

#define SR_TABLE(len) \
    TABLE_DEF(len, len/4 + 1);
/* Power of two tables */
SR_POW2_TABLES
#undef SR_TABLE

/* Other factors' tables */
TABLE_DEF(53, 12);
TABLE_DEF( 7,  6);
TABLE_DEF( 9,  8);

typedef struct FFTabInitData {
    void (*func)(void);
    int factors[TX_MAX_SUB]; /* Must be sorted high -> low */
} FFTabInitData;

#define SR_TABLE(len)                                              \
static av_cold void TX_TAB(ff_tx_init_tab_ ##len)(void)            \
{                                                                  \
    double freq = 2*M_PI/len;                                      \
    TXSample *tab = TX_TAB(ff_tx_tab_ ##len);                      \
                                                                   \
    for (int i = 0; i < len/4; i++)                                \
        *tab++ = RESCALE(cos(i*freq));                             \
                                                                   \
    *tab = 0;                                                      \
}
SR_POW2_TABLES
#undef SR_TABLE

static void (*const sr_tabs_init_funcs[])(void) = {
#define SR_TABLE(len) TX_TAB(ff_tx_init_tab_ ##len),
    SR_POW2_TABLES
#undef SR_TABLE
};

static AVOnce sr_tabs_init_once[] = {
#define SR_TABLE(len) AV_ONCE_INIT,
    SR_POW2_TABLES
#undef SR_TABLE
};

static av_cold void TX_TAB(ff_tx_init_tab_53)(void)
{
    /* 5pt, doubled to eliminate AVX lane shuffles */
    TX_TAB(ff_tx_tab_53)[0] = RESCALE(cos(2 * M_PI /  5));
    TX_TAB(ff_tx_tab_53)[1] = RESCALE(cos(2 * M_PI /  5));
    TX_TAB(ff_tx_tab_53)[2] = RESCALE(cos(2 * M_PI / 10));
    TX_TAB(ff_tx_tab_53)[3] = RESCALE(cos(2 * M_PI / 10));
    TX_TAB(ff_tx_tab_53)[4] = RESCALE(sin(2 * M_PI /  5));
    TX_TAB(ff_tx_tab_53)[5] = RESCALE(sin(2 * M_PI /  5));
    TX_TAB(ff_tx_tab_53)[6] = RESCALE(sin(2 * M_PI / 10));
    TX_TAB(ff_tx_tab_53)[7] = RESCALE(sin(2 * M_PI / 10));

    /* 3pt */
    TX_TAB(ff_tx_tab_53)[ 8] = RESCALE(cos(2 * M_PI / 12));
    TX_TAB(ff_tx_tab_53)[ 9] = RESCALE(cos(2 * M_PI / 12));
    TX_TAB(ff_tx_tab_53)[10] = RESCALE(cos(2 * M_PI /  6));
    TX_TAB(ff_tx_tab_53)[11] = RESCALE(cos(8 * M_PI /  6));
}

static av_cold void TX_TAB(ff_tx_init_tab_7)(void)
{
    TX_TAB(ff_tx_tab_7)[0] = RESCALE(cos(2 * M_PI /  7));
    TX_TAB(ff_tx_tab_7)[1] = RESCALE(sin(2 * M_PI /  7));
    TX_TAB(ff_tx_tab_7)[2] = RESCALE(sin(2 * M_PI / 28));
    TX_TAB(ff_tx_tab_7)[3] = RESCALE(cos(2 * M_PI / 28));
    TX_TAB(ff_tx_tab_7)[4] = RESCALE(cos(2 * M_PI / 14));
    TX_TAB(ff_tx_tab_7)[5] = RESCALE(sin(2 * M_PI / 14));
}

static av_cold void TX_TAB(ff_tx_init_tab_9)(void)
{
    TX_TAB(ff_tx_tab_9)[0] = RESCALE(cos(2 * M_PI /  3));
    TX_TAB(ff_tx_tab_9)[1] = RESCALE(sin(2 * M_PI /  3));
    TX_TAB(ff_tx_tab_9)[2] = RESCALE(cos(2 * M_PI /  9));
    TX_TAB(ff_tx_tab_9)[3] = RESCALE(sin(2 * M_PI /  9));
    TX_TAB(ff_tx_tab_9)[4] = RESCALE(cos(2 * M_PI / 36));
    TX_TAB(ff_tx_tab_9)[5] = RESCALE(sin(2 * M_PI / 36));
    TX_TAB(ff_tx_tab_9)[6] = TX_TAB(ff_tx_tab_9)[2] + TX_TAB(ff_tx_tab_9)[5];
    TX_TAB(ff_tx_tab_9)[7] = TX_TAB(ff_tx_tab_9)[3] - TX_TAB(ff_tx_tab_9)[4];
}

static const FFTabInitData nptwo_tabs_init_data[] = {
    { TX_TAB(ff_tx_init_tab_53),      { 15, 5, 3 } },
    { TX_TAB(ff_tx_init_tab_9),       {  9 }       },
    { TX_TAB(ff_tx_init_tab_7),       {  7 }       },
};

static AVOnce nptwo_tabs_init_once[] = {
    AV_ONCE_INIT,
    AV_ONCE_INIT,
    AV_ONCE_INIT,
};

av_cold void TX_TAB(ff_tx_init_tabs)(int len)
{
    int factor_2 = ff_ctz(len);
    if (factor_2) {
        int idx = factor_2 - 3;
        for (int i = 0; i <= idx; i++)
            ff_thread_once(&sr_tabs_init_once[i],
                            sr_tabs_init_funcs[i]);
        len >>= factor_2;
    }

    for (int i = 0; i < FF_ARRAY_ELEMS(nptwo_tabs_init_data); i++) {
        int f, f_idx = 0;

        if (len <= 1)
            return;

        while ((f = nptwo_tabs_init_data[i].factors[f_idx++])) {
            if (f % len)
                continue;

            ff_thread_once(&nptwo_tabs_init_once[i],
                            nptwo_tabs_init_data[i].func);
            len /= f;
            break;
        }
    }
}

static av_always_inline void fft3(TXComplex *out, TXComplex *in,
                                  ptrdiff_t stride)
{
    TXComplex tmp[3];
    const TXSample *tab = TX_TAB(ff_tx_tab_53);
#ifdef TX_INT32
    int64_t mtmp[4];
#endif

    tmp[0] = in[0];
    BF(tmp[1].re, tmp[2].im, in[1].im, in[2].im);
    BF(tmp[1].im, tmp[2].re, in[1].re, in[2].re);

#ifdef TX_INT32
    out[0*stride].re = (int64_t)tmp[0].re + tmp[2].re;
    out[0*stride].im = (int64_t)tmp[0].im + tmp[2].im;
    mtmp[0] = (int64_t)tab[ 8] * tmp[1].re;
    mtmp[1] = (int64_t)tab[ 9] * tmp[1].im;
    mtmp[2] = (int64_t)tab[10] * tmp[2].re;
    mtmp[3] = (int64_t)tab[10] * tmp[2].im;
    out[1*stride].re = tmp[0].re - (mtmp[2] + mtmp[0] + 0x40000000 >> 31);
    out[1*stride].im = tmp[0].im - (mtmp[3] - mtmp[1] + 0x40000000 >> 31);
    out[2*stride].re = tmp[0].re - (mtmp[2] - mtmp[0] + 0x40000000 >> 31);
    out[2*stride].im = tmp[0].im - (mtmp[3] + mtmp[1] + 0x40000000 >> 31);
#else
    out[0*stride].re = tmp[0].re + tmp[2].re;
    out[0*stride].im = tmp[0].im + tmp[2].im;
    tmp[1].re = tab[ 8] * tmp[1].re;
    tmp[1].im = tab[ 9] * tmp[1].im;
    tmp[2].re = tab[10] * tmp[2].re;
    tmp[2].im = tab[10] * tmp[2].im;
    out[1*stride].re = tmp[0].re - tmp[2].re + tmp[1].re;
    out[1*stride].im = tmp[0].im - tmp[2].im - tmp[1].im;
    out[2*stride].re = tmp[0].re - tmp[2].re - tmp[1].re;
    out[2*stride].im = tmp[0].im - tmp[2].im + tmp[1].im;
#endif
}

#define DECL_FFT5(NAME, D0, D1, D2, D3, D4)                         \
static av_always_inline void NAME(TXComplex *out, TXComplex *in,    \
                                  ptrdiff_t stride)                 \
{                                                                   \
    TXComplex dc, z0[4], t[6];                                      \
    const TXSample *tab = TX_TAB(ff_tx_tab_53);                     \
                                                                    \
    dc = in[0];                                                     \
    BF(t[1].im, t[0].re, in[1].re, in[4].re);                       \
    BF(t[1].re, t[0].im, in[1].im, in[4].im);                       \
    BF(t[3].im, t[2].re, in[2].re, in[3].re);                       \
    BF(t[3].re, t[2].im, in[2].im, in[3].im);                       \
                                                                    \
    out[D0*stride].re = dc.re + (TXUSample)t[0].re + t[2].re;        \
    out[D0*stride].im = dc.im + (TXUSample)t[0].im + t[2].im;        \
                                                                    \
    SMUL(t[4].re, t[0].re, tab[0], tab[2], t[2].re, t[0].re);       \
    SMUL(t[4].im, t[0].im, tab[0], tab[2], t[2].im, t[0].im);       \
    CMUL(t[5].re, t[1].re, tab[4], tab[6], t[3].re, t[1].re);       \
    CMUL(t[5].im, t[1].im, tab[4], tab[6], t[3].im, t[1].im);       \
                                                                    \
    BF(z0[0].re, z0[3].re, t[0].re, t[1].re);                       \
    BF(z0[0].im, z0[3].im, t[0].im, t[1].im);                       \
    BF(z0[2].re, z0[1].re, t[4].re, t[5].re);                       \
    BF(z0[2].im, z0[1].im, t[4].im, t[5].im);                       \
                                                                    \
    out[D1*stride].re = dc.re + (TXUSample)z0[3].re;                 \
    out[D1*stride].im = dc.im + (TXUSample)z0[0].im;                 \
    out[D2*stride].re = dc.re + (TXUSample)z0[2].re;                 \
    out[D2*stride].im = dc.im + (TXUSample)z0[1].im;                 \
    out[D3*stride].re = dc.re + (TXUSample)z0[1].re;                 \
    out[D3*stride].im = dc.im + (TXUSample)z0[2].im;                 \
    out[D4*stride].re = dc.re + (TXUSample)z0[0].re;                 \
    out[D4*stride].im = dc.im + (TXUSample)z0[3].im;                 \
}

DECL_FFT5(fft5,     0,  1,  2,  3,  4)
DECL_FFT5(fft5_m1,  0,  6, 12,  3,  9)
DECL_FFT5(fft5_m2, 10,  1,  7, 13,  4)
DECL_FFT5(fft5_m3,  5, 11,  2,  8, 14)

static av_always_inline void fft7(TXComplex *out, TXComplex *in,
                                  ptrdiff_t stride)
{
    TXComplex dc, t[6], z[3];
    const TXComplex *tab = (const TXComplex *)TX_TAB(ff_tx_tab_7);
#ifdef TX_INT32
    int64_t mtmp[12];
#endif

    dc = in[0];
    BF(t[1].re, t[0].re, in[1].re, in[6].re);
    BF(t[1].im, t[0].im, in[1].im, in[6].im);
    BF(t[3].re, t[2].re, in[2].re, in[5].re);
    BF(t[3].im, t[2].im, in[2].im, in[5].im);
    BF(t[5].re, t[4].re, in[3].re, in[4].re);
    BF(t[5].im, t[4].im, in[3].im, in[4].im);

    out[0*stride].re = dc.re + t[0].re + t[2].re + t[4].re;
    out[0*stride].im = dc.im + t[0].im + t[2].im + t[4].im;

#ifdef TX_INT32 /* NOTE: it's possible to do this with 16 mults but 72 adds */
    mtmp[ 0] = ((int64_t)tab[0].re)*t[0].re - ((int64_t)tab[2].re)*t[4].re;
    mtmp[ 1] = ((int64_t)tab[0].re)*t[4].re - ((int64_t)tab[1].re)*t[0].re;
    mtmp[ 2] = ((int64_t)tab[0].re)*t[2].re - ((int64_t)tab[2].re)*t[0].re;
    mtmp[ 3] = ((int64_t)tab[0].re)*t[0].im - ((int64_t)tab[1].re)*t[2].im;
    mtmp[ 4] = ((int64_t)tab[0].re)*t[4].im - ((int64_t)tab[1].re)*t[0].im;
    mtmp[ 5] = ((int64_t)tab[0].re)*t[2].im - ((int64_t)tab[2].re)*t[0].im;

    mtmp[ 6] = ((int64_t)tab[2].im)*t[1].im + ((int64_t)tab[1].im)*t[5].im;
    mtmp[ 7] = ((int64_t)tab[0].im)*t[5].im + ((int64_t)tab[2].im)*t[3].im;
    mtmp[ 8] = ((int64_t)tab[2].im)*t[5].im + ((int64_t)tab[1].im)*t[3].im;
    mtmp[ 9] = ((int64_t)tab[0].im)*t[1].re + ((int64_t)tab[1].im)*t[3].re;
    mtmp[10] = ((int64_t)tab[2].im)*t[3].re + ((int64_t)tab[0].im)*t[5].re;
    mtmp[11] = ((int64_t)tab[2].im)*t[1].re + ((int64_t)tab[1].im)*t[5].re;

    z[0].re = (int32_t)(mtmp[ 0] - ((int64_t)tab[1].re)*t[2].re + 0x40000000 >> 31);
    z[1].re = (int32_t)(mtmp[ 1] - ((int64_t)tab[2].re)*t[2].re + 0x40000000 >> 31);
    z[2].re = (int32_t)(mtmp[ 2] - ((int64_t)tab[1].re)*t[4].re + 0x40000000 >> 31);
    z[0].im = (int32_t)(mtmp[ 3] - ((int64_t)tab[2].re)*t[4].im + 0x40000000 >> 31);
    z[1].im = (int32_t)(mtmp[ 4] - ((int64_t)tab[2].re)*t[2].im + 0x40000000 >> 31);
    z[2].im = (int32_t)(mtmp[ 5] - ((int64_t)tab[1].re)*t[4].im + 0x40000000 >> 31);

    t[0].re = (int32_t)(mtmp[ 6] - ((int64_t)tab[0].im)*t[3].im + 0x40000000 >> 31);
    t[2].re = (int32_t)(mtmp[ 7] - ((int64_t)tab[1].im)*t[1].im + 0x40000000 >> 31);
    t[4].re = (int32_t)(mtmp[ 8] + ((int64_t)tab[0].im)*t[1].im + 0x40000000 >> 31);
    t[0].im = (int32_t)(mtmp[ 9] + ((int64_t)tab[2].im)*t[5].re + 0x40000000 >> 31);
    t[2].im = (int32_t)(mtmp[10] - ((int64_t)tab[1].im)*t[1].re + 0x40000000 >> 31);
    t[4].im = (int32_t)(mtmp[11] - ((int64_t)tab[0].im)*t[3].re + 0x40000000 >> 31);
#else
    z[0].re = tab[0].re*t[0].re - tab[2].re*t[4].re - tab[1].re*t[2].re;
    z[1].re = tab[0].re*t[4].re - tab[1].re*t[0].re - tab[2].re*t[2].re;
    z[2].re = tab[0].re*t[2].re - tab[2].re*t[0].re - tab[1].re*t[4].re;
    z[0].im = tab[0].re*t[0].im - tab[1].re*t[2].im - tab[2].re*t[4].im;
    z[1].im = tab[0].re*t[4].im - tab[1].re*t[0].im - tab[2].re*t[2].im;
    z[2].im = tab[0].re*t[2].im - tab[2].re*t[0].im - tab[1].re*t[4].im;

    /* It's possible to do t[4].re and t[0].im with 2 multiplies only by
     * multiplying the sum of all with the average of the twiddles */

    t[0].re = tab[2].im*t[1].im + tab[1].im*t[5].im - tab[0].im*t[3].im;
    t[2].re = tab[0].im*t[5].im + tab[2].im*t[3].im - tab[1].im*t[1].im;
    t[4].re = tab[2].im*t[5].im + tab[1].im*t[3].im + tab[0].im*t[1].im;
    t[0].im = tab[0].im*t[1].re + tab[1].im*t[3].re + tab[2].im*t[5].re;
    t[2].im = tab[2].im*t[3].re + tab[0].im*t[5].re - tab[1].im*t[1].re;
    t[4].im = tab[2].im*t[1].re + tab[1].im*t[5].re - tab[0].im*t[3].re;
#endif

    BF(t[1].re, z[0].re, z[0].re, t[4].re);
    BF(t[3].re, z[1].re, z[1].re, t[2].re);
    BF(t[5].re, z[2].re, z[2].re, t[0].re);
    BF(t[1].im, z[0].im, z[0].im, t[0].im);
    BF(t[3].im, z[1].im, z[1].im, t[2].im);
    BF(t[5].im, z[2].im, z[2].im, t[4].im);

    out[1*stride].re = dc.re + z[0].re;
    out[1*stride].im = dc.im + t[1].im;
    out[2*stride].re = dc.re + t[3].re;
    out[2*stride].im = dc.im + z[1].im;
    out[3*stride].re = dc.re + z[2].re;
    out[3*stride].im = dc.im + t[5].im;
    out[4*stride].re = dc.re + t[5].re;
    out[4*stride].im = dc.im + z[2].im;
    out[5*stride].re = dc.re + z[1].re;
    out[5*stride].im = dc.im + t[3].im;
    out[6*stride].re = dc.re + t[1].re;
    out[6*stride].im = dc.im + z[0].im;
}

static av_always_inline void fft9(TXComplex *out, TXComplex *in,
                                  ptrdiff_t stride)
{
    const TXComplex *tab = (const TXComplex *)TX_TAB(ff_tx_tab_9);
    TXComplex dc, t[16], w[4], x[5], y[5], z[2];
#ifdef TX_INT32
    int64_t mtmp[12];
#endif

    dc = in[0];
    BF(t[1].re, t[0].re, in[1].re, in[8].re);
    BF(t[1].im, t[0].im, in[1].im, in[8].im);
    BF(t[3].re, t[2].re, in[2].re, in[7].re);
    BF(t[3].im, t[2].im, in[2].im, in[7].im);
    BF(t[5].re, t[4].re, in[3].re, in[6].re);
    BF(t[5].im, t[4].im, in[3].im, in[6].im);
    BF(t[7].re, t[6].re, in[4].re, in[5].re);
    BF(t[7].im, t[6].im, in[4].im, in[5].im);

    w[0].re = t[0].re - t[6].re;
    w[0].im = t[0].im - t[6].im;
    w[1].re = t[2].re - t[6].re;
    w[1].im = t[2].im - t[6].im;
    w[2].re = t[1].re - t[7].re;
    w[2].im = t[1].im - t[7].im;
    w[3].re = t[3].re + t[7].re;
    w[3].im = t[3].im + t[7].im;

    z[0].re = dc.re + t[4].re;
    z[0].im = dc.im + t[4].im;

    z[1].re = t[0].re + t[2].re + t[6].re;
    z[1].im = t[0].im + t[2].im + t[6].im;

    out[0*stride].re = z[0].re + z[1].re;
    out[0*stride].im = z[0].im + z[1].im;

#ifdef TX_INT32
    mtmp[0] = t[1].re - t[3].re + t[7].re;
    mtmp[1] = t[1].im - t[3].im + t[7].im;

    y[3].re = (int32_t)(((int64_t)tab[0].im)*mtmp[0] + 0x40000000 >> 31);
    y[3].im = (int32_t)(((int64_t)tab[0].im)*mtmp[1] + 0x40000000 >> 31);

    mtmp[0] = (int32_t)(((int64_t)tab[0].re)*z[1].re + 0x40000000 >> 31);
    mtmp[1] = (int32_t)(((int64_t)tab[0].re)*z[1].im + 0x40000000 >> 31);
    mtmp[2] = (int32_t)(((int64_t)tab[0].re)*t[4].re + 0x40000000 >> 31);
    mtmp[3] = (int32_t)(((int64_t)tab[0].re)*t[4].im + 0x40000000 >> 31);

    x[3].re = z[0].re  + (int32_t)mtmp[0];
    x[3].im = z[0].im  + (int32_t)mtmp[1];
    z[0].re = in[0].re + (int32_t)mtmp[2];
    z[0].im = in[0].im + (int32_t)mtmp[3];

    mtmp[0] = ((int64_t)tab[1].re)*w[0].re;
    mtmp[1] = ((int64_t)tab[1].re)*w[0].im;
    mtmp[2] = ((int64_t)tab[2].im)*w[0].re;
    mtmp[3] = ((int64_t)tab[2].im)*w[0].im;
    mtmp[4] = ((int64_t)tab[1].im)*w[2].re;
    mtmp[5] = ((int64_t)tab[1].im)*w[2].im;
    mtmp[6] = ((int64_t)tab[2].re)*w[2].re;
    mtmp[7] = ((int64_t)tab[2].re)*w[2].im;

    x[1].re = (int32_t)(mtmp[0] + ((int64_t)tab[2].im)*w[1].re + 0x40000000 >> 31);
    x[1].im = (int32_t)(mtmp[1] + ((int64_t)tab[2].im)*w[1].im + 0x40000000 >> 31);
    x[2].re = (int32_t)(mtmp[2] - ((int64_t)tab[3].re)*w[1].re + 0x40000000 >> 31);
    x[2].im = (int32_t)(mtmp[3] - ((int64_t)tab[3].re)*w[1].im + 0x40000000 >> 31);
    y[1].re = (int32_t)(mtmp[4] + ((int64_t)tab[2].re)*w[3].re + 0x40000000 >> 31);
    y[1].im = (int32_t)(mtmp[5] + ((int64_t)tab[2].re)*w[3].im + 0x40000000 >> 31);
    y[2].re = (int32_t)(mtmp[6] - ((int64_t)tab[3].im)*w[3].re + 0x40000000 >> 31);
    y[2].im = (int32_t)(mtmp[7] - ((int64_t)tab[3].im)*w[3].im + 0x40000000 >> 31);

    y[0].re = (int32_t)(((int64_t)tab[0].im)*t[5].re + 0x40000000 >> 31);
    y[0].im = (int32_t)(((int64_t)tab[0].im)*t[5].im + 0x40000000 >> 31);

#else
    y[3].re = tab[0].im*(t[1].re - t[3].re + t[7].re);
    y[3].im = tab[0].im*(t[1].im - t[3].im + t[7].im);

    x[3].re = z[0].re  + tab[0].re*z[1].re;
    x[3].im = z[0].im  + tab[0].re*z[1].im;
    z[0].re = dc.re + tab[0].re*t[4].re;
    z[0].im = dc.im + tab[0].re*t[4].im;

    x[1].re = tab[1].re*w[0].re + tab[2].im*w[1].re;
    x[1].im = tab[1].re*w[0].im + tab[2].im*w[1].im;
    x[2].re = tab[2].im*w[0].re - tab[3].re*w[1].re;
    x[2].im = tab[2].im*w[0].im - tab[3].re*w[1].im;
    y[1].re = tab[1].im*w[2].re + tab[2].re*w[3].re;
    y[1].im = tab[1].im*w[2].im + tab[2].re*w[3].im;
    y[2].re = tab[2].re*w[2].re - tab[3].im*w[3].re;
    y[2].im = tab[2].re*w[2].im - tab[3].im*w[3].im;

    y[0].re = tab[0].im*t[5].re;
    y[0].im = tab[0].im*t[5].im;
#endif

    x[4].re = x[1].re + x[2].re;
    x[4].im = x[1].im + x[2].im;

    y[4].re = y[1].re - y[2].re;
    y[4].im = y[1].im - y[2].im;
    x[1].re = z[0].re + x[1].re;
    x[1].im = z[0].im + x[1].im;
    y[1].re = y[0].re + y[1].re;
    y[1].im = y[0].im + y[1].im;
    x[2].re = z[0].re + x[2].re;
    x[2].im = z[0].im + x[2].im;
    y[2].re = y[2].re - y[0].re;
    y[2].im = y[2].im - y[0].im;
    x[4].re = z[0].re - x[4].re;
    x[4].im = z[0].im - x[4].im;
    y[4].re = y[0].re - y[4].re;
    y[4].im = y[0].im - y[4].im;

    out[1*stride] = (TXComplex){ x[1].re + y[1].im, x[1].im - y[1].re };
    out[2*stride] = (TXComplex){ x[2].re + y[2].im, x[2].im - y[2].re };
    out[3*stride] = (TXComplex){ x[3].re + y[3].im, x[3].im - y[3].re };
    out[4*stride] = (TXComplex){ x[4].re + y[4].im, x[4].im - y[4].re };
    out[5*stride] = (TXComplex){ x[4].re - y[4].im, x[4].im + y[4].re };
    out[6*stride] = (TXComplex){ x[3].re - y[3].im, x[3].im + y[3].re };
    out[7*stride] = (TXComplex){ x[2].re - y[2].im, x[2].im + y[2].re };
    out[8*stride] = (TXComplex){ x[1].re - y[1].im, x[1].im + y[1].re };
}

static av_always_inline void fft15(TXComplex *out, TXComplex *in,
                                   ptrdiff_t stride)
{
    TXComplex tmp[15];

    for (int i = 0; i < 5; i++)
        fft3(tmp + i, in + i*3, 5);

    fft5_m1(out, tmp +  0, stride);
    fft5_m2(out, tmp +  5, stride);
    fft5_m3(out, tmp + 10, stride);
}

static av_cold int TX_NAME(ff_tx_fft_factor_init)(AVTXContext *s,
                                                  const FFTXCodelet *cd,
                                                  uint64_t flags,
                                                  FFTXCodeletOptions *opts,
                                                  int len, int inv,
                                                  const void *scale)
{
    int ret = 0;
    TX_TAB(ff_tx_init_tabs)(len);

    if (len == 15)
        ret = ff_tx_gen_pfa_input_map(s, opts, 3, 5);
    else if (flags & FF_TX_PRESHUFFLE)
        ret = ff_tx_gen_default_map(s, opts);

    return ret;
}

#define DECL_FACTOR_S(n)                                                       \
static void TX_NAME(ff_tx_fft##n)(AVTXContext *s, void *dst,                   \
                                  void *src, ptrdiff_t stride)                 \
{                                                                              \
    fft##n((TXComplex *)dst, (TXComplex *)src, stride / sizeof(TXComplex));    \
}                                                                              \
static const FFTXCodelet TX_NAME(ff_tx_fft##n##_ns_def) = {                    \
    .name       = TX_NAME_STR("fft" #n "_ns"),                                 \
    .function   = TX_NAME(ff_tx_fft##n),                                       \
    .type       = TX_TYPE(FFT),                                                \
    .flags      = AV_TX_INPLACE | FF_TX_OUT_OF_PLACE |                         \
                  AV_TX_UNALIGNED | FF_TX_PRESHUFFLE,                          \
    .factors[0] = n,                                                           \
    .nb_factors = 1,                                                           \
    .min_len    = n,                                                           \
    .max_len    = n,                                                           \
    .init       = TX_NAME(ff_tx_fft_factor_init),                              \
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,                                         \
    .prio       = FF_TX_PRIO_BASE,                                             \
};

#define DECL_FACTOR_F(n)                                                       \
DECL_FACTOR_S(n)                                                               \
static const FFTXCodelet TX_NAME(ff_tx_fft##n##_fwd_def) = {                   \
    .name       = TX_NAME_STR("fft" #n "_fwd"),                                \
    .function   = TX_NAME(ff_tx_fft##n),                                       \
    .type       = TX_TYPE(FFT),                                                \
    .flags      = AV_TX_INPLACE | FF_TX_OUT_OF_PLACE |                         \
                  AV_TX_UNALIGNED | FF_TX_FORWARD_ONLY,                        \
    .factors[0] = n,                                                           \
    .nb_factors = 1,                                                           \
    .min_len    = n,                                                           \
    .max_len    = n,                                                           \
    .init       = TX_NAME(ff_tx_fft_factor_init),                              \
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,                                         \
    .prio       = FF_TX_PRIO_BASE,                                             \
};

DECL_FACTOR_F(3)
DECL_FACTOR_F(5)
DECL_FACTOR_F(7)
DECL_FACTOR_F(9)
DECL_FACTOR_S(15)

#define BUTTERFLIES(a0, a1, a2, a3)            \
    do {                                       \
        r0=a0.re;                              \
        i0=a0.im;                              \
        r1=a1.re;                              \
        i1=a1.im;                              \
        BF(t3, t5, t5, t1);                    \
        BF(a2.re, a0.re, r0, t5);              \
        BF(a3.im, a1.im, i1, t3);              \
        BF(t4, t6, t2, t6);                    \
        BF(a3.re, a1.re, r1, t4);              \
        BF(a2.im, a0.im, i0, t6);              \
    } while (0)

#define TRANSFORM(a0, a1, a2, a3, wre, wim)    \
    do {                                       \
        CMUL(t1, t2, a2.re, a2.im, wre, -wim); \
        CMUL(t5, t6, a3.re, a3.im, wre,  wim); \
        BUTTERFLIES(a0, a1, a2, a3);           \
    } while (0)

/* z[0...8n-1], w[1...2n-1] */
static inline void TX_NAME(ff_tx_fft_sr_combine)(TXComplex *z,
                                                 const TXSample *cos, int len)
{
    int o1 = 2*len;
    int o2 = 4*len;
    int o3 = 6*len;
    const TXSample *wim = cos + o1 - 7;
    TXUSample t1, t2, t3, t4, t5, t6, r0, i0, r1, i1;

    for (int i = 0; i < len; i += 4) {
        TRANSFORM(z[0], z[o1 + 0], z[o2 + 0], z[o3 + 0], cos[0], wim[7]);
        TRANSFORM(z[2], z[o1 + 2], z[o2 + 2], z[o3 + 2], cos[2], wim[5]);
        TRANSFORM(z[4], z[o1 + 4], z[o2 + 4], z[o3 + 4], cos[4], wim[3]);
        TRANSFORM(z[6], z[o1 + 6], z[o2 + 6], z[o3 + 6], cos[6], wim[1]);

        TRANSFORM(z[1], z[o1 + 1], z[o2 + 1], z[o3 + 1], cos[1], wim[6]);
        TRANSFORM(z[3], z[o1 + 3], z[o2 + 3], z[o3 + 3], cos[3], wim[4]);
        TRANSFORM(z[5], z[o1 + 5], z[o2 + 5], z[o3 + 5], cos[5], wim[2]);
        TRANSFORM(z[7], z[o1 + 7], z[o2 + 7], z[o3 + 7], cos[7], wim[0]);

        z   += 2*4;
        cos += 2*4;
        wim -= 2*4;
    }
}

static av_cold int TX_NAME(ff_tx_fft_sr_codelet_init)(AVTXContext *s,
                                                      const FFTXCodelet *cd,
                                                      uint64_t flags,
                                                      FFTXCodeletOptions *opts,
                                                      int len, int inv,
                                                      const void *scale)
{
    TX_TAB(ff_tx_init_tabs)(len);
    return ff_tx_gen_ptwo_revtab(s, opts);
}

#define DECL_SR_CODELET_DEF(n)                              \
static const FFTXCodelet TX_NAME(ff_tx_fft##n##_ns_def) = { \
    .name       = TX_NAME_STR("fft" #n "_ns"),              \
    .function   = TX_NAME(ff_tx_fft##n##_ns),               \
    .type       = TX_TYPE(FFT),                             \
    .flags      = FF_TX_OUT_OF_PLACE | AV_TX_INPLACE |      \
                  AV_TX_UNALIGNED | FF_TX_PRESHUFFLE,       \
    .factors[0] = 2,                                        \
    .nb_factors = 1,                                        \
    .min_len    = n,                                        \
    .max_len    = n,                                        \
    .init       = TX_NAME(ff_tx_fft_sr_codelet_init),       \
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,                      \
    .prio       = FF_TX_PRIO_BASE,                          \
};

#define DECL_SR_CODELET(n, n2, n4)                                    \
static void TX_NAME(ff_tx_fft##n##_ns)(AVTXContext *s, void *_dst,    \
                                        void *_src, ptrdiff_t stride) \
{                                                                     \
    TXComplex *src = _src;                                            \
    TXComplex *dst = _dst;                                            \
    const TXSample *cos = TX_TAB(ff_tx_tab_##n);                      \
                                                                      \
    TX_NAME(ff_tx_fft##n2##_ns)(s, dst,        src,        stride);   \
    TX_NAME(ff_tx_fft##n4##_ns)(s, dst + n4*2, src + n4*2, stride);   \
    TX_NAME(ff_tx_fft##n4##_ns)(s, dst + n4*3, src + n4*3, stride);   \
    TX_NAME(ff_tx_fft_sr_combine)(dst, cos, n4 >> 1);                 \
}                                                                     \
                                                                      \
DECL_SR_CODELET_DEF(n)

static void TX_NAME(ff_tx_fft2_ns)(AVTXContext *s, void *_dst,
                                   void *_src, ptrdiff_t stride)
{
    TXComplex *src = _src;
    TXComplex *dst = _dst;
    TXComplex tmp;

    BF(tmp.re, dst[0].re, src[0].re, src[1].re);
    BF(tmp.im, dst[0].im, src[0].im, src[1].im);
    dst[1] = tmp;
}

static void TX_NAME(ff_tx_fft4_ns)(AVTXContext *s, void *_dst,
                                   void *_src, ptrdiff_t stride)
{
    TXComplex *src = _src;
    TXComplex *dst = _dst;
    TXSample t1, t2, t3, t4, t5, t6, t7, t8;

    BF(t3, t1, src[0].re, src[1].re);
    BF(t8, t6, src[3].re, src[2].re);
    BF(dst[2].re, dst[0].re, t1, t6);
    BF(t4, t2, src[0].im, src[1].im);
    BF(t7, t5, src[2].im, src[3].im);
    BF(dst[3].im, dst[1].im, t4, t8);
    BF(dst[3].re, dst[1].re, t3, t7);
    BF(dst[2].im, dst[0].im, t2, t5);
}

static void TX_NAME(ff_tx_fft8_ns)(AVTXContext *s, void *_dst,
                                   void *_src, ptrdiff_t stride)
{
    TXComplex *src = _src;
    TXComplex *dst = _dst;
    TXUSample t1, t2, t3, t4, t5, t6, r0, i0, r1, i1;
    const TXSample cos = TX_TAB(ff_tx_tab_8)[1];

    TX_NAME(ff_tx_fft4_ns)(s, dst, src, stride);

    BF(t1, dst[5].re, src[4].re, -src[5].re);
    BF(t2, dst[5].im, src[4].im, -src[5].im);
    BF(t5, dst[7].re, src[6].re, -src[7].re);
    BF(t6, dst[7].im, src[6].im, -src[7].im);

    BUTTERFLIES(dst[0], dst[2], dst[4], dst[6]);
    TRANSFORM(dst[1], dst[3], dst[5], dst[7], cos, cos);
}

static void TX_NAME(ff_tx_fft16_ns)(AVTXContext *s, void *_dst,
                                    void *_src, ptrdiff_t stride)
{
    TXComplex *src = _src;
    TXComplex *dst = _dst;
    const TXSample *cos = TX_TAB(ff_tx_tab_16);

    TXUSample t1, t2, t3, t4, t5, t6, r0, i0, r1, i1;
    TXSample cos_16_1 = cos[1];
    TXSample cos_16_2 = cos[2];
    TXSample cos_16_3 = cos[3];

    TX_NAME(ff_tx_fft8_ns)(s, dst +  0, src +  0, stride);
    TX_NAME(ff_tx_fft4_ns)(s, dst +  8, src +  8, stride);
    TX_NAME(ff_tx_fft4_ns)(s, dst + 12, src + 12, stride);

    t1 = dst[ 8].re;
    t2 = dst[ 8].im;
    t5 = dst[12].re;
    t6 = dst[12].im;
    BUTTERFLIES(dst[0], dst[4], dst[8], dst[12]);

    TRANSFORM(dst[ 2], dst[ 6], dst[10], dst[14], cos_16_2, cos_16_2);
    TRANSFORM(dst[ 1], dst[ 5], dst[ 9], dst[13], cos_16_1, cos_16_3);
    TRANSFORM(dst[ 3], dst[ 7], dst[11], dst[15], cos_16_3, cos_16_1);
}

DECL_SR_CODELET_DEF(2)
DECL_SR_CODELET_DEF(4)
DECL_SR_CODELET_DEF(8)
DECL_SR_CODELET_DEF(16)
DECL_SR_CODELET(32,16,8)
DECL_SR_CODELET(64,32,16)
DECL_SR_CODELET(128,64,32)
DECL_SR_CODELET(256,128,64)
DECL_SR_CODELET(512,256,128)
DECL_SR_CODELET(1024,512,256)
DECL_SR_CODELET(2048,1024,512)
DECL_SR_CODELET(4096,2048,1024)
DECL_SR_CODELET(8192,4096,2048)
DECL_SR_CODELET(16384,8192,4096)
DECL_SR_CODELET(32768,16384,8192)
DECL_SR_CODELET(65536,32768,16384)
DECL_SR_CODELET(131072,65536,32768)
DECL_SR_CODELET(262144,131072,65536)
DECL_SR_CODELET(524288,262144,131072)
DECL_SR_CODELET(1048576,524288,262144)
DECL_SR_CODELET(2097152,1048576,524288)

static av_cold int TX_NAME(ff_tx_fft_init)(AVTXContext *s,
                                           const FFTXCodelet *cd,
                                           uint64_t flags,
                                           FFTXCodeletOptions *opts,
                                           int len, int inv,
                                           const void *scale)
{
    int ret;
    int is_inplace = !!(flags & AV_TX_INPLACE);
    FFTXCodeletOptions sub_opts = {
        .map_dir = is_inplace ? FF_TX_MAP_SCATTER : FF_TX_MAP_GATHER,
    };

    flags &= ~FF_TX_OUT_OF_PLACE; /* We want the subtransform to be */
    flags |=  AV_TX_INPLACE;      /* in-place */
    flags |=  FF_TX_PRESHUFFLE;   /* This function handles the permute step */

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, &sub_opts, len, inv, scale)))
        return ret;

    if (is_inplace && (ret = ff_tx_gen_inplace_map(s, len)))
        return ret;

    return 0;
}

static av_cold int TX_NAME(ff_tx_fft_inplace_small_init)(AVTXContext *s,
                                                         const FFTXCodelet *cd,
                                                         uint64_t flags,
                                                         FFTXCodeletOptions *opts,
                                                         int len, int inv,
                                                         const void *scale)
{
    if (!(s->tmp = av_malloc(len*sizeof(*s->tmp))))
        return AVERROR(ENOMEM);
    flags &= ~AV_TX_INPLACE;
    return TX_NAME(ff_tx_fft_init)(s, cd, flags, opts, len, inv, scale);
}

/* Compilers can't vectorize this anyway without assuming AVX2, which they
 * generally don't, at least without -march=native -mtune=native */
static void TX_NAME(ff_tx_remap)(TXComplex *dst, const TXComplex *src,
                                 const int *map, const int len)
{
    for (int i = 0; i < len; i++)
        dst[i] = src[map[i]];
}

static void TX_NAME(ff_tx_fft)(AVTXContext *s, void *_dst,
                               void *_src, ptrdiff_t stride)
{
    TXComplex *src = _src;
    TXComplex *dst1 = s->flags & AV_TX_INPLACE ? s->tmp : _dst;
    TXComplex *dst2 = _dst;
    int *map = s->sub[0].map;
    int len = s->len;

    TX_NAME(ff_tx_remap)(dst1, src, map, len);
    s->fn[0](&s->sub[0], dst2, dst1, stride);
}

static void TX_NAME(ff_tx_fft_inplace)(AVTXContext *s, void *_dst,
                                       void *_src, ptrdiff_t stride)
{
    TXComplex *src = _src;
    TXComplex *dst = _dst;
    TXComplex tmp;
    const int *map = s->sub->map;
    const int *inplace_idx = s->map;
    int src_idx, dst_idx;

    src_idx = *inplace_idx++;
    do {
        tmp = src[src_idx];
        dst_idx = map[src_idx];
        do {
            FFSWAP(TXComplex, tmp, src[dst_idx]);
            dst_idx = map[dst_idx];
        } while (dst_idx != src_idx); /* Can be > as well, but was less predictable */
        src[dst_idx] = tmp;
    } while ((src_idx = *inplace_idx++));

    s->fn[0](&s->sub[0], dst, src, stride);
}

static const FFTXCodelet TX_NAME(ff_tx_fft_def) = {
    .name       = TX_NAME_STR("fft"),
    .function   = TX_NAME(ff_tx_fft),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE,
    .factors[0] = TX_FACTOR_ANY,
    .nb_factors = 1,
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_fft_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_inplace_small_def) = {
    .name       = TX_NAME_STR("fft_inplace_small"),
    .function   = TX_NAME(ff_tx_fft),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | AV_TX_INPLACE,
    .factors[0] = TX_FACTOR_ANY,
    .nb_factors = 1,
    .min_len    = 2,
    .max_len    = 65536,
    .init       = TX_NAME(ff_tx_fft_inplace_small_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE - 256,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_inplace_def) = {
    .name       = TX_NAME_STR("fft_inplace"),
    .function   = TX_NAME(ff_tx_fft_inplace),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | AV_TX_INPLACE,
    .factors[0] = TX_FACTOR_ANY,
    .nb_factors = 1,
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_fft_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE - 512,
};

static av_cold int TX_NAME(ff_tx_fft_init_naive_small)(AVTXContext *s,
                                                       const FFTXCodelet *cd,
                                                       uint64_t flags,
                                                       FFTXCodeletOptions *opts,
                                                       int len, int inv,
                                                       const void *scale)
{
    const double phase = s->inv ? 2.0*M_PI/len : -2.0*M_PI/len;

    if (!(s->exp = av_malloc(len*len*sizeof(*s->exp))))
        return AVERROR(ENOMEM);

    for (int i = 0; i < len; i++) {
        for (int j = 0; j < len; j++) {
            const double factor = phase*i*j;
            s->exp[i*len+j] = (TXComplex){
                RESCALE(cos(factor)),
                RESCALE(sin(factor)),
            };
        }
    }

    return 0;
}

static av_always_inline void stockham0(const int n, const int z, TXComplex *y, const TXComplex *x,
                                       const TXComplex **exp)
{
    if (n <= 2) {
        const TXComplex *xa = x + 0;
        const TXComplex *xb = x + z;
        TXComplex *y0 = y + 0;
        TXComplex *yz = y + z;

        for (int q = 0; q < z; q++) {
            const TXComplex a = xa[q];
            const TXComplex b = xb[q];

            y0[q].re = a.re + b.re;
            y0[q].im = a.im + b.im;
            yz[q].re = a.re - b.re;
            yz[q].im = a.im - b.im;
        }
    } else if (n >= 4) {
        const int m = n / 2;

        for (int p = 0; p < m; p++) {
            const int s2p0 = z * (2 * p + 0);
            const int s2p1 = z * (2 * p + 1);
            const TXComplex wp = exp[0][p];
            const int sp0 = z * (p + 0);
            const int spm = z * (p + m);
            const TXComplex *xa = x + sp0;
            const TXComplex *xb = x + spm;
            TXComplex *y0 = y + s2p0;
            TXComplex *y1 = y + s2p1;

            for (int q = 0; q < z; q++) {
                const TXComplex a = xa[q];
                const TXComplex b = xb[q];
                TXComplex t;

                t.re = a.re - b.re;
                t.im = a.im - b.im;
                y0[q].re = a.re + b.re;
                y0[q].im = a.im + b.im;
                CMUL3(y1[q], t, wp);
            }
        }

        exp[0] += m;
    }
}

static av_cold int TX_NAME(ff_tx_fft_init_stockham)(AVTXContext *s,
                                                    const FFTXCodelet *cd,
                                                    uint64_t flags,
                                                    FFTXCodeletOptions *opts,
                                                    int len, int inv,
                                                    const void *scale)
{
    s->scale_d = *((SCALE_TYPE *)scale);
    s->scale_f = s->scale_d;

    if (!(s->exp = av_mallocz(len*sizeof(TXComplex))))
        return AVERROR(ENOMEM);

    {
        const double invf = s->inv ? 1.0 : -1.0;
        TXComplex *exp = s->exp;

        for (int n = len; n > 2; n /= 2) {
            const double w = 2.0 * M_PI * invf / n;

            for (int m = 0; m < n/2; m++) {
                exp[m].re = RESCALE(cos(w * m));
                exp[m].im = RESCALE(sin(w * m));
            }
            exp += n/2;
        }
    }

    if (!(s->tmp = av_mallocz(len*2*sizeof(TXComplex))))
        return AVERROR(ENOMEM);

    return 0;
}

static void TX_NAME(ff_tx_fft_stockham16)(AVTXContext *s, void *_dst, void *_src,
                                          ptrdiff_t stride)
{
    const TXComplex *exp = s->exp;
    TXComplex *tmp = s->tmp;
    TXComplex *tm0 = tmp+16;
    TXComplex *src = _src;
    TXComplex *dst = _dst;

    stride /= sizeof(*dst);

    stockham0(16, 1, tmp, src, &exp);
    stockham0(8,  2, tm0, tmp, &exp);
    stockham0(4,  4, tmp, tm0, &exp);
    stockham0(2,  8, tm0, tmp, &exp);

    for (int i = 0; i < 16; i++)
        dst[i*stride] = tm0[i];
}

static void TX_NAME(ff_tx_fft_stockham32)(AVTXContext *s, void *_dst, void *_src,
                                          ptrdiff_t stride)
{
    const TXComplex *exp = s->exp;
    TXComplex *tmp = s->tmp;
    TXComplex *tm0 = tmp+32;
    TXComplex *src = _src;
    TXComplex *dst = _dst;

    stride /= sizeof(*dst);

    stockham0(32, 1, tmp, src, &exp);
    stockham0(16, 2, tm0, tmp, &exp);
    stockham0(8,  4, tmp, tm0, &exp);
    stockham0(4,  8, tm0, tmp, &exp);
    stockham0(2, 16, tmp, tm0, &exp);

    for (int i = 0; i < 32; i++)
        dst[i*stride] = tmp[i];
}

static void TX_NAME(ff_tx_fft_stockham64)(AVTXContext *s, void *_dst, void *_src,
                                          ptrdiff_t stride)
{
    const TXComplex *exp = s->exp;
    TXComplex *tmp = s->tmp;
    TXComplex *tm0 = tmp+64;
    TXComplex *src = _src;
    TXComplex *dst = _dst;

    stride /= sizeof(*dst);

    stockham0(64, 1, tmp, src, &exp);
    stockham0(32, 2, tm0, tmp, &exp);
    stockham0(16, 4, tmp, tm0, &exp);
    stockham0(8,  8, tm0, tmp, &exp);
    stockham0(4, 16, tmp, tm0, &exp);
    stockham0(2, 32, tm0, tmp, &exp);

    for (int i = 0; i < 64; i++)
        dst[i*stride] = tm0[i];
}

static void TX_NAME(ff_tx_fft_stockham128)(AVTXContext *s, void *_dst, void *_src,
                                           ptrdiff_t stride)
{
    const TXComplex *exp = s->exp;
    TXComplex *tmp = s->tmp;
    TXComplex *tm0 = tmp+128;
    TXComplex *src = _src;
    TXComplex *dst = _dst;

    stride /= sizeof(*dst);

    stockham0(128, 1, tmp, src, &exp);
    stockham0(64,  2, tm0, tmp, &exp);
    stockham0(32,  4, tmp, tm0, &exp);
    stockham0(16,  8, tm0, tmp, &exp);
    stockham0(8,  16, tmp, tm0, &exp);
    stockham0(4,  32, tm0, tmp, &exp);
    stockham0(2,  64, tmp, tm0, &exp);

    for (int i = 0; i < 128; i++)
        dst[i*stride] = tmp[i];
}

static void TX_NAME(ff_tx_fft_stockham256)(AVTXContext *s, void *_dst, void *_src,
                                           ptrdiff_t stride)
{
    const TXComplex *exp = s->exp;
    TXComplex *tmp = s->tmp;
    TXComplex *tm0 = tmp+256;
    TXComplex *src = _src;
    TXComplex *dst = _dst;

    stride /= sizeof(*dst);

    stockham0(256, 1, tmp, src, &exp);
    stockham0(128, 2, tm0, tmp, &exp);
    stockham0(64,  4, tmp, tm0, &exp);
    stockham0(32,  8, tm0, tmp, &exp);
    stockham0(16, 16, tmp, tm0, &exp);
    stockham0(8,  32, tm0, tmp, &exp);
    stockham0(4,  64, tmp, tm0, &exp);
    stockham0(2, 128, tm0, tmp, &exp);

    for (int i = 0; i < 256; i++)
        dst[i*stride] = tm0[i];
}

static void TX_NAME(ff_tx_fft_stockham512)(AVTXContext *s, void *_dst, void *_src,
                                           ptrdiff_t stride)
{
    const TXComplex *exp = s->exp;
    TXComplex *tmp = s->tmp;
    TXComplex *tm0 = tmp+512;
    TXComplex *src = _src;
    TXComplex *dst = _dst;

    stride /= sizeof(*dst);

    stockham0(512, 1, tmp, src, &exp);
    stockham0(256, 2, tm0, tmp, &exp);
    stockham0(128, 4, tmp, tm0, &exp);
    stockham0(64,  8, tm0, tmp, &exp);
    stockham0(32, 16, tmp, tm0, &exp);
    stockham0(16, 32, tm0, tmp, &exp);
    stockham0(8,  64, tmp, tm0, &exp);
    stockham0(4, 128, tm0, tmp, &exp);
    stockham0(2, 256, tmp, tm0, &exp);

    for (int i = 0; i < 512; i++)
        dst[i*stride] = tmp[i];
}

static void TX_NAME(ff_tx_fft_stockham1024)(AVTXContext *s, void *_dst, void *_src,
                                            ptrdiff_t stride)
{
    const TXComplex *exp = s->exp;
    TXComplex *tmp = s->tmp;
    TXComplex *tm0 = tmp+1024;
    TXComplex *src = _src;
    TXComplex *dst = _dst;

    stride /= sizeof(*dst);

    stockham0(1024, 1, tmp, src, &exp);
    stockham0(512,  2, tm0, tmp, &exp);
    stockham0(256,  4, tmp, tm0, &exp);
    stockham0(128,  8, tm0, tmp, &exp);
    stockham0(64,  16, tmp, tm0, &exp);
    stockham0(32,  32, tm0, tmp, &exp);
    stockham0(16,  64, tmp, tm0, &exp);
    stockham0(8,  128, tm0, tmp, &exp);
    stockham0(4,  256, tmp, tm0, &exp);
    stockham0(2,  512, tm0, tmp, &exp);

    for (int i = 0; i < 1024; i++)
        dst[i*stride] = tm0[i];
}

static void TX_NAME(ff_tx_fft_stockham2048)(AVTXContext *s, void *_dst, void *_src,
                                            ptrdiff_t stride)
{
    const TXComplex *exp = s->exp;
    TXComplex *tmp = s->tmp;
    TXComplex *tm0 = tmp+2048;
    TXComplex *src = _src;
    TXComplex *dst = _dst;

    stride /= sizeof(*dst);

    stockham0(2048, 1, tmp, src, &exp);
    stockham0(1024, 2, tm0, tmp, &exp);
    stockham0(512,  4, tmp, tm0, &exp);
    stockham0(256,  8, tm0, tmp, &exp);
    stockham0(128, 16, tmp, tm0, &exp);
    stockham0(64,  32, tm0, tmp, &exp);
    stockham0(32,  64, tmp, tm0, &exp);
    stockham0(16, 128, tm0, tmp, &exp);
    stockham0(8,  256, tmp, tm0, &exp);
    stockham0(4,  512, tm0, tmp, &exp);
    stockham0(2, 1024, tmp, tm0, &exp);

    for (int i = 0; i < 2048; i++)
        dst[i*stride] = tmp[i];
}

static void TX_NAME(ff_tx_fft_stockham4096)(AVTXContext *s, void *_dst, void *_src,
                                            ptrdiff_t stride)
{
    const TXComplex *exp = s->exp;
    TXComplex *tmp = s->tmp;
    TXComplex *tm0 = tmp+4096;
    TXComplex *src = _src;
    TXComplex *dst = _dst;

    stride /= sizeof(*dst);

    stockham0(4096, 1, tmp, src, &exp);
    stockham0(2048, 2, tm0, tmp, &exp);
    stockham0(1024, 4, tmp, tm0, &exp);
    stockham0(512,  8, tm0, tmp, &exp);
    stockham0(256, 16, tmp, tm0, &exp);
    stockham0(128, 32, tm0, tmp, &exp);
    stockham0(64,  64, tmp, tm0, &exp);
    stockham0(32, 128, tm0, tmp, &exp);
    stockham0(16, 256, tmp, tm0, &exp);
    stockham0(8,  512, tm0, tmp, &exp);
    stockham0(4, 1024, tmp, tm0, &exp);
    stockham0(2, 2048, tm0, tmp, &exp);

    for (int i = 0; i < 4096; i++)
        dst[i*stride] = tm0[i];
}

static void TX_NAME(ff_tx_fft_stockham8192)(AVTXContext *s, void *_dst, void *_src,
                                            ptrdiff_t stride)
{
    const TXComplex *exp = s->exp;
    TXComplex *tmp = s->tmp;
    TXComplex *tm0 = tmp+8192;
    TXComplex *src = _src;
    TXComplex *dst = _dst;

    stride /= sizeof(*dst);

    stockham0(8192, 1, tmp, src, &exp);
    stockham0(4096, 2, tm0, tmp, &exp);
    stockham0(2048, 4, tmp, tm0, &exp);
    stockham0(1024, 8, tm0, tmp, &exp);
    stockham0(512, 16, tmp, tm0, &exp);
    stockham0(256, 32, tm0, tmp, &exp);
    stockham0(128, 64, tmp, tm0, &exp);
    stockham0(64, 128, tm0, tmp, &exp);
    stockham0(32, 256, tmp, tm0, &exp);
    stockham0(16, 512, tm0, tmp, &exp);
    stockham0(8, 1024, tmp, tm0, &exp);
    stockham0(4, 2048, tm0, tmp, &exp);
    stockham0(2, 4096, tmp, tm0, &exp);

    for (int i = 0; i < 8192; i++)
        dst[i*stride] = tmp[i];
}

static const FFTXCodelet TX_NAME(ff_tx_fft_stockham16_def) = {
    .name       = TX_NAME_STR("fft_stockham16"),
    .function   = TX_NAME(ff_tx_fft_stockham16),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | AV_TX_INPLACE,
    .factors[0] = 2,
    .nb_factors = 1,
    .min_len    = 16,
    .max_len    = 16,
    .init       = TX_NAME(ff_tx_fft_init_stockham),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE+128,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_stockham32_def) = {
    .name       = TX_NAME_STR("fft_stockham32"),
    .function   = TX_NAME(ff_tx_fft_stockham32),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | AV_TX_INPLACE,
    .factors[0] = 2,
    .nb_factors = 1,
    .min_len    = 32,
    .max_len    = 32,
    .init       = TX_NAME(ff_tx_fft_init_stockham),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE+128,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_stockham64_def) = {
    .name       = TX_NAME_STR("fft_stockham64"),
    .function   = TX_NAME(ff_tx_fft_stockham64),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | AV_TX_INPLACE,
    .factors[0] = 2,
    .nb_factors = 1,
    .min_len    = 64,
    .max_len    = 64,
    .init       = TX_NAME(ff_tx_fft_init_stockham),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE+128,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_stockham128_def) = {
    .name       = TX_NAME_STR("fft_stockham128"),
    .function   = TX_NAME(ff_tx_fft_stockham128),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | AV_TX_INPLACE,
    .factors[0] = 2,
    .nb_factors = 1,
    .min_len    = 128,
    .max_len    = 128,
    .init       = TX_NAME(ff_tx_fft_init_stockham),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE+128,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_stockham256_def) = {
    .name       = TX_NAME_STR("fft_stockham256"),
    .function   = TX_NAME(ff_tx_fft_stockham256),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | AV_TX_INPLACE,
    .factors[0] = 2,
    .nb_factors = 1,
    .min_len    = 256,
    .max_len    = 256,
    .init       = TX_NAME(ff_tx_fft_init_stockham),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE+128,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_stockham512_def) = {
    .name       = TX_NAME_STR("fft_stockham512"),
    .function   = TX_NAME(ff_tx_fft_stockham512),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | AV_TX_INPLACE,
    .factors[0] = 2,
    .nb_factors = 1,
    .min_len    = 512,
    .max_len    = 512,
    .init       = TX_NAME(ff_tx_fft_init_stockham),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE+128,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_stockham1024_def) = {
    .name       = TX_NAME_STR("fft_stockham1024"),
    .function   = TX_NAME(ff_tx_fft_stockham1024),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | AV_TX_INPLACE,
    .factors[0] = 2,
    .nb_factors = 1,
    .min_len    = 1024,
    .max_len    = 1024,
    .init       = TX_NAME(ff_tx_fft_init_stockham),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE+128,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_stockham2048_def) = {
    .name       = TX_NAME_STR("fft_stockham2048"),
    .function   = TX_NAME(ff_tx_fft_stockham2048),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | AV_TX_INPLACE,
    .factors[0] = 2,
    .nb_factors = 1,
    .min_len    = 2048,
    .max_len    = 2048,
    .init       = TX_NAME(ff_tx_fft_init_stockham),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE+128,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_stockham4096_def) = {
    .name       = TX_NAME_STR("fft_stockham4096"),
    .function   = TX_NAME(ff_tx_fft_stockham4096),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | AV_TX_INPLACE,
    .factors[0] = 2,
    .nb_factors = 1,
    .min_len    = 4096,
    .max_len    = 4096,
    .init       = TX_NAME(ff_tx_fft_init_stockham),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE+128,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_stockham8192_def) = {
    .name       = TX_NAME_STR("fft_stockham8192"),
    .function   = TX_NAME(ff_tx_fft_stockham8192),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | AV_TX_INPLACE,
    .factors[0] = 2,
    .nb_factors = 1,
    .min_len    = 8192,
    .max_len    = 8192,
    .init       = TX_NAME(ff_tx_fft_init_stockham),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE+128,
};

static int add_mod(int x, int y, int p)
{
    return (x >= p-y) ? (x+(y-p)) : x+y;
}

static int mulmod(int x, int y, int p)
{
    int r = 0;

    if (y > x)
        FFSWAP(int, x, y);

    while (y) {
        r = add_mod(r, x*(y&1), p);
        y >>= 1;
        x = add_mod(x, x, p);
    }

    return r;
}

static int powmod(int64_t x, unsigned int y, int p)
{
    int res = 1;

    x = x % p;

    while (y > 0) {
        if (y & 1)
            res = (res*x) % p;
        y = y >> 1;
        x = (x*x) % p;
    }

    return res;
}

static int get_prime_factors(int n, int *primef)
{
    int i, size = 0;

    primef[size++] = 2;
    do {
        n >>= 1;
    } while ((n & 1) == 0);

    if (n == 1)
        return size;

    for (i = 3; i * i <= n; i += 2) {
        if (!(n % i)) {
            primef[size++] = i;
            do {
                n /= i;
            } while (!(n % i));
        }
    }

    if (n == 1)
        return size;

    primef[size++] = n;

    return size;
}

static int generator(int p)
{
    int n = 2, i, size, primef[16], pm1 = p-1;

    if (p == 2)
        return 1;

    size = get_prime_factors(pm1, primef);
    for (i = 0; i < size; i++) {
        if (powmod(n, pm1 / primef[i], p) == 1) {
            i = -1;
            n++;
        }
    }

    return n;
}

static void radix_map(int *map, const int n, const int r)
{
    for (int i = 0; i < n; i++)
        map[i] = i;

    for (int i = 0, j = 0; i < n-1; i++) {
        int k;

        if (i < j)
            FFSWAP(int, map[i], map[j]);

        k = (r - 1) * n / r;
        for (; k <= j;) {
            j -= k;
            k /= r;
        }
        j += k / (r - 1);
    }
}

static av_cold int TX_NAME(ff_tx_fft_init_radix3)(AVTXContext *s,
                                                  const FFTXCodelet *cd,
                                                  uint64_t flags,
                                                  FFTXCodeletOptions *opts,
                                                  int len, int inv,
                                                  const void *scale)
{
    const double invf = s->inv ? 1.0 : -1.0;
    const double phase = 2.0*M_PI * invf;
    const int n = len;
    const int r = 3;
    TXComplex *exp;
    int *map;

    if (!(s->tmp = av_mallocz(2*n*sizeof(TXComplex))))
        return AVERROR(ENOMEM);

    map = (int *)(s->tmp + n);
    radix_map(map, n, r);

    if (!(s->exp = av_mallocz((1+n)*sizeof(*s->exp))))
        return AVERROR(ENOMEM);

    exp = s->exp;
    exp[0] = (TXComplex){RESCALE(-0.5), RESCALE(invf * sqrt(3.0) * 0.5)};

    for (int m = r, z = 0; m <= n; m *= r) {
        const int mr = m/r;

        for (int j = 0; j < mr; j++) {
            for (int i = 1; i < r; i++) {
                const double ww = (j*i) * phase / m;

                exp[1+z++] = (TXComplex){RESCALE(cos(ww)), RESCALE(sin(ww))};
            }
        }
    }

    return 0;
}

static void TX_NAME(ff_tx_fft_radix3)(AVTXContext *s, void *_dst, void *_src,
                                      ptrdiff_t stride)
{
    const TXComplex w31 = s->exp[0];
    const TXComplex *w = s->exp + 1;
    const int n = s->len;
    const int *map = (int *)(s->tmp + n);
    TXComplex *tmp = s->tmp;
    TXComplex *dst = _dst;
    TXComplex *src = _src;
    const int r = 3;

    stride /= sizeof(*dst);

    TX_NAME(ff_tx_remap)(tmp, src, map, n);
    src = tmp;

    for (int m = r, idx = 0; m <= n; m *= r) {
        TXComplex *srci = src;
        const int nm = n/m;
        const int mr = m/r;

        for (int i = 0; i < nm; i++) {
            TXComplex *srci0 = srci + mr*0;
            TXComplex *srci1 = srci + mr*1;
            TXComplex *srci2 = srci + mr*2;

            for (int j = 0; j < mr; j++) {
                TXComplex z1, s0, s1, s2;

                s0 = srci0[j];
                CMUL3(z1, w[idx+j*2+0], srci1[j]);
                CMUL3(s2, w[idx+j*2+1], srci2[j]);
                s1.re = z1.re - s2.re;
                s1.im = z1.im - s2.im;
                s2.re = 2*z1.re - s1.re;
                s2.im = 2*z1.im - s1.im;
                z1.re = s2.re + s0.re;
                z1.im = s2.im + s0.im;
                s2.re = s0.re + MULT(w31.re, s2.re);
                s2.im = s0.im + MULT(w31.re, s2.im);
                s0.re = s2.re + MULT(w31.im, s1.im);
                s0.im = s2.im - MULT(w31.im, s1.re);
                s1.re = 2*s2.re - s0.re;
                s1.im = 2*s2.im - s0.im;

                srci0[j] = z1;
                srci1[j] = s1;
                srci2[j] = s0;
            }

            srci += m;
        }

        idx += mr * 2;
    }

    if (stride == 1) {
        memcpy(dst, src, n * sizeof(*dst));
        return;
    }

    for (int i = 0; i < n; i++) {
        dst[0] = src[i];
        dst += stride;
    }
}

static const FFTXCodelet TX_NAME(ff_tx_fft_radix3_def) = {
    .name       = TX_NAME_STR("fft_radix3"),
    .function   = TX_NAME(ff_tx_fft_radix3),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE | FF_TX_OUT_OF_PLACE,
    .factors[0] = 3,
    .nb_factors = 1,
    .min_len    = 3,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_fft_init_radix3),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE,
};

static av_cold int TX_NAME(ff_tx_fft_init_radix5)(AVTXContext *s,
                                                  const FFTXCodelet *cd,
                                                  uint64_t flags,
                                                  FFTXCodeletOptions *opts,
                                                  int len, int inv,
                                                  const void *scale)
{
    const double invf = s->inv ? 1.0 : -1.0;
    const double phase = 2.0*M_PI * invf;
    const int n = len;
    const int r = 5;
    TXComplex *exp;
    int *map;

    if (!(s->tmp = av_mallocz(2*n*sizeof(TXComplex))))
        return AVERROR(ENOMEM);

    map = (int *)(s->tmp + n);
    radix_map(map, n, r);

    if (!(s->exp = av_mallocz((2+n)*sizeof(*s->exp))))
        return AVERROR(ENOMEM);

    exp = s->exp;
    exp[0] = (TXComplex){RESCALE(0.25), RESCALE(sqrt(5.0) * 0.25)};
    exp[1] = (TXComplex){RESCALE(sqrt((5.0-sqrt(5.0))/(5.0+sqrt(5.0)))), RESCALE(-0.5 * invf * sqrt(2.5 + sqrt(5.0)*0.5))};

    for (int m = r, z = 0; m <= n; m *= r) {
        const int mr = m/r;

        for (int j = 0; j < mr; j++) {
            for (int i = 1; i < r; i++) {
                const double ww = (j*i) * phase / m;

                exp[2+z++] = (TXComplex){RESCALE(cos(ww)), RESCALE(sin(ww))};
            }
        }
    }

    return 0;
}

static void TX_NAME(ff_tx_fft_radix5)(AVTXContext *s, void *_dst, void *_src,
                                      ptrdiff_t stride)
{
    const TXComplex w51 = s->exp[0];
    const TXComplex w52 = s->exp[1];
    const TXComplex *w = s->exp + 2;
    const int n = s->len;
    const int *map = (int *)(s->tmp + n);
    TXComplex *tmp = s->tmp;
    TXComplex *dst = _dst;
    TXComplex *src = _src;
    const int r = 5;

    stride /= sizeof(*dst);

    TX_NAME(ff_tx_remap)(tmp, src, map, n);
    src = tmp;

    for (int m = r, idx = 0; m <= n; m *= r) {
        TXComplex *srci = src;
        const int nm = n/m;
        const int mr = m/r;

        for (int i = 0; i < nm; i++) {
            TXComplex *srci0 = srci + mr*0;
            TXComplex *srci1 = srci + mr*1;
            TXComplex *srci2 = srci + mr*2;
            TXComplex *srci3 = srci + mr*3;
            TXComplex *srci4 = srci + mr*4;

            for (int j = 0; j < mr; j++) {
                TXComplex z0, z1, z2, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, t1, t2, t3, t4, t5;

                z0 = srci0[j];
                CMUL3(z1, w[idx+j*4+0], srci1[j]);
                CMUL3(z2, w[idx+j*4+1], srci2[j]);
                CMUL3(s3, w[idx+j*4+2], srci3[j]);
                CMUL3(s1, w[idx+j*4+3], srci4[j]);
                s1.re = z1.re - s1.re;
                s1.im = z1.im - s1.im;
                s2.re = 2*z1.re - s1.re;
                s2.im = 2*z1.im - s1.im;
                s3.re = z2.re - s3.re;
                s3.im = z2.im - s3.im;
                s4.re = 2*z2.re - s3.re;
                s4.im = 2*z2.im - s3.im;
                s5.re = s2.re + s4.re;
                s5.im = s2.im + s4.im;
                s6.re = s2.re - s4.re;
                s6.im = s2.im - s4.im;
                s7.re = z0.re - MULT(w51.re, s5.re);
                s7.im = z0.im - MULT(w51.re, s5.im);
                s8.re = s7.re - MULT(w51.im, s6.re);
                s8.im = s7.im - MULT(w51.im, s6.im);
                s9.re = 2*s7.re - s8.re;
                s9.im = 2*s7.im - s8.im;
                s10.re = s1.re + MULT(w52.re, s3.re);
                s10.im = s1.im + MULT(w52.re, s3.im);
                s11.re = MULT(w52.re, s1.re) - s3.re;
                s11.im = MULT(w52.re, s1.im) - s3.im;
                t1.re = s9.re + MULT(w52.im, s10.im);
                t1.im = s9.im - MULT(w52.im, s10.re);
                t2.re = s8.re + MULT(w52.im, s11.im);
                t2.im = s8.im - MULT(w52.im, s11.re);
                t3.re = 2*s8.re - t2.re;
                t3.im = 2*s8.im - t2.im;
                t4.re = 2*s9.re - t1.re;
                t4.im = 2*s9.im - t1.im;
                t5.re = z0.re + s5.re;
                t5.im = z0.im + s5.im;

                srci0[j] = t5;
                srci1[j] = t1;
                srci2[j] = t2;
                srci3[j] = t3;
                srci4[j] = t4;
            }

            srci += m;
        }

        idx += mr * 4;
    }

    if (stride == 1) {
        memcpy(dst, src, n * sizeof(*dst));
        return;
    }

    for (int i = 0; i < n; i++) {
        dst[0] = src[i];
        dst += stride;
    }
}

static const FFTXCodelet TX_NAME(ff_tx_fft_radix5_def) = {
    .name       = TX_NAME_STR("fft_radix5"),
    .function   = TX_NAME(ff_tx_fft_radix5),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE | FF_TX_OUT_OF_PLACE,
    .factors[0] = 5,
    .nb_factors = 1,
    .min_len    = 5,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_fft_init_radix5),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE,
};

static int len_is_prime(int n)
{
    if (n == 2 || n == 3)
        return 1;

    if (n <= 1 || n % 2 == 0 || n % 3 == 0)
        return 0;

    for (int i = 5; i * i <= n; i += 6) {
        if (n % i == 0 || n % (i + 2) == 0)
            return 0;
    }

    return 1;
}

static av_cold int TX_NAME(ff_tx_fft_init_rader)(AVTXContext *s,
                                                 const FFTXCodelet *cd,
                                                 uint64_t flags,
                                                 FFTXCodeletOptions *opts,
                                                 int len, int inv,
                                                 const void *scale)
{
    int gen, igen, ret;
    const int plen = FFALIGN(len, av_cpu_max_align());
    const int len2 = len-1;
    const double phase = s->inv ? 2.0*M_PI/len : -2.0*M_PI/len;
    const TXSample ifactor = RESCALE(1.0 / len2);
    TXComplex *exp;
    int *imap, *map;

    if (!len_is_prime(len) || len_is_prime(len-1))
        return AVERROR(EINVAL);

    flags &= ~FF_TX_PRESHUFFLE;
    flags &= ~AV_TX_INPLACE;

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, NULL, len2, 0, scale)))
        return ret;

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, NULL, len2, 1, scale)))
        return ret;

    if (!(s->tmp = av_mallocz(len*2*sizeof(*map))))
        return AVERROR(ENOMEM);

    if (!(s->exp = av_mallocz(plen*3*sizeof(*s->exp))))
        return AVERROR(ENOMEM);

    map = (int *)s->tmp;
    imap = map+len;
    gen = generator(len);
    igen = powmod(gen, len-2, len);
    exp = s->exp + plen;

    for (int i = 0, gp = 1, igp = 1; i < len2; i++) {
        double factor;

        map[i] = gp;
        imap[i] = igp;
        factor = phase*imap[i];
        gp = mulmod(gp, gen, len);
        igp = mulmod(igp, igen, len);

        exp[i] = (TXComplex){
            RESCALE(cos(factor)),
            RESCALE(sin(factor)),
        };

        exp[i].re = MULT(exp[i].re, ifactor);
        exp[i].im = MULT(exp[i].im, ifactor);
    }

    s->fn[0](&s->sub[0], s->exp, exp, sizeof(TXComplex));

    return 0;
}

static void get_two_factors(int x, int *M, int *N)
{
    int i = lrint(sqrt(x));

    while ((x % i) != 0)
        i--;

    *M = i;
    *N = x/i;
}

static av_cold int TX_NAME(ff_tx_fft_init_bailey)(AVTXContext *s,
                                                  const FFTXCodelet *cd,
                                                  uint64_t flags,
                                                  FFTXCodeletOptions *opts,
                                                  int len, int inv,
                                                  const void *scale)
{
    const double phase = s->inv ? 2.0*M_PI/len : -2.0*M_PI/len;
    TXComplex *exp;
    int ret, m, n;

    get_two_factors(len, &m, &n);
    if ((m * n) != len || av_popcount(m) <= 1 || av_popcount(n) <= 1)
        return AVERROR(EINVAL);

    flags &= ~FF_TX_PRESHUFFLE;
    flags &= ~AV_TX_INPLACE;

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, NULL, m, s->inv, scale)))
        return ret;

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, NULL, n, s->inv, scale)))
        return ret;

    if (!(s->exp = av_mallocz(len*3*sizeof(*s->exp))))
        return AVERROR(ENOMEM);

    exp = s->exp;
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            const double factor = phase*i*j;
            exp[j] = (TXComplex){
                RESCALE(cos(factor)),
                RESCALE(sin(factor)),
            };
        }

        exp += n;
    }

    return 0;
}

static av_cold int TX_NAME(ff_tx_fft_init_bailey_slow)(AVTXContext *s,
                                                       const FFTXCodelet *cd,
                                                       uint64_t flags,
                                                       FFTXCodeletOptions *opts,
                                                       int len, int inv,
                                                       const void *scale)
{
    const double phase = s->inv ? 2.0*M_PI/len : -2.0*M_PI/len;
    TXComplex *exp;
    int ret, m, n;

    get_two_factors(len, &m, &n);
    if ((m * n) != len || m <= 1 || n <= 1)
        return AVERROR(EINVAL);

    flags &= ~FF_TX_PRESHUFFLE;
    flags &= ~AV_TX_INPLACE;

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, NULL, m, s->inv, scale)))
        return ret;

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, NULL, n, s->inv, scale)))
        return ret;

    if (!(s->exp = av_mallocz(len*3*sizeof(*s->exp))))
        return AVERROR(ENOMEM);

    exp = s->exp;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            const double factor = phase*i*j;
            exp[j] = (TXComplex){
                RESCALE(cos(factor)),
                RESCALE(sin(factor)),
            };
        }

        exp += m;
    }

    return 0;
}

static av_cold int TX_NAME(ff_tx_fft_init_bluestein)(AVTXContext *s,
                                                     const FFTXCodelet *cd,
                                                     uint64_t flags,
                                                     FFTXCodeletOptions *opts,
                                                     int len, int inv,
                                                     const void *scale)
{
    const double phase = s->inv ? M_PI/len : -M_PI/len;
    const int len2 = 1 << av_ceil_log2(2*len-1);
    TXComplex *w;
    int ret;

    flags &= ~FF_TX_PRESHUFFLE;
    flags &= ~AV_TX_INPLACE;

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, NULL, len2, 0, scale)))
        return ret;

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, NULL, len2, 1, scale)))
        return ret;

    if (!(s->exp = av_mallocz(len2*4*sizeof(*s->exp))))
        return AVERROR(ENOMEM);

    s->exp[0] = (TXComplex){
        RESCALE(cos(0.0)),
        RESCALE(sin(0.0)),
    };
    for (int i = 1; i < len; i++) {
        const double factor = phase*i*i;
        s->exp[i] = (TXComplex){
            RESCALE(cos(factor)),
            RESCALE(sin(factor)),
        };
        s->exp[len2-i] = s->exp[i];
    }

    w = s->exp + len2 * 2;
    for (int i = 0; i < len2; i++) {
        w[i].re =  s->exp[i].re;
        w[i].im = -s->exp[i].im;
    }

    s->fn[0](&s->sub[0], s->exp + len2, w, sizeof(TXComplex));

    return 0;
}

static void TX_NAME(ff_tx_fft_naive)(AVTXContext *s, void *_dst, void *_src,
                                     ptrdiff_t stride)
{
    TXComplex *src = _src;
    TXComplex *dst = _dst;
    const int n = s->len;
    double phase = s->inv ? 2.0*M_PI/n : -2.0*M_PI/n;

    stride /= sizeof(*dst);

    for (int i = 0; i < n; i++) {
        TXComplex tmp = { 0 };
        for (int j = 0; j < n; j++) {
            const double factor = phase*i*j;
            const TXComplex mult = {
                RESCALE(cos(factor)),
                RESCALE(sin(factor)),
            };
            TXComplex res;
            CMUL3(res, src[j], mult);
            tmp.re += res.re;
            tmp.im += res.im;
        }
        dst[i*stride] = tmp;
    }
}

static void TX_NAME(ff_tx_fft_naive_small)(AVTXContext *s, void *_dst, void *_src,
                                           ptrdiff_t stride)
{
    const TXComplex *exp = s->exp;
    TXComplex *src = _src;
    TXComplex *dst = _dst;
    const int n = s->len;

    stride /= sizeof(*dst);

    for (int i = 0; i < n; i++) {
        TXComplex tmp = { 0 };
        for (int j = 0; j < n; j++) {
            TXComplex res;
            const TXComplex mult = exp[j];
            CMUL3(res, src[j], mult);
            tmp.re += res.re;
            tmp.im += res.im;
        }
        dst[i*stride] = tmp;
        exp += n;
    }
}

static void transpose_matrix_(TXComplex *out, const TXComplex *in,
                              const int row, const int col, const int n, const int m,
                              const int sn, const int sm)
{
    const int block = 64 / sizeof(*out);

    if (m > block || n > block) {
        if (n >= m) {
            const int half_n = n / 2;

            transpose_matrix_(out, in, row, col, half_n, m, sn, sm);
            transpose_matrix_(out, in, row, col+half_n, n-half_n, m, sn, sm);
        } else {
            const int half_m = m / 2;

            transpose_matrix_(out, in, row, col, n, half_m, sn, sm);
            transpose_matrix_(out, in, row+half_m, col, n, m-half_m, sn, sm);
        }
    } else {
        const TXComplex *src = in + row * sn;
        TXComplex *dst0 = out + col * sm;
        const int rlimit = row + m;
        const int climit = col + n;

        for (int i = row; i < rlimit; i++) {
            TXComplex *dst = dst0;

            for (int j = col; j < climit; j++) {
                dst[i] = src[j];
                dst += sm;
            }

            src += sn;
        }
    }
}

static void transpose_matrix(TXComplex *out, const TXComplex *in,
                             const int n, const int m)
{
    if (n == 7 && m == 7) {
        transpose_matrix_(out, in, 0, 0, 7, 7, 7, 7);
    } else if (n == 5 && m == 5) {
        transpose_matrix_(out, in, 0, 0, 5, 5, 5, 5);
    } else {
        transpose_matrix_(out, in, 0, 0, n, m, n, m);
    }
}

static void TX_NAME(ff_tx_fft_bailey)(AVTXContext *s, void *_dst, void *_src,
                                      ptrdiff_t stride)
{
    const int len = s->len;
    const int m = s->sub[0].len;
    const int n = s->sub[1].len;
    const ptrdiff_t n_stride = n * sizeof(TXComplex);
    const ptrdiff_t m_stride = m * stride;
    const TXComplex *exp = s->exp;
    AVTXContext *sub0 = &s->sub[0];
    AVTXContext *sub1 = &s->sub[1];
    const av_tx_fn fn0 = s->fn[0];
    const av_tx_fn fn1 = s->fn[1];
    TXComplex *tmp2 = ((TXComplex *)s->exp) + len;
    TXComplex *tmp = tmp2 + len;
    TXComplex *src = _src;
    TXComplex *dst = _dst;

    stride /= sizeof(*dst);

    transpose_matrix(tmp2, src, n, m);

    for (int i = 0; i < n; i++) {
        fn0(sub0, tmp, tmp2, n_stride);
        tmp2 += m;
        tmp++;
    }

    tmp2 = ((TXComplex *)s->exp) + len;
    tmp = tmp2 + len;
    for (int i = 1; i < len; i++) {
        const TXComplex x = tmp[i];

        CMUL3(tmp[i], x, exp[i]);
    }

    for (int i = 0; i < m; i++) {
        fn1(sub1, dst, tmp, m_stride);
        tmp += n;
        dst += stride;
    }
}

static void TX_NAME(ff_tx_fft_bailey_slow)(AVTXContext *s, void *_dst, void *_src,
                                           ptrdiff_t stride)
{
    const int len = s->len;
    const int m = s->sub[0].len;
    const int n = s->sub[1].len;
    const TXComplex *exp = s->exp;
    AVTXContext *sub0 = &s->sub[0];
    AVTXContext *sub1 = &s->sub[1];
    const av_tx_fn fn0 = s->fn[0];
    const av_tx_fn fn1 = s->fn[1];
    TXComplex *tmp2 = ((TXComplex *)s->exp) + len;
    TXComplex *tmp = tmp2 + len;
    TXComplex *src = _src;
    TXComplex *dst = _dst;

    stride /= sizeof(*dst);

    transpose_matrix(tmp2, src, n, m);

    for (int i = 0; i < n; i++) {
        fn0(sub0, tmp, tmp2, sizeof(TXComplex));
        tmp2 += m;
        tmp += m;
    }

    tmp2 = ((TXComplex *)s->exp) + len;
    tmp = tmp2 + len;
    for (int i = m+1; i < len; i++) {
        const TXComplex x = tmp[i];

        CMUL3(tmp[i], x, exp[i]);
    }

    transpose_matrix(tmp2, tmp, m, n);

    for (int i = 0; i < m; i++) {
        fn1(sub1, tmp, tmp2, sizeof(TXComplex));
        tmp2 += n;
        tmp += n;
    }

    tmp2 = ((TXComplex *)s->exp) + len;
    tmp = tmp2 + len;

    if (stride == 1)
        tmp2 = dst;

    transpose_matrix(tmp2, tmp, n, m);

    if (stride == 1)
        return;

    for (int i = 0; i < len; i++) {
        dst[0] = tmp2[i];
        dst += stride;
    }
}

static void TX_NAME(ff_tx_fft_rader)(AVTXContext *s, void *_dst, void *_src,
                                     ptrdiff_t stride)
{
    const int n = s->len;
    const int pn = FFALIGN(n, av_cpu_max_align());
    const int m = n-1;
    const TXComplex *y = s->exp;
    TXComplex *src = _src;
    TXComplex *dst = _dst;
    const int *map = (const int *)s->tmp;
    const int *imap = map+n;
    TXComplex *w = s->exp+pn;
    TXComplex *ww = w+pn;
    TXComplex src0 = src[0];

    stride /= sizeof(*dst);

    TX_NAME(ff_tx_remap)(ww, src, map, m);
    s->fn[0](&s->sub[0], w, ww, sizeof(TXComplex));

    dst[0].re = src0.re + w[0].re;
    dst[0].im = src0.im + w[0].im;

    for (int i = 0; i < m; i++) {
        const TXComplex x = w[i];

        CMUL3(w[i], x, y[i]);
    }

    w[0].re += src0.re;
    w[0].im += src0.im;

    s->fn[1](&s->sub[1], ww, w, sizeof(TXComplex));

    for (int i = 0; i < m; i++)
        dst[imap[i]*stride] = ww[i];
}

static void TX_NAME(ff_tx_fft_bluestein)(AVTXContext *s, void *_dst, void *_src,
                                         ptrdiff_t stride)
{
    const int n = s->len;
    TXComplex *src = _src;
    TXComplex *dst = _dst;
    const int m = s->sub[0].len;
    const TXComplex *y = s->exp + m;
    TXComplex *w = s->exp + 2*m;
    TXComplex *ww = w + m;
    const TXComplex *exp = s->exp;
    const TXSample scale = RESCALE(1.0/m);

    stride /= sizeof(*dst);

    for (int i = 0; i < n; i++)
        CMUL3(ww[i], src[i], exp[i]);
    memset(ww+n, 0, sizeof(*ww)*(m-n));

    s->fn[0](&s->sub[0], w, ww, sizeof(TXComplex));

    for (int i = 0; i < m; i++) {
        const TXComplex x = w[i];
        CMUL3(w[i], x, y[i]);
    }

    s->fn[1](&s->sub[1], ww, w, sizeof(TXComplex));

    for (int i = 0; i < n; i++) {
        TXComplex x;
        CMUL3(x, ww[i], exp[i]);
        dst[i*stride].re = MULT(x.re, scale);
        dst[i*stride].im = MULT(x.im, scale);
    }
}

static av_always_inline void cpx_add(TXComplex *out,
                                     const TXComplex *in1,
                                     const TXComplex *in2)
{
    out->re = in1->re + in2->re;
    out->im = in1->im + in2->im;
}

static av_always_inline void cpx_sub(TXComplex *out,
                                     const TXComplex *in1,
                                     const TXComplex *in2)
{
    out->re = in1->re - in2->re;
    out->im = in1->im - in2->im;
}

static av_always_inline void cpx_mul_s(TXComplex *out,
                                       const TXComplex *in,
                                       const TXSample s)
{
    out->re = MULT(in->re, s);
    out->im = MULT(in->im, s);
}

static av_always_inline void cpx_out(TXComplex *head,
                                     TXComplex *tail,
                                     const TXComplex *A,
                                     const TXComplex *B)
{
    head->re = A->re - B->im;
    head->im = A->im + B->re;
    tail->re = A->re + B->im;
    tail->im = A->im - B->re;
}

static av_always_inline void cpx_mla(TXComplex *out,
                                     const TXComplex *in1,
                                     const TXComplex *in2,
                                     const TXSample s)
{
    out->re = in1->re + MULT(in2->re, s);
    out->im = in1->im + MULT(in2->im, s);
}

static av_always_inline void cpx_neg(TXComplex *out,
                                     const TXComplex *in)
{
    out->re = -in->re;
    out->im = -in->im;
}

static av_always_inline void cpx_zero(TXComplex *out)
{
    out->re = 0;
    out->im = 0;
}

static av_always_inline void like_terms(TXComplex *add,
                                        TXComplex *sub,
                                        const TXComplex *in,
                                        const int r)
{
    const int m = (r-1)/2;

    if (r&1) {
        add[0] = in[0];
        cpx_neg(&sub[0], &in[0]);
    }

    for (int h = 1, t = r-1; h <= m; h++, t--) {
        cpx_add(&add[h], &in[h], &in[t]);
        cpx_sub(&sub[h], &in[h], &in[t]);
    }
}

static av_always_inline void out_special(TXComplex *out,
                                         const TXComplex *add,
                                         const TXComplex *sub,
                                         const int r)
{
    const int m = r/2;

    cpx_zero(&out[0]);
    cpx_zero(&out[m]);

    for (int i = 0; i <= m; i++)
        cpx_add(&out[0], &out[0], &add[i]);

    if (r&1)
        return;

    for (int i = 0; i < m; i += 2) {
        cpx_add(&out[m], &out[m], &add[i+0]);
        cpx_sub(&out[m], &out[m], &add[i+1]);
    }
}

static av_always_inline void out_pair(TXComplex *out,
                                      const TXComplex *add,
                                      const TXComplex *sub,
                                      const TXComplex *Wr,
                                      const int k,
                                      const int r)
{
    TXComplex P, Q;

    cpx_mla(&P, &add[0], &add[1], Wr[k-1].re);
    cpx_mul_s(&Q, &sub[1], Wr[k-1].im);

    Wr += (r/2) * (k-1);
    for (int i = 2; i <= r/2; i++) {
        cpx_mla(&P, &P, &add[i], Wr[i-1].re);
        cpx_mla(&Q, &Q, &sub[i], Wr[i-1].im);
    }

    cpx_out(&out[k], &out[r-k], &P, &Q);
}

static int init_twiddles(AVTXContext *s, const int len)
{
    const double phase = s->inv ? 2.0*M_PI/len : -2.0*M_PI/len;
    TXComplex *exp;

    if (!(s->exp = av_mallocz((len/2)*(len/2)*sizeof(*s->exp))))
        return AVERROR(ENOMEM);

    exp = s->exp;
    for (int i = 0; i < len/2; i++) {
        for (int j = 0; j < len/2; j++) {
            const double factor = phase*(i+1)*(j+1);
            exp[j] = (TXComplex){
                RESCALE(cos(factor)),
                RESCALE(sin(factor)),
            };
        }

        exp += len/2;
    }

    return 0;
}

static av_cold int TX_NAME(ff_tx_fft_auto_init)(AVTXContext *s,
                                                const FFTXCodelet *cd,
                                                uint64_t flags,
                                                FFTXCodeletOptions *opts,
                                                int len, int inv,
                                                const void *scale)
{
    return init_twiddles(s, len);
}

#define DECL_BUTTERFLY_CODELET(n) \
static void TX_NAME(ff_tx_fft##n##_butterfly)(AVTXContext *s,\
                                    void *_dst, void *_src, \
                                    ptrdiff_t stride)       \
{                                                           \
    TXComplex stack[n], *tmp;                               \
    TXComplex add[n], sub[n];                               \
    TXComplex *src = _src;                                  \
    TXComplex *dst = _dst;                                  \
    TXComplex *W = s->exp;                                  \
                                                            \
    stride /= sizeof(*dst);                                 \
                                                            \
    if (stride == 1)                                        \
        tmp = dst;                                          \
    else                                                    \
        tmp = stack;                                        \
                                                            \
    like_terms(add, sub, src, n);                           \
    out_special(tmp, add, sub, n);                          \
                                                            \
    for (int i = 1; i <= n/2; i++)                          \
        out_pair(tmp, add, sub, W, i, n);                   \
                                                            \
    if (stride == 1)                                        \
        return;                                             \
                                                            \
    for (int i = 0; i < n; i++) {                           \
        dst[0] = tmp[i];                                    \
        dst += stride;                                      \
    }                                                       \
}                                                           \
                                                            \
static const FFTXCodelet TX_NAME(ff_tx_fft##n##_butterfly_def) = { \
    .name       = TX_NAME_STR("fft" #n "_butterfly"),       \
    .function   = TX_NAME(ff_tx_fft##n##_butterfly),        \
    .type       = TX_TYPE(FFT),                             \
    .flags      = FF_TX_OUT_OF_PLACE | AV_TX_INPLACE |      \
                  AV_TX_UNALIGNED,                          \
    .factors[0] = n,                                        \
    .nb_factors = 1,                                        \
    .min_len    = n,                                        \
    .max_len    = n,                                        \
    .init       = TX_NAME(ff_tx_fft_auto_init),             \
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,                      \
    .prio       = FF_TX_PRIO_BASE+512,                      \
};

DECL_BUTTERFLY_CODELET(3)
DECL_BUTTERFLY_CODELET(5)
DECL_BUTTERFLY_CODELET(7)
DECL_BUTTERFLY_CODELET(11)
DECL_BUTTERFLY_CODELET(13)
DECL_BUTTERFLY_CODELET(17)
DECL_BUTTERFLY_CODELET(19)
DECL_BUTTERFLY_CODELET(23)
DECL_BUTTERFLY_CODELET(29)
DECL_BUTTERFLY_CODELET(31)

static const FFTXCodelet TX_NAME(ff_tx_fft_naive_small_def) = {
    .name       = TX_NAME_STR("fft_naive_small"),
    .function   = TX_NAME(ff_tx_fft_naive_small),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE,
    .factors[0] = TX_FACTOR_ANY,
    .nb_factors = 1,
    .min_len    = 2,
    .max_len    = 1024,
    .init       = TX_NAME(ff_tx_fft_init_naive_small),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_MIN/2,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_bluestein_def) = {
    .name       = TX_NAME_STR("fft_bluestein"),
    .function   = TX_NAME(ff_tx_fft_bluestein),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE | FF_TX_OUT_OF_PLACE,
    .factors[0] = TX_FACTOR_ANY,
    .nb_factors = 1,
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_fft_init_bluestein),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_MIN/3,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_rader_def) = {
    .name       = TX_NAME_STR("fft_rader"),
    .function   = TX_NAME(ff_tx_fft_rader),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE | FF_TX_OUT_OF_PLACE,
    .factors[0] = TX_FACTOR_ANY,
    .nb_factors = 1,
    .min_len    = 5,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_fft_init_rader),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_MIN/4,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_bailey_def) = {
    .name       = TX_NAME_STR("fft_bailey"),
    .function   = TX_NAME(ff_tx_fft_bailey),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE | FF_TX_OUT_OF_PLACE,
    .factors[0] = TX_FACTOR_ANY,
    .nb_factors = 1,
    .min_len    = 6,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_fft_init_bailey),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_MIN/5,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_bailey_slow_def) = {
    .name       = TX_NAME_STR("fft_bailey_slow"),
    .function   = TX_NAME(ff_tx_fft_bailey_slow),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE | FF_TX_OUT_OF_PLACE,
    .factors[0] = TX_FACTOR_ANY,
    .nb_factors = 1,
    .min_len    = 6,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_fft_init_bailey_slow),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_MIN/4,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_naive_def) = {
    .name       = TX_NAME_STR("fft_naive"),
    .function   = TX_NAME(ff_tx_fft_naive),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE,
    .factors[0] = TX_FACTOR_ANY,
    .nb_factors = 1,
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = NULL,
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_MIN,
};

static av_cold int TX_NAME(ff_tx_fft_pfa_init)(AVTXContext *s,
                                               const FFTXCodelet *cd,
                                               uint64_t flags,
                                               FFTXCodeletOptions *opts,
                                               int len, int inv,
                                               const void *scale)
{
    int ret, *tmp, ps = flags & FF_TX_PRESHUFFLE, nb_decomp;
    FFTXCodeletOptions sub_opts = { .map_dir = FF_TX_MAP_GATHER };
    size_t extra_tmp_len = 0;
    int len_list[TX_MAX_DECOMPOSITIONS];

    if ((ret = ff_tx_decompose_length(len_list, TX_TYPE(FFT), len, inv)) < 0)
        return ret;

    nb_decomp = ret;
    /* Two iterations to test both orderings. */
    for (int i = 0; i < nb_decomp; i++) {
        int len1 = len_list[i];
        int len2 = len / len1;

        /* Our ptwo transforms don't support striding the output. */
        if (len2 & (len2 - 1))
            FFSWAP(int, len1, len2);

        ff_tx_clear_ctx(s);

        /* First transform */
        sub_opts.map_dir = FF_TX_MAP_GATHER;
        flags &= ~AV_TX_INPLACE;
        flags |=  FF_TX_OUT_OF_PLACE;
        flags |=  FF_TX_PRESHUFFLE; /* This function handles the permute step */
        ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, &sub_opts,
                               len1, inv, scale);

        if (ret == AVERROR(ENOMEM)) {
            return ret;
        } else if (ret < 0) { /* Try again without a preshuffle flag */
            flags &= ~FF_TX_PRESHUFFLE;
            ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, &sub_opts,
                                   len1, inv, scale);
            if (ret == AVERROR(ENOMEM))
                return ret;
            else if (ret < 0)
                continue;
        }

        /* Second transform. */
        sub_opts.map_dir = FF_TX_MAP_SCATTER;
        flags |=  FF_TX_PRESHUFFLE;
retry:
        flags &= ~FF_TX_OUT_OF_PLACE;
        flags |=  AV_TX_INPLACE;
        ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, &sub_opts,
                               len2, inv, scale);

        if (ret == AVERROR(ENOMEM)) {
            return ret;
        } else if (ret < 0) { /* Try again with an out-of-place transform */
            flags |= FF_TX_OUT_OF_PLACE;
            flags &= ~AV_TX_INPLACE;
            ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, &sub_opts,
                                   len2, inv, scale);
            if (ret == AVERROR(ENOMEM)) {
                return ret;
            } else if (ret < 0) {
                if (flags & FF_TX_PRESHUFFLE) { /* Retry again without a preshuf flag */
                    flags &= ~FF_TX_PRESHUFFLE;
                    goto retry;
                } else {
                    continue;
                }
            }
        }

        /* Success */
        break;
    }

    /* If nothing was successful, error out */
    if (ret < 0)
        return ret;

    /* Generate PFA map */
    if ((ret = ff_tx_gen_compound_mapping(s, opts, 0,
                                          s->sub[0].len, s->sub[1].len)))
        return ret;

    if (!(s->tmp = av_malloc(len*sizeof(*s->tmp))))
        return AVERROR(ENOMEM);

    /* Flatten input map */
    tmp = (int *)s->tmp;
    for (int k = 0; k < len; k += s->sub[0].len) {
        memcpy(tmp, &s->map[k], s->sub[0].len*sizeof(*tmp));
        for (int i = 0; i < s->sub[0].len; i++)
            s->map[k + i] = tmp[s->sub[0].map[i]];
    }

    /* Only allocate extra temporary memory if we need it */
    if (!(s->sub[1].flags & AV_TX_INPLACE))
        extra_tmp_len = len;
    else if (!ps)
        extra_tmp_len = s->sub[0].len;

    if (extra_tmp_len && !(s->exp = av_malloc(extra_tmp_len*sizeof(*s->exp))))
        return AVERROR(ENOMEM);

    return 0;
}

static void TX_NAME(ff_tx_fft_pfa)(AVTXContext *s, void *_out,
                                   void *_in, ptrdiff_t stride)
{
    const int n = s->sub[0].len, m = s->sub[1].len, l = s->len;
    const int *in_map = s->map, *out_map = in_map + l;
    const int *sub_map = s->sub[1].map;
    const ptrdiff_t m_stride = m * sizeof(TXComplex);
    TXComplex *tmp1 = s->sub[1].flags & AV_TX_INPLACE ? s->tmp : s->exp;
    TXComplex *tmp2 = s->tmp;
    TXComplex *exp = s->exp;
    TXComplex *in = _in, *out = _out;
    AVTXContext *sub0 = &s->sub[0];
    AVTXContext *sub1 = &s->sub[1];
    const av_tx_fn fn0 = s->fn[0];
    const av_tx_fn fn1 = s->fn[1];

    stride /= sizeof(*out);

    for (int i = 0; i < m; i++) {
        TX_NAME(ff_tx_remap)(exp, in, in_map, n);
        in_map += n;
        fn0(sub0, &tmp2[sub_map[i]], exp, m_stride);
    }

    for (int i = 0; i < n; i++) {
        fn1(sub1, tmp1, tmp2, sizeof(TXComplex));
        tmp1 += m;
        tmp2 += m;
    }

    tmp1 = s->sub[1].flags & AV_TX_INPLACE ? s->tmp : s->exp;
    for (int i = 0; i < l; i++) {
        out[0] = tmp1[out_map[i]];
        out += stride;
    }
}

static void TX_NAME(ff_tx_fft_pfa_ns)(AVTXContext *s, void *_out,
                                      void *_in, ptrdiff_t stride)
{
    const int n = s->sub[0].len, m = s->sub[1].len, l = s->len;
    const int *in_map = s->map, *out_map = in_map + l;
    const int *sub_map = s->sub[1].map;
    TXComplex *tmp1 = s->sub[1].flags & AV_TX_INPLACE ? s->tmp : s->exp;
    TXComplex *tmp2 = s->tmp;
    TXComplex *in = _in, *out = _out;
    const ptrdiff_t tstride = m*sizeof(TXComplex);
    AVTXContext *sub0 = &s->sub[0];
    AVTXContext *sub1 = &s->sub[1];
    const av_tx_fn fn0 = s->fn[0];
    const av_tx_fn fn1 = s->fn[1];

    stride /= sizeof(*out);

    for (int i = 0; i < m; i++) {
        fn0(sub0, &tmp2[sub_map[i]], in, tstride);
        in += n;
    }

    for (int i = 0; i < n; i++) {
        fn1(sub1, tmp1, tmp2, sizeof(TXComplex));
        tmp1 += m;
        tmp2 += m;
    }

    tmp1 = s->sub[1].flags & AV_TX_INPLACE ? s->tmp : s->exp;
    for (int i = 0; i < l; i++) {
        out[0] = tmp1[out_map[i]];
        out += stride;
    }
}

static const FFTXCodelet TX_NAME(ff_tx_fft_pfa_def) = {
    .name       = TX_NAME_STR("fft_pfa"),
    .function   = TX_NAME(ff_tx_fft_pfa),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE | FF_TX_OUT_OF_PLACE,
    .factors    = { 7, 5, 3, 2, TX_FACTOR_ANY },
    .nb_factors = 2,
    .min_len    = 2*3,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_fft_pfa_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE,
};

static const FFTXCodelet TX_NAME(ff_tx_fft_pfa_ns_def) = {
    .name       = TX_NAME_STR("fft_pfa_ns"),
    .function   = TX_NAME(ff_tx_fft_pfa_ns),
    .type       = TX_TYPE(FFT),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE | FF_TX_OUT_OF_PLACE |
                  FF_TX_PRESHUFFLE,
    .factors    = { 7, 5, 3, 2, TX_FACTOR_ANY },
    .nb_factors = 2,
    .min_len    = 2*3,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_fft_pfa_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE,
};

static av_cold int TX_NAME(ff_tx_mdct_naive_init)(AVTXContext *s,
                                                  const FFTXCodelet *cd,
                                                  uint64_t flags,
                                                  FFTXCodeletOptions *opts,
                                                  int len, int inv,
                                                  const void *scale)
{
    s->scale_d = *((SCALE_TYPE *)scale);
    s->scale_f = s->scale_d;
    return 0;
}

static void TX_NAME(ff_tx_mdct_naive_fwd)(AVTXContext *s, void *_dst,
                                          void *_src, ptrdiff_t stride)
{
    TXSample *src = _src;
    TXSample *dst = _dst;
    double scale = s->scale_d;
    int len = s->len;
    const double phase = M_PI/(4.0*len);

    stride /= sizeof(*dst);

    for (int i = 0; i < len; i++) {
        double sum = 0.0;
        for (int j = 0; j < len*2; j++) {
            int a = (2*j + 1 + len) * (2*i + 1);
            sum += UNSCALE(src[j]) * cos(a * phase);
        }
        dst[i*stride] = RESCALE(sum*scale);
    }
}

static void TX_NAME(ff_tx_mdct_naive_inv)(AVTXContext *s, void *_dst,
                                          void *_src, ptrdiff_t stride)
{
    TXSample *src = _src;
    TXSample *dst = _dst;
    double scale = s->scale_d;
    int len = s->len >> 1;
    int len2 = len*2;
    const double phase = M_PI/(4.0*len2);

    stride /= sizeof(*src);

    for (int i = 0; i < len; i++) {
        double sum_d = 0.0;
        double sum_u = 0.0;
        double i_d = phase * (4*len  - 2*i - 1);
        double i_u = phase * (3*len2 + 2*i + 1);
        for (int j = 0; j < len2; j++) {
            double a = (2 * j + 1);
            double a_d = cos(a * i_d);
            double a_u = cos(a * i_u);
            double val = UNSCALE(src[j*stride]);
            sum_d += a_d * val;
            sum_u += a_u * val;
        }
        dst[i +   0] = RESCALE( sum_d*scale);
        dst[i + len] = RESCALE(-sum_u*scale);
    }
}

static const FFTXCodelet TX_NAME(ff_tx_mdct_naive_fwd_def) = {
    .name       = TX_NAME_STR("mdct_naive_fwd"),
    .function   = TX_NAME(ff_tx_mdct_naive_fwd),
    .type       = TX_TYPE(MDCT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | FF_TX_FORWARD_ONLY,
    .factors    = { 2, TX_FACTOR_ANY }, /* MDCTs need an even length */
    .nb_factors = 2,
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_mdct_naive_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_MIN,
};

static const FFTXCodelet TX_NAME(ff_tx_mdct_naive_inv_def) = {
    .name       = TX_NAME_STR("mdct_naive_inv"),
    .function   = TX_NAME(ff_tx_mdct_naive_inv),
    .type       = TX_TYPE(MDCT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | FF_TX_INVERSE_ONLY,
    .factors    = { 2, TX_FACTOR_ANY },
    .nb_factors = 2,
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_mdct_naive_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_MIN,
};

static av_cold int TX_NAME(ff_tx_mdct_init)(AVTXContext *s,
                                            const FFTXCodelet *cd,
                                            uint64_t flags,
                                            FFTXCodeletOptions *opts,
                                            int len, int inv,
                                            const void *scale)
{
    int ret;
    FFTXCodeletOptions sub_opts = {
        .map_dir = !inv ? FF_TX_MAP_SCATTER : FF_TX_MAP_GATHER,
    };

    s->scale_d = *((SCALE_TYPE *)scale);
    s->scale_f = s->scale_d;

    flags &= ~FF_TX_OUT_OF_PLACE; /* We want the subtransform to be */
    flags |=  AV_TX_INPLACE;      /* in-place */
    flags |=  FF_TX_PRESHUFFLE;   /* First try with an in-place transform */

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, &sub_opts, len >> 1,
                                inv, scale))) {
        flags &= ~FF_TX_PRESHUFFLE; /* Now try with a generic FFT */
        if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, &sub_opts, len >> 1,
                                    inv, scale)))
            return ret;
    }

    s->map = av_malloc((len >> 1)*sizeof(*s->map));
    if (!s->map)
        return AVERROR(ENOMEM);

    /* If we need to preshuffle copy the map from the subcontext */
    if (s->sub[0].flags & FF_TX_PRESHUFFLE) {
        memcpy(s->map, s->sub->map, (len >> 1)*sizeof(*s->map));
    } else {
        for (int i = 0; i < len >> 1; i++)
            s->map[i] = i;
    }

    if ((ret = TX_TAB(ff_tx_mdct_gen_exp)(s, inv ? s->map : NULL)))
        return ret;

    /* Saves a multiply in a hot path. */
    if (inv)
        for (int i = 0; i < (s->len >> 1); i++)
            s->map[i] <<= 1;

    return 0;
}

static void TX_NAME(ff_tx_mdct_fwd)(AVTXContext *s, void *_dst, void *_src,
                                    ptrdiff_t stride)
{
    TXSample *src = _src, *dst = _dst;
    TXComplex *exp = s->exp, tmp, *z = _dst;
    const int len2 = s->len >> 1;
    const int len4 = s->len >> 2;
    const int len3 = len2 * 3;
    const int *sub_map = s->map;

    stride /= sizeof(*dst);

    for (int i = 0; i < len2; i++) { /* Folding and pre-reindexing */
        const int k = 2*i;
        const int idx = sub_map[i];
        if (k < len2) {
            tmp.re = FOLD(-src[ len2 + k],  src[1*len2 - 1 - k]);
            tmp.im = FOLD(-src[ len3 + k], -src[1*len3 - 1 - k]);
        } else {
            tmp.re = FOLD(-src[ len2 + k], -src[5*len2 - 1 - k]);
            tmp.im = FOLD( src[-len2 + k], -src[1*len3 - 1 - k]);
        }
        CMUL(z[idx].im, z[idx].re, tmp.re, tmp.im, exp[i].re, exp[i].im);
    }

    s->fn[0](&s->sub[0], z, z, sizeof(TXComplex));

    for (int i = 0; i < len4; i++) {
        const int i0 = len4 + i, i1 = len4 - i - 1;
        TXComplex src1 = { z[i1].re, z[i1].im };
        TXComplex src0 = { z[i0].re, z[i0].im };

        CMUL(dst[2*i1*stride + stride], dst[2*i0*stride], src0.re, src0.im,
             exp[i0].im, exp[i0].re);
        CMUL(dst[2*i0*stride + stride], dst[2*i1*stride], src1.re, src1.im,
             exp[i1].im, exp[i1].re);
    }
}

static void TX_NAME(ff_tx_mdct_inv)(AVTXContext *s, void *_dst, void *_src,
                                    ptrdiff_t stride)
{
    TXComplex *z = _dst, *exp = s->exp;
    const TXSample *src = _src, *in1, *in2;
    const int len2 = s->len >> 1;
    const int len4 = s->len >> 2;
    const int *sub_map = s->map;

    stride /= sizeof(*src);
    in1 = src;
    in2 = src + ((len2*2) - 1) * stride;

    for (int i = 0; i < len2; i++) {
        int k = sub_map[i];
        TXComplex tmp = { in2[-k*stride], in1[k*stride] };
        CMUL3(z[i], tmp, exp[i]);
    }

    s->fn[0](&s->sub[0], z, z, sizeof(TXComplex));

    exp += len2;
    for (int i = 0; i < len4; i++) {
        const int i0 = len4 + i, i1 = len4 - i - 1;
        TXComplex src1 = { z[i1].im, z[i1].re };
        TXComplex src0 = { z[i0].im, z[i0].re };

        CMUL(z[i1].re, z[i0].im, src1.re, src1.im, exp[i1].im, exp[i1].re);
        CMUL(z[i0].re, z[i1].im, src0.re, src0.im, exp[i0].im, exp[i0].re);
    }
}

static const FFTXCodelet TX_NAME(ff_tx_mdct_fwd_def) = {
    .name       = TX_NAME_STR("mdct_fwd"),
    .function   = TX_NAME(ff_tx_mdct_fwd),
    .type       = TX_TYPE(MDCT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | FF_TX_FORWARD_ONLY,
    .factors    = { 2, TX_FACTOR_ANY },
    .nb_factors = 2,
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_mdct_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE,
};

static const FFTXCodelet TX_NAME(ff_tx_mdct_inv_def) = {
    .name       = TX_NAME_STR("mdct_inv"),
    .function   = TX_NAME(ff_tx_mdct_inv),
    .type       = TX_TYPE(MDCT),
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | FF_TX_INVERSE_ONLY,
    .factors    = { 2, TX_FACTOR_ANY },
    .nb_factors = 2,
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_mdct_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE,
};

static av_cold int TX_NAME(ff_tx_mdct_inv_full_init)(AVTXContext *s,
                                                     const FFTXCodelet *cd,
                                                     uint64_t flags,
                                                     FFTXCodeletOptions *opts,
                                                     int len, int inv,
                                                     const void *scale)
{
    int ret;

    s->scale_d = *((SCALE_TYPE *)scale);
    s->scale_f = s->scale_d;

    flags &= ~AV_TX_FULL_IMDCT;

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(MDCT), flags, NULL, len, 1, scale)))
        return ret;

    return 0;
}

static void TX_NAME(ff_tx_mdct_inv_full)(AVTXContext *s, void *_dst,
                                         void *_src, ptrdiff_t stride)
{
    int len  = s->len << 1;
    int len2 = len >> 1;
    int len4 = len >> 2;
    TXSample *dst = _dst;

    s->fn[0](&s->sub[0], dst + len4, _src, stride);

    stride /= sizeof(*dst);

    for (int i = 0; i < len4; i++) {
        dst[            i*stride] = -dst[(len2 - i - 1)*stride];
        dst[(len - i - 1)*stride] =  dst[(len2 + i + 0)*stride];
    }
}

static const FFTXCodelet TX_NAME(ff_tx_mdct_inv_full_def) = {
    .name       = TX_NAME_STR("mdct_inv_full"),
    .function   = TX_NAME(ff_tx_mdct_inv_full),
    .type       = TX_TYPE(MDCT),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE |
                  FF_TX_OUT_OF_PLACE | AV_TX_FULL_IMDCT,
    .factors    = { 2, TX_FACTOR_ANY },
    .nb_factors = 2,
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_mdct_inv_full_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE,
};

static av_cold int TX_NAME(ff_tx_mdct_pfa_init)(AVTXContext *s,
                                                const FFTXCodelet *cd,
                                                uint64_t flags,
                                                FFTXCodeletOptions *opts,
                                                int len, int inv,
                                                const void *scale)
{
    int ret, sub_len;
    FFTXCodeletOptions sub_opts = { .map_dir = FF_TX_MAP_SCATTER };

    len >>= 1;
    sub_len = len / cd->factors[0];

    s->scale_d = *((SCALE_TYPE *)scale);
    s->scale_f = s->scale_d;

    flags &= ~FF_TX_OUT_OF_PLACE; /* We want the subtransform to be */
    flags |=  AV_TX_INPLACE;      /* in-place */
    flags |=  FF_TX_PRESHUFFLE;   /* This function handles the permute step */

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, &sub_opts,
                                sub_len, inv, scale)))
        return ret;

    if ((ret = ff_tx_gen_compound_mapping(s, opts, s->inv, cd->factors[0], sub_len)))
        return ret;

    /* Our 15-point transform is also a compound one, so embed its input map */
    if (cd->factors[0] == 15)
        TX_EMBED_INPUT_PFA_MAP(s->map, len, 3, 5);

    if ((ret = TX_TAB(ff_tx_mdct_gen_exp)(s, inv ? s->map : NULL)))
        return ret;

    /* Saves multiplies in loops. */
    for (int i = 0; i < len; i++)
        s->map[i] <<= 1;

    if (!(s->tmp = av_malloc(len*sizeof(*s->tmp))))
        return AVERROR(ENOMEM);

    TX_TAB(ff_tx_init_tabs)(len / sub_len);

    return 0;
}

#define DECL_COMP_IMDCT(N)                                                     \
static void TX_NAME(ff_tx_mdct_pfa_##N##xM_inv)(AVTXContext *s, void *_dst,    \
                                                void *_src, ptrdiff_t stride)  \
{                                                                              \
    TXComplex fft##N##in[N];                                                   \
    TXComplex *z = _dst, *exp = s->exp;                                        \
    const TXSample *src = _src, *in1, *in2;                                    \
    const int len4 = s->len >> 2;                                              \
    const int len2 = s->len >> 1;                                              \
    const int m = s->sub->len;                                                 \
    const int *in_map = s->map, *out_map = in_map + N*m;                       \
    const int *sub_map = s->sub->map;                                          \
                                                                               \
    stride /= sizeof(*src); /* To convert it from bytes */                     \
    in1 = src;                                                                 \
    in2 = src + ((N*m*2) - 1) * stride;                                        \
                                                                               \
    for (int i = 0; i < len2; i += N) {                                        \
        for (int j = 0; j < N; j++) {                                          \
            const int k = in_map[j];                                           \
            TXComplex tmp = { in2[-k*stride], in1[k*stride] };                 \
            CMUL3(fft##N##in[j], tmp, exp[j]);                                 \
        }                                                                      \
        fft##N(s->tmp + *(sub_map++), fft##N##in, m);                          \
        exp += N;                                                              \
        in_map += N;                                                           \
    }                                                                          \
                                                                               \
    for (int i = 0; i < N; i++)                                                \
        s->fn[0](&s->sub[0], s->tmp + m*i, s->tmp + m*i, sizeof(TXComplex));   \
                                                                               \
    for (int i = 0; i < len4; i++) {                                           \
        const int i0 = len4 + i, i1 = len4 - i - 1;                            \
        const int s0 = out_map[i0], s1 = out_map[i1];                          \
        TXComplex src1 = { s->tmp[s1].im, s->tmp[s1].re };                     \
        TXComplex src0 = { s->tmp[s0].im, s->tmp[s0].re };                     \
                                                                               \
        CMUL(z[i1].re, z[i0].im, src1.re, src1.im, exp[i1].im, exp[i1].re);    \
        CMUL(z[i0].re, z[i1].im, src0.re, src0.im, exp[i0].im, exp[i0].re);    \
    }                                                                          \
}                                                                              \
                                                                               \
static const FFTXCodelet TX_NAME(ff_tx_mdct_pfa_##N##xM_inv_def) = {           \
    .name       = TX_NAME_STR("mdct_pfa_" #N "xM_inv"),                        \
    .function   = TX_NAME(ff_tx_mdct_pfa_##N##xM_inv),                         \
    .type       = TX_TYPE(MDCT),                                               \
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | FF_TX_INVERSE_ONLY,   \
    .factors    = { N, TX_FACTOR_ANY },                                        \
    .nb_factors = 2,                                                           \
    .min_len    = N*2,                                                         \
    .max_len    = TX_LEN_UNLIMITED,                                            \
    .init       = TX_NAME(ff_tx_mdct_pfa_init),                                \
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,                                         \
    .prio       = FF_TX_PRIO_BASE,                                             \
};

DECL_COMP_IMDCT(3)
DECL_COMP_IMDCT(5)
DECL_COMP_IMDCT(7)
DECL_COMP_IMDCT(9)
DECL_COMP_IMDCT(15)

#define DECL_COMP_MDCT(N)                                                      \
static void TX_NAME(ff_tx_mdct_pfa_##N##xM_fwd)(AVTXContext *s, void *_dst,    \
                                                void *_src, ptrdiff_t stride)  \
{                                                                              \
    TXComplex fft##N##in[N];                                                   \
    TXSample *src = _src, *dst = _dst;                                         \
    TXComplex *exp = s->exp, tmp;                                              \
    const int m = s->sub->len;                                                 \
    const int len4 = N*m;                                                      \
    const int len3 = len4 * 3;                                                 \
    const int len8 = s->len >> 2;                                              \
    const int *in_map = s->map, *out_map = in_map + N*m;                       \
    const int *sub_map = s->sub->map;                                          \
                                                                               \
    stride /= sizeof(*dst);                                                    \
                                                                               \
    for (int i = 0; i < m; i++) { /* Folding and pre-reindexing */             \
        for (int j = 0; j < N; j++) {                                          \
            const int k = in_map[i*N + j];                                     \
            if (k < len4) {                                                    \
                tmp.re = FOLD(-src[ len4 + k],  src[1*len4 - 1 - k]);          \
                tmp.im = FOLD(-src[ len3 + k], -src[1*len3 - 1 - k]);          \
            } else {                                                           \
                tmp.re = FOLD(-src[ len4 + k], -src[5*len4 - 1 - k]);          \
                tmp.im = FOLD( src[-len4 + k], -src[1*len3 - 1 - k]);          \
            }                                                                  \
            CMUL(fft##N##in[j].im, fft##N##in[j].re, tmp.re, tmp.im,           \
                 exp[k >> 1].re, exp[k >> 1].im);                              \
        }                                                                      \
        fft##N(s->tmp + sub_map[i], fft##N##in, m);                            \
    }                                                                          \
                                                                               \
    for (int i = 0; i < N; i++)                                                \
        s->fn[0](&s->sub[0], s->tmp + m*i, s->tmp + m*i, sizeof(TXComplex));   \
                                                                               \
    for (int i = 0; i < len8; i++) {                                           \
        const int i0 = len8 + i, i1 = len8 - i - 1;                            \
        const int s0 = out_map[i0], s1 = out_map[i1];                          \
        TXComplex src1 = { s->tmp[s1].re, s->tmp[s1].im };                     \
        TXComplex src0 = { s->tmp[s0].re, s->tmp[s0].im };                     \
                                                                               \
        CMUL(dst[2*i1*stride + stride], dst[2*i0*stride], src0.re, src0.im,    \
             exp[i0].im, exp[i0].re);                                          \
        CMUL(dst[2*i0*stride + stride], dst[2*i1*stride], src1.re, src1.im,    \
             exp[i1].im, exp[i1].re);                                          \
    }                                                                          \
}                                                                              \
                                                                               \
static const FFTXCodelet TX_NAME(ff_tx_mdct_pfa_##N##xM_fwd_def) = {           \
    .name       = TX_NAME_STR("mdct_pfa_" #N "xM_fwd"),                        \
    .function   = TX_NAME(ff_tx_mdct_pfa_##N##xM_fwd),                         \
    .type       = TX_TYPE(MDCT),                                               \
    .flags      = AV_TX_UNALIGNED | FF_TX_OUT_OF_PLACE | FF_TX_FORWARD_ONLY,   \
    .factors    = { N, TX_FACTOR_ANY },                                        \
    .nb_factors = 2,                                                           \
    .min_len    = N*2,                                                         \
    .max_len    = TX_LEN_UNLIMITED,                                            \
    .init       = TX_NAME(ff_tx_mdct_pfa_init),                                \
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,                                         \
    .prio       = FF_TX_PRIO_BASE,                                             \
};

DECL_COMP_MDCT(3)
DECL_COMP_MDCT(5)
DECL_COMP_MDCT(7)
DECL_COMP_MDCT(9)
DECL_COMP_MDCT(15)

static av_cold int TX_NAME(ff_tx_rdft_init)(AVTXContext *s,
                                            const FFTXCodelet *cd,
                                            uint64_t flags,
                                            FFTXCodeletOptions *opts,
                                            int len, int inv,
                                            const void *scale)
{
    int ret;
    double f, m;
    TXSample *tab;
    uint64_t r2r = flags & AV_TX_REAL_TO_REAL;
    int len4 = FFALIGN(len, 4) / 4;

    s->scale_d = *((SCALE_TYPE *)scale);
    s->scale_f = s->scale_d;

    flags &= ~(AV_TX_REAL_TO_REAL | AV_TX_REAL_TO_IMAGINARY);

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(FFT), flags, NULL, len >> 1, inv, scale)))
        return ret;

    if (!(s->exp = av_mallocz((8 + 2*len4)*sizeof(*s->exp))))
        return AVERROR(ENOMEM);

    tab = (TXSample *)s->exp;

    f = 2*M_PI/len;

    m = (inv ? 2*s->scale_d : s->scale_d);

    *tab++ = RESCALE((inv ? 0.5 : 1.0) * m);
    *tab++ = RESCALE(inv ? 0.5*m : 1.0*m);
    *tab++ = RESCALE( m);
    *tab++ = RESCALE(-m);

    *tab++ = RESCALE( (0.5 - 0.0) * m);
    if (r2r)
        *tab++ = 1 / s->scale_f;
    else
        *tab++ = RESCALE( (0.0 - 0.5) * m);
    *tab++ = RESCALE( (0.5 - inv) * m);
    *tab++ = RESCALE(-(0.5 - inv) * m);

    for (int i = 0; i < len4; i++) {
        *tab++ = RESCALE(cos(i*f));
        *tab++ = RESCALE(cos(((len - i*4)/4.0)*f)) * (inv ? 1 : -1);
    }

    return 0;
}

#define DECL_RDFT(n, inv)                                                      \
static void TX_NAME(ff_tx_rdft_ ##n)(AVTXContext *s, void *_dst,               \
                                     void *_src, ptrdiff_t stride)             \
{                                                                              \
    const int len2 = s->len >> 1;                                              \
    const int len4 = FFALIGN(s->len, 4) >> 2;                                  \
    const TXSample *fact = (void *)s->exp;                                     \
    const TXSample *exp = fact + 8;                                            \
    TXComplex *data = inv ? _src : _dst;                                       \
    TXComplex t[3];                                                            \
                                                                               \
    if (!inv)                                                                  \
        s->fn[0](&s->sub[0], data, _src, sizeof(TXComplex));                   \
    else                                                                       \
        data[0].im = data[len2].re;                                            \
                                                                               \
    /* The DC value's both components are real, but we need to change them     \
     * into complex values. Also, the middle of the array is special-cased.    \
     * These operations can be done before or after the loop. */               \
    t[0].re = data[0].re;                                                      \
    data[0].re = t[0].re + data[0].im;                                         \
    data[0].im = t[0].re - data[0].im;                                         \
    data[   0].re = MULT(fact[0], data[   0].re);                              \
    data[   0].im = MULT(fact[1], data[   0].im);                              \
    if (!(len2 & 1)) {                                                         \
        data[len4].re = MULT(fact[2], data[len4].re);                          \
        data[len4].im = MULT(fact[3], data[len4].im);                          \
    }                                                                          \
                                                                               \
    for (int i = 1; i < len4; i++) {                                           \
        /* Separate even and odd FFTs */                                       \
        t[0].im = MULT(fact[5], (data[i].im - data[len2 - i].im));             \
        t[0].re = MULT(fact[4], (data[i].re + data[len2 - i].re));             \
        t[1].im = MULT(fact[7], (data[i].re - data[len2 - i].re));             \
        t[1].re = MULT(fact[6], (data[i].im + data[len2 - i].im));             \
                                                                               \
        /* Apply twiddle factors to the odd FFT and add to the even FFT */     \
        CMUL(t[2].re, t[2].im, t[1].re, t[1].im, exp[i*2 + 0], exp[i*2 + 1]);  \
                                                                               \
        data[       i].re = t[2].re + t[0].re;                                 \
        data[       i].im = t[2].im - t[0].im;                                 \
        data[len2 - i].re = t[0].re - t[2].re;                                 \
        data[len2 - i].im = t[0].im + t[2].im;                                 \
    }                                                                          \
                                                                               \
    if (inv) {                                                                 \
        s->fn[0](&s->sub[0], _dst, data, sizeof(TXComplex));                   \
    } else {                                                                   \
        /* Move [0].im to the last position, as convention requires */         \
        data[len2].re = data[0].im;                                            \
        data[   0].im = data[len2].im = 0;                                     \
    }                                                                          \
}                                                                              \
                                                                               \
static const FFTXCodelet TX_NAME(ff_tx_rdft_ ##n## _def) = {                   \
    .name       = TX_NAME_STR("rdft_" #n),                                     \
    .function   = TX_NAME(ff_tx_rdft_ ##n),                                    \
    .type       = TX_TYPE(RDFT),                                               \
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE | FF_TX_OUT_OF_PLACE |       \
                  (inv ? FF_TX_INVERSE_ONLY : FF_TX_FORWARD_ONLY),             \
    .factors    = { 2, TX_FACTOR_ANY },                                        \
    .nb_factors = 2,                                                           \
    .min_len    = 4,                                                           \
    .max_len    = TX_LEN_UNLIMITED,                                            \
    .init       = TX_NAME(ff_tx_rdft_init),                                    \
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,                                         \
    .prio       = FF_TX_PRIO_BASE,                                             \
};

DECL_RDFT(r2c,  0)
DECL_RDFT(c2r,  1)

#define DECL_RDFT_HALF(n, mode, mod2)                                          \
static void TX_NAME(ff_tx_rdft_ ##n)(AVTXContext *s, void *_dst,               \
                                        void *_src, ptrdiff_t stride)          \
{                                                                              \
    const int len = s->len;                                                    \
    const int len2 = len >> 1;                                                 \
    const int len4 = len >> 2;                                                 \
    const int aligned_len4 = FFALIGN(len, 4)/4;                                \
    const TXSample *fact = (void *)s->exp;                                     \
    const TXSample *tcos = fact + 8;                                           \
    const TXSample *tsin = tcos + aligned_len4;                                \
    TXComplex *data = _dst;                                                    \
    TXSample *out = _dst; /* Half-complex is forward-only */                   \
    TXSample tmp_dc;                                                           \
    av_unused TXSample tmp_mid;                                                \
    TXSample tmp[4];                                                           \
    TXComplex sf, sl;                                                          \
                                                                               \
    s->fn[0](&s->sub[0], _dst, _src, sizeof(TXComplex));                       \
                                                                               \
    tmp_dc = data[0].re;                                                       \
    data[   0].re = tmp_dc + data[0].im;                                       \
    tmp_dc        = tmp_dc - data[0].im;                                       \
                                                                               \
    data[   0].re = MULT(fact[0], data[   0].re);                              \
    tmp_dc        = MULT(fact[1],        tmp_dc);                              \
    data[len4].re = MULT(fact[2], data[len4].re);                              \
                                                                               \
    if (!mod2) {                                                               \
        data[len4].im = MULT(fact[3], data[len4].im);                          \
    } else {                                                                   \
        sf = data[len4];                                                       \
        sl = data[len4 + 1];                                                   \
        if (mode == AV_TX_REAL_TO_REAL)                                        \
            tmp[0] = MULT(fact[4], (sf.re + sl.re));                           \
        else                                                                   \
            tmp[0] = MULT(fact[5], (sf.im - sl.im));                           \
        tmp[1] = MULT(fact[6], (sf.im + sl.im));                               \
        tmp[2] = MULT(fact[7], (sf.re - sl.re));                               \
                                                                               \
        if (mode == AV_TX_REAL_TO_REAL) {                                      \
            tmp[3]  = tmp[1]*tcos[len4] - tmp[2]*tsin[len4];                   \
            tmp_mid = (tmp[0] - tmp[3]);                                       \
        } else {                                                               \
            tmp[3]  = tmp[1]*tsin[len4] + tmp[2]*tcos[len4];                   \
            tmp_mid = (tmp[0] + tmp[3]);                                       \
        }                                                                      \
    }                                                                          \
                                                                               \
    /* NOTE: unrolling this breaks non-mod8 lengths */                         \
    for (int i = 1; i <= len4; i++) {                                          \
        TXSample tmp[4];                                                       \
        TXComplex sf = data[i];                                                \
        TXComplex sl = data[len2 - i];                                         \
                                                                               \
        if (mode == AV_TX_REAL_TO_REAL)                                        \
            tmp[0] = MULT(fact[4], (sf.re + sl.re));                           \
        else                                                                   \
            tmp[0] = MULT(fact[5], (sf.im - sl.im));                           \
                                                                               \
        tmp[1] = MULT(fact[6], (sf.im + sl.im));                               \
        tmp[2] = MULT(fact[7], (sf.re - sl.re));                               \
                                                                               \
        if (mode == AV_TX_REAL_TO_REAL) {                                      \
            tmp[3]           = tmp[1]*tcos[i] - tmp[2]*tsin[i];                \
            out[i]           = (tmp[0] + tmp[3]);                              \
            out[len - i]     = (tmp[0] - tmp[3]);                              \
        } else {                                                               \
            tmp[3]           = tmp[1]*tsin[i] + tmp[2]*tcos[i];                \
            out[i - 1]       = (tmp[3] - tmp[0]);                              \
            out[len - i - 1] = (tmp[0] + tmp[3]);                              \
        }                                                                      \
    }                                                                          \
                                                                               \
    for (int i = 1; i < (len4 + (mode == AV_TX_REAL_TO_IMAGINARY)); i++)       \
        out[len2 - i] = out[len - i];                                          \
                                                                               \
    if (mode == AV_TX_REAL_TO_REAL) {                                          \
        out[len2] = tmp_dc;                                                    \
        if (mod2)                                                              \
            out[len4 + 1] = tmp_mid * fact[5];                                 \
    } else if (mod2) {                                                         \
        out[len4] = tmp_mid;                                                   \
    }                                                                          \
}                                                                              \
                                                                               \
static const FFTXCodelet TX_NAME(ff_tx_rdft_ ##n## _def) = {                   \
    .name       = TX_NAME_STR("rdft_" #n),                                     \
    .function   = TX_NAME(ff_tx_rdft_ ##n),                                    \
    .type       = TX_TYPE(RDFT),                                               \
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE | mode |                     \
                  FF_TX_OUT_OF_PLACE | FF_TX_FORWARD_ONLY,                     \
    .factors    = { 2 + 2*(!mod2), TX_FACTOR_ANY },                            \
    .nb_factors = 2,                                                           \
    .min_len    = 2 + 2*(!mod2),                                               \
    .max_len    = TX_LEN_UNLIMITED,                                            \
    .init       = TX_NAME(ff_tx_rdft_init),                                    \
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,                                         \
    .prio       = FF_TX_PRIO_BASE,                                             \
};

DECL_RDFT_HALF(r2r,      AV_TX_REAL_TO_REAL,      0)
DECL_RDFT_HALF(r2r_mod2, AV_TX_REAL_TO_REAL,      1)
DECL_RDFT_HALF(r2i,      AV_TX_REAL_TO_IMAGINARY, 0)
DECL_RDFT_HALF(r2i_mod2, AV_TX_REAL_TO_IMAGINARY, 1)

static av_cold int TX_NAME(ff_tx_dct_init)(AVTXContext *s,
                                           const FFTXCodelet *cd,
                                           uint64_t flags,
                                           FFTXCodeletOptions *opts,
                                           int len, int inv,
                                           const void *scale)
{
    int ret;
    double freq;
    TXSample *tab;
    SCALE_TYPE rsc = *((SCALE_TYPE *)scale);

    if (inv) {
        len *= 2;
        s->len *= 2;
        rsc *= 0.5;
    }

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(RDFT), flags, NULL, len, inv, &rsc)))
        return ret;

    s->exp = av_malloc((len/2)*3*sizeof(TXSample));
    if (!s->exp)
        return AVERROR(ENOMEM);

    tab = (TXSample *)s->exp;

    freq = M_PI/(len*2);

    for (int i = 0; i < len; i++)
        tab[i] = RESCALE(cos(i*freq)*(!inv + 1));

    if (inv) {
        for (int i = 0; i < len/2; i++)
            tab[len + i] = RESCALE(0.5 / sin((2*i + 1)*freq));
    } else {
        for (int i = 0; i < len/2; i++)
            tab[len + i] = RESCALE(cos((len - 2*i - 1)*freq));
    }

    return 0;
}

static void TX_NAME(ff_tx_dctII)(AVTXContext *s, void *_dst,
                                 void *_src, ptrdiff_t stride)
{
    TXSample *dst = _dst;
    TXSample *src = _src;
    const int len = s->len;
    const int len2 = len >> 1;
    const TXSample *exp = (void *)s->exp;
    TXSample next;
#ifdef TX_INT32
    int64_t tmp1, tmp2;
#else
    TXSample tmp1, tmp2;
#endif

    for (int i = 0; i < len2; i++) {
        TXSample in1 = src[i];
        TXSample in2 = src[len - i - 1];
        TXSample s    = exp[len + i];

#ifdef TX_INT32
        tmp1 = in1 + in2;
        tmp2 = in1 - in2;

        tmp1 >>= 1;
        tmp2 *= s;

        tmp2 = (tmp2 + 0x40000000) >> 31;
#else
        tmp1 = (in1 + in2)*((TXSample)0.5);
        tmp2 = (in1 - in2)*s;
#endif

        src[i]           = tmp1 + tmp2;
        src[len - i - 1] = tmp1 - tmp2;
    }

    s->fn[0](&s->sub[0], dst, src, sizeof(TXComplex));

    next = dst[len];

    for (int i = len - 2; i > 0; i -= 2) {
        TXSample tmp;

        CMUL(tmp, dst[i], exp[len - i], exp[i], dst[i + 0], dst[i + 1]);

        dst[i + 1] = next;

        next += tmp;
    }

#ifdef TX_INT32
    tmp1 = ((int64_t)exp[0]) * ((int64_t)dst[0]);
    dst[0] = (tmp1 + 0x40000000) >> 31;
#else
    dst[0] = exp[0] * dst[0];
#endif
    dst[1] = next;
}

static void TX_NAME(ff_tx_dctIII)(AVTXContext *s, void *_dst,
                                  void *_src, ptrdiff_t stride)
{
    TXSample *dst = _dst;
    TXSample *src = _src;
    const int len = s->len;
    const int len2 = len >> 1;
    const TXSample *exp = (void *)s->exp;
#ifdef TX_INT32
    int64_t  tmp1, tmp2 = src[len - 1];
    tmp2 = (2*tmp2 + 0x40000000) >> 31;
#else
    TXSample tmp1, tmp2 = 2*src[len - 1];
#endif

    src[len] = tmp2;

    for (int i = len - 2; i >= 2; i -= 2) {
        TXSample val1 = src[i - 0];
        TXSample val2 = src[i - 1] - src[i + 1];

        CMUL(src[i + 1], src[i], exp[len - i], exp[i], val1, val2);
    }

    s->fn[0](&s->sub[0], dst, src, sizeof(float));

    for (int i = 0; i < len2; i++) {
        TXSample in1 = dst[i];
        TXSample in2 = dst[len - i - 1];
        TXSample c   = exp[len + i];

        tmp1 = in1 + in2;
        tmp2 = in1 - in2;
        tmp2 *= c;
#ifdef TX_INT32
        tmp2 = (tmp2 + 0x40000000) >> 31;
#endif

        dst[i]            = tmp1 + tmp2;
        dst[len - i - 1]  = tmp1 - tmp2;
    }
}

static const FFTXCodelet TX_NAME(ff_tx_dctII_def) = {
    .name       = TX_NAME_STR("dctII"),
    .function   = TX_NAME(ff_tx_dctII),
    .type       = TX_TYPE(DCT),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE |
                  FF_TX_OUT_OF_PLACE | FF_TX_FORWARD_ONLY,
    .factors    = { 2, TX_FACTOR_ANY },
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_dct_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE,
};

static const FFTXCodelet TX_NAME(ff_tx_dctIII_def) = {
    .name       = TX_NAME_STR("dctIII"),
    .function   = TX_NAME(ff_tx_dctIII),
    .type       = TX_TYPE(DCT),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE |
                  FF_TX_OUT_OF_PLACE | FF_TX_INVERSE_ONLY,
    .factors    = { 2, TX_FACTOR_ANY },
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_dct_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE,
};

static av_cold int TX_NAME(ff_tx_dcstI_init)(AVTXContext *s,
                                             const FFTXCodelet *cd,
                                             uint64_t flags,
                                             FFTXCodeletOptions *opts,
                                             int len, int inv,
                                             const void *scale)
{
    int ret;
    SCALE_TYPE rsc = *((SCALE_TYPE *)scale);

    if (inv) {
        len *= 2;
        s->len *= 2;
        rsc *= 0.5;
    }

    /* We want a half-complex RDFT */
    flags |= cd->type == TX_TYPE(DCT_I) ? AV_TX_REAL_TO_REAL :
                                          AV_TX_REAL_TO_IMAGINARY;

    if ((ret = ff_tx_init_subtx(s, TX_TYPE(RDFT), flags, NULL,
                                (len - 1 + 2*(cd->type == TX_TYPE(DST_I)))*2,
                                0, &rsc)))
        return ret;

    s->tmp = av_mallocz((len + 1)*2*sizeof(TXSample));
    if (!s->tmp)
        return AVERROR(ENOMEM);

    return 0;
}

static void TX_NAME(ff_tx_dctI)(AVTXContext *s, void *_dst,
                                void *_src, ptrdiff_t stride)
{
    TXSample *dst = _dst;
    TXSample *src = _src;
    const int len = s->len - 1;
    TXSample *tmp = (TXSample *)s->tmp;

    stride /= sizeof(TXSample);

    for (int i = 0; i < len; i++)
        tmp[i] = tmp[2*len - i] = src[i * stride];

    tmp[len] = src[len * stride]; /* Middle */

    s->fn[0](&s->sub[0], dst, tmp, sizeof(TXSample));
}

static void TX_NAME(ff_tx_dstI)(AVTXContext *s, void *_dst,
                                void *_src, ptrdiff_t stride)
{
    TXSample *dst = _dst;
    TXSample *src = _src;
    const int len = s->len + 1;
    TXSample *tmp = (void *)s->tmp;

    stride /= sizeof(TXSample);

    tmp[0] = 0;

    for (int i = 1; i < len; i++) {
        TXSample a = src[(i - 1) * stride];
        tmp[i] = -a;
        tmp[2*len - i] = a;
    }

    tmp[len] = 0; /* i == n, Nyquist */

    s->fn[0](&s->sub[0], dst, tmp, sizeof(float));
}

static const FFTXCodelet TX_NAME(ff_tx_dctI_def) = {
    .name       = TX_NAME_STR("dctI"),
    .function   = TX_NAME(ff_tx_dctI),
    .type       = TX_TYPE(DCT_I),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE | FF_TX_OUT_OF_PLACE,
    .factors    = { 2, TX_FACTOR_ANY },
    .nb_factors = 2,
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_dcstI_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE,
};

static const FFTXCodelet TX_NAME(ff_tx_dstI_def) = {
    .name       = TX_NAME_STR("dstI"),
    .function   = TX_NAME(ff_tx_dstI),
    .type       = TX_TYPE(DST_I),
    .flags      = AV_TX_UNALIGNED | AV_TX_INPLACE | FF_TX_OUT_OF_PLACE,
    .factors    = { 2, TX_FACTOR_ANY },
    .nb_factors = 2,
    .min_len    = 2,
    .max_len    = TX_LEN_UNLIMITED,
    .init       = TX_NAME(ff_tx_dcstI_init),
    .cpu_flags  = FF_TX_CPU_FLAGS_ALL,
    .prio       = FF_TX_PRIO_BASE,
};

int TX_TAB(ff_tx_mdct_gen_exp)(AVTXContext *s, int *pre_tab)
{
    int off = 0;
    int len4 = s->len >> 1;
    double scale = s->scale_d;
    const double theta = (scale < 0 ? len4 : 0) + 1.0/8.0;
    size_t alloc = pre_tab ? 2*len4 : len4;

    if (!(s->exp = av_malloc_array(alloc, sizeof(*s->exp))))
        return AVERROR(ENOMEM);

    scale = sqrt(fabs(scale));

    if (pre_tab)
        off = len4;

    for (int i = 0; i < len4; i++) {
        const double alpha = M_PI_2 * (i + theta) / len4;
        s->exp[off + i] = (TXComplex){ RESCALE(cos(alpha) * scale),
                                       RESCALE(sin(alpha) * scale) };
    }

    if (pre_tab)
        for (int i = 0; i < len4; i++)
            s->exp[i] = s->exp[len4 + pre_tab[i]];

    return 0;
}

const FFTXCodelet * const TX_NAME(ff_tx_codelet_list)[] = {
    /* Split-Radix codelets */
    &TX_NAME(ff_tx_fft2_ns_def),
    &TX_NAME(ff_tx_fft4_ns_def),
    &TX_NAME(ff_tx_fft8_ns_def),
    &TX_NAME(ff_tx_fft16_ns_def),
    &TX_NAME(ff_tx_fft32_ns_def),
    &TX_NAME(ff_tx_fft64_ns_def),
    &TX_NAME(ff_tx_fft128_ns_def),
    &TX_NAME(ff_tx_fft256_ns_def),
    &TX_NAME(ff_tx_fft512_ns_def),
    &TX_NAME(ff_tx_fft1024_ns_def),
    &TX_NAME(ff_tx_fft2048_ns_def),
    &TX_NAME(ff_tx_fft4096_ns_def),
    &TX_NAME(ff_tx_fft8192_ns_def),
    &TX_NAME(ff_tx_fft16384_ns_def),
    &TX_NAME(ff_tx_fft32768_ns_def),
    &TX_NAME(ff_tx_fft65536_ns_def),
    &TX_NAME(ff_tx_fft131072_ns_def),
    &TX_NAME(ff_tx_fft262144_ns_def),
    &TX_NAME(ff_tx_fft524288_ns_def),
    &TX_NAME(ff_tx_fft1048576_ns_def),
    &TX_NAME(ff_tx_fft2097152_ns_def),

    /* Stockham codelets */
    &TX_NAME(ff_tx_fft_stockham16_def),
    &TX_NAME(ff_tx_fft_stockham32_def),
    &TX_NAME(ff_tx_fft_stockham64_def),
    &TX_NAME(ff_tx_fft_stockham128_def),
    &TX_NAME(ff_tx_fft_stockham256_def),
    &TX_NAME(ff_tx_fft_stockham512_def),
    &TX_NAME(ff_tx_fft_stockham1024_def),
    &TX_NAME(ff_tx_fft_stockham2048_def),
    &TX_NAME(ff_tx_fft_stockham4096_def),
    &TX_NAME(ff_tx_fft_stockham8192_def),

    /* Prime factor codelets */
    &TX_NAME(ff_tx_fft3_ns_def),
    &TX_NAME(ff_tx_fft5_ns_def),
    &TX_NAME(ff_tx_fft7_ns_def),
    &TX_NAME(ff_tx_fft9_ns_def),
    &TX_NAME(ff_tx_fft15_ns_def),

    /* We get these for free */
    &TX_NAME(ff_tx_fft3_fwd_def),
    &TX_NAME(ff_tx_fft5_fwd_def),
    &TX_NAME(ff_tx_fft7_fwd_def),
    &TX_NAME(ff_tx_fft9_fwd_def),

    /* Butterfly prime transforms */
    &TX_NAME(ff_tx_fft3_butterfly_def),
    &TX_NAME(ff_tx_fft5_butterfly_def),
    &TX_NAME(ff_tx_fft7_butterfly_def),
    &TX_NAME(ff_tx_fft11_butterfly_def),
    &TX_NAME(ff_tx_fft13_butterfly_def),
    &TX_NAME(ff_tx_fft17_butterfly_def),
    &TX_NAME(ff_tx_fft19_butterfly_def),
    &TX_NAME(ff_tx_fft23_butterfly_def),
    &TX_NAME(ff_tx_fft29_butterfly_def),
    &TX_NAME(ff_tx_fft31_butterfly_def),

    /* Radix-X transforms */
    &TX_NAME(ff_tx_fft_radix3_def),
    &TX_NAME(ff_tx_fft_radix5_def),

    /* Standalone transforms */
    &TX_NAME(ff_tx_fft_def),
    &TX_NAME(ff_tx_fft_inplace_def),
    &TX_NAME(ff_tx_fft_inplace_small_def),
    &TX_NAME(ff_tx_fft_pfa_def),
    &TX_NAME(ff_tx_fft_pfa_ns_def),
    &TX_NAME(ff_tx_fft_naive_def),
    &TX_NAME(ff_tx_fft_naive_small_def),
    &TX_NAME(ff_tx_fft_bluestein_def),
    &TX_NAME(ff_tx_fft_bailey_def),
    &TX_NAME(ff_tx_fft_bailey_slow_def),
    &TX_NAME(ff_tx_fft_rader_def),
    &TX_NAME(ff_tx_mdct_fwd_def),
    &TX_NAME(ff_tx_mdct_inv_def),
    &TX_NAME(ff_tx_mdct_pfa_3xM_fwd_def),
    &TX_NAME(ff_tx_mdct_pfa_5xM_fwd_def),
    &TX_NAME(ff_tx_mdct_pfa_7xM_fwd_def),
    &TX_NAME(ff_tx_mdct_pfa_9xM_fwd_def),
    &TX_NAME(ff_tx_mdct_pfa_15xM_fwd_def),
    &TX_NAME(ff_tx_mdct_pfa_3xM_inv_def),
    &TX_NAME(ff_tx_mdct_pfa_5xM_inv_def),
    &TX_NAME(ff_tx_mdct_pfa_7xM_inv_def),
    &TX_NAME(ff_tx_mdct_pfa_9xM_inv_def),
    &TX_NAME(ff_tx_mdct_pfa_15xM_inv_def),
    &TX_NAME(ff_tx_mdct_naive_fwd_def),
    &TX_NAME(ff_tx_mdct_naive_inv_def),
    &TX_NAME(ff_tx_mdct_inv_full_def),
    &TX_NAME(ff_tx_rdft_r2c_def),
    &TX_NAME(ff_tx_rdft_r2r_def),
    &TX_NAME(ff_tx_rdft_r2r_mod2_def),
    &TX_NAME(ff_tx_rdft_r2i_def),
    &TX_NAME(ff_tx_rdft_r2i_mod2_def),
    &TX_NAME(ff_tx_rdft_c2r_def),
    &TX_NAME(ff_tx_dctII_def),
    &TX_NAME(ff_tx_dctIII_def),
    &TX_NAME(ff_tx_dctI_def),
    &TX_NAME(ff_tx_dstI_def),

    NULL,
};
