/*
 * Bink video 2 decoder
 * Copyright (c) 2014 Konstantin Shishkov
 * Copyright (c) 2019 Paul B Mahol
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

#ifndef AVCODEC_BINK2F_H
#define AVCODEC_BINK2F_H

#include <stdint.h>
#include "avcodec.h"
#include "get_bits.h"
#include "bink2.h"

static const uint8_t bink2f_quant_codes[16] = {
    0x01, 0x02, 0x04, 0x08, 0x10, 0x30, 0x50, 0x70,
    0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0,
};

static const uint8_t bink2f_quant_bits[16] = {
    1, 2, 3, 4, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8,
};

static const uint16_t bink2f_ac_val_codes[2][13] = {
    {
        0x04, 0x01, 0x02, 0x00, 0x08, 0x18, 0xF8, 0x178, 0x138,
        0x38, 0x1B8, 0x78, 0xB8
    },
    {
        0x0A, 0x01, 0x04, 0x08, 0x06, 0x00, 0x02, 0x1A, 0x2A,
        0x16A, 0x1EA, 0x6A, 0xEA
    },
};

static const uint8_t bink2f_ac_val_bits[2][13] = {
    { 3, 1, 2, 4, 5, 6, 8, 9, 9, 9, 9, 9, 9 },
    { 6, 1, 3, 4, 3, 4, 4, 5, 7, 9, 9, 9, 9 },
};

static const uint16_t bink2f_ac_skip_codes[2][NUM_AC_SKIPS] = {
    {
        0x00, 0x01, 0x0D, 0x15, 0x45, 0x85, 0xA5, 0x165,
        0x65, 0x1E5, 0xE5, 0x25, 0x03, 0x05
    },
    {
        0x00, 0x01, 0x03, 0x07, 0x1F, 0x1B, 0x0F, 0x2F,
        0x5B, 0xDB, 0x1DB, 0x3B, 0x05, 0x0B
    }
};

static const uint8_t bink2f_ac_skip_bits[2][NUM_AC_SKIPS] = {
    { 1, 3, 4, 5, 7, 8, 8, 9, 9, 9, 9, 8, 2, 8 },
    { 1, 3, 4, 4, 5, 7, 6, 6, 8, 9, 9, 6, 3, 5 }
};

static const uint8_t bink2f_skips[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 62, 0, 0, 0,
};

static const float bink2f_dc_quant[16] = {
    4, 4, 4, 4, 4, 6, 7, 8, 10, 12, 16, 24, 32, 48, 64, 128
};

static const float bink2f_ac_quant[16] = {
    1.0, 2.0, 2.5, 3.0, 3.5, 4.0, 6.0, 7.0, 8.0, 12.0, 16.0, 24.0, 32.0, 48.0, 64.0, 128.0
};

static const float bink2f_luma_intra_qmat[64] = {
    0.125,    0.190718, 0.16332,  0.235175, 0.3,      0.392847, 0.345013, 0.210373,
    0.208056, 0.288582, 0.317145, 0.387359, 0.450788, 0.790098, 0.562995, 0.263095,
    0.228649, 0.294491, 0.341421, 0.460907, 0.653281, 0.731424, 0.60988,  0.252336,
    0.205778, 0.346585, 0.422498, 0.501223, 0.749621, 1.004719, 0.636379, 0.251428,
    0.225,    0.381436, 0.604285, 0.823113, 0.85,     1.070509, 0.69679,  0.265553,
    0.235708, 0.476783, 0.70576,  0.739104, 0.795516, 0.802512, 0.600616, 0.249289,
    0.331483, 0.600528, 0.689429, 0.692062, 0.69679,  0.643138, 0.43934,  0.188511,
    0.248309, 0.440086, 0.42807,  0.397419, 0.386259, 0.270966, 0.192244, 0.094199,
};

static const float bink2f_luma_inter_qmat[64] = {
    0.125,    0.17338,  0.16332,  0.146984, 0.128475, 0.106393, 0.077046, 0.043109,
    0.17338,  0.240485, 0.226532, 0.203873, 0.1782,   0.147571, 0.109474, 0.062454,
    0.16332,  0.226532, 0.219321, 0.202722, 0.181465, 0.149711, 0.112943, 0.062584,
    0.146984, 0.203873, 0.202722, 0.201647, 0.183731, 0.153976, 0.11711,  0.065335,
    0.128475, 0.1782,   0.181465, 0.183731, 0.177088, 0.155499, 0.120267, 0.068016,
    0.106393, 0.147571, 0.149711, 0.153976, 0.155499, 0.145756, 0.116636, 0.068495,
    0.077046, 0.109474, 0.112943, 0.11711,  0.120267, 0.116636, 0.098646, 0.060141,
    0.043109, 0.062454, 0.062584, 0.065335, 0.068016, 0.068495, 0.060141, 0.038853,
};

static const float bink2f_chroma_qmat[64] = {
    0.125,      0.17338,    0.217761,   0.383793,   0.6875,     0.54016501, 0.37207201, 0.18968099,
    0.17338,    0.28056601, 0.32721299, 0.74753499, 0.95358998, 0.74923098, 0.51607901, 0.26309499,
    0.217761,   0.32721299, 0.66387498, 1.056244,   0.89826202, 0.70576,    0.48613599, 0.24783,
    0.383793,   0.74753499, 1.056244,   0.95059502, 0.80841398, 0.635167,   0.437511,   0.223041,
    0.6875,     0.95358998, 0.89826202, 0.80841398, 0.6875,     0.54016501, 0.37207201, 0.18968099,
    0.54016501, 0.74923098, 0.70576,    0.635167,   0.54016501, 0.42440501, 0.292335,   0.149031,
    0.37207201, 0.51607901, 0.48613599, 0.437511,   0.37207201, 0.292335,   0.201364,   0.102655,
    0.18968099, 0.26309499, 0.24783,    0.223041,   0.18968099, 0.149031,   0.102655,   0.052333001
};

static const uint8_t bink2f_luma_scan[64] = {
     0,  2,  1,  8,  9, 17, 10, 16,
    24,  3, 18, 25, 32, 11, 33, 26,
     4, 40, 19, 12, 27, 41, 34,  5,
    20, 48,  6, 28, 15, 42, 23, 35,
    21, 13, 14,  7, 31, 43, 49, 36,
    22, 56, 39, 50, 30, 44, 29, 51,
    57, 47, 58, 59, 63, 61, 55, 38,
    52, 62, 45, 37, 60, 46, 54, 53
};

static const uint8_t bink2f_chroma_scan[64] = {
     0,  1,  8,  2,  9, 16, 10, 17,
     3, 24, 11, 18, 25, 13, 14,  4,
    15,  5,  6,  7, 12, 19, 20, 21,
    22, 23, 26, 27, 28, 29, 30, 31,
    32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 43, 44, 45, 46, 47,
    48, 49, 50, 51, 52, 53, 54, 55,
    56, 57, 58, 59, 60, 61, 62, 63
};

static inline void bink2f_idct_1d(float *blk, int step)
{
    float t00 =  blk[2 * step] + blk[6 * step];
    float t01 = (blk[2 * step] - blk[6 * step]) * 1.4142135f - t00;
    float t02 =  blk[0 * step] + blk[4 * step];
    float t03 =  blk[0 * step] - blk[4 * step];
    float t04 =  blk[3 * step] + blk[5 * step];
    float t05 =  blk[3 * step] - blk[5 * step];
    float t06 =  blk[1 * step] + blk[7 * step];
    float t07 =  blk[1 * step] - blk[7 * step];
    float t08 = t02 + t00;
    float t09 = t02 - t00;
    float t10 = t03 + t01;
    float t11 = t03 - t01;
    float t12 = t06 + t04;
    float t13 = (t06 - t04) * 1.4142135f;
    float t14 = (t07 - t05) * 1.847759f;
    float t15 = t05 * 2.613126f + t14 - t12;
    float t16 = t13 - t15;
    float t17 = t07 * 1.0823922f - t14 + t16;

    blk[0*step] = t08 + t12;
    blk[1*step] = t10 + t15;
    blk[2*step] = t11 + t16;
    blk[3*step] = t09 - t17;
    blk[4*step] = t09 + t17;
    blk[5*step] = t11 - t16;
    blk[6*step] = t10 - t15;
    blk[7*step] = t08 - t12;
}

static void bink2f_idct_put(uint8_t *dst, int stride, float *block)
{
    block[0] += 512.f;

    for (int i = 0; i < 8; i++)
        bink2f_idct_1d(block + i, 8);
    for (int i = 0; i < 8; i++) {
        bink2f_idct_1d(block, 1);
        for (int j = 0; j < 8; j++)
            dst[j] = av_clip_uint8(lrintf(block[j] - 512.0f));
        block += 8;
        dst += stride;
    }
}

static void bink2f_idct_add(uint8_t *dst, int stride,
                            float *block)
{
    block[0] += 512.f;

    for (int i = 0; i < 8; i++)
        bink2f_idct_1d(block + i, 8);
    for (int i = 0; i < 8; i++) {
        bink2f_idct_1d(block, 1);
        for (int j = 0; j < 8; j++)
            dst[j] = av_clip_uint8(dst[j] + lrintf(block[j] - 512.0f));
        block += 8;
        dst += stride;
    }
}

static int bink2f_decode_delta_q(GetBitContext *gb)
{
    int dq = get_vlc2(gb, bink2f_quant_vlc.table, bink2f_quant_vlc.bits, 1);

    if (dq < 0)
        return AVERROR_INVALIDDATA;
    if (dq && get_bits1(gb))
        dq = -dq;

    return dq;
}

static unsigned bink2f_decode_cbp_luma(GetBitContext *gb, unsigned prev_cbp)
{
    unsigned cbp, cbp4, cbplo, cbphi;

    if (get_bits1(gb)) {
        if (get_bits1(gb))
            return prev_cbp;
        cbplo = prev_cbp & 0xFFFF;
    } else {
        cbplo = 0;
        cbp4 = (prev_cbp >> 4) & 0xF;
        for (int i = 0; i < 4; i++) {
            if (!get_bits1(gb))
                cbp4 = get_bits(gb, 4);
            cbplo |= cbp4 << (i * 4);
        }
    }
    cbphi = 0;
    cbp = cbplo;
    cbp4 = prev_cbp >> 20 & 0xF;
    for (int i = 0; i < 4; i++) {
        if (av_popcount(cbp & 0xF)) {
            if (av_popcount(cbp & 0xF) == 1) {
                cbp4 = 0;
                for (int j = 1; j < 16; j <<= 1) {
                    if ((j & cbp) && get_bits1(gb))
                        cbp4 |= j;
                }
            } else if (!get_bits1(gb)) {
                cbp4 = 0;
                for (int j = 1; j < 16; j <<= 1) {
                    if ((j & cbp) && get_bits1(gb))
                        cbp4 |= j;
                }
            }
        } else {
            cbp4 = 0;
        }
        cbp4 &= cbp;
        cbphi = (cbphi >> 4) | (cbp4 << 0x1c);
        cbp >>= 4;
    }
    return cbphi | cbplo;
}

static unsigned bink2f_decode_cbp_chroma(GetBitContext *gb, unsigned prev_cbp)
{
    unsigned cbplo, cbphi;

    if (get_bits1(gb)) {
        if (get_bits1(gb))
            return prev_cbp;
        cbplo = prev_cbp & 0xF;
    } else {
        cbplo = get_bits(gb, 4);
    }

    cbphi = 0;
    if (av_popcount(cbplo & 0xF)) {
        if (av_popcount(cbplo & 0xF) != 1) {
            cbphi = (prev_cbp >> 16) & cbplo;
            if (get_bits1(gb))
                return cbplo | (cbphi << 16);
        }
        cbphi = 0;
        for (int j = 1; j < 16; j <<= 1) {
            if ((j & cbplo) && get_bits1(gb))
                cbphi |= j;
        }
    }
    return cbplo | (cbphi << 16);
}

static const uint8_t q_dc_bits[16] = {
    1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 6,
};

static void bink2f_predict_dc(Bink2Context *c,
                              int is_luma, float mindc, float maxdc,
                              int flags, float tdc[16])
{
    float *LTdc = c->prev_dc[FFMAX(c->mb_pos - 1, 0)].dc[c->comp];
    float *Tdc = c->prev_dc[c->mb_pos].dc[c->comp];
    float *Ldc = c->current_dc[FFMAX(c->mb_pos - 1, 0)].dc[c->comp];
    float *dc = c->current_dc[c->mb_pos].dc[c->comp];

    if (is_luma && (flags & 0x20) && (flags & 0x80)) {
        dc[0]  = av_clipf((mindc < 0 ? 0 : 1024.f) + tdc[0], mindc, maxdc);
        dc[1]  = av_clipf(dc[0] + tdc[1], mindc, maxdc);
        dc[2]  = av_clipf(DC_MPRED2(dc[0], dc[1]) + tdc[2], mindc, maxdc);
        dc[3]  = av_clipf(DC_MPRED(dc[0], dc[2], dc[1]) + tdc[3], mindc, maxdc);
        dc[4]  = av_clipf(DC_MPRED2(dc[1], dc[3]) + tdc[4], mindc, maxdc);
        dc[5]  = av_clipf(dc[4] + tdc[5], mindc, maxdc);
        dc[6]  = av_clipf(DC_MPRED(dc[1], dc[3], dc[4]) + tdc[6], mindc, maxdc);
        dc[7]  = av_clipf(DC_MPRED(dc[4], dc[6], dc[5]) + tdc[7], mindc, maxdc);
        dc[8]  = av_clipf(DC_MPRED2(dc[2], dc[3]) + tdc[8], mindc, maxdc);
        dc[9]  = av_clipf(DC_MPRED(dc[2], dc[8], dc[3]) + tdc[9], mindc, maxdc);
        dc[10] = av_clipf(DC_MPRED2(dc[8], dc[9]) + tdc[10], mindc, maxdc);
        dc[11] = av_clipf(DC_MPRED(dc[8], dc[10], dc[9]) + tdc[11], mindc, maxdc);
        dc[12] = av_clipf(DC_MPRED(dc[3], dc[9], dc[6]) + tdc[12], mindc, maxdc);
        dc[13] = av_clipf(DC_MPRED(dc[6], dc[12], dc[7]) + tdc[13], mindc, maxdc);
        dc[14] = av_clipf(DC_MPRED(dc[9], dc[11], dc[12]) + tdc[14], mindc, maxdc);
        dc[15] = av_clipf(DC_MPRED(dc[12], dc[14], dc[13]) + tdc[15], mindc, maxdc);
    } else if (is_luma && (flags & 0x80)) {
        dc[0]  = av_clipf(DC_MPRED2(Ldc[5], Ldc[7]) + tdc[0], mindc, maxdc);
        dc[1]  = av_clipf(dc[0] + tdc[1], mindc, maxdc);
        dc[2]  = av_clipf(DC_MPRED(Ldc[5], Ldc[7], dc[0]) + tdc[2], mindc, maxdc);
        dc[3]  = av_clipf(DC_MPRED(dc[0], dc[2], dc[1]) + tdc[3], mindc, maxdc);
        dc[4]  = av_clipf(DC_MPRED2(dc[1], dc[3]) + tdc[4], mindc, maxdc);
        dc[5]  = av_clipf(dc[4] + tdc[5], mindc, maxdc);
        dc[6]  = av_clipf(DC_MPRED(dc[1], dc[3], dc[4]) + tdc[6], mindc, maxdc);
        dc[7]  = av_clipf(DC_MPRED(dc[4], dc[6], dc[5]) + tdc[7], mindc, maxdc);
        dc[8]  = av_clipf(DC_MPRED(Ldc[7], Ldc[13], dc[2]) + tdc[8], mindc, maxdc);
        dc[9]  = av_clipf(DC_MPRED(dc[2], dc[8], dc[3]) + tdc[9], mindc, maxdc);
        dc[10] = av_clipf(DC_MPRED(Ldc[13], Ldc[15], dc[8]) + tdc[10], mindc, maxdc);
        dc[11] = av_clipf(DC_MPRED(dc[8], dc[10], dc[9]) + tdc[11], mindc, maxdc);
        dc[12] = av_clipf(DC_MPRED(dc[3], dc[9], dc[6]) + tdc[12], mindc, maxdc);
        dc[13] = av_clipf(DC_MPRED(dc[6], dc[12], dc[7]) + tdc[13], mindc, maxdc);
        dc[14] = av_clipf(DC_MPRED(dc[9], dc[11], dc[12]) + tdc[14], mindc, maxdc);
        dc[15] = av_clipf(DC_MPRED(dc[12], dc[14], dc[13]) + tdc[15], mindc, maxdc);
    } else if (is_luma && (flags & 0x20)) {
        dc[0]  = av_clipf(DC_MPRED2(Tdc[10], Tdc[11]) + tdc[0], mindc, maxdc);
        dc[1]  = av_clipf(DC_MPRED(Tdc[10], dc[0], Tdc[11]) + tdc[1], mindc, maxdc);
        dc[2]  = av_clipf(DC_MPRED2(dc[0], dc[1]) + tdc[2], mindc, maxdc);
        dc[3]  = av_clipf(DC_MPRED(dc[0], dc[2], dc[1]) + tdc[3], mindc, maxdc);
        dc[4]  = av_clipf(DC_MPRED(Tdc[11], dc[1], Tdc[14]) + tdc[4], mindc, maxdc);
        dc[5]  = av_clipf(DC_MPRED(Tdc[14], dc[4], Tdc[15]) + tdc[5], mindc, maxdc);
        dc[6]  = av_clipf(DC_MPRED(dc[1], dc[3], dc[4]) + tdc[6], mindc, maxdc);
        dc[7]  = av_clipf(DC_MPRED(dc[4], dc[6], dc[5]) + tdc[7], mindc, maxdc);
        dc[8]  = av_clipf(DC_MPRED2(dc[2], dc[3]) + tdc[8], mindc, maxdc);
        dc[9]  = av_clipf(DC_MPRED(dc[2], dc[8], dc[3]) + tdc[9], mindc, maxdc);
        dc[10] = av_clipf(DC_MPRED2(dc[8], dc[9]) + tdc[10], mindc, maxdc);
        dc[11] = av_clipf(DC_MPRED(dc[8], dc[10], dc[9]) + tdc[11], mindc, maxdc);
        dc[12] = av_clipf(DC_MPRED(dc[3], dc[9], dc[6]) + tdc[12], mindc, maxdc);
        dc[13] = av_clipf(DC_MPRED(dc[6], dc[12], dc[7]) + tdc[13], mindc, maxdc);
        dc[14] = av_clipf(DC_MPRED(dc[9], dc[11], dc[12]) + tdc[14], mindc, maxdc);
        dc[15] = av_clipf(DC_MPRED(dc[12], dc[14], dc[13]) + tdc[15], mindc, maxdc);
    } else if (is_luma) {
        dc[0]  = av_clipf(DC_MPRED(LTdc[15], Ldc[5], Tdc[10]) + tdc[0], mindc, maxdc);
        dc[1]  = av_clipf(DC_MPRED(Tdc[10], dc[0], Tdc[11]) + tdc[1], mindc, maxdc);
        dc[2]  = av_clipf(DC_MPRED(Ldc[5], Ldc[7], dc[0]) + tdc[2], mindc, maxdc);
        dc[3]  = av_clipf(DC_MPRED(dc[0], dc[2], dc[1]) + tdc[3], mindc, maxdc);
        dc[4]  = av_clipf(DC_MPRED(Tdc[11], dc[1], Tdc[14]) + tdc[4], mindc, maxdc);
        dc[5]  = av_clipf(DC_MPRED(Tdc[14], dc[4], Tdc[15]) + tdc[5], mindc, maxdc);
        dc[6]  = av_clipf(DC_MPRED(dc[1], dc[3], dc[4]) + tdc[6], mindc, maxdc);
        dc[7]  = av_clipf(DC_MPRED(dc[4], dc[6], dc[5]) + tdc[7], mindc, maxdc);
        dc[8]  = av_clipf(DC_MPRED(Ldc[7], Ldc[13], dc[2]) + tdc[8], mindc, maxdc);
        dc[9]  = av_clipf(DC_MPRED(dc[2], dc[8], dc[3]) + tdc[9], mindc, maxdc);
        dc[10] = av_clipf(DC_MPRED(Ldc[13], Ldc[15], dc[8]) + tdc[10], mindc, maxdc);
        dc[11] = av_clipf(DC_MPRED(dc[8], dc[10], dc[9]) + tdc[11], mindc, maxdc);
        dc[12] = av_clipf(DC_MPRED(dc[3], dc[9], dc[6]) + tdc[12], mindc, maxdc);
        dc[13] = av_clipf(DC_MPRED(dc[6], dc[12], dc[7]) + tdc[13], mindc, maxdc);
        dc[14] = av_clipf(DC_MPRED(dc[9], dc[11], dc[12]) + tdc[14], mindc, maxdc);
        dc[15] = av_clipf(DC_MPRED(dc[12], dc[14], dc[13]) + tdc[15], mindc, maxdc);
    } else if (!is_luma && (flags & 0x20) && (flags & 0x80)) {
        dc[0] = av_clipf((mindc < 0 ? 0 : 1024.f) + tdc[0], mindc, maxdc);
        dc[1] = av_clipf(dc[0] + tdc[1], mindc, maxdc);
        dc[2] = av_clipf(DC_MPRED2(dc[0], dc[1]) + tdc[2], mindc, maxdc);
        dc[3] = av_clipf(DC_MPRED(dc[0], dc[2], dc[1]) + tdc[3], mindc, maxdc);
    } else if (!is_luma && (flags & 0x80)) {
        dc[0] = av_clipf(DC_MPRED2(Ldc[1], Ldc[3]) + tdc[0], mindc, maxdc);
        dc[1] = av_clipf(dc[0] + tdc[1], mindc, maxdc);
        dc[2] = av_clipf(DC_MPRED(Ldc[1], Ldc[3], dc[0]) + tdc[2], mindc, maxdc);
        dc[3] = av_clipf(DC_MPRED(dc[0], dc[2], dc[1]) + tdc[3], mindc, maxdc);
    } else if (!is_luma && (flags & 0x20)) {
        dc[0] = av_clipf(DC_MPRED2(Tdc[2], Tdc[3]) + tdc[0], mindc, maxdc);
        dc[1] = av_clipf(DC_MPRED(Tdc[2], dc[0], Tdc[3]) + tdc[1], mindc, maxdc);
        dc[2] = av_clipf(DC_MPRED2(dc[0], dc[1]) + tdc[2], mindc, maxdc);
        dc[3] = av_clipf(DC_MPRED(dc[0], dc[2], dc[1]) + tdc[3], mindc, maxdc);
    } else if (!is_luma) {
        dc[0] = av_clipf(DC_MPRED(LTdc[3], Ldc[1], Tdc[2]) + tdc[0], mindc, maxdc);
        dc[1] = av_clipf(DC_MPRED(Tdc[2], dc[0], Tdc[3]) + tdc[1], mindc, maxdc);
        dc[2] = av_clipf(DC_MPRED(Ldc[1], Ldc[3], dc[0]) + tdc[2], mindc, maxdc);
        dc[3] = av_clipf(DC_MPRED(dc[0], dc[2], dc[1]) + tdc[3], mindc, maxdc);
    }
}

static void bink2f_decode_dc(Bink2Context *c, GetBitContext *gb, float *dc,
                             int is_luma, int q, int mindc, int maxdc,
                             int flags)
{
    const int num_dc = is_luma ? 16 : 4;
    float tdc[16] = { 0 };
    int dc_bits;

    dc_bits = get_bits(gb, 3);
    if (dc_bits == 7)
        dc_bits += get_bits(gb, 2);
    if (!dc_bits) {
        memset(dc, 0, sizeof(*dc) * num_dc);
    } else {
        for (int j = 0; j < num_dc; j += 4) {
            for (int i = 0; i < 4; i++)
                tdc[i + j] = get_bits(gb, dc_bits);

            for (int i = 0; i < 4; i++)
                if (tdc[i + j] && get_bits1(gb))
                    tdc[i + j] = -tdc[i + j];
        }
    }

    if ((flags & 0x20) && (flags & 0x80) && mindc >= 0) {
        int bits = q_dc_bits[q] + dc_bits;

        if (bits < 10) {
            int dc_val = get_bits(gb, 10 - bits);

            if (dc_val) {
               dc_val <<= dc_bits;
               if (get_bits1(gb))
                   dc_val = -dc_val;
            }
            tdc[0] += dc_val;
        }
    }

    for (int i = 0; i < num_dc; i++)
        tdc[i] *= bink2f_dc_quant[q];

    bink2f_predict_dc(c, is_luma, mindc, maxdc, flags, tdc);
}

static int bink2f_decode_ac(GetBitContext *gb, const uint8_t *scan,
                            float block[4][64], unsigned cbp,
                            float q, const float qmat[64])
{
    int idx, next, val, skip;
    VLC *val_vlc, *skip_vlc;

    for (int i = 0; i < 4; i++, cbp >>= 1) {
        memset(block[i], 0, sizeof(**block) * 64);

        if (!(cbp & 1))
            continue;

        if (cbp & 0x10000) {
            val_vlc = &bink2f_ac_val1_vlc;
            skip_vlc = &bink2f_ac_skip1_vlc;
        } else {
            val_vlc = &bink2f_ac_val0_vlc;
            skip_vlc = &bink2f_ac_skip0_vlc;
        }

        next = 0;
        idx  = 1;
        while (idx < 64) {
            val = get_vlc2(gb, val_vlc->table, val_vlc->bits, 1);
            if (val < 0)
                return AVERROR_INVALIDDATA;
            if (val) {
                if (val >= 4) {
                    val -= 3;
                    val = get_bits(gb, val) + (1 << val) + 2;
                }
                if (get_bits1(gb))
                    val = -val;
            }

            block[i][scan[idx]] = val * q * qmat[scan[idx]];
            if (idx > 62)
                break;
            idx++;
            next--;
            if (next < 1) {
                skip = get_vlc2(gb, skip_vlc->table, skip_vlc->bits, 1);
                if (skip < 0)
                    return AVERROR_INVALIDDATA;
                next = bink2_next_skips[skip];
                skip = bink2f_skips[skip];
                if (skip == 11)
                    skip = get_bits(gb, 6);
                idx += skip;
            }
        }
    }

    return 0;
}

static int bink2f_decode_intra_luma(Bink2Context *c,
                                    float block[4][64],
                                    unsigned *prev_cbp, int *prev_q,
                                    uint8_t *dst, int stride,
                                    int flags)
{
    GetBitContext *gb = &c->gb;
    float *dc = c->current_dc[c->mb_pos].dc[c->comp];
    int q, dq, ret;
    unsigned cbp;

    *prev_cbp = cbp = bink2f_decode_cbp_luma(gb, *prev_cbp);
    dq = bink2f_decode_delta_q(gb);
    q = *prev_q + dq;
    if (q < 0 || q >= 16)
        return AVERROR_INVALIDDATA;
    *prev_q = q;

    bink2f_decode_dc(c, gb, dc, 1, q, 0, 2047, flags);

    for (int i = 0; i < 4; i++) {
        ret = bink2f_decode_ac(gb, bink2f_luma_scan, block, cbp >> (4 * i),
                               bink2f_ac_quant[q], bink2f_luma_intra_qmat);
        if (ret < 0)
            return ret;

        for (int j = 0; j < 4; j++) {
            block[j][0] = dc[i * 4 + j] * 0.125f;
            bink2f_idct_put(dst + (luma_repos[i*4+j]&3) * 8 +
                            (luma_repos[i*4+j]>>2) * 8 * stride, stride, block[j]);
        }
    }

    return 0;
}

static int bink2f_decode_intra_chroma(Bink2Context *c,
                                      float block[4][64],
                                      unsigned *prev_cbp, int *prev_q,
                                      uint8_t *dst, int stride,
                                      int flags)
{
    GetBitContext *gb = &c->gb;
    float *dc = c->current_dc[c->mb_pos].dc[c->comp];
    int q, dq, ret;
    unsigned cbp;

    *prev_cbp = cbp = bink2f_decode_cbp_chroma(gb, *prev_cbp);
    dq = bink2f_decode_delta_q(gb);
    q = *prev_q + dq;
    if (q < 0 || q >= 16)
        return AVERROR_INVALIDDATA;
    *prev_q = q;

    bink2f_decode_dc(c, gb, dc, 0, q, 0, 2047, flags);

    ret = bink2f_decode_ac(gb, bink2f_chroma_scan, block, cbp,
                           bink2f_ac_quant[q], bink2f_chroma_qmat);
    if (ret < 0)
        return ret;

    for (int j = 0; j < 4; j++) {
        block[j][0] = dc[j] * 0.125f;
        bink2f_idct_put(dst + (j & 1) * 8 + (j >> 1) * 8 * stride, stride, block[j]);
    }

    return 0;
}

static int bink2f_decode_inter_luma(Bink2Context *c,
                                    float block[4][64],
                                    unsigned *prev_cbp, int *prev_q,
                                    uint8_t *dst, int stride,
                                    int flags)
{
    GetBitContext *gb = &c->gb;
    float *dc = c->current_dc[c->mb_pos].dc[c->comp];
    unsigned cbp;
    int q, dq;

    *prev_cbp = cbp = bink2f_decode_cbp_luma(gb, *prev_cbp);
    dq = bink2f_decode_delta_q(gb);
    q = *prev_q + dq;
    if (q < 0 || q >= 16)
        return AVERROR_INVALIDDATA;
    *prev_q = q;

    bink2f_decode_dc(c, gb, dc, 1, q, -1023, 1023, 0xA8);

    for (int i = 0; i < 4; i++) {
        bink2f_decode_ac(gb, bink2f_luma_scan, block, cbp >> (i * 4),
                         bink2f_ac_quant[q], bink2f_luma_inter_qmat);
        for (int j = 0; j < 4; j++) {
            block[j][0] = dc[i * 4 + j] * 0.125f;
            bink2f_idct_add(dst + (luma_repos[i*4+j]&3) * 8 +
                            (luma_repos[i*4+j]>>2) * 8 * stride, stride,
                            block[j]);
        }
    }

    return 0;
}

static int bink2f_decode_inter_chroma(Bink2Context *c,
                                      float block[4][64],
                                      unsigned *prev_cbp, int *prev_q,
                                      uint8_t *dst, int stride,
                                      int flags)
{
    GetBitContext *gb = &c->gb;
    float *dc = c->current_dc[c->mb_pos].dc[c->comp];
    unsigned cbp;
    int q, dq;

    *prev_cbp = cbp = bink2f_decode_cbp_chroma(gb, *prev_cbp);
    dq = bink2f_decode_delta_q(gb);
    q = *prev_q + dq;
    if (q < 0 || q >= 16)
        return AVERROR_INVALIDDATA;
    *prev_q = q;

    bink2f_decode_dc(c, gb, dc, 0, q, -1023, 1023, 0xA8);

    bink2f_decode_ac(gb, bink2f_chroma_scan, block, cbp,
                     bink2f_ac_quant[q], bink2f_chroma_qmat);

    for (int i = 0; i < 4; i++) {
        block[i][0] = dc[i] * 0.125f;
        bink2f_idct_add(dst + (i & 1) * 8 + (i >> 1) * 8 * stride, stride,
                        block[i]);
    }

    return 0;
}

static void bink2f_predict_mv(Bink2Context *c, int x, int y, int flags, MVectors mv)
{
    MVectors *c_mv = &c->current_mv[c->mb_pos].mv;
    MVectors *l_mv = &c->current_mv[FFMAX(c->mb_pos - 1, 0)].mv;
    MVectors *lt_mv = &c->prev_mv[FFMAX(c->mb_pos - 1, 0)].mv;
    MVectors *t_mv = &c->prev_mv[c->mb_pos].mv;

    if (!(flags & 0x80)) {
        if (flags & 0x20) {
            c_mv->v[0][0] = mv.v[0][0] + mid_pred(t_mv->v[0][0], t_mv->v[2][0], t_mv->v[3][0]);
            c_mv->v[0][1] = mv.v[0][1] + mid_pred(t_mv->v[0][1], t_mv->v[2][1], t_mv->v[3][1]);
            c_mv->v[1][0] = mv.v[1][0] + mid_pred(t_mv->v[2][0], t_mv->v[3][0], c_mv->v[0][0]);
            c_mv->v[1][1] = mv.v[1][1] + mid_pred(t_mv->v[2][1], t_mv->v[3][1], c_mv->v[0][1]);
            c_mv->v[2][0] = mv.v[2][0] + mid_pred(t_mv->v[2][0], c_mv->v[0][0], c_mv->v[1][0]);
            c_mv->v[2][1] = mv.v[2][1] + mid_pred(t_mv->v[2][1], c_mv->v[0][1], c_mv->v[1][1]);
            c_mv->v[3][0] = mv.v[3][0] + mid_pred(c_mv->v[0][0], c_mv->v[1][0], c_mv->v[2][0]);
            c_mv->v[3][1] = mv.v[3][1] + mid_pred(c_mv->v[0][1], c_mv->v[1][1], c_mv->v[2][1]);
        } else {
            c_mv->v[0][0] = mv.v[0][0] + mid_pred(lt_mv->v[3][0], t_mv->v[2][0], l_mv->v[1][0]);
            c_mv->v[0][1] = mv.v[0][1] + mid_pred(lt_mv->v[3][1], t_mv->v[2][1], l_mv->v[1][1]);
            c_mv->v[1][0] = mv.v[1][0] + mid_pred( t_mv->v[2][0], t_mv->v[3][0], c_mv->v[0][0]);
            c_mv->v[1][1] = mv.v[1][1] + mid_pred( t_mv->v[2][1], t_mv->v[3][1], c_mv->v[0][1]);
            c_mv->v[2][0] = mv.v[2][0] + mid_pred( t_mv->v[2][0], c_mv->v[0][0], c_mv->v[1][0]);
            c_mv->v[2][1] = mv.v[2][1] + mid_pred( t_mv->v[2][1], c_mv->v[0][1], c_mv->v[1][1]);
            c_mv->v[3][0] = mv.v[3][0] + mid_pred( c_mv->v[0][0], c_mv->v[1][0], c_mv->v[2][0]);
            c_mv->v[3][1] = mv.v[3][1] + mid_pred( c_mv->v[0][1], c_mv->v[1][1], c_mv->v[2][1]);
        }
    } else {
        if (flags & 0x20) {
            c_mv->v[0][0] = mv.v[0][0];
            c_mv->v[0][1] = mv.v[0][1];
            c_mv->v[1][0] = mv.v[1][0];
            c_mv->v[1][1] = mv.v[1][1];
            c_mv->v[2][0] = mv.v[2][0];
            c_mv->v[2][1] = mv.v[2][1];
            c_mv->v[3][0] = mv.v[3][0];
            c_mv->v[3][1] = mv.v[3][1];
        } else {
            c_mv->v[0][0] = mv.v[0][0] + mid_pred(l_mv->v[0][0], l_mv->v[1][0], l_mv->v[3][0]);
            c_mv->v[0][1] = mv.v[0][1] + mid_pred(l_mv->v[0][1], l_mv->v[1][1], l_mv->v[3][1]);
            c_mv->v[2][0] = mv.v[2][0] + mid_pred(l_mv->v[1][0], l_mv->v[3][0], c_mv->v[0][0]);
            c_mv->v[2][1] = mv.v[2][1] + mid_pred(l_mv->v[1][1], l_mv->v[3][1], c_mv->v[0][1]);
            c_mv->v[1][0] = mv.v[1][0] + mid_pred(l_mv->v[1][0], c_mv->v[0][0], c_mv->v[2][0]);
            c_mv->v[1][1] = mv.v[1][1] + mid_pred(l_mv->v[1][1], c_mv->v[0][1], c_mv->v[2][1]);
            c_mv->v[3][0] = mv.v[3][0] + mid_pred(c_mv->v[0][0], c_mv->v[1][0], c_mv->v[2][0]);
            c_mv->v[3][1] = mv.v[3][1] + mid_pred(c_mv->v[0][1], c_mv->v[1][1], c_mv->v[2][1]);
        }
    }
}

#define CH1FILTER(src)    ((6*(src)[0] + 2*(src)[1] + 4) >> 3)
#define CH2FILTER(src)    ((  (src)[0] +   (src)[1] + 1) >> 1)
#define CH3FILTER(src)    ((2*(src)[0] + 6*(src)[1] + 4) >> 3)

#define CV1FILTER(src, i)    ((6*(src)[0] + 2*(src)[i] + 4) >> 3)
#define CV2FILTER(src, i)    ((  (src)[0] +   (src)[i] + 1) >> 1)
#define CV3FILTER(src, i)    ((2*(src)[0] + 6*(src)[i] + 4) >> 3)

static void bink2f_c_mc(Bink2Context *c, int x, int y,
                        uint8_t *dst, int stride,
                        uint8_t *src, int sstride,
                        int width, int height,
                        int mv_x, int mv_y,
                        int mode)
{
    uint8_t *msrc;
    uint8_t temp[8*9];

    if (mv_x < 0 || mv_x >= width ||
        mv_y < 0 || mv_y >= height)
        return;

    msrc = src + mv_x + mv_y * sstride;

    switch (mode) {
    case 0:
        copy_block8(dst, msrc, stride, sstride, 8);
        break;
    case 1:
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CH1FILTER(msrc + i));
            dst  += stride;
            msrc += sstride;
        }
        break;
    case 2:
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CH2FILTER(msrc + i));
            dst  += stride;
            msrc += sstride;
        }
        break;
    case 3:
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CH3FILTER(msrc + i));
            dst  += stride;
            msrc += sstride;
        }
        break;
    case 4:
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i*stride] = av_clip_uint8(CV1FILTER(msrc + i*sstride, sstride));
            dst  += 1;
            msrc += 1;
        }
        break;
    case 5:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = av_clip_uint8(CH1FILTER(msrc + j));
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV1FILTER(temp+j*8+i, 8));
            dst  += stride;
        }
        break;
    case 6:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = av_clip_uint8(CH2FILTER(msrc + j));
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV1FILTER(temp+j*8+i, 8));
            dst  += stride;
        }
        break;
    case 7:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = av_clip_uint8(CH3FILTER(msrc + j));
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV1FILTER(temp+j*8+i, 8));
            dst  += stride;
        }
        break;
    case 8:
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i*stride] = av_clip_uint8(CV2FILTER(msrc + i*sstride, sstride));
            dst  += 1;
            msrc += 1;
        }
        break;
    case 9:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = av_clip_uint8(CH1FILTER(msrc + j));
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV2FILTER(temp+j*8+i, 8));
            dst  += stride;
        }
        break;
    case 10:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = av_clip_uint8(CH2FILTER(msrc + j));
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV2FILTER(temp+j*8+i, 8));
            dst  += stride;
        }
        break;
    case 11:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = av_clip_uint8(CH3FILTER(msrc + j));
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV2FILTER(temp+j*8+i, 8));
            dst  += stride;
        }
        break;
    case 12:
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i*stride] = av_clip_uint8(CV3FILTER(msrc + i*sstride, sstride));
            dst  += 1;
            msrc += 1;
        }
        break;
    case 13:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = av_clip_uint8(CH1FILTER(msrc + j));
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV3FILTER(temp+j*8+i, 8));
            dst  += stride;
        }
        break;
    case 14:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = av_clip_uint8(CH2FILTER(msrc + j));
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV3FILTER(temp+j*8+i, 8));
            dst  += stride;
        }
        break;
    case 15:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = av_clip_uint8(CH3FILTER(msrc + j));
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV3FILTER(temp+j*8+i, 8));
            dst  += stride;
        }
        break;
    }
}

static void bink2f_y_mc(Bink2Context *c, int x, int y,
                        uint8_t *dst, int stride,
                        uint8_t *src, int sstride,
                        int width, int height,
                        int mv_x, int mv_y, int mode)
{
    uint8_t *msrc;

    if (mv_x < 0 || mv_x >= width ||
        mv_y < 0 || mv_y >= height)
        return;

    msrc = src + mv_x + mv_y * sstride;

    if (mode == 0) {
        copy_block16(dst, msrc, stride, sstride, 16);
    } else if (mode == 1) {
        for (int j = 0; j < 16; j++) {
            for (int i = 0; i < 16; i++)
                dst[i] = av_clip_uint8(LHFILTER(msrc + i));
            dst  += stride;
            msrc += sstride;
        }
    } else if (mode == 2) {
        for (int j = 0; j < 16; j++) {
            for (int i = 0; i < 16; i++)
                dst[i*stride] = av_clip_uint8(LVFILTER(msrc + i*sstride, sstride));
            dst  += 1;
            msrc += 1;
        }
    } else if (mode == 3) {
        uint8_t temp[21 * 16];

        msrc -= 2 * sstride;
        for (int i = 0; i < 21; i++) {
            for (int j = 0; j < 16; j++)
                temp[i*16+j] = av_clip_uint8(LHFILTER(msrc + j));
            msrc += sstride;
        }
        for (int j = 0; j < 16; j++) {
            for (int i = 0; i < 16; i++)
                dst[i] = av_clip_uint8(LVFILTER(temp+(j+2)*16+i, 16));
            dst  += stride;
        }
    }
}

static int bink2f_mcompensate_chroma(Bink2Context *c, int x, int y,
                                     uint8_t *dst, int stride,
                                     uint8_t *src, int sstride,
                                     int width, int height)
{
    MVectors *mv = &c->current_mv[c->mb_pos].mv;
    int mv_x, mv_y, mode;

    mv_x  = (mv->v[0][0] >> 2) + x;
    mv_y  = (mv->v[0][1] >> 2) + y;
    mode  =  mv->v[0][0] & 3;
    mode |= (mv->v[0][1] & 3) << 2;
    bink2f_c_mc(c, x, y, dst + x, stride, src, sstride, width, height, mv_x, mv_y, mode);

    mv_x = (mv->v[1][0] >> 2) + x + 8;
    mv_y = (mv->v[1][1] >> 2) + y;
    mode  =  mv->v[1][0] & 3;
    mode |= (mv->v[1][1] & 3) << 2;
    bink2f_c_mc(c, x, y, dst + x + 8, stride, src, sstride, width, height, mv_x, mv_y, mode);

    mv_x = (mv->v[2][0] >> 2) + x;
    mv_y = (mv->v[2][1] >> 2) + y + 8;
    mode  =  mv->v[2][0] & 3;
    mode |= (mv->v[2][1] & 3) << 2;
    bink2f_c_mc(c, x, y, dst + x + 8 * stride, stride, src, sstride, width, height, mv_x, mv_y, mode);

    mv_x = (mv->v[3][0] >> 2) + x + 8;
    mv_y = (mv->v[3][1] >> 2) + y + 8;
    mode  =  mv->v[3][0] & 3;
    mode |= (mv->v[3][1] & 3) << 2;
    bink2f_c_mc(c, x, y, dst + x + 8 + 8 * stride, stride, src, sstride, width, height, mv_x, mv_y, mode);

    return 0;
}

static float bink2f_average_block(uint8_t *src, int stride)
{
    int sum = 0;

    for (int i = 0; i < 8; i++) {
        int avg_a = (src[i+0*stride] + src[i+1*stride] + 1) >> 1;
        int avg_b = (src[i+2*stride] + src[i+3*stride] + 1) >> 1;
        int avg_c = (src[i+4*stride] + src[i+5*stride] + 1) >> 1;
        int avg_d = (src[i+6*stride] + src[i+7*stride] + 1) >> 1;
        int avg_e = (avg_a + avg_b + 1) >> 1;
        int avg_f = (avg_c + avg_d + 1) >> 1;
        int avg_g = (avg_e + avg_f + 1) >> 1;
        sum += avg_g;
    }

    return sum;
}

static void bink2f_average_chroma(Bink2Context *c, int x, int y,
                                  uint8_t *src, int stride,
                                  float *dc)
{
    for (int i = 0; i < 4; i++) {
        int X = i & 1;
        int Y = i >> 1;
        dc[i] = bink2f_average_block(src + x + X * 8 + (y + Y * 8) * stride, stride);
    }
}

static void bink2f_average_luma(Bink2Context *c, int x, int y,
                                uint8_t *src, int stride,
                                float *dc)
{
    for (int i = 0; i < 16; i++) {
        int I = luma_repos[i];
        int X = I & 3;
        int Y = I >> 2;
        dc[i] = bink2f_average_block(src + x + X * 8 + (y + Y * 8) * stride, stride);
    }
}

static int bink2f_mcompensate_luma(Bink2Context *c, int x, int y,
                                   uint8_t *dst, int stride,
                                   uint8_t *src, int sstride,
                                   int width, int height)
{
    MVectors *mv = &c->current_mv[c->mb_pos].mv;
    int mv_x, mv_y, mode;

    mv_x  = (mv->v[0][0] >> 1) + x;
    mv_y  = (mv->v[0][1] >> 1) + y;
    mode  =  mv->v[0][0] & 1;
    mode |= (mv->v[0][1] & 1) << 1;
    bink2f_y_mc(c, x, y, dst + x, stride, src, sstride, width, height, mv_x, mv_y, mode);

    mv_x  = (mv->v[1][0] >> 1) + x + 16;
    mv_y  = (mv->v[1][1] >> 1) + y;
    mode  =  mv->v[1][0] & 1;
    mode |= (mv->v[1][1] & 1) << 1;
    bink2f_y_mc(c, x, y, dst + x + 16, stride, src, sstride, width, height, mv_x, mv_y, mode);

    mv_x  = (mv->v[2][0] >> 1) + x;
    mv_y  = (mv->v[2][1] >> 1) + y + 16;
    mode  =  mv->v[2][0] & 1;
    mode |= (mv->v[2][1] & 1) << 1;
    bink2f_y_mc(c, x, y, dst + x + 16 * stride, stride, src, sstride, width, height, mv_x, mv_y, mode);

    mv_x  = (mv->v[3][0] >> 1) + x + 16;
    mv_y  = (mv->v[3][1] >> 1) + y + 16;
    mode  =  mv->v[3][0] & 1;
    mode |= (mv->v[3][1] & 1) << 1;
    bink2f_y_mc(c, x, y, dst + x + 16 + 16 * stride, stride, src, sstride, width, height, mv_x, mv_y, mode);

    return 0;
}

static int bink2f_decode_mv(Bink2Context *c, GetBitContext *gb, int x, int y,
                            int flags, MVectors *mv)
{
    for (int i = 0; i < 2; i++) {
        int val = 0, bits = get_bits(gb, 3);

        if (bits == 7)
            bits += get_bits(gb, 2);
        if (bits) {
            for (int j = 0; j < 4; j++)
                mv->v[j][i] = get_bits(gb, bits);
            for (int j = 0; j < 4; j++)
                if (mv->v[j][i] && get_bits1(gb))
                    mv->v[j][i] = -mv->v[j][i];
        }

        if ((flags & 0x80) && (flags & 0x20)) {
            val = get_bits(gb, 5) * 16;
            if (val && get_bits1(gb))
                val = -val;
        }

        mv->v[0][i] += val;
        mv->v[1][i] += val;
        mv->v[2][i] += val;
        mv->v[3][i] += val;
    }

    return 0;
}

static int bink2f_decode_slice(Bink2Context *c,
                               uint8_t *dst[4], int stride[4],
                               uint8_t *src[4], int sstride[4],
                               int is_kf, int start, int end)
{
    GetBitContext *gb = &c->gb;
    int w = c->avctx->width;
    int h = c->avctx->height;
    int flags, ret = 0;

    memset(c->prev_mv, 0, ((c->avctx->width + 31) / 32) * sizeof(*c->prev_mv));

    for (int y = start; y < end; y += 32) {
        unsigned y_cbp_intra = 0, u_cbp_intra = 0, v_cbp_intra = 0, a_cbp_intra = 0;
        unsigned y_cbp_inter = 0, u_cbp_inter = 0, v_cbp_inter = 0, a_cbp_inter = 0;
        int y_intra_q = 8, u_intra_q = 8, v_intra_q = 8, a_intra_q = 8;
        int y_inter_q = 8, u_inter_q = 8, v_inter_q = 8, a_inter_q = 8;

        memset(c->current_mv, 0, ((c->avctx->width + 31) / 32) * sizeof(*c->current_mv));

        for (int x = 0; x < c->avctx->width; x += 32) {
            MVectors mv = { 0 };
            int type = is_kf ? INTRA_BLOCK : get_bits(gb, 2);

            c->mb_pos = x / 32;
            c->current_dc[c->mb_pos].block_type = type;
            flags = 0;
            if (y == start)
                flags |= 0x80;
            if (!x)
                flags |= 0x20;
            if (x == 32)
                flags |= 0x200;
            if (x + 32 >= c->avctx->width)
                flags |= 0x40;

            switch (type) {
            case INTRA_BLOCK:
                if (!(flags & 0xA0) && c->prev_dc[c->mb_pos - 1].block_type != INTRA_BLOCK) {
                    bink2f_average_luma  (c, x  -32, -32, dst[0], stride[0], c->prev_dc[c->mb_pos - 1].dc[0]);
                    bink2f_average_chroma(c, x/2-16, -16, dst[2], stride[2], c->prev_dc[c->mb_pos - 1].dc[1]);
                    bink2f_average_chroma(c, x/2-16, -16, dst[1], stride[1], c->prev_dc[c->mb_pos - 1].dc[2]);
                }
                if (!(flags & 0x20) && c->current_dc[c->mb_pos - 1].block_type != INTRA_BLOCK) {
                    bink2f_average_luma  (c, x  -32, 0, dst[0], stride[0], c->current_dc[c->mb_pos - 1].dc[0]);
                    bink2f_average_chroma(c, x/2-16, 0, dst[2], stride[2], c->current_dc[c->mb_pos - 1].dc[1]);
                    bink2f_average_chroma(c, x/2-16, 0, dst[1], stride[1], c->current_dc[c->mb_pos - 1].dc[2]);
                }
                if ((flags & 0x20) && !(flags & 0x80) && c->prev_dc[c->mb_pos + 1].block_type != INTRA_BLOCK) {
                    bink2f_average_luma  (c, x  +32, -32, dst[0], stride[0], c->prev_dc[c->mb_pos + 1].dc[0]);
                    bink2f_average_chroma(c, x/2+16, -16, dst[2], stride[2], c->prev_dc[c->mb_pos + 1].dc[1]);
                    bink2f_average_chroma(c, x/2+16, -16, dst[1], stride[1], c->prev_dc[c->mb_pos + 1].dc[2]);
                }
                if (!(flags & 0x80) && c->prev_dc[c->mb_pos].block_type != INTRA_BLOCK) {
                    bink2f_average_luma  (c, x,   -32, dst[0], stride[0], c->prev_dc[c->mb_pos].dc[0]);
                    bink2f_average_chroma(c, x/2, -16, dst[2], stride[2], c->prev_dc[c->mb_pos].dc[1]);
                    bink2f_average_chroma(c, x/2, -16, dst[1], stride[1], c->prev_dc[c->mb_pos].dc[2]);
                }

                bink2f_predict_mv(c, x, y, flags, mv);
                c->comp = 0;
                ret = bink2f_decode_intra_luma(c, c->block, &y_cbp_intra, &y_intra_q,
                                               dst[0] + x, stride[0], flags);
                if (ret < 0)
                    goto fail;
                c->comp = 1;
                ret = bink2f_decode_intra_chroma(c, c->block, &u_cbp_intra, &u_intra_q,
                                                 dst[2] + x/2, stride[2], flags);
                if (ret < 0)
                    goto fail;
                c->comp = 2;
                ret = bink2f_decode_intra_chroma(c, c->block, &v_cbp_intra, &v_intra_q,
                                                 dst[1] + x/2, stride[1], flags);
                if (ret < 0)
                    goto fail;
                if (c->has_alpha) {
                    c->comp = 3;
                    ret = bink2f_decode_intra_luma(c, c->block, &a_cbp_intra, &a_intra_q,
                                                   dst[3] + x, stride[3], flags);
                    if (ret < 0)
                        goto fail;
                }
                break;
            case SKIP_BLOCK:
                copy_block16(dst[0] + x, src[0] + x + sstride[0] * y,
                             stride[0], sstride[0], 32);
                copy_block16(dst[0] + x + 16, src[0] + x + 16 + sstride[0] * y,
                             stride[0], sstride[0], 32);
                copy_block16(dst[1] + (x/2), src[1] + (x/2) + sstride[1] * (y/2),
                             stride[1], sstride[1], 16);
                copy_block16(dst[2] + (x/2), src[2] + (x/2) + sstride[2] * (y/2),
                             stride[2], sstride[2], 16);
                if (c->has_alpha) {
                    copy_block16(dst[3] + x, src[3] + x + sstride[3] * y,
                                 stride[3], sstride[3], 32);
                    copy_block16(dst[3] + x + 16, src[3] + x + 16 + sstride[3] * y,
                                 stride[3], sstride[3], 32);
                }
                break;
            case MOTION_BLOCK:
                bink2f_decode_mv(c, gb, x, y, flags, &mv);
                bink2f_predict_mv(c, x, y, flags, mv);
                c->comp = 0;
                ret = bink2f_mcompensate_luma(c, x, y,
                                              dst[0], stride[0],
                                              src[0], sstride[0],
                                              w, h);
                if (ret < 0)
                    goto fail;
                c->comp = 1;
                ret = bink2f_mcompensate_chroma(c, x/2, y/2,
                                                dst[2], stride[2],
                                                src[2], sstride[2],
                                                w/2, h/2);
                if (ret < 0)
                    goto fail;
                c->comp = 2;
                ret = bink2f_mcompensate_chroma(c, x/2, y/2,
                                                dst[1], stride[1],
                                                src[1], sstride[1],
                                                w/2, h/2);
                if (ret < 0)
                    goto fail;
                break;
            case RESIDUE_BLOCK:
                bink2f_decode_mv(c, gb, x, y, flags, &mv);
                bink2f_predict_mv(c, x, y, flags, mv);
                ret = bink2f_mcompensate_luma(c, x, y,
                                              dst[0], stride[0],
                                              src[0], sstride[0],
                                              w, h);
                if (ret < 0)
                    goto fail;
                ret = bink2f_mcompensate_chroma(c, x/2, y/2,
                                                dst[2], stride[2],
                                                src[2], sstride[2],
                                                w/2, h/2);
                if (ret < 0)
                    goto fail;
                ret = bink2f_mcompensate_chroma(c, x/2, y/2,
                                                dst[1], stride[1],
                                                src[1], sstride[1],
                                                w/2, h/2);
                if (ret < 0)
                    goto fail;
                c->comp = 0;
                ret = bink2f_decode_inter_luma(c, c->block, &y_cbp_inter, &y_inter_q,
                                               dst[0] + x, stride[0], flags);
                if (ret < 0)
                    goto fail;
                c->comp = 1;
                ret = bink2f_decode_inter_chroma(c, c->block, &u_cbp_inter, &u_inter_q,
                                                 dst[2] + x/2, stride[2], flags);
                if (ret < 0)
                    goto fail;
                c->comp = 2;
                ret = bink2f_decode_inter_chroma(c, c->block, &v_cbp_inter, &v_inter_q,
                                                 dst[1] + x/2, stride[1], flags);
                if (ret < 0)
                    goto fail;
                if (c->has_alpha) {
                    c->comp = 3;
                    ret = bink2f_decode_inter_luma(c, c->block, &a_cbp_inter, &a_inter_q,
                                                   dst[3] + x, stride[3], flags);
                    if (ret < 0)
                        goto fail;
                }
                break;
            default:
                return AVERROR_INVALIDDATA;
            }
        }

        dst[0] += stride[0] * 32;
        dst[1] += stride[1] * 16;
        dst[2] += stride[2] * 16;
        dst[3] += stride[3] * 32;

        FFSWAP(MVPredict *, c->current_mv, c->prev_mv);
        FFSWAP(DCPredict *, c->current_dc, c->prev_dc);
    }
fail:

    return ret;
}

#endif /* AVCODEC_BINK2F_H */
