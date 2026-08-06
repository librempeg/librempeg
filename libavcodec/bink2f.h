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
    1.f, 2.f, 2.5f, 3.f, 3.5f, 4.f, 6.f, 7.f, 8.f, 12.f, 16.f, 24.f, 32.f, 48.f, 64.f, 128.f
};

static const int32_t bink2f_luma_intra_qmat[64] = {
    0x3E000000, 0x3E550CA2, 0x3E6A22F7, 0x3E52B778, 0x3E666666, 0x3E715D70, 0x3EA9B824, 0x3E7E44B7,
    0x3E434B94, 0x3E93C105, 0x3E96C786, 0x3EB17397, 0x3EC34B94, 0x3EF41CE7, 0x3F19BC34, 0x3EE152F4,
    0x3E273D5C, 0x3EA260D4, 0x3EAECEBC, 0x3ED851A8, 0x3F1AB26C, 0x3F34ACB0, 0x3F307E6B, 0x3EDB2BFE,
    0x3E70D1B7, 0x3EC653EB, 0x3EEBFC01, 0x3F005027, 0x3F52B789, 0x3F3D35EB, 0x3F312AFA, 0x3ECB7A81,
    0x3E99999A, 0x3EE6CDAF, 0x3F273D6C, 0x3F3FE729, 0x3F59999A, 0x3F4BA6F0, 0x3F3260D4, 0x3EC5C3BD,
    0x3EC9233E, 0x3F4A43DD, 0x3F3B3E9A, 0x3F809AA2, 0x3F890670, 0x3F4D716D, 0x3F24A4B1, 0x3E8ABC0E,
    0x3EB0A58B, 0x3F102071, 0x3F1C2118, 0x3F22E9BC, 0x3F3260D4, 0x3F19C1F8, 0x3EE0F12C, 0x3E44DB9C,
    0x3E576C05, 0x3E86B463, 0x3E81322F, 0x3E80BB2C, 0x3E87F690, 0x3E7F459E, 0x3E410907, 0x3DC0EB68
};

static const int32_t bink2f_luma_inter_qmat[64] = {
    0x3E000000, 0x3E318A87, 0x3E273D5C, 0x3E1682F9, 0x3E038EF3, 0x3DD9E493, 0x3D9DCA4B, 0x3D309310,
    0x3E318A87, 0x3E7641B3, 0x3E67F801, 0x3E50C415, 0x3E367A10, 0x3E171CDA, 0x3DE033E8, 0x3D7FCFC4,
    0x3E273D5C, 0x3E67F801, 0x3E6095AF, 0x3E4F965B, 0x3E39D1F6, 0x3E194DD7, 0x3DE74EA9, 0x3D802C0A,
    0x3E1682F9, 0x3E50C415, 0x3E4F965B, 0x3E4E7C8D, 0x3E3C23FB, 0x3E1DABE2, 0x3DEFD75E, 0x3D85CE5B,
    0x3E038EF3, 0x3E367A10, 0x3E39D1F6, 0x3E3C23FB, 0x3E35568F, 0x3E1F3B21, 0x3DF64E8B, 0x3D8B4BF9,
    0x3DD9E493, 0x3E171CDA, 0x3E194DD7, 0x3E1DABE2, 0x3E1F3B21, 0x3E154110, 0x3DEEDEDB, 0x3D8C471B,
    0x3D9DCA4B, 0x3DE033E8, 0x3DE74EA9, 0x3DEFD75E, 0x3DF64E8B, 0x3DEEDEDB, 0x3DCA06EA, 0x3D765669,
    0x3D309310, 0x3D7FCFC4, 0x3D802C0A, 0x3D85CE5B, 0x3D8B4BF9, 0x3D8C471B, 0x3D765669, 0x3D1F2453
};

static const int32_t bink2f_chroma_qmat[64] = {
    0x3E000000, 0x3E318A87, 0x3E5EFCBD, 0x3EC48084, 0x3F300000, 0x3F0A4841, 0x3EBE8039, 0x3E423BBC,
    0x3E318A87, 0x3E8FA659, 0x3EA78876, 0x3F3F5E74, 0x3F741E79, 0x3F3FCD9A, 0x3F041DC1, 0x3E86B463,
    0x3E5EFCBD, 0x3EA78876, 0x3F29F3B6, 0x3F873301, 0x3F65F480, 0x3F34ACB0, 0x3EF8E6D1, 0x3E7DC726,
    0x3EC48084, 0x3F3F5E74, 0x3F873301, 0x3F735A32, 0x3F4EF438, 0x3F229A4E, 0x3EE00171, 0x3E6464DC,
    0x3F300000, 0x3F741E79, 0x3F65F480, 0x3F4EF438, 0x3F300000, 0x3F0A4841, 0x3EBE8039, 0x3E423BBC,
    0x3F0A4841, 0x3F3FCD9A, 0x3F34ACB0, 0x3F229A4E, 0x3F0A4841, 0x3ED94B9D, 0x3E95ACEF, 0x3E189B95,
    0x3EBE8039, 0x3F041DC1, 0x3EF8E6D1, 0x3EE00171, 0x3EBE8039, 0x3E95ACEF, 0x3E4E325D, 0x3DD23CC9,
    0x3E423BBC, 0x3E86B463, 0x3E7DC726, 0x3E6464DC, 0x3E423BBC, 0x3E189B95, 0x3DD23CC9, 0x3D565B21
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
    const float A = av_int2float(0x3fec835e);
    const float B = av_int2float(0x40273d75);
    const float C = av_int2float(0x3f8a8bd4);
    float t00 =  blk[2 * step] + blk[6 * step];
    float t01 = (blk[2 * step] - blk[6 * step]) * M_SQRT2f - t00;
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
    float t13 = (t06 - t04) * M_SQRT2f;
    float t14 = (t07 - t05) * A;
    float t15 = t05 * B + t14 - t12;
    float t16 = t13 - t15;
    float t17 = t07 * C - t14 + t16;

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
    block[0] += 512.5f;

    for (int i = 0; i < 8; i++)
        bink2f_idct_1d(block + i, 8);
    for (int i = 0; i < 8; i++)
        bink2f_idct_1d(block + i * 8, 1);

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++)
            dst[j] = av_clip_uint8(truncf(block[i*8+j]) - 512);
        dst += stride;
    }
}

static void bink2f_idct_add(uint8_t *dst, int stride, float *block)
{
    block[0] += 512.5f;

    for (int i = 0; i < 8; i++)
        bink2f_idct_1d(block + i, 8);
    for (int i = 0; i < 8; i++)
        bink2f_idct_1d(block + i * 8, 1);

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++)
            dst[j] = av_clip_uint8(dst[j] + truncf(block[i*8+j]) - 512);
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

#undef DC_MPRED
#undef DC_MPRED2
#define DC_MPRED(A, B, C) av_clipf((C) + (B) - (A), FFMIN3(A, B, C), FFMAX3(A, B, C))
#define DC_MPRED2(A, B) av_clipf(2.f * (A) - (B), fminf(A, B), fmaxf(A, B))

static void bink2f_predict_dc(Bink2Context *c,
                              int is_luma, float mindc, float maxdc,
                              int flags, float tdc[16])
{
    float *dc = c->current_dc[c->mb_pos].dc[c->comp];

    if (is_luma && (flags & 0x20) && (flags & 0x80)) {
        dc[0]  = av_clipf(tdc[0], mindc, maxdc);
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
        const float *Ldc = c->current_dc[c->mb_pos - 1].dc[c->comp];

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
        const float *Tdc = c->prev_dc[c->mb_pos].dc[c->comp];

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
        const float *LTdc = c->prev_dc[c->mb_pos - 1].dc[c->comp];
        const float *Ldc = c->current_dc[c->mb_pos - 1].dc[c->comp];
        const float *Tdc = c->prev_dc[c->mb_pos].dc[c->comp];

        dc[0]  = av_clipf(DC_MPRED(LTdc[15], Ldc[5], Tdc[10]) + tdc[0], mindc, maxdc);
        dc[1]  = av_clipf(DC_MPRED(Tdc[10], Tdc[11], dc[0]) + tdc[1], mindc, maxdc);
        dc[2]  = av_clipf(DC_MPRED(Ldc[5], Ldc[7], dc[0]) + tdc[2], mindc, maxdc);
        dc[3]  = av_clipf(DC_MPRED(dc[0], dc[2], dc[1]) + tdc[3], mindc, maxdc);
        dc[4]  = av_clipf(DC_MPRED(Tdc[11], dc[1], Tdc[14]) + tdc[4], mindc, maxdc);
        dc[5]  = av_clipf(DC_MPRED(Tdc[14], dc[4], Tdc[15]) + tdc[5], mindc, maxdc);
        dc[6]  = av_clipf(DC_MPRED(dc[1], dc[3], dc[4]) + tdc[6], mindc, maxdc);
        dc[7]  = av_clipf(DC_MPRED(dc[4], dc[6], dc[5]) + tdc[7], mindc, maxdc);
        dc[8]  = av_clipf(DC_MPRED(Ldc[7], dc[2], Ldc[13]) + tdc[8], mindc, maxdc);
        dc[9]  = av_clipf(DC_MPRED(dc[2], dc[8], dc[3]) + tdc[9], mindc, maxdc);
        dc[10] = av_clipf(DC_MPRED(Ldc[13], Ldc[15], dc[8]) + tdc[10], mindc, maxdc);
        dc[11] = av_clipf(DC_MPRED(dc[8], dc[10], dc[9]) + tdc[11], mindc, maxdc);
        dc[12] = av_clipf(DC_MPRED(dc[3], dc[9], dc[6]) + tdc[12], mindc, maxdc);
        dc[13] = av_clipf(DC_MPRED(dc[6], dc[12], dc[7]) + tdc[13], mindc, maxdc);
        dc[14] = av_clipf(DC_MPRED(dc[9], dc[11], dc[12]) + tdc[14], mindc, maxdc);
        dc[15] = av_clipf(DC_MPRED(dc[12], dc[14], dc[13]) + tdc[15], mindc, maxdc);
    } else if (!is_luma && (flags & 0x20) && (flags & 0x80)) {
        dc[0] = av_clipf(tdc[0], mindc, maxdc);
        dc[1] = av_clipf(dc[0] + tdc[1], mindc, maxdc);
        dc[2] = av_clipf(DC_MPRED2(dc[0], dc[1]) + tdc[2], mindc, maxdc);
        dc[3] = av_clipf(DC_MPRED(dc[0], dc[2], dc[1]) + tdc[3], mindc, maxdc);
    } else if (!is_luma && (flags & 0x80)) {
        const float *Ldc = c->current_dc[c->mb_pos - 1].dc[c->comp];

        dc[0] = av_clipf(DC_MPRED2(Ldc[1], Ldc[3]) + tdc[0], mindc, maxdc);
        dc[1] = av_clipf(dc[0] + tdc[1], mindc, maxdc);
        dc[2] = av_clipf(DC_MPRED(Ldc[1], Ldc[3], dc[0]) + tdc[2], mindc, maxdc);
        dc[3] = av_clipf(DC_MPRED(dc[0], dc[2], dc[1]) + tdc[3], mindc, maxdc);
    } else if (!is_luma && (flags & 0x20)) {
        const float *Tdc = c->prev_dc[c->mb_pos].dc[c->comp];

        dc[0] = av_clipf(DC_MPRED2(Tdc[2], Tdc[3]) + tdc[0], mindc, maxdc);
        dc[1] = av_clipf(DC_MPRED(Tdc[2], dc[0], Tdc[3]) + tdc[1], mindc, maxdc);
        dc[2] = av_clipf(DC_MPRED2(dc[0], dc[1]) + tdc[2], mindc, maxdc);
        dc[3] = av_clipf(DC_MPRED(dc[0], dc[2], dc[1]) + tdc[3], mindc, maxdc);
    } else if (!is_luma) {
        const float *Ldc = c->current_dc[c->mb_pos - 1].dc[c->comp];
        const float *LTdc = c->prev_dc[c->mb_pos - 1].dc[c->comp];
        const float *Tdc = c->prev_dc[c->mb_pos].dc[c->comp];

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

    if ((flags & 0x20) && (flags & 0x80) && mindc >= 0)
        tdc[0] += 1024.f;

    bink2f_predict_dc(c, is_luma, mindc, maxdc, flags, tdc);
}

static int bink2f_decode_ac(GetBitContext *gb, const uint8_t *scan,
                            float block[4][64], unsigned cbp,
                            float q, const int32_t qmat[64])
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

            block[i][scan[idx]] = val * q * av_int2float(qmat[(scan[idx]/8) + (scan[idx]&7)*8]);
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

    bink2f_decode_dc(c, gb, dc, 0, 0, -1023, 1023, 0xA8);

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

    if (!(flags & 0x80)) {
        if (flags & 0x20) {
            const MVectors *t_mv = &c->prev_mv[c->mb_pos].mv;

            c_mv->v[0][0] = mv.v[0][0] + mid_pred(t_mv->v[0][0], t_mv->v[2][0], t_mv->v[3][0]);
            c_mv->v[0][1] = mv.v[0][1] + mid_pred(t_mv->v[0][1], t_mv->v[2][1], t_mv->v[3][1]);
            c_mv->v[1][0] = mv.v[1][0] + mid_pred(t_mv->v[2][0], t_mv->v[3][0], c_mv->v[0][0]);
            c_mv->v[1][1] = mv.v[1][1] + mid_pred(t_mv->v[2][1], t_mv->v[3][1], c_mv->v[0][1]);
            c_mv->v[2][0] = mv.v[2][0] + mid_pred(t_mv->v[2][0], c_mv->v[0][0], c_mv->v[1][0]);
            c_mv->v[2][1] = mv.v[2][1] + mid_pred(t_mv->v[2][1], c_mv->v[0][1], c_mv->v[1][1]);
            c_mv->v[3][0] = mv.v[3][0] + mid_pred(c_mv->v[0][0], c_mv->v[1][0], c_mv->v[2][0]);
            c_mv->v[3][1] = mv.v[3][1] + mid_pred(c_mv->v[0][1], c_mv->v[1][1], c_mv->v[2][1]);
        } else {
            const MVectors *l_mv = &c->current_mv[c->mb_pos - 1].mv;
            const MVectors *lt_mv = &c->prev_mv[c->mb_pos - 1].mv;
            const MVectors *t_mv = &c->prev_mv[c->mb_pos].mv;

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
            const MVectors *l_mv = &c->current_mv[c->mb_pos - 1].mv;

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

#undef CH1FILTER
#undef CH2FILTER
#undef CH3FILTER
#define CH1FILTER(src, shift) ((3*(src)[0] +   (src)[1] + (1<<shift)/2) >> shift)
#define CH2FILTER(src, shift) ((2*(src)[0] + 2*(src)[1] + (1<<shift)/2) >> shift)
#define CH3FILTER(src, shift) ((  (src)[0] + 3*(src)[1] + (1<<shift)/2) >> shift)

#undef CV1FILTER
#undef CV2FILTER
#undef CV3FILTER
#define CV1FILTER(src, i, shift) ((3*(src)[0] +   (src)[i] + (1<<shift)/2) >> shift)
#define CV2FILTER(src, i, shift) ((2*(src)[0] + 2*(src)[i] + (1<<shift)/2) >> shift)
#define CV3FILTER(src, i, shift) ((  (src)[0] + 3*(src)[i] + (1<<shift)/2) >> shift)

static void bink2f_c_mc(Bink2Context *c, int x, int y,
                        uint8_t *dst, int stride,
                        uint8_t *src, int sstride,
                        int width, int height,
                        int mv_x, int mv_y,
                        int mode)
{
    uint16_t temp[8*9];
    uint8_t *msrc;

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
                dst[i] = av_clip_uint8(CH1FILTER(msrc + i, 2));
            dst  += stride;
            msrc += sstride;
        }
        break;
    case 2:
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CH2FILTER(msrc + i, 2));
            dst  += stride;
            msrc += sstride;
        }
        break;
    case 3:
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CH3FILTER(msrc + i, 2));
            dst  += stride;
            msrc += sstride;
        }
        break;
    case 4:
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i*stride] = av_clip_uint8(CV1FILTER(msrc + i*sstride, sstride, 2));
            dst  += 1;
            msrc += 1;
        }
        break;
    case 5:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = CH1FILTER(msrc + j, 0);
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV1FILTER(temp+j*8+i, 8, 4));
            dst  += stride;
        }
        break;
    case 6:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = CH2FILTER(msrc + j, 0);
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV1FILTER(temp+j*8+i, 8, 4));
            dst  += stride;
        }
        break;
    case 7:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = CH3FILTER(msrc + j, 0);
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV1FILTER(temp+j*8+i, 8, 4));
            dst  += stride;
        }
        break;
    case 8:
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i*stride] = av_clip_uint8(CV2FILTER(msrc + i*sstride, sstride, 2));
            dst  += 1;
            msrc += 1;
        }
        break;
    case 9:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = CH1FILTER(msrc + j, 0);
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV2FILTER(temp+j*8+i, 8, 4));
            dst  += stride;
        }
        break;
    case 10:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = CH2FILTER(msrc + j, 0);
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV2FILTER(temp+j*8+i, 8, 4));
            dst  += stride;
        }
        break;
    case 11:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = CH3FILTER(msrc + j, 0);
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV2FILTER(temp+j*8+i, 8, 4));
            dst  += stride;
        }
        break;
    case 12:
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i*stride] = av_clip_uint8(CV3FILTER(msrc + i*sstride, sstride, 2));
            dst  += 1;
            msrc += 1;
        }
        break;
    case 13:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = CH1FILTER(msrc + j, 0);
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV3FILTER(temp+j*8+i, 8, 4));
            dst  += stride;
        }
        break;
    case 14:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = CH2FILTER(msrc + j, 0);
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV3FILTER(temp+j*8+i, 8, 4));
            dst  += stride;
        }
        break;
    case 15:
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 8; j++)
                temp[i*8+j] = CH3FILTER(msrc + j, 0);
            msrc += sstride;
        }
        for (int j = 0; j < 8; j++) {
            for (int i = 0; i < 8; i++)
                dst[i] = av_clip_uint8(CV3FILTER(temp+j*8+i, 8, 4));
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
        int16_t temp[21 * 16];

        msrc -= 2 * sstride;
        for (int i = 0; i < 21; i++) {
            for (int j = 0; j < 16; j++)
                temp[i*16+j] = LHFILTER(msrc + j);
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
    const int w = c->avctx->width;
    const int h = c->avctx->height;
    int flags, ret = 0;

    memset(c->prev_mv, 0, ((w + 31) / 32) * sizeof(*c->prev_mv));

    for (int y = start; y < end; y += 32) {
        unsigned y_cbp_intra = 0, u_cbp_intra = 0, v_cbp_intra = 0, a_cbp_intra = 0;
        unsigned y_cbp_inter = 0, u_cbp_inter = 0, v_cbp_inter = 0, a_cbp_inter = 0;
        int y_intra_q = 8, u_intra_q = 8, v_intra_q = 8, a_intra_q = 8;
        int y_inter_q = 8, u_inter_q = 8, v_inter_q = 8, a_inter_q = 8;

        memset(c->current_mv, 0, ((w + 31) / 32) * sizeof(*c->current_mv));

        for (int x = 0; x < w; x += 32) {
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
            if (x + 32 >= w)
                flags |= 0x40;

            switch (type) {
            case INTRA_BLOCK:
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

            if (c->current_dc[c->mb_pos].block_type != INTRA_BLOCK) {
                bink2f_average_luma  (c, x, 0, dst[0], stride[0], c->current_dc[c->mb_pos].dc[0]);
                bink2f_average_chroma(c, x/2, 0, dst[2], stride[2], c->current_dc[c->mb_pos].dc[1]);
                bink2f_average_chroma(c, x/2, 0, dst[1], stride[1], c->current_dc[c->mb_pos].dc[2]);
                if (c->has_alpha)
                    bink2f_average_luma(c, x, 0, dst[3], stride[3], c->current_dc[c->mb_pos].dc[3]);
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
