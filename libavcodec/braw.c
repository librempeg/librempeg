/*
 * Blackmagic RAW decoder
 *
 * Copyright (c) 2019-2025 Paul B Mahol
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "libavutil/mem_internal.h"
#include "libavutil/opt.h"
#include "libavutil/thread.h"

#define CACHED_BITSTREAM_READER !ARCH_X86_32

#include "avcodec.h"
#include "decode.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "get_bits.h"
#include "internal.h"
#include "idctdsp.h"
#include "internal.h"
#include "thread.h"
#include "threadframe.h"
#include "vlc.h"

static const uint8_t half_scan[] = {
     0, 1, 2, 3, 8, 9, 16, 17, 10, 11, 4, 5, 6, 7, 12, 13, 18,
    19, 24, 25, 26, 27, 20, 21, 14,15, 22, 23, 28, 29, 30, 31,
};

static const int16_t dcval_tab[2][16] = {
  {    -1,     -3,     -7,    -15,
      -31,    -63,   -127,   -255,
     -511,  -1023,  -2047,  -4095,
    -8191, -16383, -32767,      0 },
  {     1,      2,      4,      8,
       16,     32,     64,    128,
      256,    512,   1024,   2048,
     4096,   8192,  16384,      0 },
};

static const uint8_t dc_bits[16] = {
    2, 3, 3, 3, 3, 3, 5, 5, 6, 12, 12, 12, 12, 5, 12, 13,
};

static const uint16_t dc_codes[16] = {
    0x0, 0x2, 0x3, 0x4, 0x5, 0x6, 0x1C, 0x1D, 0x3E, 0xFF4, 0xFF5, 0xFF7, 0xFED, 0x1E, 0xFFE, 0x1FFE,
};

static const uint8_t dc_table[16][3] = {
    {  0, 0, 15 }, {  0, 1,  0 }, {  1, 1,  1 }, {  2, 1,  2 },
    {  3, 1,  3 }, {  4, 1,  4 }, {  5, 1,  5 }, {  6, 1,  6 },
    {  7, 1,  7 }, {  8, 1,  8 }, {  9, 1,  9 }, { 10, 1, 10 },
    { 11, 1, 11 }, { 12, 1, 12 }, { 13, 1, 13 }, { 14, 1, 14 },
};

static const uint8_t ac_bits[194] = {
    2, 2, 3, 4, 4, 4, 5, 5, 5, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10,
    10, 10, 10, 11, 11, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 15, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 18, 18, 18, 18,
    18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
    18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
};

static const uint32_t ac_codes[194] = {
    0x000000, 0x000001, 0x000004, 0x00000B, 0x00000C, 0x00000A,
    0x00001A, 0x00001B, 0x00001C, 0x00003A, 0x00003B, 0x000078,
    0x000079, 0x00007A, 0x00007B, 0x0000F8, 0x0000F9, 0x0000FA,
    0x0001F6, 0x0001F7, 0x0001F8, 0x0001F9, 0x0001FA, 0x0003F6,
    0x0003F7, 0x0003F8, 0x0003F9, 0x0003FA, 0x0007F8, 0x0007F9,
    0x000FED, 0x000FF4, 0x000FF5, 0x000FF7, 0x000FEC, 0x000FF6,
    0x001FDC, 0x001FDD, 0x001FDE, 0x001FDF, 0x007FC0, 0x00FF84,
    0x00FF85, 0x00FF86, 0x00FF87, 0x00FF88, 0x00FF89, 0x00FF8A,
    0x00FF8B, 0x00FF8C, 0x00FF8D, 0x00FF8E, 0x00FF8F, 0x00FF90,
    0x00FF91, 0x00FF92, 0x00FF93, 0x00FF94, 0x00FF95, 0x00FF96,
    0x00FF97, 0x00FF98, 0x00FF99, 0x00FF9A, 0x00FF9B, 0x00FF9C,
    0x00FF9D, 0x00FF9E, 0x00FF9F, 0x00FFA0, 0x00FFA1, 0x00FFA2,
    0x00FFA3, 0x00FFA4, 0x00FFA5, 0x00FFA6, 0x00FFA7, 0x00FFA8,
    0x00FFA9, 0x00FFAA, 0x00FFAB, 0x00FFAC, 0x00FFAE, 0x00FFAF,
    0x00FFB0, 0x00FFB1, 0x00FFB2, 0x00FFB3, 0x00FFB4, 0x00FFB6,
    0x00FFB7, 0x00FFB8, 0x00FFB9, 0x00FFBA, 0x00FFBB, 0x00FFBC,
    0x00FFBE, 0x00FFBF, 0x00FFC0, 0x00FFC1, 0x00FFC2, 0x00FFC3,
    0x00FFC4, 0x00FFC5, 0x00FFC7, 0x00FFC8, 0x00FFC9, 0x00FFCA,
    0x00FFCB, 0x00FFCC, 0x00FFCD, 0x00FFCE, 0x00FFD0, 0x00FFD1,
    0x00FFD2, 0x00FFD3, 0x00FFD4, 0x00FFD5, 0x00FFD6, 0x00FFD7,
    0x00FFD9, 0x00FFDA, 0x00FFDB, 0x00FFDC, 0x00FFDD, 0x00FFDE,
    0x00FFDF, 0x00FFE0, 0x00FFE2, 0x00FFE3, 0x00FFE4, 0x00FFE5,
    0x00FFE6, 0x00FFE7, 0x00FFE8, 0x00FFE9, 0x00FFEB, 0x00FFEC,
    0x00FFED, 0x00FFEE, 0x00FFEF, 0x00FFF0, 0x00FFF1, 0x00FFF2,
    0x00FFF3, 0x00FFF5, 0x00FFF6, 0x00FFF7, 0x00FFF8, 0x00FFF9,
    0x00FFFA, 0x00FFFB, 0x00FFFC, 0x00FFFD, 0x03FEB5, 0x03FEB6,
    0x03FEB7, 0x03FED5, 0x03FED6, 0x03FED7, 0x03FEF5, 0x03FEF6,
    0x03FEF7, 0x03FF19, 0x03FEB4, 0x03FF1A, 0x03FF1B, 0x03FED4,
    0x03FF3D, 0x03FF3E, 0x03FEF4, 0x03FF3F, 0x03FF61, 0x03FF18,
    0x03FF62, 0x03FF63, 0x03FF3C, 0x03FF85, 0x03FF86, 0x03FF60,
    0x03FF87, 0x03FFA9, 0x03FF84, 0x03FFAA, 0x03FFAB, 0x03FFA8,
    0x03FFD1, 0x03FFD2, 0x03FFD0, 0x03FFD3, 0x03FFF9, 0x03FFF8,
    0x03FFFA, 0x03FFFB,
};

static const uint8_t ac_table[194][2] = {
    { 0, 1 }, { 0, 2 }, { 0, 3 }, { 0, 4 }, { 1, 1 }, { 255, 0 },
    { 0, 5 }, { 1, 2 }, { 2, 1 }, { 3, 1 }, { 4, 1 }, { 0, 6 },
    { 1, 3 }, { 5, 1 }, { 6, 1 }, { 0, 7 }, { 2, 2 }, { 7, 1 },
    { 1, 4 }, { 3, 2 }, { 8, 1 }, { 9, 1 }, { 10, 1 }, { 0, 8 },
    { 2, 3 }, { 4, 2 }, { 11, 1 }, { 12, 1 }, { 13, 1 }, { 15, 0 },
    { 0, 12 },{ 0, 9 }, { 0, 10 }, { 0, 11 }, { 1, 5 }, { 6, 2 },
    { 2, 4 }, { 3, 3 }, { 5, 2 }, { 7, 2 }, { 8, 2 }, { 1, 6 },
    { 1, 7 }, { 1, 8 }, { 1, 9 }, { 1, 10 }, { 2, 5 }, { 2, 6 },
    { 2, 7 }, { 2, 8 }, { 2, 9 }, { 2, 10 }, { 3, 4 }, { 3, 5 },
    { 3, 6 }, { 3, 7 }, { 3, 8 }, { 3, 9 }, { 3, 10 }, { 4, 3 },
    { 4, 4 }, { 4, 5 }, { 4, 6 }, { 4, 7 }, { 4, 8 }, { 4, 9 },
    { 4, 10 }, { 5, 3 }, { 5, 4 }, { 5, 5 }, { 5, 6 }, { 5, 7 },
    { 5, 8 }, { 5, 9 }, { 5, 10 }, { 6, 3 }, { 6, 4 }, { 6, 5 },
    { 6, 6 }, { 6, 7 }, { 6, 8 }, { 6, 9 }, { 7, 3 }, { 7, 4 },
    { 7, 5 }, { 7, 6 }, { 7, 7 }, { 7, 8 }, { 7, 9 }, { 8, 3 },
    { 8, 4 }, { 8, 5 }, { 8, 6 }, { 8, 7 }, { 8, 8 }, { 8, 9 },
    { 9, 2 }, { 9, 3 }, { 9, 4 }, { 9, 5 }, { 9, 6 }, { 9, 7 },
    { 9, 8 }, { 9, 9 }, { 10, 2 }, { 10, 3 }, { 10, 4 }, { 10, 5 },
    { 10, 6 }, { 10, 7 }, { 10, 8 }, { 10, 9 }, { 11, 2 }, { 11, 3 },
    { 11, 4 }, { 11, 5 }, { 11, 6 }, { 11, 7 }, { 11, 8 }, { 11, 9 },
    { 12, 2 }, { 12, 3 }, { 12, 4 }, { 12, 5 }, { 12, 6 }, { 12, 7 },
    { 12, 8 }, { 12, 9 }, { 13, 2 }, { 13, 3 }, { 13, 4 }, { 13, 5 },
    { 13, 6 }, { 13, 7 }, { 13, 8 }, { 13, 9 }, { 14, 1 }, { 14, 2 },
    { 14, 3 }, { 14, 4 }, { 14, 5 }, { 14, 6 }, { 14, 7 }, { 14, 8 },
    { 14, 9 }, { 15, 1 }, { 15, 2 }, { 15, 3 }, { 15, 4 }, { 15, 5 },
    { 15, 6 }, { 15, 7 }, { 15, 8 }, { 15, 9 }, { 1, 11 }, { 1, 12 },
    { 2, 11 }, { 2, 12 }, { 3, 11 }, { 3, 12 }, { 4, 11 }, { 4, 12 },
    { 5, 11 }, { 5, 12 }, { 6, 10 }, { 6, 11 }, { 6, 12 }, { 7, 10 },
    { 7, 11 }, { 7, 12 }, { 8, 10 }, { 8, 11 }, { 8, 12 }, { 9, 10 },
    { 9, 11 }, { 9, 12 }, { 10, 10 }, { 10, 11 }, { 10, 12 }, { 11, 10 },
    { 11, 11 }, { 11, 12 }, { 12, 10 }, { 12, 11 }, { 12, 12 }, { 13, 10 },
    { 13, 11 }, { 13, 12 }, { 14, 10 }, { 14, 11 }, { 14, 12 }, { 15, 10 },
    { 15, 11 }, { 15, 12 },
};

static VLCElem dc_vlc[544];
static VLCElem ac_vlc[2474];

typedef struct BRAWContext {
    const AVClass    *class;

    GetByteContext    gb;
    GetBitContext     gbit;

    uint32_t          header_size;

    int               offsets[4][2];

    int               nb_tiles_w;
    int               nb_tiles_h;
    int               tile_size_w[256];
    int               tile_offset_w[257];
    int               tile_size_h;
    int               blocks_w_in_tile;
    int               blocks_h_in_tile;

    int               qscale;
    int               qscale2;
    int               qscale3;
    int               quant[2][64];

    DECLARE_ALIGNED(32, int16_t, block)[4][64];
    DECLARE_ALIGNED(32, uint8_t, out)[4][128];
} BRAWContext;

static int parse_frame_metadata(AVCodecContext *avctx)
{
    BRAWContext *s = avctx->priv_data;
    GetByteContext *gb = &s->gb;

    s->header_size = bytestream2_get_be32(gb);
    if (s->header_size <= 8 ||
        s->header_size - 8 >= bytestream2_get_bytes_left(gb))
        return AVERROR_INVALIDDATA;
    if (bytestream2_get_le32(gb) != MKTAG('b','m','d','f'))
        return AVERROR_INVALIDDATA;
    bytestream2_skip(gb, s->header_size - 8);

    return 0;
}

static int decode_block(AVCodecContext *avctx,
                        int16_t *dst, const int *quant,
                        int *prev_dc, int max,
                        const uint8_t *scan)
{
    BRAWContext *s = avctx->priv_data;
    GetBitContext *gbit = &s->gbit;
    int dc_idx, sgnbit = 0, sign, len, val, code;

    memset(dst, 0, 64 * 2);

    dc_idx = get_vlc2(gbit, dc_vlc, 9, 2);
    if (dc_idx < 0)
        return AVERROR_INVALIDDATA;

    sign = dc_table[dc_idx][1];
    sgnbit = get_bitsz(gbit, sign);
    len = dc_table[dc_idx][0];
    code = get_bitsz(gbit, len);
    val = code + *prev_dc + dcval_tab[sgnbit][dc_table[dc_idx][2]];
    *prev_dc = val;
    dst[0] = val * quant[0] + 0x8000 >> 16;

    for (int i = 0;;) {
        int skip, len, ac_idx = get_vlc2(gbit, ac_vlc, 11, 3);

        if (ac_idx < 0)
            return AVERROR_INVALIDDATA;
        skip = ac_table[ac_idx][0];
        if (skip == 255)
            break;

        len = ac_table[ac_idx][1];
        val = get_bits_long(gbit, len);
        i = i + 1 + skip;
        if (i >= max)
            return AVERROR_INVALIDDATA;
        if (len && val < 1 << (len - 1))
            val -= (1 << len) - 1;
        dst[scan[i]] = val * quant[scan[i]] + 0x8000 >> 16;
    }

    return 0;
}

static void idct4(int16_t *in, int step)
{
    const float a = 0.270598050073099f;
    const float b = 0.653281482438188f;
    const float c = 0.5f;
    float va = in[0 * step] * c;
    float vb = in[2 * step] * c;
    float vc = in[1 * step];
    float vd = in[3 * step];
    float vab = va + vb;
    float vamb = va - vb;
    float vcavdb = vc * a - vd * b;
    float vdavcb = vd * a + vc * b;
    float d0 = vab + vdavcb;
    float d1 = vamb + vcavdb;
    float d2 = vamb - vcavdb;
    float d3 = vab - vdavcb;
    in[0 * step] = d0;
    in[1 * step] = d1;
    in[2 * step] = d2;
    in[3 * step] = d3;
}

static void idct8(int16_t *src, int step)
{
    const float N_SQRT2 = 1.4142135623730951f;
    const float c0 = 1.0823922002923942f;
    const float c1 = 2.6131259297527532f;
    const float c2 = 1.8477590650225735f;
    const float c3 = 0.353553390593274f;
    const float c4 = 0.461939766255643f;
    const float c5 = 0.191341716182545f;
    const float c6 = 0.277785116509801f;
    const float c7 = 0.490392640201615f;
    const float c8 = 0.097545161008064f;
    const float c9 = 0.415734806151273f;
    float t0, t1, t2, t3, t4, t5, t6, t7;
    float t4m7, t5p6, t5m6, t4p7, t2m3, t2p3;
    float x0, x1, x2, x3, x4, x5, x6, x7, x8;
    float x9, x10, x11, x12, x13, x14, x15, x16;
    float d0, d1, d2, d3, d4, d5, d6, d7;
    t0 = src[0 * step] * c3;
    t5 = src[1 * step] * c7;
    t2 = src[2 * step] * c4;
    t7 = src[3 * step] * c9;
    t1 = src[4 * step] * c3;
    t4 = src[5 * step] * c6;
    t3 = src[6 * step] * c5;
    t6 = src[7 * step] * c8;

    t4m7 = t4 - t7;
    t5p6 = t5 + t6;
    t5m6 = t5 - t6;
    t4p7 = t4 + t7;
    t2m3 = t2 - t3;
    t2p3 = t2 + t3;
    x0 = t5p6 - t4p7;
    x1 = t5p6 + t4p7;
    x2 = t4m7 + t5m6;
    x3 = t2m3 * N_SQRT2;
    x4 = x2 * c2 - t4m7 * c1;
    x5 = x0 * N_SQRT2;
    x6 = t5m6 * c0 - x2 * c2;
    x7 = t0 + t1;
    x8 = t0 - t1;
    x9 = x3 - t2p3;
    x10 = x4 - x1;
    x11 = x5 - x10;
    x12 = x7 + t2p3;
    x13 = x8 + x9;
    x14 = x8 - x9;
    x15 = x7 - t2p3;
    x16 = -x6 - x11;
    d0 = x12 + x1;
    d1 = x13 + x10;
    d2 = x14 + x11;
    d3 = x15 + x16;
    d4 = x15 - x16;
    d5 = x14 - x11;
    d6 = x13 - x10;
    d7 = x12 - x1;

    src[0 * step] = d0;
    src[1 * step] = d1;
    src[2 * step] = d2;
    src[3 * step] = d3;
    src[4 * step] = d4;
    src[5 * step] = d5;
    src[6 * step] = d6;
    src[7 * step] = d7;
}

static void idct4_put(uint8_t *ddst, ptrdiff_t stride, int16_t *block)
{
    uint16_t *dst = (uint16_t *)ddst;

    for (int i = 0; i < 4; i++)
        idct8(block + i * 8, 1);
    for (int i = 0; i < 8; i++)
        idct4(block + i, 8);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 8; j++)
            dst[j] = av_clip_uintp2(block[j] + 2048, 12);
        block += 8;
        dst += stride;
    }
}

static void idct8_put(uint8_t *ddst, ptrdiff_t stride, int16_t *block)
{
    uint16_t *dst = (uint16_t *)ddst;

    for (int i = 0; i < 8; i++)
        idct8(block + i, 8);
    for (int i = 0; i < 8; i++) {
        idct8(block, 1);
        for (int j = 0; j < 8; j++)
            dst[j] = av_clip_uintp2(block[j] + 2048, 12);
        block += 8;
        dst += stride;
    }
}

static int decode_tile(AVCodecContext *avctx, AVFrame *frame, int tile_x, int tile_y,
                       int blocks_w_in_tile, int blocks_h_in_tile)
{
    BRAWContext *s = avctx->priv_data;
    GetBitContext *gbit = &s->gbit;
    int prev_dc[3];
    int ret = 0;

    prev_dc[0] = prev_dc[1] = prev_dc[2] = 0;

    for (int y = 0; y < blocks_h_in_tile; y++) {
        int pos_y = y * 8 + tile_y * s->tile_size_h;

        for (int x = 0; x < blocks_w_in_tile; x++) {
            int pos_x = s->tile_offset_w[tile_x] + x * 16;

            ret = decode_block(avctx, s->block[0], s->quant[0], &prev_dc[0], 64, ff_zigzag_direct);
            if (ret < 0)
                return ret;
            ret = decode_block(avctx, s->block[1], s->quant[0], &prev_dc[0], 64, ff_zigzag_direct);
            if (ret < 0)
                return ret;
            ret = decode_block(avctx, s->block[2], s->quant[1], &prev_dc[1], 32, half_scan);
            if (ret < 0)
                return ret;
            ret = decode_block(avctx, s->block[3], s->quant[1], &prev_dc[2], 32, half_scan);
            if (ret < 0)
                return ret;

            idct8_put(s->out[0], 8, s->block[0]);
            idct8_put(s->out[1], 8, s->block[1]);

            idct4_put(s->out[2], 8, s->block[2]);
            idct4_put(s->out[3], 8, s->block[3]);

            {
                uint16_t *g = (uint16_t *)(frame->data[0] + pos_y * frame->linesize[0] + pos_x * 2);
                uint16_t *b = (uint16_t *)(frame->data[1] + pos_y * frame->linesize[1] + pos_x * 2);
                uint16_t *r = (uint16_t *)(frame->data[2] + pos_y * frame->linesize[2] + pos_x * 2);
                const uint16_t *Y0 = (const uint16_t *)s->out[0];
                const uint16_t *Y1 = (const uint16_t *)s->out[1];
                const uint16_t *B  = (const uint16_t *)s->out[2];
                const uint16_t *R  = (const uint16_t *)s->out[3];
                int bv, rv, gv;

                for (int yy = 0; yy < 8; yy++) {
                    for (int xx = 0; xx < 8; xx++) {
                        bv = B[xx/2]-2048;
                        rv = R[xx/2]-2048;
                        gv = Y0[xx];
                        bv = Y0[xx] + bv;
                        rv = Y0[xx] + rv;

                        g[xx+0] = av_clip_uintp2(gv, 12);
                        b[xx+0] = av_clip_uintp2(bv, 12);
                        r[xx+0] = av_clip_uintp2(rv, 12);

                        bv = B[4+(xx/2)]-2048;
                        rv = R[4+(xx/2)]-2048;
                        gv = Y1[xx];
                        bv = Y1[xx] + bv;
                        rv = Y1[xx] + rv;

                        g[xx+8] = av_clip_uintp2(gv, 12);
                        b[xx+8] = av_clip_uintp2(bv, 12);
                        r[xx+8] = av_clip_uintp2(rv, 12);
                    }

                    Y0 += 8;
                    Y1 += 8;
                    if ((yy & 1)) {
                        B += 8;
                        R += 8;
                    }

                    g += frame->linesize[0]/2;
                    b += frame->linesize[1]/2;
                    r += frame->linesize[2]/2;
                }
            }

            skip_bits(gbit, 8);
            if (get_bits_left(gbit) < 0)
                return AVERROR_INVALIDDATA;
        }
    }

    av_log(avctx, AV_LOG_DEBUG, "bits left: %d\n", get_bits_left(gbit));

    return ret;
}

static int get_offset(AVCodecContext *avctx, int x)
{
    BRAWContext *s = avctx->priv_data;
    GetByteContext *gb = &s->gb;
    int a, b, nb, idx = 4;

    a = bytestream2_get_byte(gb);
    b = bytestream2_get_byte(gb);

    nb = a * 2 + b;
    if (nb != 0) {
        int i = 0, j = 0;

        while (i = j, (nb >> j & 1) == 0) {
            j++;
        }

        idx = 4 - i;
    }

    s->offsets[x][0] = a << (idx & 0x1f);
    s->offsets[x][1] = b << (idx & 0x1f);

    av_log(avctx, AV_LOG_DEBUG, "offset:%d %dx%d\n", x, s->offsets[x][0], s->offsets[x][1]);

    return 0;
}

static int get_offsets(AVCodecContext *avctx)
{
    for (int i = 0; i < 4; i++)
        get_offset(avctx, i);

    return 0;
}

static int decode_quants(AVCodecContext *avctx)
{
    BRAWContext *s = avctx->priv_data;
    GetByteContext *gb = &s->gb;
    int scale;

    for (int n = 0; n < 64; n++)
        s->quant[0][n] = bytestream2_get_be16(gb);

    for (int n = 0; n < 32; n++)
        s->quant[1][n] = bytestream2_get_be16(gb);

    scale = (s->qscale + 4) * 2048;
    for (int n = 0; n < 64; n++)
        s->quant[0][n] *= scale;

    scale = (s->qscale + 4) * 1024;
    for (int n = 0; n < 32; n++)
        s->quant[1][n] *= scale;

    if (s->qscale2 && s->qscale2 * 2048 <= s->quant[0][0])
        s->quant[0][0] = s->qscale2 * 2048;

    if (s->qscale3 && s->qscale3 * 2048 <= s->quant[1][0])
        s->quant[1][0] = s->qscale3 * 2048;

    return 0;
}

static int decode_tiles(AVCodecContext *avctx, AVFrame *frame)
{
    BRAWContext *s = avctx->priv_data;
    GetBitContext *gbit = &s->gbit;
    GetByteContext *gb = &s->gb;
    uint32_t braw_size;
    int version, ret, w, h;

    if (bytestream2_get_le32(gb) != MKTAG('b','r','a','w'))
        return AVERROR_INVALIDDATA;
    braw_size = bytestream2_get_be32(gb);
    if (braw_size < 4352)
        return AVERROR_INVALIDDATA;
    if (braw_size - 8 > bytestream2_get_bytes_left(gb))
        return AVERROR_INVALIDDATA;

    version = bytestream2_get_byte(gb);
    av_log(avctx, AV_LOG_DEBUG, "version: %d\n", version);
    if (version != 1 && version != 2)
        return AVERROR_INVALIDDATA;
    s->qscale = bytestream2_get_byte(gb);
    av_log(avctx, AV_LOG_DEBUG, "qscale: %d\n", s->qscale);

    s->nb_tiles_w = bytestream2_get_byte(gb);
    s->nb_tiles_h = bytestream2_get_byte(gb);
    av_log(avctx, AV_LOG_DEBUG, "nb_tiles: %dx%d\n", s->nb_tiles_w, s->nb_tiles_h);
    if (!s->nb_tiles_w ||
        !s->nb_tiles_h ||
        s->nb_tiles_w * s->nb_tiles_h > 1024)
        return AVERROR_INVALIDDATA;

    w = bytestream2_get_be16(gb);
    h = bytestream2_get_be16(gb);
    av_log(avctx, AV_LOG_DEBUG, "WxH: %dx%d\n", w, h);

    s->tile_size_h = bytestream2_get_be16(gb);
    av_log(avctx, AV_LOG_DEBUG, "tile_size_h = %d\n", s->tile_size_h);
    if (s->tile_size_h & 7)
        return AVERROR_INVALIDDATA;

    bytestream2_skip(gb, 2);

    ret = get_offsets(avctx);
    if (ret < 0)
        return ret;
    bytestream2_skip(gb, 4);
    s->qscale2 = bytestream2_get_be16(gb);
    s->qscale3 = bytestream2_get_be16(gb);
    bytestream2_skip(gb, 28);

    s->tile_offset_w[0] = 0;

    for (int x = 0; x < s->nb_tiles_w; x++) {
        s->tile_size_w[x] = bytestream2_get_be16(gb);
        s->tile_offset_w[x+1] = s->tile_offset_w[x] + s->tile_size_w[x];
        av_log(avctx, AV_LOG_DEBUG, "tile_size_w[%d] = %d\n", x, s->tile_size_w[x]);
        av_log(avctx, AV_LOG_DEBUG, "tile_offset_w[%d] = %d\n", x+1, s->tile_offset_w[x+1]);
        if (s->tile_offset_w[x+1] > w)
            return AVERROR_INVALIDDATA;
    }

    ret = ff_set_dimensions(avctx, s->tile_offset_w[s->nb_tiles_w], s->tile_size_h * s->nb_tiles_h);
    if (ret < 0)
        return ret;
    avctx->width  = w;
    avctx->height = h;

    ret = ff_thread_get_buffer(avctx, frame, 0);
    if (ret < 0)
        return ret;

    bytestream2_seek(gb, 0x1180, SEEK_SET);

    ret = decode_quants(avctx);
    if (ret < 0)
        return ret;

    bytestream2_seek(gb, 0x180, SEEK_SET);

    for (int x = 0; x < s->nb_tiles_w; x++) {
        s->blocks_w_in_tile = s->tile_size_w[x] / 16;

        for (int y = 0; y < s->nb_tiles_h; y++) {
            uint32_t tile_offset = bytestream2_get_be32(gb);
            int last_tile = (x == (s->nb_tiles_w - 1)) && (y == (s->nb_tiles_h - 1));
            int tile_size = last_tile ? bytestream2_size(gb) - tile_offset - s->header_size : bytestream2_peek_be32(gb) - tile_offset;

            if (tile_offset > bytestream2_size(gb) || tile_size <= 0)
                return AVERROR_INVALIDDATA;

            av_log(avctx, AV_LOG_DEBUG, "%dx%d: tile_bitstream_size: 0x%X, tile_bitstream_offset: 0x%X\n", x, y, tile_size, tile_offset + s->header_size);
            ret = init_get_bits8(gbit, gb->buffer_start + tile_offset + s->header_size, tile_size);
            if (ret < 0)
                return ret;

            if (y == s->nb_tiles_h - 1)
                s->blocks_h_in_tile = (h - y * s->tile_size_h) / 8;
            else
                s->blocks_h_in_tile = s->tile_size_h / 8;

            ret = decode_tile(avctx, frame, x, y, s->blocks_w_in_tile, s->blocks_h_in_tile);
            if (ret < 0)
                return ret;
        }
    }

    return 0;
}

static int braw_decode_frame(AVCodecContext *avctx,
                             AVFrame *frame, int *got_frame,
                             AVPacket *avpkt)
{
    BRAWContext *s = avctx->priv_data;
    int ret;

    if (avpkt->size <= 4608)
        return AVERROR_INVALIDDATA;

    bytestream2_init(&s->gb, avpkt->data, avpkt->size);
    ret = parse_frame_metadata(avctx);
    if (ret < 0)
        return ret;

    ret = decode_tiles(avctx, frame);
    if (ret < 0)
        return ret;

    frame->pict_type = AV_PICTURE_TYPE_I;
    frame->flags |= AV_FRAME_FLAG_KEY;

    *got_frame = 1;

    return avpkt->size;
}

static av_cold void braw_static_init(void)
{
    VLC_INIT_STATIC_TABLE(dc_vlc, 9, 16, dc_bits, 1, 1, dc_codes, 2, 2, 0);
    VLC_INIT_STATIC_TABLE(ac_vlc, 11, 194, ac_bits, 1, 1, ac_codes, 4, 4, 0);
}

static av_cold int braw_decode_init(AVCodecContext *avctx)
{
    static AVOnce init_once = AV_ONCE_INIT;
    int ret;

    avctx->pix_fmt = AV_PIX_FMT_GBRP12;
    avctx->bits_per_raw_sample = 12;

    ret = ff_thread_once(&init_once, braw_static_init);
    if (ret)
        return AVERROR_UNKNOWN;

    return 0;
}

static const AVOption options[] = {
    { NULL },
};

static const AVClass braw_decoder_class = {
    .class_name = "braw decoder",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_braw_decoder = {
    .p.name           = "braw",
    CODEC_LONG_NAME("Blackmagic RAW"),
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_BRAW,
    .priv_data_size   = sizeof(BRAWContext),
    .init             = braw_decode_init,
    FF_CODEC_DECODE_CB(braw_decode_frame),
    .p.capabilities   = AV_CODEC_CAP_DR1 |
                        AV_CODEC_CAP_FRAME_THREADS,
    .caps_internal    = FF_CODEC_CAP_INIT_CLEANUP,
    .p.priv_class     = &braw_decoder_class,
};
