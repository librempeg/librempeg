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

#include <stddef.h>
#include <stdint.h>

#define BITSTREAM_WRITER_LE
#include "put_bits.h"

#include "deflate.h"

static const uint16_t run_len[256] = {
     3,  4,  5,  6,  7,  8,  9, 10, 11, 11, 13, 13, 15, 15, 17, 17,
    19, 19, 19, 19, 23, 23, 23, 23, 27, 27, 27, 27, 31, 31, 31, 31,
    35, 35, 35, 35, 35, 35, 35, 35, 43, 43, 43, 43, 43, 43, 43, 43,
    51, 51, 51, 51, 51, 51, 51, 51, 59, 59, 59, 59, 59, 59, 59, 59,
    67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67, 67,
    83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 83,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115, 115,
    131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131,
    163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163,
    195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195,
    227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 258,
};

static const uint16_t run_sym[256] = {
    257, 258, 259, 260, 261, 262, 263, 264, 265, 265, 266, 266, 267, 267, 268, 268, 269, 269, 269, 269, 270, 270, 270, 270, 271, 271, 271, 271, 272, 272, 272, 272,
    273, 273, 273, 273, 273, 273, 273, 273, 274, 274, 274, 274, 274, 274, 274, 274, 275, 275, 275, 275, 275, 275, 275, 275, 276, 276, 276, 276, 276, 276, 276, 276,
    277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 277, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278, 278,
    279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280, 280,
    281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281, 281,
    282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282, 282,
    283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283, 283,
    284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 284, 285
};

static const uint8_t run_extra[256] = {
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 0
};

static const uint8_t dst_sym[512] = {
     0,  1,  2,  3,  4,  4,  5,  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  8,  8,  8,  8,  8,  9,  9,  9,  9,  9,  9,  9,  9,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
    15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
    15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
    17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
    17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
    17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17
};

static const uint8_t dst_extra[512] = {
    0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7
};

static const uint8_t large_dst_sym[128] = {
    18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 22, 22, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 25,
    26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27,
    28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28,
    29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29
};

static const uint8_t large_dst_extra[128] = {
     8,  8,  8,  8,  9,  9,  9,  9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
    13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
    13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13
};

static uint32_t get_adler32(uint32_t adler, const uint8_t *src,
                            int height, int width, ptrdiff_t linesize)
{
    const uint32_t divisor = 65521;
    const uint32_t max_chunk_len = 5552;
    uint32_t s1 = adler & 0xFFFF;
    uint32_t s2 = adler >> 16;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width;) {
            size_t n = FFMIN(width, max_chunk_len & ~3);
            const uint8_t *p = src + y * linesize;

            x += n;
            do {
                if (n >= 4) {
                    uint32_t byte_0_sum = 0;
                    uint32_t byte_1_sum = 0;
                    uint32_t byte_2_sum = 0;
                    uint32_t byte_3_sum = 0;
                    uint32_t s1_sum = 0;

                    do {
                        s1_sum += s1;
                        s1 += p[0] + p[1] + p[2] + p[3];
                        byte_0_sum += p[0];
                        byte_1_sum += p[1];
                        byte_2_sum += p[2];
                        byte_3_sum += p[3];
                        p += 4;
                        n -= 4;
                    } while (n >= 4);

                    s2 += (4 * (s1_sum + byte_0_sum)) + (3 * byte_1_sum) +
                        (2 * byte_2_sum) + byte_3_sum;
                }

                for (; n; n--, p++) {
                    s1 += *p;
                    s2 += s1;
                }

                s1 %= divisor;
                s2 %= divisor;
            } while (0);
        }
    }

    return (s2 << 16) | s1;
}

static void aligned_copy_bits(PutBitContext *pb, const uint8_t *src, int length)
{
    memcpy(put_bits_ptr(pb), src, length);
    skip_put_bytes(pb, length);
}

static void deflate_raw_block(DeflateContext *s, int len)
{
    const ptrdiff_t linesize = s->linesize;
    PutBitContext *pb = &s->pb;
    const uint8_t *src = s->src;
    const int width = s->width;
    int y = s->y, x = s->x;

    put_bits(pb, 2, 0);
    align_put_bits(pb);
    put_bits(pb, 16, len);
    put_bits(pb, 16, ~len);
    align_put_bits(pb);
    flush_put_bits(pb);

    do {
        const int ilen = FFMIN(len, width - x);

        aligned_copy_bits(pb, src + y * linesize + x, ilen);

        x += ilen;
        len -= ilen;
        if (x >= width) {
            y++;
            x = 0;
        }
    } while (len > 0);

    s->y = y;
    s->x = x;
}

typedef struct DeflateSymFreq {
    uint16_t key, index;
} DeflateSymFreq;

static DeflateSymFreq *radix_sort(unsigned num_syms,
                                  DeflateSymFreq *pSyms0, DeflateSymFreq *pSyms1)
{
    uint32_t total_passes = 2, pass_shift, pass, i, hist[256 * 2] = { 0 };
    DeflateSymFreq *pCur_syms = pSyms0, *pNew_syms = pSyms1;

    for (int i = 0; i < num_syms; i++) {
        unsigned freq = pSyms0[i].key;
        hist[freq & 0xFF]++;
        hist[256 + ((freq >> 8) & 0xFF)]++;
    }

    while ((total_passes > 1) && (num_syms == hist[(total_passes - 1) * 256]))
        total_passes--;

    for (pass_shift = 0, pass = 0; pass < total_passes; pass++, pass_shift += 8) {
        const uint32_t *pHist = &hist[pass << 8];
        unsigned offsets[256], cur_ofs = 0;

        for (i = 0; i < 256; i++) {
            offsets[i] = cur_ofs;
            cur_ofs += pHist[i];
        }
        for (i = 0; i < num_syms; i++)
            pNew_syms[offsets[(pCur_syms[i].key >> pass_shift) & 0xFF]++] = pCur_syms[i];

        FFSWAP(DeflateSymFreq *, pCur_syms, pNew_syms);
    }

    return pCur_syms;
}

static void minimum_redundancy(DeflateSymFreq *A, int n)
{
    int root, leaf, next, avbl, used, dpth;

    if (n == 0) {
        return;
    } else if (n == 1) {
        A[0].key = 1;
        return;
    }

    A[0].key += A[1].key;
    root = 0;
    leaf = 2;
    for (next = 1; next < n - 1; next++) {
        if (leaf >= n || A[root].key < A[leaf].key) {
            A[next].key = A[root].key;
            A[root++].key = (uint16_t)next;
        } else {
            A[next].key = A[leaf++].key;
        }
        if (leaf >= n || (root < next && A[root].key < A[leaf].key)) {
            A[next].key = (uint16_t)(A[next].key + A[root].key);
            A[root++].key = (uint16_t)next;
        } else {
            A[next].key = (uint16_t)(A[next].key + A[leaf++].key);
        }
    }
    A[n - 2].key = 0;
    for (next = n - 3; next >= 0; next--)
        A[next].key = A[A[next].key].key + 1;
    avbl = 1;
    used = dpth = 0;
    root = n - 2;
    next = n - 1;
    while (avbl > 0) {
        while (root >= 0 && (int)A[root].key == dpth) {
            used++;
            root--;
        }
        while (avbl > used) {
            A[next--].key = (uint16_t)(dpth);
            avbl--;
        }
        avbl = 2 * used;
        dpth++;
        used = 0;
    }
}

static void enforce_max_code_size(int *pNum_codes, int code_list_len, int max_code_size)
{
    uint32_t total = 0;

    if (code_list_len <= 1)
        return;

    for (int i = max_code_size + 1; i <= 32; i++)
        pNum_codes[max_code_size] += pNum_codes[i];

    for (int i = max_code_size; i > 0; i--)
        total += (((uint32_t)pNum_codes[i]) << (max_code_size - i));

    while (total != (1UL << max_code_size)) {
        pNum_codes[max_code_size]--;
        for (int i = max_code_size - 1; i > 0; i--) {
            if (pNum_codes[i]) {
                pNum_codes[i]--;
                pNum_codes[i + 1] += 2;
                break;
            }
        }
        total--;
    }
}

static void optimize_huffman(DeflateContext *s, uint16_t *count_table, uint8_t *code_sizes, uint16_t *codes,
                             int table_len, int code_size_limit, int static_table)
{
    int num_codes[1 + 32] = { 0 };
    unsigned next_code[32 + 1];

    if (static_table) {
        for (int i = 0; i < table_len; i++)
            num_codes[code_sizes[i]]++;
    } else {
        DeflateSymFreq syms0[288], syms1[288], *pSyms;
        int num_used_syms = 0;

        for (int i = 0; i < table_len; i++) {
            if (count_table[i]) {
                syms0[num_used_syms].key = count_table[i];
                syms0[num_used_syms++].index = i;
            }
        }

        pSyms = radix_sort(num_used_syms, syms0, syms1);
        minimum_redundancy(pSyms, num_used_syms);

        for (int i = 0; i < num_used_syms; i++)
            num_codes[pSyms[i].key]++;

        enforce_max_code_size(num_codes, num_used_syms, code_size_limit);

        memset(code_sizes, 0, table_len * sizeof(*code_sizes));
        memset(codes, 0, table_len * sizeof(*codes));

        for (int i = 1, j = num_used_syms; i <= code_size_limit; i++)
            for (int l = num_codes[i]; l > 0; l--)
                code_sizes[pSyms[--j].index] = i;
    }

    next_code[1] = 0;
    for (int j = 0, i = 2; i <= code_size_limit; i++)
        next_code[i] = j = ((j + num_codes[i-1]) << 1);

    for (int i = 0; i < table_len; i++) {
        unsigned rev_code = 0, code, code_size;

        if ((code_size = code_sizes[i]) == 0)
            continue;

        code = next_code[code_size]++;
        for (int l = code_size; l > 0; l--, code >>= 1)
            rev_code = (rev_code << 1) | (code & 1);

        codes[i] = rev_code;
    }
}

#define RLE_PREV_CODE_SIZE() \
{ \
    if (rle_repeat_count > 0) { \
        if (rle_repeat_count < 3) { \
            s->huf_count[prev_code_size] += rle_repeat_count; \
            while (rle_repeat_count--) \
                packed_code_sizes[num_packed_code_sizes++] = prev_code_size; \
        } else { \
            s->huf_count[16]++; \
            packed_code_sizes[num_packed_code_sizes++] = 16; \
            packed_code_sizes[num_packed_code_sizes++] = rle_repeat_count - 3; \
        } \
        rle_repeat_count = 0; \
    } \
}

#define RLE_ZERO_CODE_SIZE() \
{ \
    if (rle_z_count > 0) { \
        if (rle_z_count < 3) { \
            s->huf_count[0] += rle_z_count; \
            while (rle_z_count--) \
                packed_code_sizes[num_packed_code_sizes++] = 0; \
        } else if (rle_z_count <= 10) { \
            s->huf_count[17]++; \
            packed_code_sizes[num_packed_code_sizes++] = 17; \
            packed_code_sizes[num_packed_code_sizes++] = rle_z_count - 3; \
        } else { \
            s->huf_count[18]++; \
            packed_code_sizes[num_packed_code_sizes++] = 18; \
            packed_code_sizes[num_packed_code_sizes++] = rle_z_count - 11; \
        } \
        rle_z_count = 0; \
    } \
}

static void deflate_block(PutBitContext *pb, int len,
                          const uint16_t *lit, const uint16_t *dst,
                          const uint8_t *dsc, const uint8_t *run,
                          const uint8_t *code_sizes, const uint16_t *codes,
                          const uint8_t *dst_code_sizes, const uint16_t *dst_codes)
{
    for (int i = 0; i < len; i++) {
        uint16_t l = lit[i];
        uint16_t d = dst[i];
        uint8_t  c = dsc[i];
        uint8_t  r = run[i];

        put_bits(pb, code_sizes[l], codes[l]);
        if (d > 0) {
            const int len_extra_size = run_extra[r];
            int extra_distance_size = 0;

            if (len_extra_size > 0) {
                const int run_extra_code = r+3 - run_len[r];

                put_bits(pb, len_extra_size, run_extra_code);
            }

            d--;
            if (d <= 512) {
                extra_distance_size = dst_extra[d];
            } else {
                extra_distance_size = large_dst_extra[d>>8];
            }

            put_bits(pb, dst_code_sizes[c], dst_codes[c]);

            if (extra_distance_size > 0)
                put_bits(pb, extra_distance_size, d & ((1<<extra_distance_size)-1));
        }
    }

    put_bits(pb, code_sizes[256], codes[256]);
}

static void deflate_dynamic_block(DeflateContext *s, int len)
{
    PutBitContext *pb = &s->pb;
    static const uint8_t swizzle[] = { 16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15 };
    int num_lit_codes, num_dist_codes, num_bit_lengths;
    unsigned i, total_code_sizes_to_pack, num_packed_code_sizes, packed_code_sizes_index;
    uint8_t code_sizes_to_pack[288 + 32], packed_code_sizes[288 + 32], prev_code_size = 0xFF;
    uint8_t *dst_code_sizes = s->dst_code_sizes[1];
    uint8_t *code_sizes = s->lit_code_sizes[1];
    uint16_t *dst_codes = s->dst_codes[1];
    uint16_t *codes = s->lit_codes[1];
    const uint16_t *lit = s->lit;
    const uint16_t *dst = s->dst;
    const uint8_t *dsc = s->dsc;
    const uint8_t *run = s->run;
    int rle_repeat_count = 0;
    int rle_z_count = 0;

    optimize_huffman(s, s->lit_count, s->lit_code_sizes[1], s->lit_codes[1], 288, 15, 0);
    optimize_huffman(s, s->dst_count, s->dst_code_sizes[1], s->dst_codes[1], 32, 15, 0);

    for (num_lit_codes = 286; num_lit_codes > 257; num_lit_codes--)
        if (s->lit_code_sizes[1][num_lit_codes - 1])
            break;
    for (num_dist_codes = 30; num_dist_codes > 1; num_dist_codes--)
        if (s->dst_code_sizes[1][num_dist_codes - 1])
            break;

    memcpy(code_sizes_to_pack, s->lit_code_sizes[1], num_lit_codes);
    memcpy(code_sizes_to_pack + num_lit_codes, s->dst_code_sizes[1], num_dist_codes);
    total_code_sizes_to_pack = num_lit_codes + num_dist_codes;
    num_packed_code_sizes = 0;
    rle_z_count = 0;

    memset(s->huf_count, 0, sizeof(s->huf_count));
    for (i = 0; i < total_code_sizes_to_pack; i++) {
        uint8_t code_size = code_sizes_to_pack[i];
        if (!code_size) {
            RLE_PREV_CODE_SIZE();
            if (++rle_z_count == 138)
                RLE_ZERO_CODE_SIZE();
        } else {
            RLE_ZERO_CODE_SIZE();
            if (code_size != prev_code_size) {
                RLE_PREV_CODE_SIZE();
                s->huf_count[code_size]++;
                packed_code_sizes[num_packed_code_sizes++] = code_size;
            } else if (++rle_repeat_count == 6) {
                RLE_PREV_CODE_SIZE();
            }
        }
        prev_code_size = code_size;
    }

    if (rle_repeat_count) {
        RLE_PREV_CODE_SIZE();
    } else {
        RLE_ZERO_CODE_SIZE();
    }

    optimize_huffman(s, s->huf_count, s->huf_code_sizes[1], s->huf_codes[1], 19, 7, 0);

    put_bits(pb, 2, 2);

    put_bits(pb, 5, num_lit_codes - 257);
    put_bits(pb, 5, num_dist_codes - 1);

    for (num_bit_lengths = 18; num_bit_lengths >= 0; num_bit_lengths--)
        if (s->huf_code_sizes[1][swizzle[num_bit_lengths]])
            break;
    num_bit_lengths = FFMAX(4, (num_bit_lengths + 1));
    put_bits(pb, 4, num_bit_lengths - 4);
    for (int i = 0; i < num_bit_lengths; i++)
        put_bits(pb, 3, s->huf_code_sizes[1][swizzle[i]]);

    for (packed_code_sizes_index = 0; packed_code_sizes_index < num_packed_code_sizes;) {
        unsigned code = packed_code_sizes[packed_code_sizes_index++];

        put_bits(pb, s->huf_code_sizes[1][code], s->huf_codes[1][code]);
        if (code >= 16)
            put_bits(pb, "\02\03\07"[code - 16],
                     packed_code_sizes[packed_code_sizes_index++]);
    }

    deflate_block(pb, len, lit, dst, dsc, run, code_sizes, codes, dst_code_sizes, dst_codes);
}

static void deflate_static_block(DeflateContext *s, int len)
{
    uint8_t *dst_code_sizes = s->dst_code_sizes[0];
    uint8_t *code_sizes = s->lit_code_sizes[0];
    uint16_t *dst_codes = s->dst_codes[0];
    uint16_t *codes = s->lit_codes[0];
    uint8_t *p = s->lit_code_sizes[0];
    const uint16_t *lit = s->lit;
    const uint16_t *dst = s->dst;
    const uint8_t *dsc = s->dsc;
    const uint8_t *run = s->run;
    PutBitContext *pb = &s->pb;

    if (!s->fixed_cb_initialized) {
        for (int i = 0; i < 144; i++)
            p[i] = 8;
        for (int i = 144; i < 256; i++)
            p[i] = 9;
        for (int i = 256; i < 280; i++)
            p[i] = 7;
        for (int i = 280; i < 288; i++)
            p[i] = 8;

        memset(s->dst_code_sizes[0], 5, 32);

        optimize_huffman(s, s->lit_count, s->lit_code_sizes[0], s->lit_codes[0], 288, 15, 1);
        optimize_huffman(s, s->dst_count, s->dst_code_sizes[0], s->dst_codes[0], 32, 15, 1);
        s->fixed_cb_initialized = 1;
    }

    put_bits(pb, 2, 1);

    deflate_block(pb, len, lit, dst, dsc, run, code_sizes, codes, dst_code_sizes, dst_codes);
}

static uint8_t get_distance_code(int dist)
{
    dist--;

    if (dist <= 512) {
        return dst_sym[dist];
    } else {
        return large_dst_sym[dist>>8];
    }
}

static int mmcmp(const uint8_t *a, const uint8_t *b, int len)
{
    int pos = 0;

    while (len >= 8) {
        if (AV_RN64(a+pos) != AV_RN64(b+pos))
            break;

        pos += 8;
        len -= 8;
    }

    while (len > 0) {
        if (a[pos] != b[pos])
            break;

        pos += 1;
        len -= 1;
    }

    return pos;
}

static int get_runlen(const uint8_t *src, int len, int back, int *dist)
{
    int max_run = 0;

    *dist = 0;

    for (int k = 1; k <= back; k++) {
        int r = 0;

        switch (k) {
        case 1:
            for (; src[-1] == src[r] && r < len; r++);
            break;
        case 2:
            for (; !memcmp(src-2, src+r, 2) && r+1 < len; r += 2);
            break;
        case 3:
            for (; !memcmp(src-3, src+r, 3) && r+2 < len; r += 3);
            break;
        case 4:
            for (; !memcmp(src-4, src+r, 4) && r+3 < len; r += 4);
            break;
        case 5:
            for (; !memcmp(src-5, src+r, 5) && r+4 < len; r += 5);
            break;
        case 6:
            for (; !memcmp(src-6, src+r, 6) && r+5 < len; r += 6);
            break;
        case 7:
            for (; !memcmp(src-7, src+r, 7) && r+6 < len; r += 7);
            break;
        case 8:
            for (; !memcmp(src-8, src+r, 8) && r+7 < len; r += 8);
            break;
        default:
            for (; !memcmp(src-k, src+r, k) && r+k-1 < len; r += k);
            break;
        }

        if (r > max_run) {
            max_run = r;
            *dist = k;
        }
    }

    if (max_run < 3) {
        max_run = 0;
        *dist = 0;
    }

    return max_run;
}

static void deflate_import(DeflateContext *s, int len)
{
    const ptrdiff_t linesize = s->linesize;
    int y = s->y, x = s->x, j = 0;
    const uint8_t *src = s->src;
    const int width = s->width;
    uint8_t *dst = s->val;

    do {
        const int ilen = FFMIN(len, width - x);
        const uint8_t *isrc = src + y * linesize + x;

        memcpy(dst + j, isrc, ilen);

        j += ilen;
        x += ilen;
        len -= ilen;
        if (x >= width) {
            y++;
            x = 0;
        }
    } while (len > 0);

    s->y = y;
    s->x = x;
}

static void deflate_read(DeflateContext *s, int len)
{
    const int width = s->width;
    uint16_t *lit = s->lit;
    uint16_t *dst = s->dst;
    uint8_t *dsc = s->dsc;
    uint8_t *run = s->run;
    uint8_t *src = s->val;
    int j = 0, i = 0;

    memset(s->lit, 0, sizeof(s->lit));
    memset(s->dsc, 0, sizeof(s->dsc));
    memset(s->dst, 0, sizeof(s->dst));
    memset(s->run, 0, sizeof(s->run));

    do {
        const int max_len = FFMIN(len, 258);
        int max_run, dist, r;

        lit[j] = src[i];

        max_run = get_runlen(src+i, max_len, FFMIN(i, 8), &dist);
        if (i >= width) {
            const uint8_t *tsrc = src+i - width;

            r = mmcmp(tsrc, src+i, max_len);
            if (r >= max_run && r >= 3) {
                max_run = r;
                dist = width;
            }
        }

        r = max_run;
        run[j] = r ? r-3 : 0;
        dst[j] = dist;
        if (dist > 0) {
            lit[j] = run_sym[run[j]];
            dsc[j] = get_distance_code(dist);
        }

        i += r + !r;
        len -= r + !r;
        j++;
    } while (len > 0);

    s->nb_literals = j;
}

static void deflate_count(DeflateContext *s, int len)
{
    uint16_t *clit = s->lit_count;
    uint16_t *clen = s->dst_count;
    const uint16_t *lit = s->lit;
    const uint16_t *dst = s->dst;
    const uint8_t *dsc = s->dsc;
    int have_dst = 0;

    memset(s->lit_count, 0, sizeof(s->lit_count));
    memset(s->huf_count, 0, sizeof(s->huf_count));
    memset(s->dst_count, 0, sizeof(s->dst_count));

    for (int i = 0; i < len; i++) {
        clit[lit[i]]++;
        if (dst[i] > 0)
            clen[dsc[i]]++;
    }

    for (int i = 0; i < 32; i++) {
        if (clen[i] > 0) {
            have_dst = 1;
            break;
        }
    }

    clen[0] += !have_dst;
    clit[256] = 1;
}

int ff_deflate(DeflateContext *s,
               uint8_t *dst, int dst_len,
               const uint8_t *src, int height,
               int width, ptrdiff_t linesize)
{
    PutBitContext *pb = &s->pb;
    unsigned level_hint;
    uint16_t hdr;
    int y, x;

    s->x = 0;
    s->y = 0;
    s->src = src;
    s->width = width;
    s->height = height;
    s->nb_literals = 0;
    s->linesize = linesize;

    init_put_bits(pb, dst, dst_len);

    hdr = (8 << 8) | (7 << 12);
    level_hint = 3;
    hdr |= level_hint << 6;
    hdr |= 31 - (hdr % 31);

    put_bits(pb, 16, av_bswap16(hdr));

    y = s->y, x = s->x;
    do {
        const unsigned remain_len = (height - y) * width - x;
        int len = FFMIN(UINT16_MAX, remain_len);
        const int bfinal = len == remain_len;
        int btype = 2;

        deflate_import(s, len);
        deflate_read(s, len);
        deflate_count(s, s->nb_literals);

        put_bits(pb, 1, bfinal);

        switch (btype) {
        case 0:
            deflate_raw_block(s, len);
            break;
        case 1:
            deflate_static_block(s, s->nb_literals);
            break;
        case 2:
            deflate_dynamic_block(s, s->nb_literals);
            break;
        default:
            return AVERROR_BUG;
        }

        y = s->y, x = s->x;
    } while (y < height);

    align_put_bits(pb);
    put_bits32(pb, av_bswap32(get_adler32(1, src, height, width, linesize)));
    flush_put_bits(pb);

    return put_bytes_output(pb);
}

size_t ff_deflate_bound(const size_t in_nbytes)
{
    size_t max_blocks = FFMAX(((in_nbytes + UINT16_MAX-1) / UINT16_MAX), 1);

    return (5LL * max_blocks) + in_nbytes + 4+2;
}
