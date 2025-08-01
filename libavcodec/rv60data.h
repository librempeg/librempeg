/*
 * RV60 decoder
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

#ifndef AVCODEC_RV60DATA_H
#define AVCODEC_RV60DATA_H

#include <stdint.h>

static const uint8_t rv60_candidate_intra_angles[6] = {
    0, 1, 10, 26, 18, 2
};

static const uint8_t rv60_ipred_angle[9] = {
    0, 2, 5, 9, 13, 17, 21, 26, 32
};

static const uint16_t rv60_ipred_inv_angle[9] = {
    0, 4096, 1638, 910, 630, 482, 390, 315, 256
};

static const uint8_t rv60_avail_mask[64] = {
    0, 1, 0, 3, 0, 1, 0, 7, 0, 1, 0, 3, 0, 1, 0, 0xF,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t rv60_edge1[4] = {
    0, 2, 2, 2
};

static const uint8_t rv60_edge2[4] = {
    0, 3, 3, 3
};

static const uint8_t rv60_qp_to_idx[64] = {
    0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3,
    3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 0, 0
};

static const uint16_t rv60_quants_b[32] = {
     60,   67,   76,   85,   96,  108,  121,  136,
    152,  171,  192,  216,  242,  272,  305,  341,
    383,  432,  481,  544,  606,  683,  767,  854,
    963, 1074, 1212, 1392, 1566, 1708, 1978, 2211
};

static const uint8_t rv60_chroma_quant_dc[32] = {
     0,  0,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13,
    14, 15, 15, 16, 17, 18, 18, 19, 20, 20, 21, 21, 22, 22, 23, 23
};

static const uint8_t rv60_chroma_quant_ac[32] = {
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
    16, 17, 17, 18, 19, 20, 20, 21, 22, 22, 23, 23, 24, 24, 25, 25
};

static const uint8_t rv60_dsc_to_lx[][4] = {
    {0, 0, 0, 1}, {0, 0, 0, 2}, {0, 0, 1, 0},
    {0, 0, 1, 1}, {0, 0, 1, 2}, {0, 0, 2, 0}, {0, 0, 2, 1},
    {0, 0, 2, 2}, {0, 1, 0, 0}, {0, 1, 0, 1}, {0, 1, 0, 2},
    {0, 1, 1, 0}, {0, 1, 1, 1}, {0, 1, 1, 2}, {0, 1, 2, 0},
    {0, 1, 2, 1}, {0, 1, 2, 2}, {0, 2, 0, 0}, {0, 2, 0, 1},
    {0, 2, 0, 2}, {0, 2, 1, 0}, {0, 2, 1, 1}, {0, 2, 1, 2},
    {0, 2, 2, 0}, {0, 2, 2, 1}, {0, 2, 2, 2}, {1, 0, 0, 0},
    {1, 0, 0, 1}, {1, 0, 0, 2}, {1, 0, 1, 0}, {1, 0, 1, 1},
    {1, 0, 1, 2}, {1, 0, 2, 0}, {1, 0, 2, 1}, {1, 0, 2, 2},
    {1, 1, 0, 0}, {1, 1, 0, 1}, {1, 1, 0, 2}, {1, 1, 1, 0},
    {1, 1, 1, 1}, {1, 1, 1, 2}, {1, 1, 2, 0}, {1, 1, 2, 1},
    {1, 1, 2, 2}, {1, 2, 0, 0}, {1, 2, 0, 1}, {1, 2, 0, 2},
    {1, 2, 1, 0}, {1, 2, 1, 1}, {1, 2, 1, 2}, {1, 2, 2, 0},
    {1, 2, 2, 1}, {1, 2, 2, 2}, {2, 0, 0, 0}, {2, 0, 0, 1},
    {2, 0, 0, 2}, {2, 0, 1, 0}, {2, 0, 1, 1}, {2, 0, 1, 2},
    {2, 0, 2, 0}, {2, 0, 2, 1}, {2, 0, 2, 2}, {2, 1, 0, 0},
    {2, 1, 0, 1}, {2, 1, 0, 2}, {2, 1, 1, 0}, {2, 1, 1, 1},
    {2, 1, 1, 2}, {2, 1, 2, 0}, {2, 1, 2, 1}, {2, 1, 2, 2},
    {2, 2, 0, 0}, {2, 2, 0, 1}, {2, 2, 0, 2}, {2, 2, 1, 0},
    {2, 2, 1, 1}, {2, 2, 1, 2}, {2, 2, 2, 0}, {2, 2, 2, 1},
    {2, 2, 2, 2}, {3, 0, 0, 0}, {3, 0, 0, 1}, {3, 0, 0, 2},
    {3, 0, 1, 0}, {3, 0, 1, 1}, {3, 0, 1, 2}, {3, 0, 2, 0},
    {3, 0, 2, 1}, {3, 0, 2, 2}, {3, 1, 0, 0}, {3, 1, 0, 1},
    {3, 1, 0, 2}, {3, 1, 1, 0}, {3, 1, 1, 1}, {3, 1, 1, 2},
    {3, 1, 2, 0}, {3, 1, 2, 1}, {3, 1, 2, 2}, {3, 2, 0, 0},
    {3, 2, 0, 1}, {3, 2, 0, 2}, {3, 2, 1, 0}, {3, 2, 1, 1},
    {3, 2, 1, 2}, {3, 2, 2, 0}, {3, 2, 2, 1}, {3, 2, 2, 2},
};

static const uint8_t rv60_deblock_limits[32][4] = {
    {0, 0, 128,  0}, {0, 0, 128,  0}, {0, 0, 128,  0}, {0, 0, 128,  0},
    {0, 0, 128,  0}, {0, 0, 128,  0}, {0, 0, 128,  0}, {0, 0, 128,  0},
    {0, 0, 128,  3}, {0, 1, 128,  3}, {0, 1, 122,  3}, {1, 1,  96,  4},
    {1, 1,  75,  4}, {1, 1,  59,  4}, {1, 1,  47,  6}, {1, 1,  37,  6},
    {1, 1,  29,  6}, {1, 2,  23,  7}, {1, 2,  18,  8}, {1, 2,  15,  8},
    {1, 2,  13,  9}, {2, 3,  11,  9}, {2, 3,  10, 10}, {2, 3,   9, 10},
    {2, 4,   8, 11}, {3, 4,   7, 11}, {3, 5,   6, 12}, {3, 5,   5, 13},
    {3, 5,   4, 14}, {4, 7,   3, 15}, {5, 8,   2, 16}, {5, 9,   1, 17}
};

#endif /* AVCODEC_RV60DATA_H */
