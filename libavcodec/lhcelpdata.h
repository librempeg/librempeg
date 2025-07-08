/*
 * L&H CELP decoder
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

#ifndef AVCODEC_LHCELPDATA_H
#define AVCODEC_LHCELPDATA_H

#include <stdint.h>

static const int16_t lhcelp_init[10] = {
     1638,  4915,  8192, 11468, 14745, 18022, 21299, 24576,
    27852, 31129
};

static const int16_t lhcelp_sf1[10] = {
    153, 528, 1020, 1664, 2509, 3617, 5070, 6975, 9471, 12745
};

static const int16_t lhcelp_sf2[16] = {
    9, 58, 105, 162, 228, 307, 401, 512, 643, 798, 983, 1201, 1459, 1765, 2127, 2557
};

static const int16_t lhcelp_fast[34] = {
    32767, 32735, 32703, 32671, 32639, 32607, 32575, 32543,
    32511, 32479, 32447, 32415, 32383, 32351, 32319, 32287,
    32255,     8,   461,   652,   798,   922,  1031,  1129,
     1220,  1304,  1383,  1458,  1530,  1598,  1663,  1726,
     1787,  1846
};

static const int16_t lhcelp_slow[258] = {
    32767, 32512, 32256, 32000, 31744, 31488, 31232, 30976,
    30720, 30464, 30208, 29952, 29696, 29440, 29184, 28928,
    28672, 28416, 28160, 27904, 27648, 27392, 27136, 26880,
    26624, 26368, 26112, 25856, 25600, 25344, 25088, 24832,
    24576, 24320, 24064, 23808, 23552, 23296, 23040, 22784,
    22528, 22272, 22016, 21760, 21504, 21248, 20992, 20736,
    20480, 20224, 19968, 19712, 19456, 19200, 18944, 18688,
    18432, 18176, 17920, 17664, 17408, 17152, 16896, 16640,
    16384, 16128, 15872, 15616, 15360, 15104, 14848, 14592,
    14336, 14080, 13824, 13568, 13312, 13056, 12800, 12544,
    12288, 12032, 11776, 11520, 11264, 11008, 10752, 10496,
    10240,  9984,  9728,  9472,  9216,  8960,  8704,  8448,
     8192,  7936,  7680,  7424,  7168,  6912,  6656,  6400,
     6144,  5888,  5632,  5376,  5120,  4864,  4608,  4352,
     4096,  3840,  3584,  3328,  3072,  2816,  2560,  2304,
     2048,  1792,  1536,  1280,  1024,   768,   512,   256,
        0,     0,  1304,  1846,  2262,  2614,  2924,  3206,
     3465,  3707,  3934,  4150,  4355,  4552,  4741,  4923,
     5100,  5271,  5437,  5598,  5755,  5909,  6059,  6206,
     6350,  6491,  6630,  6766,  6899,  7031,  7160,  7288,
     7414,  7538,  7660,  7781,  7900,  8018,  8135,  8250,
     8364,  8477,  8589,  8699,  8809,  8917,  9025,  9131,
     9237,  9342,  9446,  9549,  9651,  9753,  9854,  9954,
    10053, 10152, 10251, 10348, 10445, 10542, 10638, 10733,
    10828, 10922, 11016, 11109, 11202, 11295, 11387, 11478,
    11570, 11660, 11751, 11841, 11931, 12020, 12109, 12198,
    12286, 12374, 12462, 12549, 12637, 12723, 12810, 12896,
    12983, 13068, 13154, 13240, 13325, 13410, 13495, 13579,
    13664, 13748, 13832, 13916, 14000, 14083, 14167, 14250,
    14333, 14416, 14499, 14582, 14665, 14747, 14830, 14912,
    14994, 15076, 15158, 15240, 15322, 15404, 15486, 15568,
    15650, 15731, 15813, 15894, 15976, 16058, 16139, 16221,
    16302, 16384
};

static const int16_t lhcelp_cb0[8] = {
      819,  1392,  1843,  2048,  2293,  2785,  3440,  4096
};

static const int16_t lhcelp_cb1[16] = {
     3440,  3768,  4096,  4423,  4792,  5242,  5775,  6348,
     6963,  7782,  8601,  9420, 10240, 11059, 11878, 12697
};

static const int16_t lhcelp_cb2[16] = {
     8192,  8601,  9256,  9912, 10526, 11059, 11714, 12369,
    13025, 13680, 14336, 15155, 15974, 16793, 17612, 18432
};

static const int16_t lhcelp_cb3[16] = {
    14745, 15400, 16056, 17203, 18841, 20316, 22118, 23756
};

static const int16_t lhcelp_cb4[16] = {
    22609, 23592, 24576, 25395, 26214, 27115, 28098, 29081
};

static const int16_t *lhcelp_cbs[5] = {
    lhcelp_cb0, lhcelp_cb1, lhcelp_cb2, lhcelp_cb3, lhcelp_cb4
};

static const int16_t lhcelp_scale_a0[9] = {
     1604,  3522,  5816,  8559, 11839, 15761, 20451, 26060, 32767
};

static const int16_t lhcelp_scale_a12[5] = {
     3110,  7402, 13324, 21494, 32767
};

static const int16_t lhcelp_scale_a3[3] = {
     5816, 15761, 32767
};

static const int16_t lhcelp_scale_a4[2] = {
    10125, 32767
};

static const int16_t lhcelp_scale_b0[7] = {
     1825,  4057,  6787, 10125, 14207, 19199, 25303
};

static const int16_t lhcelp_scale_b12[3] = {
     4057, 10125, 19199
};

static const int16_t lhcelp_scale_b3[1] = {
    10125
};

static const int16_t *lhcelp_scale_a[5] = { /* use lhcelp_scale_length[] for lengths */
    lhcelp_scale_a0, lhcelp_scale_a12, lhcelp_scale_a12, lhcelp_scale_a3, lhcelp_scale_a4
};

static const int16_t *lhcelp_scale_b[4] = { /* use lhcelp_scale_length[] - 2 for lengths */
    lhcelp_scale_b0, lhcelp_scale_b12, lhcelp_scale_b12, lhcelp_scale_b3
};

static const uint8_t lhcelp_scale_length[5] = {
    9, 5, 5, 3, 2
};

static const int32_t lhcelp_output_filter[9] = {
    0xdaa58cc0, 0x6f017a, 0x49d6e380, 0x6f017a, 0xdaa58cc0, 0xfffff026, 0xffffa4d3, 0x631, 0x278f
};

#endif /* AVCODEC_LHCELPDATA_H */
