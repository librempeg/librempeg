/*
 * Common MSMPEG-4 and VC-1 tables and VLC init code
 * Copyright (c) 2001 Fabrice Bellard
 * Copyright (c) 2002-2004 Michael Niedermayer <michaelni@gmx.at>
 *
 * msmpeg4v1 & v2 stuff by Michael Niedermayer <michaelni@gmx.at>
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

/**
 * @file
 * Common MSMPEG-4 and VC-1 tables and VLC init code
 */

#include "msmpeg4_vc1_data.h"
#include "vlc.h"
#include "libavutil/attributes.h"
#include "libavutil/thread.h"

VLCElem ff_msmp4_mb_i_vlc[536];
const VLCElem *ff_msmp4_dc_vlc[2][2];

static av_cold void msmp4_vc1_vlcs_init(void)
{
    static VLCElem vlc_buf[1158 + 1118 + 1476 + 1216];
    VLCInitState state = VLC_INIT_STATE(vlc_buf);

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            ff_msmp4_dc_vlc[i][j] =
                ff_vlc_init_tables(&state, MSMP4_DC_VLC_BITS, 120,
                                   &ff_msmp4_dc_tables[i][j][0][1], 8, 4,
                                   &ff_msmp4_dc_tables[i][j][0][0], 8, 4, 0);
        }
    }
    VLC_INIT_STATIC_TABLE(ff_msmp4_mb_i_vlc, MSMP4_MB_INTRA_VLC_BITS, 64,
                          &ff_msmp4_mb_i_table[0][1], 4, 2,
                          &ff_msmp4_mb_i_table[0][0], 4, 2, 0);
}

av_cold void ff_msmp4_vc1_vlcs_init_once(void)
{
    static AVOnce init_static_once = AV_ONCE_INIT;
    ff_thread_once(&init_static_once, msmp4_vc1_vlcs_init);
}

/* intra picture macroblock coded block pattern */
const uint16_t ff_msmp4_mb_i_table[64][2] = {
    {  0x1, 1 }, {  0x17,  6 }, {   0x9,  5 }, {   0x5,  5 },
    {  0x6, 5 }, {  0x47,  9 }, {  0x20,  7 }, {  0x10,  7 },
    {  0x2, 5 }, {  0x7c,  9 }, {  0x3a,  7 }, {  0x1d,  7 },
    {  0x2, 6 }, {  0xec,  9 }, {  0x77,  8 }, {   0x0,  8 },
    {  0x3, 5 }, {  0xb7,  9 }, {  0x2c,  7 }, {  0x13,  7 },
    {  0x1, 6 }, { 0x168, 10 }, {  0x46,  8 }, {  0x3f,  8 },
    { 0x1e, 6 }, { 0x712, 13 }, {  0xb5,  9 }, {  0x42,  8 },
    { 0x22, 7 }, { 0x1c5, 11 }, { 0x11e, 10 }, {  0x87,  9 },
    {  0x6, 4 }, {   0x3,  9 }, {  0x1e,  7 }, {  0x1c,  6 },
    { 0x12, 7 }, { 0x388, 12 }, {  0x44,  9 }, {  0x70,  9 },
    { 0x1f, 6 }, { 0x23e, 11 }, {  0x39,  8 }, {  0x8e,  9 },
    {  0x1, 7 }, { 0x1c6, 11 }, {  0xb6,  9 }, {  0x45,  9 },
    { 0x14, 6 }, { 0x23f, 11 }, {  0x7d,  9 }, {  0x18,  9 },
    {  0x7, 7 }, { 0x1c7, 11 }, {  0x86,  9 }, {  0x19,  9 },
    { 0x15, 6 }, { 0x1db, 10 }, {   0x2,  9 }, {  0x46,  9 },
    {  0xd, 8 }, { 0x713, 13 }, { 0x1da, 10 }, { 0x169, 10 },
};


const uint32_t ff_msmp4_dc_tables[2][2][120][2] = {
{
    /* dc table 0 */
    {
    {     0x1,  1 }, {     0x1,  2 }, {     0x1,  4 }, {     0x1,  5 },
    {     0x5,  5 }, {     0x7,  5 }, {     0x8,  6 }, {     0xc,  6 },
    {     0x0,  7 }, {     0x2,  7 }, {    0x12,  7 }, {    0x1a,  7 },
    {     0x3,  8 }, {     0x7,  8 }, {    0x27,  8 }, {    0x37,  8 },
    {     0x5,  9 }, {    0x4c,  9 }, {    0x6c,  9 }, {    0x6d,  9 },
    {     0x8, 10 }, {    0x19, 10 }, {    0x9b, 10 }, {    0x1b, 10 },
    {    0x9a, 10 }, {    0x13, 11 }, {    0x34, 11 }, {    0x35, 11 },
    {    0x61, 12 }, {    0x48, 13 }, {    0xc4, 13 }, {    0x4a, 13 },
    {    0xc6, 13 }, {    0xc7, 13 }, {    0x92, 14 }, {   0x18b, 14 },
    {    0x93, 14 }, {   0x183, 14 }, {   0x182, 14 }, {    0x96, 14 },
    {    0x97, 14 }, {   0x180, 14 }, {   0x314, 15 }, {   0x315, 15 },
    {   0x605, 16 }, {   0x604, 16 }, {   0x606, 16 }, {   0xc0e, 17 },
    { 0x303cd, 23 }, { 0x303c9, 23 }, { 0x303c8, 23 }, { 0x303ca, 23 },
    { 0x303cb, 23 }, { 0x303cc, 23 }, { 0x303ce, 23 }, { 0x303cf, 23 },
    { 0x303d0, 23 }, { 0x303d1, 23 }, { 0x303d2, 23 }, { 0x303d3, 23 },
    { 0x303d4, 23 }, { 0x303d5, 23 }, { 0x303d6, 23 }, { 0x303d7, 23 },
    { 0x303d8, 23 }, { 0x303d9, 23 }, { 0x303da, 23 }, { 0x303db, 23 },
    { 0x303dc, 23 }, { 0x303dd, 23 }, { 0x303de, 23 }, { 0x303df, 23 },
    { 0x303e0, 23 }, { 0x303e1, 23 }, { 0x303e2, 23 }, { 0x303e3, 23 },
    { 0x303e4, 23 }, { 0x303e5, 23 }, { 0x303e6, 23 }, { 0x303e7, 23 },
    { 0x303e8, 23 }, { 0x303e9, 23 }, { 0x303ea, 23 }, { 0x303eb, 23 },
    { 0x303ec, 23 }, { 0x303ed, 23 }, { 0x303ee, 23 }, { 0x303ef, 23 },
    { 0x303f0, 23 }, { 0x303f1, 23 }, { 0x303f2, 23 }, { 0x303f3, 23 },
    { 0x303f4, 23 }, { 0x303f5, 23 }, { 0x303f6, 23 }, { 0x303f7, 23 },
    { 0x303f8, 23 }, { 0x303f9, 23 }, { 0x303fa, 23 }, { 0x303fb, 23 },
    { 0x303fc, 23 }, { 0x303fd, 23 }, { 0x303fe, 23 }, { 0x303ff, 23 },
    { 0x60780, 24 }, { 0x60781, 24 }, { 0x60782, 24 }, { 0x60783, 24 },
    { 0x60784, 24 }, { 0x60785, 24 }, { 0x60786, 24 }, { 0x60787, 24 },
    { 0x60788, 24 }, { 0x60789, 24 }, { 0x6078a, 24 }, { 0x6078b, 24 },
    { 0x6078c, 24 }, { 0x6078d, 24 }, { 0x6078e, 24 }, { 0x6078f, 24 },
    },
    {
    {      0x0,  2 }, {      0x1,  2 }, {      0x5,  3 }, {      0x9,  4 },
    {      0xd,  4 }, {     0x11,  5 }, {     0x1d,  5 }, {     0x1f,  5 },
    {     0x21,  6 }, {     0x31,  6 }, {     0x38,  6 }, {     0x33,  6 },
    {     0x39,  6 }, {     0x3d,  6 }, {     0x61,  7 }, {     0x79,  7 },
    {     0x80,  8 }, {     0xc8,  8 }, {     0xca,  8 }, {     0xf0,  8 },
    {     0x81,  8 }, {     0xc0,  8 }, {     0xc9,  8 }, {    0x107,  9 },
    {    0x106,  9 }, {    0x196,  9 }, {    0x183,  9 }, {    0x1e3,  9 },
    {    0x1e2,  9 }, {    0x20a, 10 }, {    0x20b, 10 }, {    0x609, 11 },
    {    0x412, 11 }, {    0x413, 11 }, {    0x60b, 11 }, {    0x411, 11 },
    {    0x60a, 11 }, {    0x65f, 11 }, {    0x410, 11 }, {    0x65d, 11 },
    {    0x65e, 11 }, {    0xcb8, 12 }, {    0xc10, 12 }, {    0xcb9, 12 },
    {   0x1823, 13 }, {   0x3045, 14 }, {   0x6089, 15 }, {   0xc110, 16 },
    { 0x304448, 22 }, { 0x304449, 22 }, { 0x30444a, 22 }, { 0x30444b, 22 },
    { 0x30444c, 22 }, { 0x30444d, 22 }, { 0x30444e, 22 }, { 0x30444f, 22 },
    { 0x304450, 22 }, { 0x304451, 22 }, { 0x304452, 22 }, { 0x304453, 22 },
    { 0x304454, 22 }, { 0x304455, 22 }, { 0x304456, 22 }, { 0x304457, 22 },
    { 0x304458, 22 }, { 0x304459, 22 }, { 0x30445a, 22 }, { 0x30445b, 22 },
    { 0x30445c, 22 }, { 0x30445d, 22 }, { 0x30445e, 22 }, { 0x30445f, 22 },
    { 0x304460, 22 }, { 0x304461, 22 }, { 0x304462, 22 }, { 0x304463, 22 },
    { 0x304464, 22 }, { 0x304465, 22 }, { 0x304466, 22 }, { 0x304467, 22 },
    { 0x304468, 22 }, { 0x304469, 22 }, { 0x30446a, 22 }, { 0x30446b, 22 },
    { 0x30446c, 22 }, { 0x30446d, 22 }, { 0x30446e, 22 }, { 0x30446f, 22 },
    { 0x304470, 22 }, { 0x304471, 22 }, { 0x304472, 22 }, { 0x304473, 22 },
    { 0x304474, 22 }, { 0x304475, 22 }, { 0x304476, 22 }, { 0x304477, 22 },
    { 0x304478, 22 }, { 0x304479, 22 }, { 0x30447a, 22 }, { 0x30447b, 22 },
    { 0x30447c, 22 }, { 0x30447d, 22 }, { 0x30447e, 22 }, { 0x30447f, 22 },
    { 0x608880, 23 }, { 0x608881, 23 }, { 0x608882, 23 }, { 0x608883, 23 },
    { 0x608884, 23 }, { 0x608885, 23 }, { 0x608886, 23 }, { 0x608887, 23 },
    { 0x608888, 23 }, { 0x608889, 23 }, { 0x60888a, 23 }, { 0x60888b, 23 },
    { 0x60888c, 23 }, { 0x60888d, 23 }, { 0x60888e, 23 }, { 0x60888f, 23 },
    }
},
{
    /* dc table 1 */
    {
    {      0x2,  2 }, {      0x3,  2 }, {      0x3,  3 }, {      0x2,  4 },
    {      0x5,  4 }, {      0x1,  5 }, {      0x3,  5 }, {      0x8,  5 },
    {      0x0,  6 }, {      0x5,  6 }, {      0xd,  6 }, {      0xf,  6 },
    {     0x13,  6 }, {      0x8,  7 }, {     0x18,  7 }, {     0x1c,  7 },
    {     0x24,  7 }, {      0x4,  8 }, {      0x6,  8 }, {     0x12,  8 },
    {     0x32,  8 }, {     0x3b,  8 }, {     0x4a,  8 }, {     0x4b,  8 },
    {      0xb,  9 }, {     0x26,  9 }, {     0x27,  9 }, {     0x66,  9 },
    {     0x74,  9 }, {     0x75,  9 }, {     0x14, 10 }, {     0x1c, 10 },
    {     0x1f, 10 }, {     0x1d, 10 }, {     0x2b, 11 }, {     0x3d, 11 },
    {    0x19d, 11 }, {    0x19f, 11 }, {     0x54, 12 }, {    0x339, 12 },
    {    0x338, 12 }, {    0x33d, 12 }, {     0xab, 13 }, {     0xf1, 13 },
    {    0x678, 13 }, {     0xf2, 13 }, {    0x1e0, 14 }, {    0x1e1, 14 },
    {    0x154, 14 }, {    0xcf2, 14 }, {    0x3cc, 15 }, {    0x2ab, 15 },
    {   0x19e7, 15 }, {    0x3ce, 15 }, {   0x19e6, 15 }, {    0x554, 16 },
    {    0x79f, 16 }, {    0x555, 16 }, {    0xf3d, 17 }, {    0xf37, 17 },
    {    0xf3c, 17 }, {    0xf35, 17 }, {   0x1e6d, 18 }, {   0x1e68, 18 },
    {   0x3cd8, 19 }, {   0x3cd3, 19 }, {   0x3cd9, 19 }, {   0x79a4, 20 },
    {  0xf34ba, 25 }, {  0xf34b4, 25 }, {  0xf34b5, 25 }, {  0xf34b6, 25 },
    {  0xf34b7, 25 }, {  0xf34b8, 25 }, {  0xf34b9, 25 }, {  0xf34bb, 25 },
    {  0xf34bc, 25 }, {  0xf34bd, 25 }, {  0xf34be, 25 }, {  0xf34bf, 25 },
    { 0x1e6940, 26 }, { 0x1e6941, 26 }, { 0x1e6942, 26 }, { 0x1e6943, 26 },
    { 0x1e6944, 26 }, { 0x1e6945, 26 }, { 0x1e6946, 26 }, { 0x1e6947, 26 },
    { 0x1e6948, 26 }, { 0x1e6949, 26 }, { 0x1e694a, 26 }, { 0x1e694b, 26 },
    { 0x1e694c, 26 }, { 0x1e694d, 26 }, { 0x1e694e, 26 }, { 0x1e694f, 26 },
    { 0x1e6950, 26 }, { 0x1e6951, 26 }, { 0x1e6952, 26 }, { 0x1e6953, 26 },
    { 0x1e6954, 26 }, { 0x1e6955, 26 }, { 0x1e6956, 26 }, { 0x1e6957, 26 },
    { 0x1e6958, 26 }, { 0x1e6959, 26 }, { 0x1e695a, 26 }, { 0x1e695b, 26 },
    { 0x1e695c, 26 }, { 0x1e695d, 26 }, { 0x1e695e, 26 }, { 0x1e695f, 26 },
    { 0x1e6960, 26 }, { 0x1e6961, 26 }, { 0x1e6962, 26 }, { 0x1e6963, 26 },
    { 0x1e6964, 26 }, { 0x1e6965, 26 }, { 0x1e6966, 26 }, { 0x1e6967, 26 },
    },
    {
    {       0x0,  2 }, {       0x1,  2 }, {       0x4,  3 }, {       0x7,  3 },
    {       0xb,  4 }, {       0xd,  4 }, {      0x15,  5 }, {      0x28,  6 },
    {      0x30,  6 }, {      0x32,  6 }, {      0x52,  7 }, {      0x62,  7 },
    {      0x66,  7 }, {      0xa6,  8 }, {      0xc6,  8 }, {      0xcf,  8 },
    {     0x14f,  9 }, {     0x18e,  9 }, {     0x19c,  9 }, {     0x29d, 10 },
    {     0x33a, 10 }, {     0x538, 11 }, {     0x63c, 11 }, {     0x63e, 11 },
    {     0x63f, 11 }, {     0x676, 11 }, {     0xa73, 12 }, {     0xc7a, 12 },
    {     0xcef, 12 }, {    0x14e5, 13 }, {    0x19dd, 13 }, {    0x29c8, 14 },
    {    0x29c9, 14 }, {    0x63dd, 15 }, {    0x33b8, 14 }, {    0x33b9, 14 },
    {    0xc7b6, 16 }, {    0x63d8, 15 }, {    0x63df, 15 }, {    0xc7b3, 16 },
    {    0xc7b4, 16 }, {    0xc7b5, 16 }, {    0x63de, 15 }, {    0xc7b7, 16 },
    {    0xc7b8, 16 }, {    0xc7b9, 16 }, {   0x18f65, 17 }, {   0x31ec8, 18 },
    {  0xc7b248, 24 }, {  0xc7b249, 24 }, {  0xc7b24a, 24 }, {  0xc7b24b, 24 },
    {  0xc7b24c, 24 }, {  0xc7b24d, 24 }, {  0xc7b24e, 24 }, {  0xc7b24f, 24 },
    {  0xc7b250, 24 }, {  0xc7b251, 24 }, {  0xc7b252, 24 }, {  0xc7b253, 24 },
    {  0xc7b254, 24 }, {  0xc7b255, 24 }, {  0xc7b256, 24 }, {  0xc7b257, 24 },
    {  0xc7b258, 24 }, {  0xc7b259, 24 }, {  0xc7b25a, 24 }, {  0xc7b25b, 24 },
    {  0xc7b25c, 24 }, {  0xc7b25d, 24 }, {  0xc7b25e, 24 }, {  0xc7b25f, 24 },
    {  0xc7b260, 24 }, {  0xc7b261, 24 }, {  0xc7b262, 24 }, {  0xc7b263, 24 },
    {  0xc7b264, 24 }, {  0xc7b265, 24 }, {  0xc7b266, 24 }, {  0xc7b267, 24 },
    {  0xc7b268, 24 }, {  0xc7b269, 24 }, {  0xc7b26a, 24 }, {  0xc7b26b, 24 },
    {  0xc7b26c, 24 }, {  0xc7b26d, 24 }, {  0xc7b26e, 24 }, {  0xc7b26f, 24 },
    {  0xc7b270, 24 }, {  0xc7b271, 24 }, {  0xc7b272, 24 }, {  0xc7b273, 24 },
    {  0xc7b274, 24 }, {  0xc7b275, 24 }, {  0xc7b276, 24 }, {  0xc7b277, 24 },
    {  0xc7b278, 24 }, {  0xc7b279, 24 }, {  0xc7b27a, 24 }, {  0xc7b27b, 24 },
    {  0xc7b27c, 24 }, {  0xc7b27d, 24 }, {  0xc7b27e, 24 }, {  0xc7b27f, 24 },
    { 0x18f6480, 25 }, { 0x18f6481, 25 }, { 0x18f6482, 25 }, { 0x18f6483, 25 },
    { 0x18f6484, 25 }, { 0x18f6485, 25 }, { 0x18f6486, 25 }, { 0x18f6487, 25 },
    { 0x18f6488, 25 }, { 0x18f6489, 25 }, { 0x18f648a, 25 }, { 0x18f648b, 25 },
    { 0x18f648c, 25 }, { 0x18f648d, 25 }, { 0x18f648e, 25 }, { 0x18f648f, 25 },
    }
}
};

const uint8_t ff_wmv1_scantable[WMV1_SCANTABLE_COUNT][64] = {
    { 0x00, 0x08, 0x01, 0x02, 0x09, 0x10, 0x18, 0x11,
      0x0A, 0x03, 0x04, 0x0B, 0x12, 0x19, 0x20, 0x28,
      0x30, 0x38, 0x29, 0x21, 0x1A, 0x13, 0x0C, 0x05,
      0x06, 0x0D, 0x14, 0x1B, 0x22, 0x31, 0x39, 0x3A,
      0x32, 0x2A, 0x23, 0x1C, 0x15, 0x0E, 0x07, 0x0F,
      0x16, 0x1D, 0x24, 0x2B, 0x33, 0x3B, 0x3C, 0x34,
      0x2C, 0x25, 0x1E, 0x17, 0x1F, 0x26, 0x2D, 0x35,
      0x3D, 0x3E, 0x36, 0x2E, 0x27, 0x2F, 0x37, 0x3F, },
    { 0x00, 0x08, 0x01, 0x02, 0x09, 0x10, 0x18, 0x11,
      0x0A, 0x03, 0x04, 0x0B, 0x12, 0x19, 0x20, 0x28,
      0x21, 0x30, 0x1A, 0x13, 0x0C, 0x05, 0x06, 0x0D,
      0x14, 0x1B, 0x22, 0x29, 0x38, 0x31, 0x39, 0x2A,
      0x23, 0x1C, 0x15, 0x0E, 0x07, 0x0F, 0x16, 0x1D,
      0x24, 0x2B, 0x32, 0x3A, 0x33, 0x3B, 0x2C, 0x25,
      0x1E, 0x17, 0x1F, 0x26, 0x2D, 0x34, 0x3C, 0x35,
      0x3D, 0x2E, 0x27, 0x2F, 0x36, 0x3E, 0x37, 0x3F, },
    { 0x00, 0x01, 0x08, 0x02, 0x03, 0x09, 0x10, 0x18,
      0x11, 0x0A, 0x04, 0x05, 0x0B, 0x12, 0x19, 0x20,
      0x28, 0x30, 0x21, 0x1A, 0x13, 0x0C, 0x06, 0x07,
      0x0D, 0x14, 0x1B, 0x22, 0x29, 0x38, 0x31, 0x39,
      0x2A, 0x23, 0x1C, 0x15, 0x0E, 0x0F, 0x16, 0x1D,
      0x24, 0x2B, 0x32, 0x3A, 0x33, 0x2C, 0x25, 0x1E,
      0x17, 0x1F, 0x26, 0x2D, 0x34, 0x3B, 0x3C, 0x35,
      0x2E, 0x27, 0x2F, 0x36, 0x3D, 0x3E, 0x37, 0x3F, },
    { 0x00, 0x08, 0x10, 0x01, 0x18, 0x20, 0x28, 0x09,
      0x02, 0x03, 0x0A, 0x11, 0x19, 0x30, 0x38, 0x29,
      0x21, 0x1A, 0x12, 0x0B, 0x04, 0x05, 0x0C, 0x13,
      0x1B, 0x22, 0x31, 0x39, 0x32, 0x2A, 0x23, 0x1C,
      0x14, 0x0D, 0x06, 0x07, 0x0E, 0x15, 0x1D, 0x24,
      0x2B, 0x33, 0x3A, 0x3B, 0x34, 0x2C, 0x25, 0x1E,
      0x16, 0x0F, 0x17, 0x1F, 0x26, 0x2D, 0x3C, 0x35,
      0x2E, 0x27, 0x2F, 0x36, 0x3D, 0x3E, 0x37, 0x3F, }
};
