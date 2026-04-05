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

#include <float.h>

#include "libavutil/mem.h"
#include "libavutil/mem_internal.h"
#include "libavutil/tx.h"
#include "libavutil/thread.h"
#include "libavutil/float_dsp.h"

#include "avcodec.h"
#include "get_bits.h"
#include "codec_internal.h"
#include "decode.h"

#define MAX_CHANNELS 2
#define MAX_BUFFER_SIZE 256

static const uint8_t weights_tab[8][34] = {
    {
        1, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3,
        3, 3, 3, 3, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 8
    },
    {
        0, 1, 1, 2, 3, 4, 4, 4, 4, 5, 6, 6, 6, 6, 6, 7, 7, 7,
        7, 7, 7, 7, 8, 8, 8, 9, 10, 10, 11, 11, 12, 12, 12, 12,
    },
    {
        0, 1, 1, 2, 3, 3, 3, 3, 3, 4, 4, 5, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 6, 6, 6, 7, 8, 9, 9, 10, 10, 11, 11, 11,
    },
    {
        0, 1, 3, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10,
    },
    {
        0, 1, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 10, 11,
        11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13,
    },
    {
        1, 0, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 7, 8, 8, 8, 9, 9,
        9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11,
    },
    {
        0, 0, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4,
        4, 4, 4, 4, 4, 4, 5, 5, 6, 7, 7, 7, 8, 9, 9, 9, 9,
    },
    {
        0, 0, 1, 2, 3, 4, 4, 5, 5, 6, 7, 7, 8, 8, 8, 8, 9, 9, 9,
        9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12,
    },
};

static const uint8_t gcurves[50][50] = {
    {
        128,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        31,225,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        17,128,239,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        12, 69,187,244,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        10, 43,128,213,246,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        9, 31, 87,169,225,247,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        8, 24, 62,128,194,232,248,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        8, 19, 47, 97,159,209,237,248,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        7, 17, 37, 75,128,181,219,239,249,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        7, 15, 31, 59,103,153,197,225,241,249,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        7, 13, 26, 48, 83,128,173,208,230,243,249,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        6, 12, 23, 41, 69,107,149,187,215,233,244,250,255,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        6, 11, 20, 35, 58, 90,128,166,198,221,236,245,250,255,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        6, 11, 18, 31, 49, 76,110,146,180,207,225,238,245,250,255,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        6, 10, 17, 27, 43, 66, 95,128,161,190,213,229,239,246,250,255,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        6, 10, 15, 24, 38, 57, 82,112,144,174,199,218,232,241,246,250,255,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        6, 9, 14, 22, 34, 50, 72, 98,128,158,184,206,222,234,242,247,250,255,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        6, 9, 13, 20, 31, 45, 63, 87,114,142,169,193,211,225,236,243,247,250,255,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        6, 9, 13, 19, 28, 40, 56, 77,101,128,155,179,200,216,228,237,243,247,250,255,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        6, 8, 12, 18, 26, 36, 51, 69, 91,115,141,165,187,205,220,230,238,244,248,250,255,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        6, 8, 12, 17, 24, 33, 46, 62, 81,104,128,152,175,194,210,223,232,239,244,248,250,255,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        6, 8, 11, 16, 22, 31, 42, 56, 74, 94,116,140,162,182,200,214,225,234,240,245,248,250,255,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 8, 11, 15, 21, 28, 38, 51, 67, 85,106,128,150,171,189,205,218,228,235,241,245,248,251,255,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 8, 10, 14, 19, 26, 35, 47, 61, 78, 97,117,139,159,178,195,209,221,230,237,242,246,248,251,255,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 7, 10, 14, 18, 25, 33, 43, 56, 71, 88,108,128,148,168,185,200,213,223,231,238,242,246,249,251,
        255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 7, 10, 13, 17, 23, 31, 40, 51, 65, 81, 99,118,138,157,175,191,205,216,225,233,239,243,246,249,
        251,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 7, 9, 13, 17, 22, 29, 37, 47, 60, 75, 91,109,128,147,165,181,196,209,219,227,234,239,243,247,
        249,251,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 7, 9, 12, 16, 21, 27, 35, 44, 55, 69, 84,101,119,137,155,172,187,201,212,221,229,235,240,244,
        247,249,251,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 7, 9, 12, 15, 20, 25, 32, 41, 51, 64, 78, 94,110,128,146,162,178,192,205,215,224,231,236,241,
        244,247,249,251,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 7, 9, 11, 15, 19, 24, 31, 38, 48, 59, 72, 87,103,119,137,153,169,184,197,208,218,225,232,237,
        241,245,247,249,251,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 7, 9, 11, 14, 18, 23, 29, 36, 45, 55, 67, 81, 96,112,128,144,160,175,189,201,211,220,227,233,
        238,242,245,247,249,251,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 7, 8, 11, 14, 17, 22, 27, 34, 42, 52, 63, 75, 89,104,120,136,152,167,181,193,204,214,222,229,
        234,239,242,245,248,249,251,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 7, 8, 11, 13, 17, 21, 26, 32, 40, 48, 59, 70, 83, 98,113,128,143,158,173,186,197,208,216,224,
        230,235,239,243,245,248,249,251,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 7, 8, 10, 13, 16, 20, 25, 31, 37, 46, 55, 66, 78, 91,106,120,136,150,165,178,190,201,210,219,
        225,231,236,240,243,246,248,249,251,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 6, 8, 10, 12, 15, 19, 24, 29, 35, 43, 52, 62, 73, 86, 99,113,128,143,157,170,183,194,204,213,
        221,227,232,237,241,244,246,248,250,251,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 6, 8, 10, 12, 15, 18, 23, 28, 34, 41, 49, 58, 69, 81, 93,107,121,135,149,163,175,187,198,207,
        215,222,228,233,238,241,244,246,248,250,251,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 6, 8, 10, 12, 15, 18, 22, 26, 32, 39, 46, 55, 65, 76, 88,101,114,128,142,155,168,180,191,201,
        210,217,224,230,234,238,241,244,246,248,250,251,255,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 6, 8, 9, 12, 14, 17, 21, 25, 31, 37, 44, 52, 61, 72, 83, 95,108,121,135,148,161,173,184,195,
        204,212,219,225,231,235,239,242,244,247,248,250,251,255,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 6, 8, 9, 11, 14, 17, 20, 24, 29, 35, 42, 49, 58, 68, 78, 90,102,115,128,141,154,166,178,188,
        198,207,214,221,227,232,236,239,242,245,247,248,250,251,255,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 6, 8, 9, 11, 13, 16, 19, 23, 28, 33, 40, 47, 55, 64, 74, 85, 97,109,122,134,147,159,171,182,
        192,201,209,216,223,228,233,237,240,243,245,247,248,250,251,255,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 6, 7, 9, 11, 13, 16, 19, 22, 27, 32, 38, 44, 52, 61, 70, 80, 92,103,116,128,140,153,164,176,
        186,195,204,212,218,224,229,234,237,240,243,245,247,249,250,251,255,255,255,255,255,255,255,255,255,
    },
    {
        5, 6, 7, 9, 11, 13, 15, 18, 22, 26, 31, 36, 42, 49, 58, 66, 76, 87, 98,110,122,134,146,158,169,
        180,190,198,207,214,220,225,230,234,238,241,243,245,247,249,250,251,255,255,255,255,255,255,255,255,
    },
    {
        5, 6, 7, 9, 10, 12, 15, 18, 21, 25, 29, 34, 40, 47, 55, 63, 72, 82, 93,104,116,128,140,152,163,
        174,184,193,201,209,216,222,227,231,235,238,241,244,246,247,249,250,251,255,255,255,255,255,255,255,
    },
    {
        5, 6, 7, 9, 10, 12, 14, 17, 20, 24, 28, 33, 39, 45, 52, 60, 69, 78, 89, 99,111,122,134,145,157,
        167,178,187,196,204,211,217,223,228,232,236,239,242,244,246,247,249,250,251,255,255,255,255,255,255,
    },
    {
        5, 6, 7, 8, 10, 12, 14, 17, 20, 23, 27, 32, 37, 43, 50, 57, 66, 75, 84, 95,105,117,128,139,151,
        161,172,181,190,199,206,213,219,224,229,233,236,239,242,244,246,248,249,250,251,255,255,255,255,255,
    },
    {
        5, 6, 7, 8, 10, 12, 14, 16, 19, 22, 26, 31, 36, 41, 48, 55, 62, 71, 80, 90,101,111,122,134,145,
        155,166,176,185,194,201,208,215,220,225,230,234,237,240,242,244,246,248,249,250,251,255,255,255,255,
    },
    {
        5, 6, 7, 8, 10, 11, 13, 16, 18, 22, 25, 29, 34, 39, 45, 52, 60, 68, 77, 86, 96,106,117,128,139,
        150,160,170,179,188,196,204,211,217,222,227,231,234,238,240,243,245,246,248,249,250,251,255,255,255,
    },
    {
        5, 6, 7, 8, 10, 11, 13, 15, 18, 21, 24, 28, 33, 38, 44, 50, 57, 65, 73, 82, 92,102,112,123,133,
        144,154,164,174,183,191,199,206,212,218,223,228,232,235,238,241,243,245,246,248,249,250,251,255,255,
    },
    {
        5, 6, 7, 8, 9, 11, 13, 15, 17, 20, 24, 27, 32, 36, 42, 48, 55, 62, 70, 78, 88, 97,107,118,128,
        138,149,159,168,178,186,194,201,208,214,220,224,229,232,236,239,241,243,245,247,248,249,250,251,255,
    },
    {
        5, 6, 7, 8, 9, 11, 13, 15, 17, 20, 23, 26, 31, 35, 40, 46, 52, 59, 67, 75, 84, 93,103,113,123,
        133,143,153,163,172,181,189,197,204,210,216,221,225,230,233,236,239,241,243,245,247,248,249,250,251,
    },
};

static const uint8_t scale_factors_a3_bits[] =
{
    2, 2, 4, 6, 6, 5, 3, 2
};

static const uint8_t scale_factors_a3_codes[] =
{
    0x00, 0x01, 0x0E, 0x3E, 0x3F, 0x1E, 0x06, 0x02
};

static const uint8_t scale_factors_a4_bits[] =
{
    2, 2, 4, 5, 6, 7, 8, 8, 8, 8, 8, 8, 6, 5, 4, 2
};

static const uint8_t scale_factors_a4_codes[] =
{
    0x01, 0x02, 0x00, 0x06, 0x0F, 0x13, 0x23, 0x24, 0x25, 0x22, 0x21, 0x20, 0x0E, 0x05, 0x01, 0x03
};

static const uint8_t scale_factors_a5_bits[] =
{
    2, 3, 3, 4, 5, 5, 6, 7, 7, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 6, 5, 5, 4, 3
};

static const uint8_t scale_factors_a5_codes[] =
{
    0x02, 0x01, 0x07, 0x0D, 0x0C, 0x18, 0x1B, 0x21, 0x3F, 0x6A, 0x6B, 0x68, 0x73, 0x79, 0x7C, 0x7D,
    0x7A, 0x7B, 0x78, 0x72, 0x44, 0x45, 0x47, 0x46, 0x69, 0x38, 0x20, 0x1D, 0x19, 0x09, 0x05, 0x00
};

static const uint8_t scale_factors_a6_bits[] =
{
    3, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4
};

static const uint8_t scale_factors_a6_codes[] =
{
    0x00, 0x01, 0x04, 0x05, 0x12, 0x13, 0x2E, 0x2F, 0x30, 0x66, 0x67, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA,
    0xDB, 0xDC, 0xDD, 0xDE, 0xDF, 0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA,
    0xEB, 0xEC, 0xED, 0xEE, 0xEF, 0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA,
    0xFB, 0xFC, 0xFD, 0xFE, 0xFF, 0x68, 0x69, 0x6A, 0x31, 0x32, 0x14, 0x15, 0x16, 0x06, 0x07, 0x08
};

static const uint8_t scale_factors_b2_bits[] =
{
    1, 2, 0, 2
};

static const uint8_t scale_factors_b2_codes[] =
{
    0x00, 0x03, 0x00, 0x02
};

static const uint8_t scale_factors_b3_bits[] =
{
    1, 3, 5, 6, 0, 6, 4, 2
};

static const uint8_t scale_factors_b3_codes[] =
{
    0x01, 0x00, 0x04, 0x0B, 0x00, 0x0A, 0x03, 0x01
};

static const uint8_t scale_factors_b4_bits[] =
{
    1, 3, 4, 5, 5, 7, 8, 8, 0, 8, 8, 7, 6, 6, 4, 3
};

static const uint8_t scale_factors_b4_codes[] =
{
    0x01, 0x01, 0x04, 0x0E, 0x0F, 0x2C, 0x5A, 0x5D, 0x00, 0x5C, 0x5B, 0x2F, 0x15, 0x14, 0x06, 0x00
};

static const uint8_t scale_factors_b5_bits[] =
{
    3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 6, 7, 7, 7, 8, 8,
    8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 3
};

static const uint8_t scale_factors_b5_codes[] =
{
    0x00, 0x05, 0x07, 0x0C, 0x04, 0x02, 0x03, 0x05, 0x09, 0x10, 0x23, 0x33, 0x36, 0x6E, 0x60, 0x65,
    0x62, 0x61, 0x63, 0x64, 0x6F, 0x6D, 0x6C, 0x6B, 0x6A, 0x68, 0x69, 0x45, 0x44, 0x37, 0x1A, 0x07
};

static const uint8_t quc_tab[16+1+1] = {
    0, 4, 8, 10, 12, 14, 16, 18, 20, 22, 24, 25, 26, 28, 30, 32, 34
};

static const VLCElem *sf_a_vlc[4];
static const VLCElem *sf_b_vlc[4];

typedef struct ChannelContext {
    int sfactors[34];

    int precisions[34];
    int precisions_fine[34];
    int precision_mask[34];

    int16_t qspec[256];
    int16_t qfine[256];

    DECLARE_ALIGNED(32, float, spec)[256];
    DECLARE_ALIGNED(32, float, temp)[256];
    DECLARE_ALIGNED(32, float, hist)[256];

    av_tx_fn tx_fn[2];
    AVTXContext *tx_ctx[2];
} ChannelContext;

typedef struct Gradient {
    int grad[34];
    int mode;
    int start_unit;
    int stop_unit;
    int start_value;
    int stop_value;
    int boundary;
} Gradient;

typedef struct LDACContext {
    AVFloatDSPContext *fdsp;

    int frame_samples;
    int nb_quants;

    Gradient g;

    DECLARE_ALIGNED(32, float, window)[2][256];

    ChannelContext ch[MAX_CHANNELS];
} LDACContext;

static av_cold void ldac_init_static(void)
{
    static VLCElem vlc_buf[4096];
    VLCInitState state = VLC_INIT_STATE(vlc_buf);

    sf_a_vlc[0] = ff_vlc_init_tables_sparse(&state, 9,
                                            FF_ARRAY_ELEMS(scale_factors_a3_bits),
                                            scale_factors_a3_bits, 1, 1, scale_factors_a3_codes, 1, 1,
                                            0, 0, 0, 0);

    sf_a_vlc[1] = ff_vlc_init_tables_sparse(&state, 9,
                                            FF_ARRAY_ELEMS(scale_factors_a4_bits),
                                            scale_factors_a4_bits, 1, 1, scale_factors_a4_codes, 1, 1,
                                            0, 0, 0, 0);

    sf_a_vlc[2] = ff_vlc_init_tables_sparse(&state, 9,
                                            FF_ARRAY_ELEMS(scale_factors_a5_bits),
                                            scale_factors_a5_bits, 1, 1, scale_factors_a5_codes, 1, 1,
                                            0, 0, 0, 0);

    sf_a_vlc[3] = ff_vlc_init_tables_sparse(&state, 9,
                                            FF_ARRAY_ELEMS(scale_factors_a6_bits),
                                            scale_factors_a6_bits, 1, 1, scale_factors_a6_codes, 1, 1,
                                            0, 0, 0, 0);

    sf_b_vlc[0] = ff_vlc_init_tables_sparse(&state, 9,
                                            FF_ARRAY_ELEMS(scale_factors_b2_bits),
                                            scale_factors_b2_bits, 1, 1, scale_factors_b2_codes, 1, 1,
                                            0, 0, 0, 0);

    sf_b_vlc[1] = ff_vlc_init_tables_sparse(&state, 9,
                                            FF_ARRAY_ELEMS(scale_factors_b3_bits),
                                            scale_factors_b3_bits, 1, 1, scale_factors_b3_codes, 1, 1,
                                            0, 0, 0, 0);

    sf_b_vlc[2] = ff_vlc_init_tables_sparse(&state, 9,
                                            FF_ARRAY_ELEMS(scale_factors_b4_bits),
                                            scale_factors_b4_bits, 1, 1, scale_factors_b4_codes, 1, 1,
                                            0, 0, 0, 0);

    sf_b_vlc[3] = ff_vlc_init_tables_sparse(&state, 9,
                                            FF_ARRAY_ELEMS(scale_factors_b5_bits),
                                            scale_factors_b5_bits, 1, 1, scale_factors_b5_codes, 1, 1,
                                            0, 0, 0, 0);
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    static AVOnce static_table_init = AV_ONCE_INIT;
    LDACContext *s = avctx->priv_data;
    int ret;

    avctx->sample_fmt = AV_SAMPLE_FMT_FLTP;

    ff_thread_once(&static_table_init, ldac_init_static);

    for (int ch = 0; ch < 2; ch++) {
        ChannelContext *c = &s->ch[ch];
        float scale = -1.f / 32768.f;

        ret = av_tx_init(&c->tx_ctx[0], &c->tx_fn[0], AV_TX_FLOAT_MDCT, 1, 128, &scale, 0);
        if (ret < 0)
            return ret;

        ret = av_tx_init(&c->tx_ctx[1], &c->tx_fn[1], AV_TX_FLOAT_MDCT, 1, 256, &scale, 0);
        if (ret < 0)
            return ret;
    }

    for (int i = 0; i < 128; i++) {
        const int size = 128;
        const int sidx = i;
        const int eidx = size - i - 1;
        const double s_c = (sin(((sidx + 0.5) / size - 0.5) * M_PI) + 1.0) * 0.5;
        const double e_c = (sin(((eidx + 0.5) / size - 0.5) * M_PI) + 1.0) * 0.5;

        s->window[0][i] = s_c / ((s_c * s_c) + (e_c * e_c));
    }

    for (int i = 0; i < 256; i++) {
        const int size = 256;
        const int sidx = i;
        const int eidx = size - i - 1;
        const double s_c = (sin(((sidx + 0.5) / size - 0.5) * M_PI) + 1.0) * 0.5;
        const double e_c = (sin(((eidx + 0.5) / size - 0.5) * M_PI) + 1.0) * 0.5;

        s->window[1][i] = s_c / ((s_c * s_c) + (e_c * e_c));
    }

    s->fdsp = avpriv_float_dsp_alloc(avctx->flags & AV_CODEC_FLAG_BITEXACT);
    if (!s->fdsp)
        return AVERROR(ENOMEM);

    return 0;
}

static const uint8_t ch_idx_to_nb_ch[] = { 1, 2, 2, 0 };
static const uint8_t sr_idx_to_spower[] = { 7, 7, 8, 8 };
static const int sr_idx_to_rate[8] = { 44100, 48000, 88200, 96000, 0, 0, 0, 0 };
static const uint8_t bs_tab[4] = { 1, 2, 1, 0 };

static void decode_gradient(AVCodecContext *avctx, GetBitContext *gb)
{
    LDACContext *s = avctx->priv_data;

    s->g.mode = get_bits(gb, 2);
    if (s->g.mode == 0) {
        s->g.start_unit = get_bits(gb, 6);
        s->g.stop_unit = get_bits(gb, 6) + 1;
        s->g.start_value = get_bits(gb, 5);
        s->g.stop_value = get_bits(gb, 5);
    } else {
        s->g.start_unit = get_bits(gb, 5);
        s->g.stop_unit = 26;
        s->g.start_value = get_bits(gb, 5);
        s->g.stop_value = 31;
    }

    s->g.boundary = get_bits(gb, 5);
}

static void calc_gradient(AVCodecContext *avctx)
{
    LDACContext *s = avctx->priv_data;
    int value_count = s->g.stop_value - s->g.start_value;
    int unit_count = s->g.stop_unit - s->g.start_unit;

    memset(s->g.grad, 0, sizeof(s->g.grad));

    for (int i = 0; i < s->g.stop_unit; i++)
        s->g.grad[i] = -s->g.start_value;

    for (int i = s->g.stop_unit; i < s->nb_quants; i++)
        s->g.grad[i] = -s->g.stop_value;

    if (unit_count > 0) {
        const uint8_t *curve = gcurves[unit_count-1];

        if (value_count > 0) {
            for (int i = s->g.start_unit; i < s->g.stop_unit; i++)
                s->g.grad[i] -= ((curve[i-s->g.start_unit] * (value_count-1)) >> 8) + 1;
        } else if (value_count < 0) {
            for (int i = s->g.start_unit; i < s->g.stop_unit; i++)
                s->g.grad[i] -= ((curve[i-s->g.start_unit] * (value_count-1)) >> 8) + 1;
        }
    }
}

static int decode_scale_factor0(AVCodecContext *avctx, ChannelContext *c, GetBitContext *gb)
{
    LDACContext *s = avctx->priv_data;
    int bits = get_bits(gb, 2) + 3;
    int offset = get_bits(gb, 5);
    int weight = get_bits(gb, 3);
    const int mask = (1<<bits)-1;
    const uint8_t *weight_tab = weights_tab[weight];
    const VLCElem *codebook = sf_a_vlc[bits-3];

    c->sfactors[0] = get_bits(gb, bits);

    for (int i = 1; i < s->nb_quants; i++) {
        int diff = get_vlc2(gb, codebook, 9, 1);

        if (diff < 0)
            return AVERROR_INVALIDDATA;
        diff = sign_extend(diff, bits);

        c->sfactors[i] = (c->sfactors[i-1] + diff) & mask;
        c->sfactors[i-1] += offset - weight_tab[i-1];
    }

    c->sfactors[s->nb_quants-1] += offset - weight_tab[s->nb_quants-1];

    return 0;
}

static int decode_scale_factor1(AVCodecContext *avctx, ChannelContext *c, GetBitContext *gb)
{
    LDACContext *s = avctx->priv_data;
    int bitlen = get_bits(gb, 2) + 2;

    if (bitlen > 4) {
        for (int i = 0; i < s->nb_quants; i++)
            c->sfactors[i] = get_bits(gb, 5);
    } else {
        int offset = get_bits(gb, 5);
        int weight = get_bits(gb, 3);
        const uint8_t *weight_tab = weights_tab[weight];

        for (int i = 0; i < s->nb_quants; i++)
            c->sfactors[i] = (int)get_bits(gb, bitlen) - weight_tab[i] + offset;
    }

    return 0;
}

static int decode_scale_factor2(AVCodecContext *avctx, ChannelContext *c, GetBitContext *gb)
{
    LDACContext *s = avctx->priv_data;
    ChannelContext *other = &s->ch[0];
    const int bits = get_bits(gb, 2) + 2;
    const VLCElem *codebook = sf_b_vlc[bits-2];

    for (int i = 0; i < s->nb_quants; i++) {
        int diff = get_vlc2(gb, codebook, 9, 1);

        if (diff < 0)
            return AVERROR_INVALIDDATA;
        diff = sign_extend(diff, bits);

        c->sfactors[i] = other->sfactors[i] + diff;
    }

    return 0;
}

static int decode_scale_factors(AVCodecContext *avctx, GetBitContext *gb, const int ch)
{
    LDACContext *s = avctx->priv_data;
    ChannelContext *c = &s->ch[ch];
    int mode = get_bits1(gb);
    int ret;

    if (ch == 0) {
        if (mode == 0)
            ret = decode_scale_factor0(avctx, c, gb);
        else
            ret = decode_scale_factor1(avctx, c, gb);
    } else {
        if (mode == 0)
            ret = decode_scale_factor0(avctx, c, gb);
        else
            ret = decode_scale_factor2(avctx, c, gb);
    }

    return ret;
}

static void calc_precision_mask(AVCodecContext *avctx, const int ch)
{
    LDACContext *s = avctx->priv_data;
    ChannelContext *c = &s->ch[ch];

    memset(c->precision_mask, 0, sizeof(c->precision_mask));

    for (int i = 1; i < s->nb_quants; i++) {
        const int delta = c->sfactors[i] - c->sfactors[i - 1];

        if (delta > 1)
            c->precision_mask[i] += FFMIN(delta - 1, 5);
        else if (delta < -1)
            c->precision_mask[i - 1] += FFMIN(-delta - 1, 5);
    }
}

static int calc_precisions(AVCodecContext *avctx, const int ch)
{
    LDACContext *s = avctx->priv_data;
    ChannelContext *c = &s->ch[ch];

    memset(c->precisions, 0, sizeof(c->precisions));

    for (int i = 0; i < s->nb_quants; i++) {
        int precision;

        precision = c->sfactors[i] + s->g.grad[i];
        switch (s->g.mode) {
        case 0:
            break;
        case 1:
            precision += c->precision_mask[i];
            if (precision > 0)
                precision >>= 1;
            break;
        case 2:
            precision += c->precision_mask[i];
            if (precision > 0)
                precision = (precision * 3) >> 3;
            break;
        case 3:
            precision += c->precision_mask[i];
            if (precision > 0)
                precision >>= 2;
            break;
        default:
            return AVERROR_BUG;
        }
        c->precisions[i] = FFMAX(precision, 1);
    }

    for (int i = 0; i < s->g.boundary; i++)
        c->precisions[i]++;

    for (int i = 0; i < s->nb_quants; i++) {
        c->precisions_fine[i] = 0;
        if (c->precisions[i] > 15) {
            c->precisions_fine[i] = c->precisions[i] - 15;
            c->precisions[i] = 15;
            if (c->precisions_fine[i] > 15)
                return AVERROR_INVALIDDATA;
        }
    }

    return 0;
}

static const uint16_t isp_tab[34+1] = {
    0, 2, 4, 6, 8, 10, 12, 14, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56,
    60, 64, 72, 80, 88, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 256,
};

static const uint8_t nsps_tab[34] = {
    2, 2, 2, 2, 2, 2, 2, 2,
    4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 8, 8, 8, 8,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
};

static const uint8_t wl_tab[16] = { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };

static const uint8_t decode2D_spectrum[8] = {
    0, 1, 2, 4, 6, 8, 9, 10
};

static const int16_t decode4D_spectrum[128] = {
    0, 1, 2, 4, 5, 6, 8, 9,10, 16, 17, 18, 20, 21, 22, 24,
    25, 26, 32, 33, 34, 36, 37, 38, 40, 41, 42, 64, 65, 66, 68, 69,
    70, 72, 73, 74, 80, 81, 82, 84, 85, 86, 88, 89, 90, 96, 97, 98,
    100, 101, 102, 104, 105, 106, 128, 129, 130, 132, 133, 134, 136, 137, 138, 144,
    145, 146, 148, 149, 150, 152, 153, 154, 160, 161, 162, 164, 165, 166, 168, 169,
    170, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

static const float quant_ss[16] = {
    2.0000000000000000e+00, 6.6666666666666663e-01, 2.8571428571428570e-01, 1.3333333333333333e-01,
    6.4516129032258063e-02, 3.1746031746031744e-02, 1.5748031496062992e-02, 7.8431372549019607e-03,
    3.9138943248532287e-03, 1.9550342130987292e-03, 9.7703957010258913e-04, 4.8840048840048840e-04,
    2.4417043096081065e-04, 1.2207776353537203e-04, 6.1037018951994385e-05, 3.0518043793392844e-05,
};

static const float quant_fss[16] =
{
    3.0518043793392844e-05, 1.0172681264464281e-05, 4.3597205419132631e-06, 2.0345362528928561e-06,
    9.8445302559331759e-07, 4.8441339354591809e-07, 2.4029955742829012e-07, 1.1967860311134448e-07,
    5.9722199204291275e-08, 2.9831909866464167e-08, 1.4908668194134265e-08, 7.4525137468602791e-09,
    3.7258019525568114e-09, 1.8627872668859698e-09, 9.3136520869755679e-10, 4.6567549848772173e-10
};

static int decode_spec(AVCodecContext *avctx, GetBitContext *gb, const int ch)
{
    LDACContext *s = avctx->priv_data;
    ChannelContext *c = &s->ch[ch];

    for (int i = 0; i < s->nb_quants; i++) {
        const int start = isp_tab[i];
        const int stop = isp_tab[i+1];
        const int nsps = nsps_tab[i];
        const int wl = wl_tab[c->precisions[i]];

        if (c->precisions[i] == 1) {
            if (nsps == 2) {
                const int x = decode2D_spectrum[get_bits(gb, 3)];
                const int idx = start;

                c->qspec[idx] = ((x>>2)&3) - 1;
                c->qspec[idx+1] = (x&3) - 1;
            } else {
                for (int j = 0, idx = start; j < nsps/4; j++, idx += 4) {
                    const int x = decode4D_spectrum[get_bits(gb, 7)];

                    c->qspec[idx] = ((x>>6)&3) - 1;
                    c->qspec[idx+1] = ((x>>4)&3) - 1;
                    c->qspec[idx+2] = ((x>>2)&3) - 1;
                    c->qspec[idx+3] = (x&3) - 1;
                }
            }
        } else {
            for (int j = start; j < stop; j++)
                c->qspec[j] = wl ? get_sbits(gb, wl) : 0;
        }
    }

    return 0;
}

static void dequantize_quant_unit(ChannelContext *c, int band)
{
    const int index = isp_tab[band];
    const int N = nsps_tab[band];
    const float step_size = quant_ss[c->precisions[band]];
    const float step_size_fine = quant_fss[c->precisions[band]];

    for (int sb = 0; sb < N; sb++) {
        const float coarse = c->qspec[index+sb] * step_size;
        const float fine = c->qfine[index+sb] * step_size_fine;

        c->spec[index + sb] = coarse + fine;
    }
}

static int decode_spec_fine(AVCodecContext *avctx, GetBitContext *gb, const int ch)
{
    LDACContext *s = avctx->priv_data;
    ChannelContext *c = &s->ch[ch];

    memset(c->qfine, 0, sizeof(c->qfine));

    for (int i = 0; i < s->nb_quants; i++) {
        if (c->precisions_fine[i] > 0) {
            const int start = isp_tab[i];
            const int stop = isp_tab[i+1];
            const int wl = wl_tab[c->precisions_fine[i]];

            for (int j = start; j < stop; j++)
                c->qfine[j] = wl ? get_sbits(gb, wl) : 0;
        }
    }

    return 0;
}

static void dequant_spec(AVCodecContext *avctx, const int ch)
{
    LDACContext *s = avctx->priv_data;
    ChannelContext *c = &s->ch[ch];

    memset(c->spec, 0, sizeof(c->spec));
    for (int i = 0; i < s->nb_quants; i++)
        dequantize_quant_unit(c, i);
}

static const float spectrum_scale[32] =
{
    3.0517578125e-5, 6.1035156250e-5, 1.2207031250e-4, 2.4414062500e-4,
    4.8828125000e-4, 9.7656250000e-4, 1.9531250000e-3, 3.9062500000e-3,
    7.8125000000e-3, 1.5625000000e-2, 3.1250000000e-2, 6.2500000000e-2,
    1.2500000000e-1, 2.5000000000e-1, 5.0000000000e-1, 1.0000000000e+0,
    2.0000000000e+0, 4.0000000000e+0, 8.0000000000e+0, 1.6000000000e+1,
    3.2000000000e+1, 6.4000000000e+1, 1.2800000000e+2, 2.5600000000e+2,
    5.1200000000e+2, 1.0240000000e+3, 2.0480000000e+3, 4.0960000000e+3,
    8.1920000000e+3, 1.6384000000e+4, 3.2768000000e+4, 6.5536000000e+4
};

static void scale_spec(AVCodecContext *avctx, const int ch)
{
    LDACContext *s = avctx->priv_data;
    ChannelContext *c = &s->ch[ch];
    const int N = s->nb_quants;
    float *const spec = c->spec;

    for (int i = 0; i < N; i++) {
        const int start = isp_tab[i];
        const int stop = isp_tab[i+1];

        if (c->sfactors[i] >= 0 &&
            c->sfactors[i] < 32) {
            for (int sb = start; sb < stop; sb++)
                spec[sb] *= spectrum_scale[c->sfactors[i]];
        }
    }
}

static void imdct(AVCodecContext *avctx, const int ch, float *output)
{
    LDACContext *s = avctx->priv_data;
    ChannelContext *c = &s->ch[ch];
    const int idx = s->frame_samples > 128;
    const int size = 1 << (7 + idx);
    const float *window = s->window[idx];
    float *input = c->spec;
    float *temp = c->temp;
    float *hist = c->hist;

    c->tx_fn[idx](c->tx_ctx[idx], temp, input, sizeof(float));
    s->fdsp->vector_fmul_window(output, hist, temp,
                                window, size >> 1);
    memcpy(hist, temp + (size >> 1), sizeof(float)*(size >> 1));
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame_ptr, AVPacket *avpkt)
{
    LDACContext *s = avctx->priv_data;
    GetBitContext gbc, *gb = &gbc;
    int ret, sr_idx, ch_idx;
    int nb_channels;
    int frame_size;

    ret = init_get_bits8(gb, avpkt->data, avpkt->size);
    if (ret < 0)
        return ret;

    if (get_bits(gb, 8) != 0xaa)
        return AVERROR_INVALIDDATA;

    sr_idx = get_bits(gb, 3);
    ch_idx = get_bits(gb, 2);
    frame_size = get_bits(gb, 9) + 1;
    /*frame_status = */get_bits(gb, 2);

    if (avpkt->size < frame_size)
        return AVERROR_INVALIDDATA;

    avctx->sample_rate = sr_idx_to_rate[sr_idx];
    avctx->ch_layout.nb_channels = nb_channels = ch_idx_to_nb_ch[ch_idx];
    s->frame_samples = 1 << sr_idx_to_spower[sr_idx];

    frame->nb_samples = s->frame_samples * bs_tab[ch_idx];
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    for (int b = 0; b < bs_tab[ch_idx]; b++) {
        const int nb_bands = get_bits(gb, 4) + 2;

        skip_bits(gb, 1);
        s->nb_quants = quc_tab[nb_bands];
        decode_gradient(avctx, gb);
        calc_gradient(avctx);

        for (int ch = 0; ch < nb_channels; ch++) {
            ret = decode_scale_factors(avctx, gb, ch);
            if (ret < 0)
                return ret;

            calc_precision_mask(avctx, ch);
            ret = calc_precisions(avctx, ch);
            if (ret < 0)
                return ret;
            decode_spec(avctx, gb, ch);
            decode_spec_fine(avctx, gb, ch);
            dequant_spec(avctx, ch);
            scale_spec(avctx, ch);

            imdct(avctx, ch, ((float *)frame->extended_data[ch]) + b * s->frame_samples);
        }

        align_get_bits(gb);
    }

    *got_frame_ptr = 1;

    return avpkt->size;
}

static av_cold int decode_close(AVCodecContext *avctx)
{
    LDACContext *s = avctx->priv_data;

    av_tx_uninit(&s->ch[0].tx_ctx[0]);
    av_tx_uninit(&s->ch[0].tx_ctx[1]);
    av_tx_uninit(&s->ch[1].tx_ctx[0]);
    av_tx_uninit(&s->ch[1].tx_ctx[1]);
    av_freep(&s->fdsp);

    return 0;
}

static av_cold void decode_flush(AVCodecContext *avctx)
{
    LDACContext *s = avctx->priv_data;

    for (int ch = 0; ch < MAX_CHANNELS; ch++)
        memset(s->ch[ch].hist, 0, 256 * sizeof(float));
}

const FFCodec ff_ldac_decoder = {
    .p.name         = "ldac",
    CODEC_LONG_NAME("LDAC"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_LDAC,
    .priv_data_size = sizeof(LDACContext),
    .init           = decode_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .flush          = decode_flush,
    .close          = decode_close,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_FLTP),
};
