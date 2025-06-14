/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/float_dsp.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/mem_internal.h"
#include "libavutil/tx.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#define BITSTREAM_READER_LE
#include "get_bits.h"

#define FRAME_LENGTH 960
#define FREQ_LENGTH 800
#define MAX_SEGMENT_LENGTH 480
#define NUM_TIME_SWITCHING_BLOCKS 4

#define NUM_MAP_BANDS 20

#define STOP_BAND      800
#define STOP_BAND4     200

#define SFM_G1         16
#define SFM_G2         8
#define SFM_G1G2       24
#define SFM_G3         12
#define SFM_N          36
#define SFM_GX         8
#define NB_SFM         44
#define WID_G1         8
#define WID_G2         16
#define WID_G3         24
#define WID_GX         32
#define NUMC_G1        128
#define NUMC_G1G2      256
#define NUMC_N         544
#define NB_VECT1       1
#define NB_VECT2       2
#define NB_VECT3       3
#define NB_VECTX       4

#define NUMC_G23       256
#define NUMC_G1SUB     32
#define NUMC_G1G2SUB   64
#define NUMC_G1G2G3SUB 136

#define QBIT_MAX1      5
#define QBIT_MAX2      9
#define OFFSETf        0.015625f
#define FCT_LVQ1f      1.1f
#define FCT_LVQ2f      6.0f

#define FLAGS_BITS     3
#define NORM0_BITS     5
#define NORMI_BITS     5
#define NUMNRMIBITS    215

#define NOALLGROUPS    0
#define ALLGROUPS      1

#define NOHUFCODE      0
#define HUFCODE        1
#define MIN16B    INT16_MIN
#define MAX16B    INT16_MAX

static const int8_t dic4[256][8] = {
    {-2,  0,  0,  0,  0,  0,  0,  0}, { 0, -2,  0,  0,  0,  0,  0,  0},
    { 0,  0, -2,  0,  0,  0,  0,  0}, { 0,  0,  0, -2,  0,  0,  0,  0},
    { 0,  0,  0,  0, -2,  0,  0,  0}, { 0,  0,  0,  0,  0, -2,  0,  0},
    { 0,  0,  0,  0,  0,  0, -2,  0}, { 0,  0,  0,  0,  0,  0,  0, -2},
    { 2,  0,  0,  0,  0,  0,  0,  0}, { 0,  2,  0,  0,  0,  0,  0,  0},
    { 0,  0,  2,  0,  0,  0,  0,  0}, { 0,  0,  0,  2,  0,  0,  0,  0},
    { 0,  0,  0,  0,  2,  0,  0,  0}, { 0,  0,  0,  0,  0,  2,  0,  0},
    { 0,  0,  0,  0,  0,  0,  2,  0}, { 0,  0,  0,  0,  0,  0,  0,  2},
    {-2, -2,  0,  0,  0,  0,  0,  0}, {-2,  0, -2,  0,  0,  0,  0,  0},
    {-2,  0,  0, -2,  0,  0,  0,  0}, {-2,  0,  0,  0, -2,  0,  0,  0},
    {-2,  0,  0,  0,  0, -2,  0,  0}, {-2,  0,  0,  0,  0,  0, -2,  0},
    {-2,  0,  0,  0,  0,  0,  0, -2}, { 0, -2, -2,  0,  0,  0,  0,  0},
    { 0, -2,  0, -2,  0,  0,  0,  0}, { 0, -2,  0,  0, -2,  0,  0,  0},
    { 0, -2,  0,  0,  0, -2,  0,  0}, { 0, -2,  0,  0,  0,  0, -2,  0},
    { 0, -2,  0,  0,  0,  0,  0, -2}, { 0,  0, -2, -2,  0,  0,  0,  0},
    { 0,  0, -2,  0, -2,  0,  0,  0}, { 0,  0, -2,  0,  0, -2,  0,  0},
    { 0,  0, -2,  0,  0,  0, -2,  0}, { 0,  0, -2,  0,  0,  0,  0, -2},
    { 0,  0,  0, -2, -2,  0,  0,  0}, { 0,  0,  0, -2,  0, -2,  0,  0},
    { 0,  0,  0, -2,  0,  0, -2,  0}, { 0,  0,  0, -2,  0,  0,  0, -2},
    { 0,  0,  0,  0, -2, -2,  0,  0}, { 0,  0,  0,  0, -2,  0, -2,  0},
    { 0,  0,  0,  0, -2,  0,  0, -2}, { 0,  0,  0,  0,  0, -2, -2,  0},
    { 0,  0,  0,  0,  0, -2,  0, -2}, { 0,  0,  0,  0,  0,  0, -2, -2},
    {-2,  2,  0,  0,  0,  0,  0,  0}, {-2,  0,  2,  0,  0,  0,  0,  0},
    {-2,  0,  0,  2,  0,  0,  0,  0}, {-2,  0,  0,  0,  2,  0,  0,  0},
    {-2,  0,  0,  0,  0,  2,  0,  0}, {-2,  0,  0,  0,  0,  0,  2,  0},
    {-2,  0,  0,  0,  0,  0,  0,  2}, { 0, -2,  2,  0,  0,  0,  0,  0},
    { 0, -2,  0,  2,  0,  0,  0,  0}, { 0, -2,  0,  0,  2,  0,  0,  0},
    { 0, -2,  0,  0,  0,  2,  0,  0}, { 0, -2,  0,  0,  0,  0,  2,  0},
    { 0, -2,  0,  0,  0,  0,  0,  2}, { 0,  0, -2,  2,  0,  0,  0,  0},
    { 0,  0, -2,  0,  2,  0,  0,  0}, { 0,  0, -2,  0,  0,  2,  0,  0},
    { 0,  0, -2,  0,  0,  0,  2,  0}, { 0,  0, -2,  0,  0,  0,  0,  2},
    { 0,  0,  0, -2,  2,  0,  0,  0}, { 0,  0,  0, -2,  0,  2,  0,  0},
    { 0,  0,  0, -2,  0,  0,  2,  0}, { 0,  0,  0, -2,  0,  0,  0,  2},
    { 0,  0,  0,  0, -2,  2,  0,  0}, { 0,  0,  0,  0, -2,  0,  2,  0},
    { 0,  0,  0,  0, -2,  0,  0,  2}, { 0,  0,  0,  0,  0, -2,  2,  0},
    { 0,  0,  0,  0,  0, -2,  0,  2}, { 0,  0,  0,  0,  0,  0, -2,  2},
    { 2, -2,  0,  0,  0,  0,  0,  0}, { 2,  0, -2,  0,  0,  0,  0,  0},
    { 2,  0,  0, -2,  0,  0,  0,  0}, { 2,  0,  0,  0, -2,  0,  0,  0},
    { 2,  0,  0,  0,  0, -2,  0,  0}, { 2,  0,  0,  0,  0,  0, -2,  0},
    { 2,  0,  0,  0,  0,  0,  0, -2}, { 0,  2, -2,  0,  0,  0,  0,  0},
    { 0,  2,  0, -2,  0,  0,  0,  0}, { 0,  2,  0,  0, -2,  0,  0,  0},
    { 0,  2,  0,  0,  0, -2,  0,  0}, { 0,  2,  0,  0,  0,  0, -2,  0},
    { 0,  2,  0,  0,  0,  0,  0, -2}, { 0,  0,  2, -2,  0,  0,  0,  0},
    { 0,  0,  2,  0, -2,  0,  0,  0}, { 0,  0,  2,  0,  0, -2,  0,  0},
    { 0,  0,  2,  0,  0,  0, -2,  0}, { 0,  0,  2,  0,  0,  0,  0, -2},
    { 0,  0,  0,  2, -2,  0,  0,  0}, { 0,  0,  0,  2,  0, -2,  0,  0},
    { 0,  0,  0,  2,  0,  0, -2,  0}, { 0,  0,  0,  2,  0,  0,  0, -2},
    { 0,  0,  0,  0,  2, -2,  0,  0}, { 0,  0,  0,  0,  2,  0, -2,  0},
    { 0,  0,  0,  0,  2,  0,  0, -2}, { 0,  0,  0,  0,  0,  2, -2,  0},
    { 0,  0,  0,  0,  0,  2,  0, -2}, { 0,  0,  0,  0,  0,  0,  2, -2},
    { 2,  2,  0,  0,  0,  0,  0,  0}, { 2,  0,  2,  0,  0,  0,  0,  0},
    { 2,  0,  0,  2,  0,  0,  0,  0}, { 2,  0,  0,  0,  2,  0,  0,  0},
    { 2,  0,  0,  0,  0,  2,  0,  0}, { 2,  0,  0,  0,  0,  0,  2,  0},
    { 2,  0,  0,  0,  0,  0,  0,  2}, { 0,  2,  2,  0,  0,  0,  0,  0},
    { 0,  2,  0,  2,  0,  0,  0,  0}, { 0,  2,  0,  0,  2,  0,  0,  0},
    { 0,  2,  0,  0,  0,  2,  0,  0}, { 0,  2,  0,  0,  0,  0,  2,  0},
    { 0,  2,  0,  0,  0,  0,  0,  2}, { 0,  0,  2,  2,  0,  0,  0,  0},
    { 0,  0,  2,  0,  2,  0,  0,  0}, { 0,  0,  2,  0,  0,  2,  0,  0},
    { 0,  0,  2,  0,  0,  0,  2,  0}, { 0,  0,  2,  0,  0,  0,  0,  2},
    { 0,  0,  0,  2,  2,  0,  0,  0}, { 0,  0,  0,  2,  0,  2,  0,  0},
    { 0,  0,  0,  2,  0,  0,  2,  0}, { 0,  0,  0,  2,  0,  0,  0,  2},
    { 0,  0,  0,  0,  2,  2,  0,  0}, { 0,  0,  0,  0,  2,  0,  2,  0},
    { 0,  0,  0,  0,  2,  0,  0,  2}, { 0,  0,  0,  0,  0,  2,  2,  0},
    { 0,  0,  0,  0,  0,  2,  0,  2}, { 0,  0,  0,  0,  0,  0,  2,  2},
    {-1, -1,  1,  1,  1,  1,  1,  1}, {-1,  1, -1,  1,  1,  1,  1,  1},
    {-1,  1,  1, -1,  1,  1,  1,  1}, {-1,  1,  1,  1, -1,  1,  1,  1},
    {-1,  1,  1,  1,  1, -1,  1,  1}, {-1,  1,  1,  1,  1,  1, -1,  1},
    {-1,  1,  1,  1,  1,  1,  1, -1}, { 1, -1, -1,  1,  1,  1,  1,  1},
    { 1, -1,  1, -1,  1,  1,  1,  1}, { 1, -1,  1,  1, -1,  1,  1,  1},
    { 1, -1,  1,  1,  1, -1,  1,  1}, { 1, -1,  1,  1,  1,  1, -1,  1},
    { 1, -1,  1,  1,  1,  1,  1, -1}, { 1,  1, -1, -1,  1,  1,  1,  1},
    { 1,  1, -1,  1, -1,  1,  1,  1}, { 1,  1, -1,  1,  1, -1,  1,  1},
    { 1,  1, -1,  1,  1,  1, -1,  1}, { 1,  1, -1,  1,  1,  1,  1, -1},
    { 1,  1,  1, -1, -1,  1,  1,  1}, { 1,  1,  1, -1,  1, -1,  1,  1},
    { 1,  1,  1, -1,  1,  1, -1,  1}, { 1,  1,  1, -1,  1,  1,  1, -1},
    { 1,  1,  1,  1, -1, -1,  1,  1}, { 1,  1,  1,  1, -1,  1, -1,  1},
    { 1,  1,  1,  1, -1,  1,  1, -1}, { 1,  1,  1,  1,  1, -1, -1,  1},
    { 1,  1,  1,  1,  1, -1,  1, -1}, { 1,  1,  1,  1,  1,  1, -1, -1},
    {-1, -1, -1, -1,  1,  1,  1,  1}, {-1, -1, -1,  1, -1,  1,  1,  1},
    {-1, -1, -1,  1,  1, -1,  1,  1}, {-1, -1, -1,  1,  1,  1, -1,  1},
    {-1, -1, -1,  1,  1,  1,  1, -1}, {-1, -1,  1, -1, -1,  1,  1,  1},
    {-1, -1,  1, -1,  1, -1,  1,  1}, {-1, -1,  1, -1,  1,  1, -1,  1},
    {-1, -1,  1, -1,  1,  1,  1, -1}, {-1, -1,  1,  1, -1, -1,  1,  1},
    {-1, -1,  1,  1, -1,  1, -1,  1}, {-1, -1,  1,  1, -1,  1,  1, -1},
    {-1, -1,  1,  1,  1,  1, -1, -1}, {-1, -1,  1,  1,  1, -1,  1, -1},
    {-1, -1,  1,  1,  1, -1, -1,  1}, {-1,  1, -1, -1, -1,  1,  1,  1},
    {-1,  1, -1, -1,  1, -1,  1,  1}, {-1,  1, -1, -1,  1,  1, -1,  1},
    {-1,  1, -1, -1,  1,  1,  1, -1}, {-1,  1, -1,  1, -1, -1,  1,  1},
    {-1,  1, -1,  1, -1,  1, -1,  1}, {-1,  1, -1,  1, -1,  1,  1, -1},
    {-1,  1, -1,  1,  1,  1, -1, -1}, {-1,  1, -1,  1,  1, -1,  1, -1},
    {-1,  1, -1,  1,  1, -1, -1,  1}, {-1,  1,  1,  1,  1, -1, -1, -1},
    {-1,  1,  1,  1, -1,  1, -1, -1}, {-1,  1,  1,  1, -1, -1,  1, -1},
    {-1,  1,  1,  1, -1, -1, -1,  1}, {-1,  1,  1, -1, -1, -1,  1,  1},
    {-1,  1,  1, -1, -1,  1, -1,  1}, {-1,  1,  1, -1, -1,  1,  1, -1},
    {-1,  1,  1, -1,  1,  1, -1, -1}, {-1,  1,  1, -1,  1, -1,  1, -1},
    {-1,  1,  1, -1,  1, -1, -1,  1}, { 1,  1,  1,  1, -1, -1, -1, -1},
    { 1,  1,  1, -1,  1, -1, -1, -1}, { 1,  1,  1, -1, -1,  1, -1, -1},
    { 1,  1,  1, -1, -1, -1,  1, -1}, { 1,  1,  1, -1, -1, -1, -1,  1},
    { 1,  1, -1,  1,  1, -1, -1, -1}, { 1,  1, -1,  1, -1,  1, -1, -1},
    { 1,  1, -1,  1, -1, -1,  1, -1}, { 1,  1, -1,  1, -1, -1, -1,  1},
    { 1,  1, -1, -1,  1,  1, -1, -1}, { 1,  1, -1, -1,  1, -1,  1, -1},
    { 1,  1, -1, -1,  1, -1, -1,  1}, { 1,  1, -1, -1, -1, -1,  1,  1},
    { 1,  1, -1, -1, -1,  1, -1,  1}, { 1,  1, -1, -1, -1,  1,  1, -1},
    { 1, -1,  1,  1,  1, -1, -1, -1}, { 1, -1,  1,  1, -1,  1, -1, -1},
    { 1, -1,  1,  1, -1, -1,  1, -1}, { 1, -1,  1,  1, -1, -1, -1,  1},
    { 1, -1,  1, -1,  1,  1, -1, -1}, { 1, -1,  1, -1,  1, -1,  1, -1},
    { 1, -1,  1, -1,  1, -1, -1,  1}, { 1, -1,  1, -1, -1, -1,  1,  1},
    { 1, -1,  1, -1, -1,  1, -1,  1}, { 1, -1,  1, -1, -1,  1,  1, -1},
    { 1, -1, -1, -1, -1,  1,  1,  1}, { 1, -1, -1, -1,  1, -1,  1,  1},
    { 1, -1, -1, -1,  1,  1, -1,  1}, { 1, -1, -1, -1,  1,  1,  1, -1},
    { 1, -1, -1,  1,  1,  1, -1, -1}, { 1, -1, -1,  1,  1, -1,  1, -1},
    { 1, -1, -1,  1,  1, -1, -1,  1}, { 1, -1, -1,  1, -1, -1,  1,  1},
    { 1, -1, -1,  1, -1,  1, -1,  1}, { 1, -1, -1,  1, -1,  1,  1, -1},
    { 1,  1, -1, -1, -1, -1, -1, -1}, { 1, -1,  1, -1, -1, -1, -1, -1},
    { 1, -1, -1,  1, -1, -1, -1, -1}, { 1, -1, -1, -1,  1, -1, -1, -1},
    { 1, -1, -1, -1, -1,  1, -1, -1}, { 1, -1, -1, -1, -1, -1,  1, -1},
    { 1, -1, -1, -1, -1, -1, -1,  1}, {-1,  1,  1, -1, -1, -1, -1, -1},
    {-1,  1, -1,  1, -1, -1, -1, -1}, {-1,  1, -1, -1,  1, -1, -1, -1},
    {-1,  1, -1, -1, -1,  1, -1, -1}, {-1,  1, -1, -1, -1, -1,  1, -1},
    {-1,  1, -1, -1, -1, -1, -1,  1}, {-1, -1,  1,  1, -1, -1, -1, -1},
    {-1, -1,  1, -1,  1, -1, -1, -1}, {-1, -1,  1, -1, -1,  1, -1, -1},
    {-1, -1,  1, -1, -1, -1,  1, -1}, {-1, -1,  1, -1, -1, -1, -1,  1},
    {-1, -1, -1,  1,  1, -1, -1, -1}, {-1, -1, -1,  1, -1,  1, -1, -1},
    {-1, -1, -1,  1, -1, -1,  1, -1}, {-1, -1, -1,  1, -1, -1, -1,  1},
    {-1, -1, -1, -1,  1,  1, -1, -1}, {-1, -1, -1, -1,  1, -1,  1, -1},
    {-1, -1, -1, -1,  1, -1, -1,  1}, {-1, -1, -1, -1, -1,  1,  1, -1},
    {-1, -1, -1, -1, -1,  1, -1,  1}, {-1, -1, -1, -1, -1, -1,  1,  1},
    {-1, -1, -1, -1, -1, -1, -1, -1}, { 1,  1,  1,  1,  1,  1,  1,  1}
};

static const int16_t RV[10] = {
    0, 1, 4, 8, 16, 32, 64, 128, 256, 512
};

static const int16_t FacLVQ2Qv[10] = {
    0, 0, 11, 10, 9, 8, 7, 6, 5, 4
};

static const int16_t FacLVQ2Mask[10] = {
    0, 0, 2047, 1023, 511, 255, 127, 63, 31, 15
};

static const int16_t FacLVQ2HalfQv[10] = {
    0, 0, 1024, 512, 256, 128, 64, 32, 16, 8
};

static const float dicn[40]={ /* Codebook for quantization of norms */
    131072.0f, 92681.900024f,
    65536.0f,  46340.950012f,
    32768.0f,  23170.475006f,
    16384.0f,  11585.237503f,
    8192.0f,   5792.618751f,
    4096.0f,   2896.309376f,
    2048.0f,   1448.154688f,
    1024.0f,   724.077344f,
    512.0f,    362.038672f,
    256.0f,    181.019336f,
    128.0f,    90.509668f,
    64.0f,     45.254834f,
    32.0f,     22.627417f,
    16.0f,     11.313708f,
    8.0f,      5.656854f,
    4.0f,      2.828427f,
    2.0f,      1.414214f,
    1.0f,      0.707107f,
    0.5f,      0.353553f,
    0.25f,     0.176777f
};

static const int16_t huffoffset[6] = {
    0, 0, 0, 4, 12, 28,
};

static const int16_t sfm_start[44] = {
    0,   8,   16,  24,  32,  40,  48,  56,  64,  72,  80,
    88,  96,  104, 112, 120, 128, 144, 160, 176, 192, 208,
    224, 240, 256, 280, 304, 328, 352, 376, 400, 424, 448,
    472, 496, 520, 544, 576, 608, 640, 672, 704, 736, 768,
};

static const int16_t sfm_end[44] = {
    8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,
    96, 104, 112, 120, 128, 144, 160, 176, 192, 208, 224,
    240, 256, 280, 304, 328, 352, 376, 400, 424, 448, 472,
    496, 520, 544, 576, 608, 640, 672, 704, 736, 768, 800,
};

static const int16_t sfmsize[44] = {
    8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,
    16, 16, 16, 16, 16, 16, 16, 16,
    24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24,
    32, 32, 32, 32, 32, 32, 32, 32
};

static const int16_t sfm_width[20] = {
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 8
};

static const int16_t a[20] = {
    8, 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 5, 7, 11
};

static const uint8_t huffsizn[32] = {
    7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 4, 4, 3, 3,
    3, 3, 4, 4, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7
};

static const int16_t huffsizc[60] = {
    1, 3, 3, 2,
    2, 2, 4, 5, 5, 4, 4, 2,
    2, 3, 4, 4, 5, 5, 5, 6, 6, 5, 5, 5, 5, 4, 4, 3,
    2, 3, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 4, 3
};

static const int8_t dicnlg2[40] = {
    34, 33, 32, 31, 30, 29, 28, 27, 26, 25,
    24, 23, 22, 21, 20, 19, 18, 17, 16, 15,
    14, 13, 12, 11, 10,  9,  8,  7,  6,  5,
    4,  3,  2,  1,  0, -1, -2, -3, -4, -5
};

typedef struct ChannelState {
    DECLARE_ALIGNED(32, float, imdct_in)[FRAME_LENGTH];
    DECLARE_ALIGNED(32, float, imdct_out)[FRAME_LENGTH];
    DECLARE_ALIGNED(32, float, imdct_prev)[FRAME_LENGTH];
    DECLARE_ALIGNED(32, float, imdct_sout)[240];
    DECLARE_ALIGNED(32, float, imdct_sprev)[240];

    float audio_q_norm[FREQ_LENGTH];
    float codebook[FREQ_LENGTH];
    float coefs_short[STOP_BAND];
    int16_t bitalloc[NB_SFM];
    int16_t ynrm[NB_SFM];
    int16_t ycof[STOP_BAND];

    float   old_hpfilt_in;
    float   old_hpfilt_out;
    float   energy_lt;
    int16_t transient_hang_over;

    int16_t nf_idx;
    int is_transient;
    int old_is_transient;
} ChannelState;

typedef struct G719Context {
    GetBitContext gb;

    int num_bits;
    int num_bits_spectrum_stationary;
    int num_bits_spectrum_transient;

    ChannelState       ch[2];

    av_tx_fn           ltx_fn;
    av_tx_fn           stx_fn;
    AVTXContext       *ltx_ctx;
    AVTXContext       *stx_ctx;
    AVFloatDSPContext *fdsp;

    DECLARE_ALIGNED(32, float, window)[960];
    DECLARE_ALIGNED(32, float, short_window)[240];
} G719Context;

static av_cold int decode_init(AVCodecContext *avctx)
{
    G719Context *s = avctx->priv_data;
    float scale = 1.f / (32768.f * 22.f);
    int ret;

    avctx->sample_fmt = AV_SAMPLE_FMT_FLTP;

    if (avctx->ch_layout.nb_channels <= 0 || avctx->ch_layout.nb_channels > 2)
        return AVERROR(EINVAL);

    s->num_bits                     = (avctx->block_align / avctx->ch_layout.nb_channels) * 8;
    s->num_bits_spectrum_stationary = s->num_bits - 3;
    s->num_bits_spectrum_transient  = s->num_bits - 1;

    s->fdsp = avpriv_float_dsp_alloc(avctx->flags & AV_CODEC_FLAG_BITEXACT);
    if (!s->fdsp)
        return AVERROR(ENOMEM);

    for (int i = 0; i < 960; i++) {
        float angle = ((i + 0.5f) * M_PI_2) / 960.f;
        s->window[i] = sinf(angle);
    }

    for (int i = 0; i < 240; i++) {
        float angle = ((i + 0.5f) * M_PI_2) / 240.f;
        s->short_window[i] = sinf(angle);
    }

    ret = av_tx_init(&s->ltx_ctx, &s->ltx_fn, AV_TX_FLOAT_MDCT, 1, 960, &scale, 0);
    if (ret < 0)
        return ret;
    return av_tx_init(&s->stx_ctx, &s->stx_fn, AV_TX_FLOAT_MDCT, 1, 240, &scale, 0);
}

static void bits2idxn(AVCodecContext *avctx,
                      int16_t N, int16_t *x)
{
    G719Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;

    *x = 0;
    for (int i = 0; i < N; i++) {
        *x <<= 1;
        if (get_bits1(gb))
            *x = *x + 1;
    }
}

static void hdecnrm(AVCodecContext *avctx, int16_t N, int16_t *index)
{
    G719Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    int16_t i, j, k, n, m;
    int16_t temp;
    int16_t *pidx;

    pidx  = index;

    m = N - 1;
    for (i=0; i < m; i++) {
        j = 0;
        k = 0;
        if (get_bits1(gb))
            j = 1;
        if (get_bits1(gb))
            k = 1;
        n = j * 2 + k;
        j = j * 4;
        temp = 16 + n - j;
        if (get_bits1(gb)) {
            temp = 12 + n + j;
            if (get_bits1(gb))
            {
                j = get_bits1(gb);
                temp = 8 + n + j * 12;
                if (get_bits1(gb)) {
                    temp = n;
                    if (get_bits1(gb))
                        temp = n + 4;
                    if (j!=0)
                        temp += 24;
                }
            }
        }
        *pidx++ = temp;
    }
}

static void recovernorm(int16_t *idxbuf, int16_t *ynrm, int16_t *normqlg2)
{
    int16_t *pidx, *pnormq;

    for (int i=0; i<2; i++) {
        pidx = ynrm + i;
        pnormq = normqlg2 + i;
        *pidx = idxbuf[i];
        *pnormq = dicnlg2[*pidx];
        pidx += 2;
        pnormq += 2;
        *pidx = idxbuf[21-i];
        *pnormq = dicnlg2[*pidx];
        pidx += 2;
        pnormq += 2;
        *pidx = idxbuf[22+i];
        *pnormq = dicnlg2[*pidx];
        pidx += 2;
        pnormq += 2;
        *pidx = idxbuf[43-i];
        *pnormq = dicnlg2[*pidx];
        pidx += 2;
        pnormq += 2;
        *pidx  = idxbuf[i+2];
        *pnormq = dicnlg2[*pidx];
        pidx += 2;
        pnormq += 2;
        *pidx = idxbuf[19-i];
        *pnormq = dicnlg2[*pidx];
        pidx += 2;
        pnormq += 2;
        *pidx = idxbuf[24+i];
        *pnormq = dicnlg2[*pidx];
        pidx += 2;
        pnormq += 2;
        *pidx = idxbuf[41-i];
        *pnormq = dicnlg2[*pidx];
    }

    for (int i = 4, j = 0; i < NB_SFM / 4; i++, j+=4) {
        ynrm[16+j+0] = idxbuf[i];
        normqlg2[16+j+0] = dicnlg2[ynrm[16+j+0]];
        ynrm[16+j+1] = idxbuf[21-i];
        normqlg2[16+j+1] = dicnlg2[ynrm[16+j+1]];
        ynrm[16+j+2] = idxbuf[22+i];
        normqlg2[16+j+2] = dicnlg2[ynrm[16+j+2]];
        ynrm[16+j+3] = idxbuf[43-i];
        normqlg2[16+j+3] = dicnlg2[ynrm[16+j+3]];
    }
}

static void sfm2mqb(int16_t spe[], int16_t spe2q[])
{
    int16_t tmp;

    for (int i = 0; i < 10; i++)
        spe2q[i] = spe[i] + 3;

    spe2q[10] = ((spe[10] + spe[11]) >> 1) + 4;
    spe2q[11] = ((spe[12] + spe[13]) >> 1) + 4;
    spe2q[12] = ((spe[14] + spe[15]) >> 1) + 4;

    spe2q[13] = ((spe[16] + spe[17]) >> 1) + 5;
    spe2q[14] = ((spe[18] + spe[19]) >> 1) + 5;

    tmp = 0;
    for (int i = 20; i < 24; i++)
        tmp += spe[i];
    spe2q[15] = (int16_t)(((int)tmp * 8192L) >> 15) + 6;

    tmp = 0;
    for (int i = 24; i < 27; i++)
        tmp += spe[i];
    spe2q[16] = (int16_t)(((int)tmp * 10923L) >> 15) + 6;

    tmp = 0;
    for (int i = 27; i < 30; i++)
        tmp += spe[i];
    spe2q[17] = (int16_t)(((int)tmp * 10923L) >> 15) + 6;

    tmp = 0;
    for (int i = 30; i < 35; i++)
        tmp += spe[i];
    spe2q[18] = (int16_t)(((int)tmp * 6553L) >> 15) + 7;

    tmp = 0;
    for (int i= 35; i < 44; i++)
        tmp += spe[i];
    spe2q[19] = (int16_t)(((int)tmp * 3641L) >> 15) + 8;
}

static void mqb2sfm(int16_t spe2q[],int16_t spe[])
{
    for (int i = 0; i < 10; i++)
        spe[i] = spe2q[i];

    spe[10] = spe2q[10];
    spe[11] = spe2q[10];

    spe[12] = spe2q[11];
    spe[13] = spe2q[11];

    spe[14] = spe2q[12];
    spe[15] = spe2q[12];

    spe[16] = spe2q[13];
    spe[17] = spe2q[13];

    spe[18] = spe2q[14];
    spe[19] = spe2q[14];

    for (int i = 20; i < 24; i++)
        spe[i] = spe2q[15];

    for (int i = 24; i < 27; i++)
        spe[i] = spe2q[16];

    for (int i = 27; i < 30; i++)
        spe[i] = spe2q[17];

    for (int i = 30; i < 35; i++)
        spe[i] = spe2q[18];

    for (int i = 35; i < 44; i++)
        spe[i] = spe2q[19];
}

static void map_quant_weight(int16_t normqlg2[], int16_t wnorm[], int is_transient)
{
    int16_t tmp16;
    int16_t spe2q[NUM_MAP_BANDS];
    int16_t spe[NB_SFM];

    int16_t spe2q_max;
    int16_t spe2q_min;
    int16_t norm_max;
    int16_t shift;
    int16_t sum;

    if (is_transient) {
        for (int sfm = 0; sfm < NB_SFM; sfm += 4) {
            int16_t sum = 0;
            for (int k = 0; k < 4; k++)
                sum = sum + normqlg2[sfm+k];
            sum = sum >> 2;

            for (int k=0; k < 4; k++)
                spe[sfm +k] = sum;
        }
    } else {
        for (int sfm = 0; sfm < NB_SFM; sfm++)
            spe[sfm] = normqlg2[sfm];
    }

    sfm2mqb(spe, spe2q);

    for (int sfm = 0; sfm < NUM_MAP_BANDS; sfm++)
        spe2q[sfm] = spe2q[sfm] - 10;

    /* spectral smoothing */
    for (int sfm = 1; sfm < NUM_MAP_BANDS; sfm++) {
        tmp16 = spe2q[sfm-1] - 4;
        if (spe2q[sfm] < tmp16)
            spe2q[sfm] = tmp16;
    }

    for (int sfm = NUM_MAP_BANDS-2; sfm >= 0; sfm--) {
        tmp16 = spe2q[sfm+1] - 8;
        if (spe2q[sfm] < tmp16)
            spe2q[sfm] = tmp16;
    }

    for (int sfm = 0; sfm < NUM_MAP_BANDS; sfm++) {
        if (spe2q[sfm] < a[sfm])
            spe2q[sfm] = a[sfm];
    }

    spe2q_max = MIN16B;
    spe2q_min = MAX16B;
    for (int sfm = 0; sfm < NUM_MAP_BANDS; sfm++) {
        spe2q[sfm] = sfm_width[sfm] - spe2q[sfm];
        spe2q_max = FFMAX(spe2q_max, spe2q[sfm]);
        spe2q_min = FFMIN(spe2q_min, spe2q[sfm]);
    }

    for (int sfm = 0; sfm < NUM_MAP_BANDS; sfm++)
        spe2q[sfm] = spe2q[sfm] - spe2q_min;

    spe2q_max = spe2q_max - spe2q_min;

    if (spe2q_max==0) {
        norm_max = 0;
    } else {
        if (spe2q_max < 0)
            spe2q_max = ~spe2q_max;
        for (norm_max = 0; spe2q_max < 0x4000; norm_max++)
            spe2q_max <<= 1;
    }

    shift = norm_max - 13;
    for (int sfm = 0; sfm < NUM_MAP_BANDS; sfm++) {
        if (shift < 0)
            spe2q[sfm] = spe2q[sfm] >> (-shift);
        else
            spe2q[sfm] = spe2q[sfm] << shift;
    }

    mqb2sfm(spe2q,spe);

    if (is_transient) {
        for (int sfm = 0; sfm < NB_SFM; sfm+=4) {
            sum = 0;
            for (int k=0; k < 4; k++)
                sum = sum + spe[sfm+k];
            sum = sum >> 2;

            for (int k=0; k < 4; k++)
                spe[sfm +k] = sum;
        }
    }

    for (int sfm = 0; sfm < NB_SFM; sfm++)
        wnorm[sfm] = spe[sfm] + normqlg2[sfm];
}

static void bitalloc(int16_t *y, int16_t *idx, int16_t sum, int16_t N, int16_t M, int16_t *r)
{
    int16_t j, k, n, v, im;
    int16_t diff, temp;

    im = 1;
    diff = sum;
    n = sum >> 3;
    for (int i = 0; i<n; i++) {
        k = 0;
        temp = y[0];
        for (int m = 1; m <= im; m++) {
            if (temp < y[m]) {
                temp = y[m];
                k = m;
            }
        }

        if (k==im)
            im++;
        j = idx[k];
        if ((sum>=sfmsize[j]) && (r[j]<M)) {
            y[k] -= 2;
            r[j]++;
            if (r[j] >= M)
                y[k] = MIN16B;
            sum -= sfmsize[j];
        } else {
            y[k] = MIN16B;
            k++;
            if (k == im)
                im++;
        }

        if ((sum<WID_G1) || (diff==sum))
            break;
        diff = sum;
        v = N - 2;
        if (k>v) {
            for (int i = 0; i < N; i++) {
                if (y[i] > MIN16B) {
                    im = i + 1;
                    break;
                }
            }
        }
    }

    if (sum >= WID_G2) {
        for (int i=0; i<N; i++) {
            j = idx[i];
            if ((j>=SFM_G1) && (j<SFM_G1G2) && (r[j]==0)) {
                r[j] = 1;
                sum -= WID_G2;
                if (sum < WID_G2)
                    break;
            }
        }
    }

    if (sum >= WID_G2) {
        for (int i = 0; i < N; i++) {
            j = idx[i];
            if ((j>=SFM_G1) && (j<SFM_G1G2) && (r[j]==1)) {
                r[j] = 2;
                sum -= WID_G2;
                if (sum<WID_G2)
                    break;
            }
        }
    }

    if (sum >= WID_G1) {
        for (int i=0; i<N; i++) {
            j = idx[i];
            if ((j<SFM_G1) && (r[j]==0)) {
                r[j] = 1;
                sum -= WID_G1;
                if (sum<WID_G1)
                    break;
            }
        }
    }

    if (sum >= WID_G1) {
        for (int i = 0; i < N; i++) {
            j = idx[i];
            if ((j<SFM_G1) && (r[j]==1)) {
                r[j] = 2;
                sum -= WID_G1;
                if (sum<WID_G1)
                    break;
            }
        }
    }
}

static void reordvct(int16_t *y, int16_t N, int16_t *idx)
{
    const int n = N - 1;

    for (int i = 0; i < n; i++) {
        int16_t im = i, k = i + 1;

        for (int j = k; j < N; j++) {
            int16_t temp = y[im] - y[j];
            if (temp < 0)
                im = j;
        }

        FFSWAP(int16_t, y[i], y[im]);
        FFSWAP(int16_t, idx[i], idx[im]);
    }
}

static void bits2idxc(AVCodecContext *avctx, int16_t N, int16_t L, int16_t *x)
{
    G719Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    int16_t m, n;

    if (L == 1) {
        n = 1;
        m = N;
    } else {
        n = N;
        m = L;
    }

    for (int k = 0; k < n; k++) {
        x[k] = 0;
        for (int i = 0; i < m; i++) {
            int16_t temp = x[k] << 1;

            temp += get_bits1(gb);
            x[k] = temp;
        }
    }
}

static void hdec2blvq(AVCodecContext *avctx,
                      int16_t N, int16_t *index)
{
    G719Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;

    for (int i = 0; i < N; i++) {
        int16_t temp = 0;
        if (get_bits1(gb)) {
            temp = 3;
            if (get_bits1(gb))
                temp = 1 + get_bits1(gb);
        }
        index[i] = temp;
    }
}

static void hdec4blvq(AVCodecContext *avctx, int16_t N, int16_t *index)
{
    G719Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;

    for (int i = 0; i<N; i++) {
        int16_t temp = 0, k;

        k = 2 * get_bits1(gb);
        k += get_bits1(gb);
        if (k != 0) {
            int16_t j = get_bits1(gb);

            temp = 1;
            if (j != 0)
                temp = 15;
            if (k != 3) {
                int16_t m = j * 2;

                m += get_bits1(gb);
                temp = m;
                if (j == 0)
                    temp = m + 13;
                if (k != 1) {
                    m = m * 2;
                    m += get_bits1(gb);
                    temp = m;
                    if (j == 0)
                        temp = m + 9;
                    if (m == 7) {
                        temp = m;
                        if (get_bits1(gb))
                            temp = m + 1;
                    }
                }
            }
        }

        index[i] = temp;
    }
}

static void hdec3blvq(AVCodecContext *avctx, int16_t N, int16_t *index)
{
    G719Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;

    for (int i = 0; i < N; i++) {
        int m, j = get_bits1(gb);
        int k = j * 2 + get_bits1(gb);
        int temp = j * 4 + k;
        if (k == 2) {
            j = get_bits1(gb);
            m = j * 2 + get_bits1(gb);
            temp = j * 2 + m + 1;
            if (m == 0)
                temp = 3 + get_bits1(gb);
        }

        index[i] = temp;
    }
}

static void hdec5blvq(AVCodecContext *avctx,
                      int16_t N, int16_t *index)
{
    G719Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;

    for (int i = 0; i < N; i++) {
        int temp = 0, n = 2 * get_bits1(gb);
        n += get_bits1(gb);
        if (n != 0) {
            int j = get_bits1(gb) * 31;
            temp = 1;
            if (n != 1) {
                int k = get_bits1(gb);
                if (n == 2) {
                    temp = 2 + k * 28;
                    if (j) {
                        temp = 3 + get_bits1(gb);
                        temp += 25 * (!!k);
                    }
                } else {
                    int m = 2 * get_bits1(gb);

                    m += get_bits1(gb);
                    temp = m + 5;
                    temp += 19 * (!!k);
                    if (j) {
                        m = k * 4 + m;
                        temp = 23;
                        if (m != 7) {
                            m *= 2;
                            m += get_bits1(gb);
                            temp = m + 9;
                        }
                    }
                }
            }
        }

        index[i] = temp;
    }
}

static int16_t unpackc(AVCodecContext *avctx,
                       int16_t *R,
                       int16_t flag, int16_t rv,
                       int16_t N1, int16_t N2,
                       int16_t L, int16_t *y)
{
    int16_t offset, nb_vecs = L >> 3, length = 0;

    if (flag == NOHUFCODE) {
        for (int n = N1; n < N2; n++) {
            int16_t v = R[n];

            if (v > 1) {
                bits2idxc(avctx, L, v, &y[rv]);
                length += v * L;
            } else if (v==1) {
                int k = rv;
                for (int i = 0; i < nb_vecs; i++) {
                    bits2idxc(avctx, 8, 1, &y[k]);
                    k += 8;
                }
                length += L;
            }
            rv += L;
        }
    } else {
        int16_t r = 0;
        int16_t hcode_l = 0;
        for (int n = N1; n < N2; n++) {
            int16_t v = R[n];
            if (v > QBIT_MAX1) {
                bits2idxc(avctx, L, v, &y[rv]);
                r += v * L;
            } else if (v > 1) {
                if (v == 2)
                    hdec2blvq(avctx, L, &y[rv]);
                else if (v == 3)
                    hdec3blvq(avctx, L, &y[rv]);
                else if (v == 4)
                    hdec4blvq(avctx, L, &y[rv]);
                else
                    hdec5blvq(avctx, L, &y[rv]);
                offset = huffoffset[v];
                for (int i = 0; i < L; i++) {
                    int k = rv + i;
                    int j = offset + y[k];
                    hcode_l += huffsizc[j];
                }
            } else if (v == 1) {
                int k = rv;
                for (int i = 0; i < nb_vecs; i++, k += 8)
                    bits2idxc(avctx, 8, 1, &y[k]);
                r += L;
            }

            rv += L;
        }

        length = hcode_l + r;
    }

    return length;
}

static void codesearch(int16_t *x, int16_t *C, int16_t R)
{
    int16_t sum;

    sum = 0;
    for (int i = 0; i < 8; i++) {
        int16_t temp = x[i] & FacLVQ2Mask[R];
        C[i] = x[i] >> FacLVQ2Qv[R];
        if ((temp>FacLVQ2HalfQv[R]) || ((temp == FacLVQ2HalfQv[R]) && (x[i] < 0)))
            C[i] += 1;
        sum += C[i];
    }

    if (sum & 1) {
        int16_t em = 0, e[8];
        int j = 0;

        for (int i = 0; i < 8; i++) {
            int16_t temp = C[i] << FacLVQ2Qv[R];
            e[i] = x[i] - temp;
            temp = e[i];
            if (e[i] < 0)
                temp = -e[i];
            if (em < temp) {
                em = temp;
                j = i;
            }
        }

        if (e[j] >= 0)
            C[j] += 1;
        else
            C[j] -= 1;
    }
}

static void idx2code(int16_t *k, int16_t *y, int16_t R)
{
    int16_t m, tmp;
    int16_t v[8], z[8];

    tmp = FacLVQ2Qv[R] - R;
    m = k[0] << 1;
    for (int i = 1; i < 8; i++)
        m += k[i];
    if (tmp < 0)
        z[0] = m >> (-tmp);
    else
        z[0] = m << tmp;
    for (int i = 1; i < 8; i++) {
        if (tmp < 0)
            z[i] = k[i] >> (-tmp);
        else
            z[i] = k[i] << tmp;
    }
    codesearch(z, v, R);
    y[0] = m - (v[0] << R);
    for (int i = 1; i < 8; i++)
        y[i] = k[i] - (v[i] << R);
}

static void dqcoefs(int16_t *y, int16_t *idxnrm, int16_t *R, int16_t N1,
                    int16_t N2, int16_t L, float *coefs, float *coefs_norm)
{
    int16_t v, rv;
    int16_t nb_vecs, pre_idx;
    int16_t x[8];
    float normq, factor;
    int16_t *pidx;
    float *pcoefs, *pcoefs_norm;

    pidx = y;
    pcoefs = coefs;
    pcoefs_norm = coefs_norm;
    nb_vecs = L >> 3;
    for (int n = N1; n < N2; n++) {
        if (!(idxnrm[n] < 40 && idxnrm[n] >= 0))
            return;
        normq = dicn[idxnrm[n]];
        v = R[n];
        if (v > 1) {
            rv = RV[v];
            factor = FCT_LVQ2f / (float)rv;
            for (int i = 0; i < nb_vecs; i++) {
                idx2code(pidx, x, v);
                for (int j = 0; j < 8; j++) {
                    *pcoefs_norm = x[j] * factor + OFFSETf;
                    *pcoefs = (*pcoefs_norm) * normq;
                    pcoefs_norm++;
                    pcoefs++;
                }
                pidx += 8;
            }
        } else if (v == 1) {
            pre_idx = MAX16B;
            for (int i=0; i<nb_vecs; i++) {
                if ((pre_idx<128) && (*pidx<16)) {
                    for (int j = 0; j < 8; j++) {
                        *pcoefs_norm = OFFSETf;
                        *pcoefs = (*pcoefs_norm) * normq;
                        pcoefs_norm++;
                        pcoefs++;
                    }
                } else {
                    for (int j = 0; j < 8; j++) {
                        *pcoefs_norm = (float)dic4[*pidx][j] / FCT_LVQ1f + OFFSETf;
                        *pcoefs = (*pcoefs_norm) * normq;
                        pcoefs_norm++;
                        pcoefs++;
                    }
                }
                pre_idx = *pidx;
                pidx += 8;
            }
        } else {
            for (int i = 0; i < L; i++) {
                *pcoefs_norm = 0.f;
                pcoefs_norm++;
                *pcoefs = 0.f;
                pcoefs++;
            }
            pidx += L;
        }
    }
}

static void dprocnf(AVCodecContext *avctx, int16_t *y,
                    int16_t idxnrm, int16_t nb_vecs,
                    float *coefs, float* coefs_norm)
{
    const float normq = dicn[idxnrm];
    int16_t pre_idx = MAX16B;

    for (int i = 0; i < nb_vecs; i++) {
        bits2idxc(avctx, 8, 1, y);
        if ((pre_idx < 128) && (*y < 16)) {
            for (int j = 0; j < 8; j++) {
                *coefs_norm = OFFSETf;
                *coefs++ = (*coefs_norm++) * normq;
            }
        } else {
            for (int j=0; j<8; j++) {
                *coefs_norm = (float)dic4[*y][j] / FCT_LVQ1f + OFFSETf;
                *coefs++ = (*coefs_norm++) * normq;
            }
        }

        pre_idx = *y;
        y += 8;
    }
}

static void dprocnobitsbfm(AVCodecContext *avctx, int16_t *R, int16_t *idx,
                           int16_t *ynrm, int16_t *ycof,
                           float *coefsq, float *coefsq_norm,
                           int16_t nb_sfm, int16_t diff)
{
    int16_t im = MAX16B, k = nb_sfm - 1;

    for (int i = k; i > 0; i--) {
        if (R[idx[i]] == 0)
            im = i;
    }

    for (int i = im; i < nb_sfm; i++) {
        int16_t m = idx[i], offset;

        if (R[m])
            continue;

        if (m < SFM_G1) {
            if (diff >= WID_G1) {
                R[m] = 1;
                offset = m * WID_G1;
                dprocnf(avctx, &ycof[offset], ynrm[m], NB_VECT1, &coefsq[offset], &coefsq_norm[offset]);
                diff -= WID_G1;
            }
        } else if (m < SFM_G1G2) {
            if (diff >= WID_G2) {
                R[m] = 1;
                offset = NUMC_G1 + (m - SFM_G1) * WID_G2;
                dprocnf(avctx, &ycof[offset], ynrm[m], NB_VECT2, &coefsq[offset], &coefsq_norm[offset]);
                diff -= WID_G2;
            }
        } else if (m < SFM_N) {
            if (diff >= WID_G3) {
                R[m] = 1;
                offset = NUMC_G1G2 + (m - SFM_G1G2) * WID_G3;
                dprocnf(avctx, &ycof[offset], ynrm[m], NB_VECT3, &coefsq[offset], &coefsq_norm[offset]);
                diff -= WID_G3;
            }
        } else {
            if (diff >= WID_GX) {
                R[m] = 1;
                offset = NUMC_N + (m - SFM_N) * WID_GX;
                dprocnf(avctx, &ycof[offset], ynrm[m], NB_VECTX, &coefsq[offset], &coefsq_norm[offset]);
                diff -= WID_GX;
            }
        }
    }
}

static int flvqdec(AVCodecContext *avctx, ChannelState *c,
                    float *coefsq_norm, int16_t *R,
                    int16_t NumSpectumBits,
                    int16_t *ynrm, int16_t is_transient)
{
    G719Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    int16_t k, v, nb_sfm;
    int16_t diff;
    int16_t hcode_l, FlagL, FlagN, FlagC;
    int16_t idx[NB_SFM], normqlg2[NB_SFM], wnorm[NB_SFM], idxbuf[NB_SFM];

    FlagL = get_bits1(gb);
    FlagN = get_bits1(gb);
    FlagC = get_bits1(gb);

    nb_sfm = FlagL == NOALLGROUPS ? SFM_N : NB_SFM;

    bits2idxn(avctx, NORM0_BITS, ynrm);

    if (!(ynrm[0] >= 0 && ynrm[0] < 40))
        return AVERROR_INVALIDDATA;

    if (FlagN == HUFCODE) {
        hdecnrm(avctx, NB_SFM, &ynrm[1]);
        hcode_l = 0;
        for (int i = 1; i < NB_SFM; i++) {
            if (!(ynrm[i] >= 0 && ynrm[i] < 32))
                return AVERROR_INVALIDDATA;
            hcode_l += huffsizn[ynrm[i]];
        }
    } else {
        for (int i = 1; i < NB_SFM; i++) {
            bits2idxn(avctx, NORMI_BITS, &ynrm[i]);
            if (!(ynrm[i] >= 0 && ynrm[i] < 40))
                return AVERROR_INVALIDDATA;
        }
        hcode_l = NUMNRMIBITS;
    }

    normqlg2[0] = dicnlg2[ynrm[0]];
    if (is_transient) {
        idxbuf[0] = ynrm[0];
        for (int i = 1; i < NB_SFM; i++)
            idxbuf[i] = ynrm[i] + idxbuf[i-1] - 15;
        recovernorm(idxbuf, ynrm, normqlg2);
        for (int i = 0; i < NB_SFM; i++)
            if (!(ynrm[i] >= 0 && ynrm[i] < 40))
                return AVERROR_INVALIDDATA;
    } else {
        for (int i = 1; i < NB_SFM; i++) {
            k = ynrm[i - 1] - 15;
            ynrm[i] += k;
            if (!(ynrm[i] >= 0 && ynrm[i] < 40))
                return AVERROR_INVALIDDATA;
            normqlg2[i] = dicnlg2[ynrm[i]];
        }
    }

    for (int i = 0; i < nb_sfm; i++)
        idx[i] = i;

    map_quant_weight(normqlg2, wnorm, is_transient);
    reordvct(wnorm, nb_sfm, idx);
    for (int i = 0; i < NB_SFM; i++)
        R[i] = 0;
    diff = NumSpectumBits - FLAGS_BITS - NORM0_BITS;
    v = diff - hcode_l;
    diff = v;
    bitalloc(wnorm, idx, diff, nb_sfm, QBIT_MAX2, R);

    hcode_l = unpackc(avctx, R, FlagC, 0, 0, SFM_G1, WID_G1, c->ycof);

    k = unpackc(avctx, R, FlagC, NUMC_G1, SFM_G1, SFM_G1G2, WID_G2, c->ycof);
    hcode_l += k;

    k = unpackc(avctx, R, FlagC, NUMC_G1G2, SFM_G1G2, SFM_N, WID_G3, c->ycof);
    hcode_l += k;

    if (nb_sfm > SFM_N) {
        k = unpackc(avctx, R, FlagC, NUMC_N, SFM_N, NB_SFM, WID_GX, c->ycof);
        hcode_l += k;
    }
    diff = v - hcode_l;

    dqcoefs(&c->ycof[0], ynrm, R, 0, SFM_G1, WID_G1, &c->imdct_in[0], &coefsq_norm[0]);
    dqcoefs(&c->ycof[NUMC_G1], ynrm, R, SFM_G1, SFM_G1G2, WID_G2, &c->imdct_in[NUMC_G1], &coefsq_norm[NUMC_G1]);
    dqcoefs(&c->ycof[NUMC_G1G2], ynrm, R, SFM_G1G2, SFM_N, WID_G3, &c->imdct_in[NUMC_G1G2], &coefsq_norm[NUMC_G1G2]);
    dqcoefs(&c->ycof[NUMC_N], ynrm, R, SFM_N, NB_SFM, WID_GX, &c->imdct_in[NUMC_N], &coefsq_norm[NUMC_N]);

    dprocnobitsbfm(avctx, R, idx, ynrm, c->ycof, c->imdct_in, coefsq_norm, nb_sfm, diff);

    return 0;
}

static void fill_spectrum(float *coeff, float *coeff_out, int16_t *R,
                          int16_t is_transient, int16_t norm[], int16_t nf_idx,
                          float *codebook)
{
    int16_t cb_size, cb_pos;
    int16_t last_sfm;
    int16_t first_coeff;
    int16_t low_coeff;
    float *src, *dst, *end;
    float normq;

    cb_size = 0;
    for (int sfm = 0; sfm < NB_SFM; sfm++) {
        if (R[sfm] != 0) {
            for (int j = sfm_start[sfm]; j < sfm_end[sfm]; j++) {
                codebook[cb_size] = coeff[j];
                cb_size++;
            }
        }
    }

    last_sfm = NB_SFM - 1;
    if (is_transient == 0) {
        for (int sfm = NB_SFM - 1; sfm >= 0; sfm--) {
            if (R[sfm] != 0) {
                last_sfm = sfm;
                break;
            }
        }
    }

    if (cb_size != 0) {
        cb_pos = 0;
        for (int sfm = 0; sfm <= last_sfm; sfm++) {
            if (R[sfm] == 0) {
                for (int j = sfm_start[sfm]; j < sfm_end[sfm]; j++) {
                    coeff[j] = codebook[cb_pos];
                    cb_pos++;
                    cb_pos = cb_pos >= cb_size ? 0 : cb_pos;
                }
            }
        }

        if (is_transient == 0) {
            low_coeff = sfm_end[last_sfm] >> 1;
            src       = coeff + sfm_end[last_sfm] - 1;

            first_coeff = sfm_end[last_sfm];
            dst = coeff + sfm_end[last_sfm];
            end = coeff + sfm_end[NB_SFM-1];

            while (dst < end) {
                while (dst < end && src >= &coeff[low_coeff])
                    *dst++ = *src--;
                src++;

                while (dst < end && src < &coeff[first_coeff])
                    *dst++ = *src++;
            }
        }

        for (int sfm = 0; sfm <= last_sfm; sfm++) {
            if (R[sfm] == 0) {
                for (int j = sfm_start[sfm]; j < sfm_end[sfm]; j++)
                    coeff[j] = coeff[j] / powf(2, nf_idx);
            }
        }
    }

    for (int sfm = 0; sfm < NB_SFM; sfm++) {
        normq = dicn[norm[sfm]];
        for (int i = sfm_start[sfm]; i < sfm_end[sfm]; i++)
            coeff_out[i] = coeff[i] * normq;
    }
}

static void de_interleave_spectrum(float *coefs, float *coefs_short)
{
    float *p1a, *p1b, *p2a, *p2b, *p3a, *p3b, *p3c, *p4a, *p4b;
    float *pcoefs, *pcoefs1, *pcoefs2;

    p1a = coefs;
    p1b = coefs + 64;
    p2a = coefs + NUMC_G1;
    p2b = coefs + NUMC_G1 + 64;
    p3a = coefs + NUMC_G23;
    p3b = coefs + NUMC_G23 + 96;
    p3c = coefs + NUMC_G23 + 192;
    p4a = coefs + NUMC_N;
    p4b = coefs + NUMC_N + 128;

    for (int i = 0; i < STOP_BAND; i += STOP_BAND4) {
        pcoefs  = coefs_short + i;
        pcoefs1 = coefs_short + 16 + i;
        for (int j=0; j<16; j++) {
            *pcoefs++  = *p1a++;
            *pcoefs1++ = *p1b++;
        }

        pcoefs  = coefs_short + NUMC_G1SUB + i;
        pcoefs1 = coefs_short + NUMC_G1SUB + 16 + i;
        for (int j = 0; j < 16; j++) {
            *pcoefs++  = *p2a++;
            *pcoefs1++ = *p2b++;
        }

        pcoefs  = coefs_short + NUMC_G1G2SUB + i;
        pcoefs1 = coefs_short + NUMC_G1G2SUB + WID_G3 + i;
        pcoefs2 = coefs_short + NUMC_G1G2SUB + 2 * WID_G3 + i;
        for (int j = 0; j < WID_G3; j++) {
            *pcoefs++  = *p3a++;
            *pcoefs1++ = *p3b++;
            *pcoefs2++ = *p3c++;
        }

        pcoefs  = coefs_short + NUMC_G1G2G3SUB + i;
        pcoefs1 = coefs_short + NUMC_G1G2G3SUB + WID_GX + i;
        for (int j = 0; j < WID_GX; j++) {
            *pcoefs++  = *p4a++;
            *pcoefs1++ = *p4b++;
        }
    }

    /* unpack the spectrum */
    p1a    = coefs_short;
    pcoefs = coefs;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < STOP_BAND4; j++)
            pcoefs[j] = p1a[j];

        for (int j = STOP_BAND4; j < FRAME_LENGTH / 4; j++)
            pcoefs[j] = 0;
        p1a += STOP_BAND4;
        pcoefs += FRAME_LENGTH / 4;
    }
}

static void inverse_transform(AVCodecContext *avctx, ChannelState *c)
{
    G719Context *s = avctx->priv_data;

    if (c->is_transient) {
        memset(c->imdct_sprev, 0, 240 * sizeof(float));
        for (int seg = 0; seg < NUM_TIME_SWITCHING_BLOCKS; seg++) {
            s->stx_fn(s->stx_ctx, c->imdct_sout,
                      c->imdct_in + seg * 240, sizeof(float));
            s->fdsp->vector_fmul_window(c->imdct_out + seg * 240, c->imdct_sprev + (240 >> 1),
                                        c->imdct_sout, s->short_window, 240 >> 1);
            memcpy(c->imdct_sprev, c->imdct_sout, 240 * sizeof(float));
        }
    } else {
        s->ltx_fn(s->ltx_ctx, c->imdct_out, c->imdct_in, sizeof(float));
    }
}

static void decode_channel(AVCodecContext *avctx, ChannelState *c, float *samples)
{
    G719Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;

    c->is_transient = get_bits1(gb);

    if (c->is_transient) {
        flvqdec(avctx, c, c->audio_q_norm, c->bitalloc, s->num_bits_spectrum_transient, c->ynrm, c->is_transient);
        c->nf_idx = 0;
    } else {
        flvqdec(avctx, c, c->audio_q_norm, c->bitalloc, s->num_bits_spectrum_stationary, c->ynrm, c->is_transient);
        bits2idxn(avctx, 2, &c->nf_idx);
    }

    fill_spectrum(c->audio_q_norm, c->imdct_in, c->bitalloc, c->is_transient, c->ynrm, c->nf_idx, c->codebook);

    if (c->is_transient)
        de_interleave_spectrum(c->imdct_in, c->coefs_short);

    c->old_is_transient = c->is_transient;

    inverse_transform(avctx, c);

    s->fdsp->vector_fmul_window(samples, c->imdct_prev + (960 >> 1),
                                c->imdct_out, s->window, 960 >> 1);

    memcpy(c->imdct_prev, c->imdct_out, 960 * sizeof(*c->imdct_out));
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame_ptr, AVPacket *avpkt)
{
    G719Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    float **samples;
    int ret;

    if (avpkt->size < (s->num_bits >> 3) * avctx->ch_layout.nb_channels)
        return AVERROR_INVALIDDATA;

    frame->nb_samples = 960;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;
    samples = (float **)frame->extended_data;

    for (int ch = 0; ch < avctx->ch_layout.nb_channels; ch++) {
        if ((ret = init_get_bits(gb, avpkt->data + (s->num_bits >> 3) * ch,
                                 s->num_bits)) < 0)
            return ret;
        decode_channel(avctx, &s->ch[ch], samples[ch]);
    }

    *got_frame_ptr = 1;

    return avpkt->size;
}

static void decode_flush(AVCodecContext *avctx)
{
    G719Context *s = avctx->priv_data;

    for (int ch = 0; ch < avctx->ch_layout.nb_channels; ch++) {
        ChannelState *c = &s->ch[ch];
        memset(c->imdct_prev, 0, sizeof(c->imdct_prev));
    }
}

static av_cold int decode_close(AVCodecContext *avctx)
{
    G719Context *s = avctx->priv_data;

    av_freep(&s->fdsp);
    av_tx_uninit(&s->ltx_ctx);
    av_tx_uninit(&s->stx_ctx);

    return 0;
}

const FFCodec ff_g719_decoder = {
    .p.name         = "g719",
    CODEC_LONG_NAME("G.719"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_G719,
    .priv_data_size = sizeof(G719Context),
    .init           = decode_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .flush          = decode_flush,
    .close          = decode_close,
    .p.capabilities = AV_CODEC_CAP_DR1,
};
