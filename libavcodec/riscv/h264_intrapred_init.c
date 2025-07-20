/*
 * Copyright (c) 2025 Tristan Matthews <tmatth@videolan.org>
 *
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

#include <stddef.h>
#include <stdint.h>
#include "config.h"
#include "libavutil/attributes.h"
#include "libavutil/cpu.h"
#include "libavutil/riscv/cpu.h"
#include "libavcodec/codec_id.h"
#include "libavcodec/h264pred.h"

#define PRED8x8(TYPE, DEPTH, SUFFIX) \
void ff_pred8x8_ ## TYPE ## _ ## DEPTH ## _ ## SUFFIX (uint8_t *src, \
                                                       ptrdiff_t stride);

#define PRED16x16(TYPE, DEPTH, SUFFIX) \
void ff_pred16x16_ ## TYPE ## _ ## DEPTH ## _ ## SUFFIX (uint8_t *src, \
                                                         ptrdiff_t stride);

/* 8-bit versions */
PRED8x8(horizontal, 8, rvv_vl128)
PRED8x8(horizontal, 8, rvv_vl256)
PRED8x8(plane, 8, rvv_vl128)
PRED8x8(plane, 8, rvv_vl256)
PRED16x16(horizontal, 8, rvv_vl128)
PRED16x16(horizontal, 8, rvv_vl256)
PRED16x16(vertical, 8, rvv_vl128)
PRED16x16(vertical, 8, rvv_vl256)
PRED16x16(dc, 8, rvv_vl128)
PRED16x16(dc, 8, rvv_vl256)
PRED16x16(128_dc, 8, rvv_vl128)
PRED16x16(128_dc, 8, rvv_vl256)
PRED16x16(left_dc, 8, rvv_vl128)
PRED16x16(left_dc, 8, rvv_vl256)
PRED16x16(top_dc, 8, rvv_vl128)
PRED16x16(top_dc, 8, rvv_vl256)
PRED16x16(plane, 8, rvv_vl128)
PRED16x16(plane, 8, rvv_vl256)

av_cold void ff_h264_pred_init_riscv(H264PredContext *h, int codec_id,
                                     const int bit_depth,
                                     const int chroma_format_idc)
{
#if HAVE_RVV
    int cpu_flags = av_get_cpu_flags();

    if (!(cpu_flags & AV_CPU_FLAG_RVV_I32) || !ff_rv_vlen_least(128)) return;

    const int vlen = 8 * ff_get_rv_vlenb();

    if (bit_depth == 8) {
        if (chroma_format_idc <= 1) {
            if (codec_id != AV_CODEC_ID_VP7 && codec_id != AV_CODEC_ID_VP8 &&
                (cpu_flags & AV_CPU_FLAG_RVB)) {
                if (vlen >= 256) {
                    h->pred8x8[PLANE_PRED8x8] = ff_pred8x8_plane_8_rvv_vl256;
                } else {
                    h->pred8x8[PLANE_PRED8x8] = ff_pred8x8_plane_8_rvv_vl128;
                }
            }
            if (vlen >= 256) {
                h->pred8x8[HOR_PRED8x8] = ff_pred8x8_horizontal_8_rvv_vl256;
            } else {
                h->pred8x8[HOR_PRED8x8] = ff_pred8x8_horizontal_8_rvv_vl128;
            }
        }
        if (vlen >= 256) {
            h->pred16x16[HOR_PRED8x8] = ff_pred16x16_horizontal_8_rvv_vl256;
            h->pred16x16[DC_PRED8x8] = ff_pred16x16_dc_8_rvv_vl256;
            h->pred16x16[LEFT_DC_PRED8x8] = ff_pred16x16_left_dc_8_rvv_vl256;
            if (cpu_flags & AV_CPU_FLAG_RVB)
                h->pred16x16[TOP_DC_PRED8x8] = ff_pred16x16_top_dc_8_rvv_vl256;
            h->pred16x16[VERT_PRED8x8] = ff_pred16x16_vertical_8_rvv_vl256;
            h->pred16x16[DC_128_PRED8x8] = ff_pred16x16_128_dc_8_rvv_vl256;
        } else {
            h->pred16x16[HOR_PRED8x8] = ff_pred16x16_horizontal_8_rvv_vl128;
            h->pred16x16[DC_PRED8x8] = ff_pred16x16_dc_8_rvv_vl128;
            h->pred16x16[LEFT_DC_PRED8x8] = ff_pred16x16_left_dc_8_rvv_vl128;
            if (cpu_flags & AV_CPU_FLAG_RVB)
                h->pred16x16[TOP_DC_PRED8x8] = ff_pred16x16_top_dc_8_rvv_vl128;
            h->pred16x16[VERT_PRED8x8] = ff_pred16x16_vertical_8_rvv_vl128;
            h->pred16x16[DC_128_PRED8x8] = ff_pred16x16_128_dc_8_rvv_vl128;
        }
        if (codec_id != AV_CODEC_ID_SVQ3 && codec_id != AV_CODEC_ID_RV40 &&
            codec_id != AV_CODEC_ID_VP7 && codec_id != AV_CODEC_ID_VP8 &&
            (cpu_flags & AV_CPU_FLAG_RVB)) {
            if (vlen >= 256) {
                h->pred16x16[PLANE_PRED8x8] = ff_pred16x16_plane_8_rvv_vl256;
            } else {
                h->pred16x16[PLANE_PRED8x8] = ff_pred16x16_plane_8_rvv_vl128;
            }
        }
    }
#endif
}
