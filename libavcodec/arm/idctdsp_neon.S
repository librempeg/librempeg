/*
 * ARM-NEON-optimized IDCT functions
 * Copyright (c) 2008 Mans Rullgard <mans@mansr.com>
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

#include "libavutil/arm/asm.S"

function ff_put_pixels_clamped_neon, export=1
        vld1.16         {d16-d19}, [r0,:128]!
        vqmovun.s16     d0, q8
        vld1.16         {d20-d23}, [r0,:128]!
        vqmovun.s16     d1, q9
        vld1.16         {d24-d27}, [r0,:128]!
        vqmovun.s16     d2, q10
        vld1.16         {d28-d31}, [r0,:128]!
        vqmovun.s16     d3, q11
        vst1.8          {d0},      [r1,:64], r2
        vqmovun.s16     d4, q12
        vst1.8          {d1},      [r1,:64], r2
        vqmovun.s16     d5, q13
        vst1.8          {d2},      [r1,:64], r2
        vqmovun.s16     d6, q14
        vst1.8          {d3},      [r1,:64], r2
        vqmovun.s16     d7, q15
        vst1.8          {d4},      [r1,:64], r2
        vst1.8          {d5},      [r1,:64], r2
        vst1.8          {d6},      [r1,:64], r2
        vst1.8          {d7},      [r1,:64], r2
        bx              lr
endfunc

function ff_put_signed_pixels_clamped_neon, export=1
        vmov.u8         d31, #128
        vld1.16         {d16-d17}, [r0,:128]!
        vqmovn.s16      d0, q8
        vld1.16         {d18-d19}, [r0,:128]!
        vqmovn.s16      d1, q9
        vld1.16         {d16-d17}, [r0,:128]!
        vqmovn.s16      d2, q8
        vld1.16         {d18-d19}, [r0,:128]!
        vadd.u8         d0, d0, d31
        vld1.16         {d20-d21}, [r0,:128]!
        vadd.u8         d1, d1, d31
        vld1.16         {d22-d23}, [r0,:128]!
        vadd.u8         d2, d2, d31
        vst1.8          {d0},      [r1,:64], r2
        vqmovn.s16      d3, q9
        vst1.8          {d1},      [r1,:64], r2
        vqmovn.s16      d4, q10
        vst1.8          {d2},      [r1,:64], r2
        vqmovn.s16      d5, q11
        vld1.16         {d24-d25}, [r0,:128]!
        vadd.u8         d3, d3, d31
        vld1.16         {d26-d27}, [r0,:128]!
        vadd.u8         d4, d4, d31
        vadd.u8         d5, d5, d31
        vst1.8          {d3},      [r1,:64], r2
        vqmovn.s16      d6, q12
        vst1.8          {d4},      [r1,:64], r2
        vqmovn.s16      d7, q13
        vst1.8          {d5},      [r1,:64], r2
        vadd.u8         d6, d6, d31
        vadd.u8         d7, d7, d31
        vst1.8          {d6},      [r1,:64], r2
        vst1.8          {d7},      [r1,:64], r2
        bx              lr
endfunc

function ff_add_pixels_clamped_neon, export=1
        mov             r3, r1
        vld1.8          {d16},   [r1,:64], r2
        vld1.16         {d0-d1}, [r0,:128]!
        vaddw.u8        q0, q0, d16
        vld1.8          {d17},   [r1,:64], r2
        vld1.16         {d2-d3}, [r0,:128]!
        vqmovun.s16     d0, q0
        vld1.8          {d18},   [r1,:64], r2
        vaddw.u8        q1, q1, d17
        vld1.16         {d4-d5}, [r0,:128]!
        vaddw.u8        q2, q2, d18
        vst1.8          {d0},    [r3,:64], r2
        vqmovun.s16     d2, q1
        vld1.8          {d19},   [r1,:64], r2
        vld1.16         {d6-d7}, [r0,:128]!
        vaddw.u8        q3, q3, d19
        vqmovun.s16     d4, q2
        vst1.8          {d2},    [r3,:64], r2
        vld1.8          {d16},   [r1,:64], r2
        vqmovun.s16     d6, q3
        vld1.16         {d0-d1}, [r0,:128]!
        vaddw.u8        q0, q0, d16
        vst1.8          {d4},    [r3,:64], r2
        vld1.8          {d17},   [r1,:64], r2
        vld1.16         {d2-d3}, [r0,:128]!
        vaddw.u8        q1, q1, d17
        vst1.8          {d6},    [r3,:64], r2
        vqmovun.s16     d0, q0
        vld1.8          {d18},   [r1,:64], r2
        vld1.16         {d4-d5}, [r0,:128]!
        vaddw.u8        q2, q2, d18
        vst1.8          {d0},    [r3,:64], r2
        vqmovun.s16     d2, q1
        vld1.8          {d19},   [r1,:64], r2
        vqmovun.s16     d4, q2
        vld1.16         {d6-d7}, [r0,:128]!
        vaddw.u8        q3, q3, d19
        vst1.8          {d2},    [r3,:64], r2
        vqmovun.s16     d6, q3
        vst1.8          {d4},    [r3,:64], r2
        vst1.8          {d6},    [r3,:64], r2
        bx              lr
endfunc
