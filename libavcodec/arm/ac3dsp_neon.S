/*
 * Copyright (c) 2011 Mans Rullgard <mans@mansr.com>
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

function ff_ac3_exponent_min_neon, export=1
        cmp             r1,  #0
        it              eq
        bxeq            lr
        push            {lr}
        mov             r12, #256
1:
        vld1.8          {q0},     [r0,:128]
        mov             lr,  r1
        add             r3,  r0,  #256
2:      vld1.8          {q1},     [r3,:128], r12
        subs            lr,  lr,  #1
        vmin.u8         q0,  q0,  q1
        bgt             2b
        subs            r2,  r2,  #16
        vst1.8          {q0},     [r0,:128]!
        bgt             1b
        pop             {pc}
endfunc

function ff_float_to_fixed24_neon, export=1
1:      vld1.32         {q0-q1},  [r1,:128]!
        vcvt.s32.f32    q0,  q0,  #24
        vld1.32         {q2-q3},  [r1,:128]!
        vcvt.s32.f32    q1,  q1,  #24
        vcvt.s32.f32    q2,  q2,  #24
        vst1.32         {q0-q1},  [r0,:128]!
        vcvt.s32.f32    q3,  q3,  #24
        vst1.32         {q2-q3},  [r0,:128]!
        subs            r2,  r2,  #16
        bgt             1b
        bx              lr
endfunc

function ff_ac3_extract_exponents_neon, export=1
        vmov.i32        q15, #8
1:
        vld1.32         {q0},     [r1,:128]!
        vabs.s32        q1,  q0
        vclz.i32        q3,  q1
        vsub.i32        q3,  q3,  q15
        vmovn.i32       d6,  q3
        vmovn.i16       d6,  q3
        vst1.32         {d6[0]},  [r0,:32]!
        subs            r2,  r2,  #4
        bgt             1b
        bx              lr
endfunc

function ff_ac3_sum_square_butterfly_int32_neon, export=1
        vmov.i64        q0,  #0
        vmov.i64        q1,  #0
        vmov.i64        q2,  #0
        vmov.i64        q3,  #0
1:
        vld1.32         {d16},    [r1]!
        vld1.32         {d17},    [r2]!
        vadd.s32        d18, d16, d17
        vsub.s32        d19, d16, d17
        vmlal.s32       q0,  d16, d16
        vmlal.s32       q1,  d17, d17
        vmlal.s32       q2,  d18, d18
        vmlal.s32       q3,  d19, d19
        subs            r3,  r3,  #2
        bgt             1b
        vadd.s64        d0,  d0,  d1
        vadd.s64        d1,  d2,  d3
        vadd.s64        d2,  d4,  d5
        vadd.s64        d3,  d6,  d7
        vst1.64         {q0-q1},  [r0]
        bx              lr
endfunc

function ff_ac3_sum_square_butterfly_float_neon, export=1
        vmov.f32        q0,  #0.0
        vmov.f32        q1,  #0.0
1:
        vld1.32         {d16},    [r1]!
        vld1.32         {d17},    [r2]!
        vadd.f32        d18, d16, d17
        vsub.f32        d19, d16, d17
        vmla.f32        d0,  d16, d16
        vmla.f32        d1,  d17, d17
        vmla.f32        d2,  d18, d18
        vmla.f32        d3,  d19, d19
        subs            r3,  r3,  #2
        bgt             1b
        vpadd.f32       d0,  d0,  d1
        vpadd.f32       d1,  d2,  d3
        vst1.32         {q0},     [r0]
        bx              lr
endfunc
