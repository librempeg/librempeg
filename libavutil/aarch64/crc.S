/*
 * Copyright (c) 2025 Zhao Zhili <quinkblack@foxmail.com>
 * Copyright (c) 2026 Shreesh Adiga <16567adigashreesh@gmail.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "asm.S"

#if HAVE_ARM_CRC
ENABLE_ARM_CRC

.macro crc32_byte
        ldrb            w5, [x2], #1
        subs            x4, x4, #1
        crc32b          w0, w0, w5
.endm

function ff_crc32_aarch64, export=1
        and             x4, x2, #7          // non-aligned buffer addr
        mov             w0, w1
        cbz             x3, 8f

        cbz             x4, 2f

        mov             x5, #8
        sub             x4, x5, x4
        cmp             x4, x3
        csel            x4, x4, x3, lo
        sub             x3, x3, x4
1:
        // loop on non-aligned buffer
        crc32_byte
        b.ne            1b
2:
        // loop on aligned buffer
        bic             x5, x3, #15
        and             x4, x3, #15
        cbz             x5, 4f
3:
        ldp             x6, x7, [x2], #16
        subs            x5, x5, #16
        crc32x          w0, w0, x6
        crc32x          w0, w0, x7
        b.ne            3b
4:
        cbz             x4, 8f
5:
        crc32_byte
        b.ne            5b
8:
        ret
endfunc

DISABLE_ARM_CRC
#endif

#if HAVE_PMULL && HAVE_EOR3
ENABLE_PMULL
ENABLE_EOR3

const reverse_shuffle, align=4
        .byte  15, 14, 13, 12
        .byte  11, 10,  9,  8
        .byte   7,  6,  5,  4
        .byte   3,  2,  1,  0
endconst

const partial_bytes_shuf_tab, align=4
        .byte  255, 254, 253, 252
        .byte  251, 250, 249, 248
        .byte  247, 246, 245, 244
        .byte  243, 242, 241, 240
        .byte   0,  1,  2,  3
        .byte   4,  5,  6,  7
        .byte   8,  9, 10, 11
        .byte  12, 13, 14, 15
endconst

// performs Vfold = pmull(Vfold, Vconst) xor pmull2(Vfold, Vconst) xor Vdata
.macro FOLD_SINGLE Vfold, Vconst, Vdata, Vtemp
        pmull           \Vtemp\().1q, \Vconst\().1d, \Vfold\().1d
        pmull2          \Vfold\().1q, \Vconst\().2d, \Vfold\().2d
        eor3            \Vfold\().16b, \Vfold\().16b, \Vdata\().16b, \Vtemp\().16b
.endm

// assume Vfold is v16 and v0 is filled with 0
// uses v17 as temp
.macro FOLD_128_TO_64 le, Vconst
.if ! \le
        fmov            d17, d16
        pmull2          v16.1q, v16.2d, \Vconst\().2d
        ext             v17.16b, v0.16b, v17.16b, #12
        eor             v16.16b, v17.16b, v16.16b
        ext             \Vconst\().16b, \Vconst\().16b, \Vconst\().16b, #8
        pmull2          v17.1q, \Vconst\().2d, v16.2d
        eor             v16.16b, v16.16b, v17.16b
.else
        ext             v17.16b, v16.16b, v0.16b, #8
        pmull           v16.1q, v16.1d, \Vconst\().1d
        eor             v16.16b, v16.16b, v17.16b
        ext             v17.16b, v0.16b, v16.16b, #12
        ext             \Vconst\().16b, \Vconst\().16b, \Vconst\().16b, #8
        pmull           v17.1q, \Vconst\().1d, v17.1d
        eor             v16.16b, v16.16b, v17.16b
.endif
.endm

// assume Vfold is v16 and v0 is filled with 0
// uses v17 as temp
.macro FOLD_64_TO_32 le, Vconst, output_reg
.if ! \le
        pmull           v17.1q, v16.1d, \Vconst\().1d
        pmull2          v17.1q, v17.2d, \Vconst\().2d
        eor             v16.16b, v16.16b, v17.16b
        fmov            \output_reg, s16
        rev             \output_reg, \output_reg
.else
        mov             v16.s[0], wzr
        pmull           v17.1q, v16.1d, \Vconst\().1d
        eor             v17.16b, v17.16b, v16.16b
        ext             \Vconst\().16b, \Vconst\().16b, \Vconst\().16b, #8
        pmull           v17.1q, v17.1d, \Vconst\().1d
        eor             v16.16b, v16.16b, v17.16b
        mov             \output_reg, v16.s[2]
.endif
.endm

#define CTX_OFFSET 4

// ff_crc[_le]_neon_pmull(const AVCRC *ctx, uint32_t crc, const uint8_t *buffer, size_t length)
// x0 - ctx; pre-computed fold constants, see crc_init_aarch64 for details on the structure.
// x1 - crc initial value
// x2 - buffer pointer
// x3 - length of buffer
//
// Additional registers used:
// x10 - buffer + length; points to end of buffer
// x11-x15 used as temp registers as needed
//
// v0 - zero register to perform vector shift during reduction using EXT
// v1 - crc input from x1
// v2 - holds the reverse shuffle values 15, 14 ... 0 to reverse vector using TBL
// v3 - holds the constant values used in PMULL/PMULL2 loaded via x0 ctx
// v4-v7 - used as temp for PMULL/PMULL2 output
// v16 - holds the primary fold register later reduced to 32 bits
// Registers v16-v19 are used in parallel 4x fold loop
// Registers v20-23 are used to load the 4 next data chunks
// Registers v17-v21 are used as temp elsewhere
.macro crc_fn_template le
.if \le
function ff_crc_le_neon_pmull, export=1
.else
function ff_crc_neon_pmull, export=1
        movrel          x9, reverse_shuffle
        ldr             q2, [x9]
.endif
        add             x10, x2, x3
        movi            v0.2d, #0
        mov             v1.16b, v0.16b
        mov             v1.s[0], w1
        cmp             x3, #64
        b.lo            8f // less than 64 bytes
        ld1             {v16.16b-v19.16b}, [x2], #64
        eor             v16.16b, v16.16b, v1.16b
.if ! \le
        tbl             v16.16b, { v16.16b }, v2.16b
        tbl             v17.16b, { v17.16b }, v2.16b
        tbl             v18.16b, { v18.16b }, v2.16b
        tbl             v19.16b, { v19.16b }, v2.16b
.endif
        sub             x11, x10, x2
        cmp             x11, #64
        b.lo            2f // reduce 4x to 1x
        ldr             q3, [x0, #(CTX_OFFSET + 0)]

1: // fold 4x loop
        ld1             {v20.16b-v23.16b}, [x2], #64
.if ! \le
        tbl             v20.16b, { v20.16b }, v2.16b
        tbl             v21.16b, { v21.16b }, v2.16b
        tbl             v22.16b, { v22.16b }, v2.16b
        tbl             v23.16b, { v23.16b }, v2.16b
.endif
        pmull           v4.1q, v16.1d, v3.1d
        pmull           v5.1q, v17.1d, v3.1d
        pmull           v6.1q, v18.1d, v3.1d
        pmull           v7.1q, v19.1d, v3.1d
        pmull2          v16.1q, v16.2d, v3.2d
        pmull2          v17.1q, v17.2d, v3.2d
        pmull2          v18.1q, v18.2d, v3.2d
        pmull2          v19.1q, v19.2d, v3.2d
        eor3            v16.16b, v16.16b, v4.16b, v20.16b
        eor3            v17.16b, v17.16b, v5.16b, v21.16b
        eor3            v18.16b, v18.16b, v6.16b, v22.16b
        eor3            v19.16b, v19.16b, v7.16b, v23.16b
        sub             x11, x10, x2
        cmp             x11, #64
        b.hs            1b // fold 4x loop

2: // reduce 4x to 1x
        ldr             q3, [x0, #(CTX_OFFSET + 16)]
        FOLD_SINGLE     v16, v3, v17, v4
        FOLD_SINGLE     v16, v3, v18, v4
        FOLD_SINGLE     v16, v3, v19, v4

3: // fold 1x pre
        sub             x11, x10, x2
        cmp             x11, #16
        b.lo            5f // partial_block

4: // fold 1x loop
        ldr             q17, [x2], #16
.if ! \le
        tbl             v17.16b, { v17.16b }, v2.16b
.endif
        FOLD_SINGLE     v16, v3, v17, v4
        sub             x11, x10, x2
        cmp             x11, #16
        b.hs            4b // fold 1x loop

5: // partial_block
        cmp             x10, x2
        b.eq            6f // reduce 128 to 64
        ldr             q17, [x10, #-16]
        movrel          x1, partial_bytes_shuf_tab
        ldr             q18, [x1, x11]
.if ! \le
        tbl             v16.16b, { v16.16b }, v2.16b
.endif
        mvn             v19.16b, v18.16b
        tbl             v20.16b, { v16.16b }, v19.16b
        cmge            v21.16b, v18.16b, #0
        bsl             v21.16b, v17.16b, v20.16b
        tbl             v16.16b, { v16.16b }, v18.16b
.if ! \le
        tbl             v16.16b, { v16.16b }, v2.16b
        tbl             v21.16b, { v21.16b }, v2.16b
.endif
        FOLD_SINGLE     v16, v3, v21, v4

6: // reduce 128 to 64
        ldr             q3, [x0, #(CTX_OFFSET + 32)]
        FOLD_128_TO_64  \le, v3

7: // reduce 64 to 32
        ldr             q3, [x0, #(CTX_OFFSET + 48)]
        FOLD_64_TO_32   \le, v3, w0
        ret

8: // less than 64 bytes
        cmp             x3, #16
        b.lo            9f // less than 16 bytes
        ldr             q16, [x2], #16
        eor             v16.16b, v16.16b, v1.16b
.if ! \le
        tbl             v16.16b, { v16.16b }, v2.16b
.endif
        ldr             q3, [x0, #(CTX_OFFSET + 16)]
        b               3b // fold 1x pre

9: // less than 16 bytes
        movrel          x15, partial_bytes_shuf_tab
        add             x15, x15, x3
        str             q0, [sp, #-16]!
        add             x9, sp, x3
        // memcpy 0 to 15 bytes from src to stack and load it to v16 with zero extension
        tbz             x3, #3, 10f
        ldr             x11, [x2]
        ldr             x12, [x10, #-8]
        str             x11, [sp]
        str             x12, [x9, #-8]
        b               12f
10:
        tbz             x3, #2, 11f
        ldr             w11, [x2]
        ldr             w12, [x10, #-4]
        str             w11, [sp]
        str             w12, [x9, #-4]
        b               12f
11:
        cbz             x3, 12f
        lsr             x11, x3, #1
        ldrb            w12, [x2]
        ldrb            w14, [x10, #-1]
        ldrb            w13, [x2, x11]
        strb            w12, [sp]
        strb            w13, [sp, x11]
        strb            w14, [x9, #-1]
12:
        ldr             q16, [sp], #16
        eor             v16.16b, v16.16b, v1.16b
        cmp             x3, #5
        b.lo            13f // less than 5 bytes
        ldr             q17, [x15]
        tbl             v16.16b, { v16.16b }, v17.16b
.if ! \le
        tbl             v16.16b, { v16.16b }, v2.16b
.endif
        b               6b // reduce 128 to 64

13: // less than 5 bytes
.if ! \le
        ldr             q17, [x15, #12]
        tbl             v16.16b, { v16.16b }, v17.16b
        rev64           v16.16b, v16.16b
.else
        ldr             q17, [x15, #8]
        tbl             v16.16b, { v16.16b }, v17.16b
.endif
        b               7b // reduce 64 to 32
endfunc
.endm

crc_fn_template 0
crc_fn_template 1

#if HAVE_ARM_CRC
ENABLE_ARM_CRC
// uses x7, x6, x4 and v31 as temporary registers.
.macro CRC_SHIFT crc_reg, nbits_reg, output_neon_reg
        mov             x7, #-2
1:
        and             x6, x\nbits_reg, #1
        lsr             x\nbits_reg, x\nbits_reg, #1
        sub             x\nbits_reg, x\nbits_reg, #16
        add             x7, x6, x7, lsl #1
        cmp             x\nbits_reg, #191
        b.hi            1b
        mvn             x6, x7
        mov             w7, #-2147483648
        lsr             w7, w7, w\nbits_reg
        lsr             x\nbits_reg, x\nbits_reg, #5
2:
        subs            x\nbits_reg, x\nbits_reg, #1
        crc32w          w7, w7, wzr
        b.ne            2b
        lsr             x4, x6, #1
        cbz             x4, 4f
        and             w\nbits_reg, w6, #1
3:
        fmov            s31, w7
        pmull           v31.8h, v31.8b, v31.8b
        fmov            x7, d31
        lsl             x7, x7, x\nbits_reg
        and             w\nbits_reg, w4, #1
        crc32x          w7, wzr, x7
        lsr             x4, x4, #1
        cbnz            x4, 3b
4:
        fmov            s\output_neon_reg, w\crc_reg
        fmov            s31, w7
        pmull           v\output_neon_reg\().1q, v\output_neon_reg\().1d, v31.1d
.endm

// This routine is based on zlib-ng's implementation based on
// https://github.com/zlib-ng/zlib-ng/commit/b5638a82e726c9941bd3a1e7a23182d038eb831f
// https://github.com/corsix/fast-crc32
function ff_crc32_pmull_eor3_aarch64, export=1
        neg             x8, x2
        tst             x8, #0xf
        b.eq            4f // buf 16b aligned
        cbz             x3, 11f
        tbz             w8, #0, 1f
        ldrb            w9, [x2], #1
        sub             x3, x3, #1
        crc32b          w1, w1, w9
1:
        tbz             w8, #1, 2f
        subs            x9, x3, #2
        b.lo            9f
        mov             x3, x9
        ldrh            w10, [x2], #2
        crc32h          w1, w1, w10
2:
        tbz             w8, #2, 3f
        subs            x9, x3, #4
        b.lo            9f
        mov             x3, x9
        ldr             w10, [x2], #4
        crc32w          w1, w1, w10
3:
        tbz             w8, #3, 4f
        subs            x9, x3, #8
        b.lo            9f
        mov             x3, x9
        ldr             x10, [x2], #8
        crc32x          w1, w1, x10

4: // buf 16b aligned
        cmp             x3, #2, lsl #12 // 8192
        b.lo            12f // 4x fold

        mov             x8, #-6148914691236517206
        ldur            q17, [x0, #(CTX_OFFSET + 0)]
        movk            x8, #43691
        mov             w10, wzr
        umulh           x8, x3, x8
        ldur            q18, [x0, #(CTX_OFFSET + 16)]
        mov             w11, wzr
        lsr             x9, x8, #7
        add             x8, x9, x9, lsl #1
        lsl             x12, x9, #4
        lsl             x13, x9, #5
        lsl             x8, x8, #4
        add             x16, x2, x12
        sub             x15, x16, #32
        sub             x17, x3, x8
        add             x8, x2, x8
        ldp             q6, q16, [x8]
        ldr             q0, [x8, #128]
        ldp             q4, q7, [x8, #32]
        sub             x3, x17, #144
        ldp             q3, q5, [x8, #64]
        ldp             q2, q1, [x8, #96]
        add             x8, x8, #144

5: // 192b hybrid fold
        pmull           v19.1q, v6.1d, v17.1d
        ldp             q26, q28, [x8]
        pmull2          v6.1q, v6.2d, v17.2d
        ldp             x16, x6, [x2]
        pmull           v20.1q, v16.1d, v17.1d
        add             x17, x2, x12
        add             x7, x2, x13
        pmull2          v16.1q, v16.2d, v17.2d
        add             x2, x2, #16
        sub             x3, x3, #144
        pmull           v21.1q, v4.1d, v17.1d
        ldp             x4, x17, [x17]
        pmull2          v4.1q, v4.2d, v17.2d
        eor3            v6.16b, v6.16b, v19.16b, v26.16b
        crc32x          w10, w10, x4
        pmull           v22.1q, v7.1d, v17.1d
        ldp             x5, x7, [x7]
        pmull2          v7.1q, v7.2d, v17.2d
        eor3            v16.16b, v16.16b, v20.16b, v28.16b
        crc32x          w11, w11, x5
        ldp             q26, q20, [x8, #32]
        cmp             x2, x15
        pmull           v23.1q, v3.1d, v17.1d
        crc32x          w16, w1, x16
        crc32x          w1, w16, x6
        pmull2          v3.1q, v3.2d, v17.2d
        crc32x          w10, w10, x17
        crc32x          w11, w11, x7
        pmull           v24.1q, v5.1d, v17.1d
        eor3            v4.16b, v4.16b, v21.16b, v26.16b
        eor3            v7.16b, v7.16b, v22.16b, v20.16b
        pmull2          v5.1q, v5.2d, v17.2d
        ldp             q21, q26, [x8, #64]
        pmull           v25.1q, v2.1d, v17.1d
        ldp             q28, q20, [x8, #96]
        pmull2          v2.1q, v2.2d, v17.2d
        ldr             q22, [x8, #128]
        add             x8, x8, #144
        pmull           v27.1q, v1.1d, v17.1d
        eor3            v3.16b, v3.16b, v23.16b, v21.16b
        pmull2          v1.1q, v1.2d, v17.2d
        eor3            v5.16b, v5.16b, v24.16b, v26.16b
        pmull           v19.1q, v0.1d, v17.1d
        pmull2          v0.1q, v0.2d, v17.2d
        eor3            v2.16b, v2.16b, v25.16b, v28.16b
        eor3            v1.16b, v1.16b, v27.16b, v20.16b
        eor3            v0.16b, v0.16b, v19.16b, v22.16b
        b.ls            5b // 192b hybrid fold

        add             x17, x2, x13
        mov             x13, #-33
        add             x12, x2, x12
        ldur            q22, [x0, #(CTX_OFFSET + 32)]
        ldp             x16, x15, [x2]
        ldur            q23, [x0, #(CTX_OFFSET + 48)]
        ldp             x14, x0, [x12]
        crc32x          w16, w1, x16
        crc32x          w10, w10, x14
        crc32x          w12, w16, x15
        ldp             x7, x17, [x17]
        crc32x          w14, w11, x7
        crc32x          w11, w10, x0
        crc32x          w10, w14, x17
        mov             w14, #1408
        madd            x14, x9, x14, x13

        pmull           v20.1q, v6.1d, v18.1d
        pmull2          v6.1q, v6.2d, v18.2d
        pmull           v21.1q, v5.1d, v18.1d
        pmull2          v5.1q, v5.2d, v18.2d
        eor3            v6.16b, v6.16b, v20.16b, v16.16b
        pmull           v20.1q, v7.1d, v18.1d
        pmull2          v7.1q, v7.2d, v18.2d
        eor3            v2.16b, v5.16b, v21.16b, v2.16b
        pmull           v16.1q, v6.1d, v18.1d
        pmull2          v6.1q, v6.2d, v18.2d
        pmull           v5.1q, v2.1d, v22.1d
        eor3            v3.16b, v7.16b, v20.16b, v3.16b
        pmull2          v2.1q, v2.2d, v22.2d

        CRC_SHIFT       12, 14, 29
        mov             w14, #1280
        eor3            v4.16b, v6.16b, v16.16b, v4.16b
        pmull           v16.1q, v1.1d, v18.1d
        pmull2          v1.1q, v1.2d, v18.2d
        pmull           v6.1q, v4.1d, v22.1d
        pmull2          v4.1q, v4.2d, v22.2d
        madd            x14, x9, x14, x13
        CRC_SHIFT       11, 14, 28
        eor3            v3.16b, v4.16b, v6.16b, v3.16b
        eor3            v4.16b, v1.16b, v16.16b, v0.16b
        pmull           v0.1q, v3.1d, v23.1d
        eor3            v2.16b, v2.16b, v5.16b, v4.16b
        pmull2          v1.1q, v3.2d, v23.2d
        mov             w12, #1152
        madd            x9, x9, x12, x13
        eor3            v0.16b, v1.16b, v0.16b, v2.16b
        CRC_SHIFT       10, 9, 27
        fmov            x9, d0
        crc32x          w9, wzr, x9
        eor3            v1.16b, v27.16b, v28.16b, v29.16b
        dup             v2.2d, v0.d[1]
        eor             v1.16b, v1.16b, v2.16b
        mov             x2, x8
        fmov            x10, d1
        crc32x          w1, w9, x10

6: // process tail (<192 bytes)
        bic             x5, x3, #15
        and             x3, x3, #0xf
        cbz             x5, 8f
7:
        ldp             x6, x7, [x2], #16
        subs            x5, x5, #16
        crc32x          w1, w1, x6
        crc32x          w1, w1, x7
        b.ne            7b
8:
        tbz             x3, #3, 9f
        ldr             x10, [x2], #8
        sub             x3, x3, #8
        crc32x          w1, w1, x10
9:
        cbz             x3, 11f
10:
        ldrb            w10, [x2], #1
        subs            x3, x3, #1
        crc32b          w1, w1, w10
        b.ne            10b
11:
        mov             w0, w1
        ret

12: // 4x fold
        cmp             x3, #192
        b.lo            6b // process tail (<192 bytes)

        ldur            q3, [x0, #(CTX_OFFSET + 64)]
        movi            v0.2d, #0
        fmov            s1, w1
        ld1             {v16.16b-v19.16b}, [x2], #64
        sub             x3, x3, #64
        eor             v16.16b, v16.16b, v1.16b
        ldur            q25, [x0, #(CTX_OFFSET + 80)]

        bic             x5, x3, #63
        and             x3, x3, #0x3f

13: // fold 4x loop
        ld1             {v20.16b-v23.16b}, [x2], #64
        pmull           v4.1q, v16.1d, v3.1d
        pmull           v5.1q, v17.1d, v3.1d
        pmull           v6.1q, v18.1d, v3.1d
        pmull           v7.1q, v19.1d, v3.1d
        pmull2          v16.1q, v16.2d, v3.2d
        pmull2          v17.1q, v17.2d, v3.2d
        pmull2          v18.1q, v18.2d, v3.2d
        pmull2          v19.1q, v19.2d, v3.2d
        subs            x5, x5, #64
        eor3            v16.16b, v16.16b, v4.16b, v20.16b
        eor3            v17.16b, v17.16b, v5.16b, v21.16b
        eor3            v18.16b, v18.16b, v6.16b, v22.16b
        eor3            v19.16b, v19.16b, v7.16b, v23.16b
        b.ne            13b // fold 4x loop

        FOLD_SINGLE     v16, v25, v17, v4
        ldur            q26, [x0, #(CTX_OFFSET + 96)]
        FOLD_SINGLE     v16, v25, v18, v4
        ldur            q27, [x0, #(CTX_OFFSET + 112)]
        FOLD_SINGLE     v16, v25, v19, v4
        FOLD_128_TO_64  1, v26
        FOLD_64_TO_32   1, v27, w1
        b               6b // process tail (<192 bytes)
endfunc
DISABLE_ARM_CRC
#endif

DISABLE_PMULL
DISABLE_EOR3
#endif
