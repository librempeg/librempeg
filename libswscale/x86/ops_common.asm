;******************************************************************************
;* Copyright (c) 2025 Niklas Haas
;*
;* This file is part of FFmpeg.
;*
;* FFmpeg is free software; you can redistribute it and/or
;* modify it under the terms of the GNU Lesser General Public
;* License as published by the Free Software Foundation; either
;* version 2.1 of the License, or (at your option) any later version.
;*
;* FFmpeg is distributed in the hope that it will be useful,
;* but WITHOUT ANY WARRANTY; without even the implied warranty of
;* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;* Lesser General Public License for more details.
;*
;* You should have received a copy of the GNU Lesser General Public
;* License along with FFmpeg; if not, write to the Free Software
;* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
;******************************************************************************

%include "ops_include.asm"

SECTION .text

;---------------------------------------------------------
; Global entry point. See `ops_include.asm` for info.

%macro process_fn 1 ; number of planes
cglobal sws_process%1_x86, 6, 7 + 2 * %1, 16
            ; Args:
            ;   execq, implq, bxd, yd as defined in ops_include.asm
            ;   bx_end and y_end are initially in tmp0d / tmp1d
            ;   (see SwsOpFunc signature)
            ;
            ; Stack layout:
            ;   [rsp +  0] = [qword] impl->cont (address of first kernel)
            ;   [rsp +  8] = [qword] &impl[1]   (restore implq after chain)
            ;   [rsp + 16] = [dword] bx start   (restore after line finish)
            ;   [rsp + 20] = [dword] bx end     (loop counter limit)
            ;   [rsp + 24] = [dword] y end      (loop counter limit)
            sub rsp, 32
            mov [rsp + 16], bxd
            mov [rsp + 20], tmp0d ; bx_end
            mov [rsp + 24], tmp1d ; y_end
            mov tmp0q, [implq + SwsOpImpl.cont]
            add implq, SwsOpImpl.next
            mov [rsp +  0], tmp0q
            mov [rsp +  8], implq
            movsxdifnidn bxq, bxd
            movsxdifnidn yq, yd

            ; load plane pointers
            mov in0q,  [execq + SwsOpExec.in0]
IF %1 > 1,  mov in1q,  [execq + SwsOpExec.in1]
IF %1 > 2,  mov in2q,  [execq + SwsOpExec.in2]
IF %1 > 3,  mov in3q,  [execq + SwsOpExec.in3]
            mov out0q, [execq + SwsOpExec.out0]
IF %1 > 1,  mov out1q, [execq + SwsOpExec.out1]
IF %1 > 2,  mov out2q, [execq + SwsOpExec.out2]
IF %1 > 3,  mov out3q, [execq + SwsOpExec.out3]
.loop:
            call [rsp] ; call into op chain
            mov implq, [rsp + 8]
            inc bxd
            cmp bxd, [rsp + 20]
            jne .loop
            ; end of line
            inc yd
            cmp yd, [rsp + 24]
            je .end
            ; bump addresses to point to start of next line
            add in0q,  [execq + SwsOpExec.in_bump0]
IF %1 > 1,  add in1q,  [execq + SwsOpExec.in_bump1]
IF %1 > 2,  add in2q,  [execq + SwsOpExec.in_bump2]
IF %1 > 3,  add in3q,  [execq + SwsOpExec.in_bump3]
            add out0q, [execq + SwsOpExec.out_bump0]
IF %1 > 1,  add out1q, [execq + SwsOpExec.out_bump1]
IF %1 > 2,  add out2q, [execq + SwsOpExec.out_bump2]
IF %1 > 3,  add out3q, [execq + SwsOpExec.out_bump3]
            mov bxd, [rsp + 16]
            ; conditionally apply y bump (if non-NULL)
            mov tmp0q, [execq + SwsOpExec.in_bump_y]
            test tmp0q, tmp0q
            jz .loop
            movsxd tmp0q, [tmp0q + yq * 4 - 4] ; load (signed) y bump
%if %1 > 3
            mov tmp1q, tmp0q
            imul tmp1q, [execq + SwsOpExec.in_stride3]
            add in3q, tmp1q
%endif
%if %1 > 2
            mov tmp1q, tmp0q
            imul tmp1q, [execq + SwsOpExec.in_stride2]
            add in2q, tmp1q
%endif
%if %1 > 1
            mov tmp1q, tmp0q
            imul tmp1q, [execq + SwsOpExec.in_stride1]
            add in1q, tmp1q
%endif
            imul tmp0q, [execq + SwsOpExec.in_stride0]
            add in0q, tmp0q
            jmp .loop
.end:
            add rsp, 32
            RET
%endmacro

process_fn 1
process_fn 2
process_fn 3
process_fn 4

;---------------------------------------------------------
; Packed shuffle fast-path

; This is a special entry point for handling a subset of operation chains
; that can be reduced down to a single `pshufb` shuffle mask. For more details
; about when this works, refer to `solve_shuffle()` in ops_optimizer.c.
;
; This macro gets instantiated for every parameter combination of
; SWS_UOP_RW_SHUFFLE, which embeds the clear value and read and write sizes.
;
; Since pshufb can't shuffle across lanes, we only call SSE4 versions for
; all shuffles that are not a clean multiple of 128 bits (e.g. rgb24 -> rgb0),
; unless we have access to AVX-512 vpermb, or if the lanes are all independent,
; in which case we can also use `pshufb` on mmsize == 32/64. This is detected
; by the `LANE_ALIGNED` condition.

%macro MOVSIZE 3 ; size, dst, src
    %if %1 <= 4
        movd %2, %3
    %elif %1 <= 8
        movq %2, %3
    %else
        movu %2, %3
    %endif
%endmacro

%macro RW_SHUFFLE 3
%assign CLEAR_VALUE %1
%assign READ_SIZE   %2
%assign WRITE_SIZE  %3

%assign LANE_ALIGNED (READ_SIZE == WRITE_SIZE && 16 % READ_SIZE == 0)
%assign MAX_SIZE     (READ_SIZE > WRITE_SIZE ? READ_SIZE : WRITE_SIZE)

; Expand read/write sizes to the true number of groups, this matches
; logic on the C side in `translate_shuffle` / `ff_sws_shuffle_mask`
%assign GROUPS       (mmsize / MAX_SIZE)
%assign READ_SIZE    (READ_SIZE * GROUPS)
%assign WRITE_SIZE   (WRITE_SIZE * GROUPS)

cglobal NAME, 6, 10, 3, exec, shuffle, bx, y, bxend, yend, src, dst, src_stride, dst_stride
%if mmsize > 16 && !LANE_ALIGNED
            ud2 ; runtime checks should prevent this variant from being called
%else
            mov srcq, [execq + SwsOpExec.in0]
            mov dstq, [execq + SwsOpExec.out0]
            mov src_strideq, [execq + SwsOpExec.in_stride0]
            mov dst_strideq, [execq + SwsOpExec.out_stride0]

            ; setup shuffle mask
            VBROADCASTI128 m0, [shuffleq]
    %if cpuflag(avx512) && CLEAR_VALUE != 0
            vpmovb2m k1, m0 ; needed for vpblendmb
    %endif

            ; setup clear value register if needed
    %if CLEAR_VALUE == 0xFF
        %if cpuflag(avx512)
            vpternlogd m2, m2, m2, 0xff
        %else
            pcmpeqb m2, m2
        %endif
    %elif CLEAR_VALUE != 0 ; clear-to-0 is implicitly handled by pshufb
            mov shuffled, CLEAR_VALUE * 0x1010101
            movd xm2, shuffled
            VPBROADCASTD m2, xm2
    %endif

            ; setup loop bounds and variables
            sub bxendd, bxd
            sub yendd, yd
            ; reuse now-unneeded regs
            %define srcidxq execq
            imul srcidxq, bxendq, -READ_SIZE
    %if READ_SIZE == WRITE_SIZE
            %define dstidxq srcidxq
    %else
            %define dstidxq shuffleq ; no longer needed reg
            imul dstidxq, bxendq, -WRITE_SIZE
    %endif
            sub srcq, srcidxq
            sub dstq, dstidxq

.loop:
            MOVSIZE READ_SIZE, m1, [srcq + srcidxq]
            pshufb m1, m0

    %if CLEAR_VALUE != 0
        %if cpuflag(avx512)
            vpblendmb m1{k1}, m1, m2
        %elif avx_enabled
            vpblendvb m1, m1, m2, m0
        %else
            pblendvb m1, m2
        %endif
    %endif

            MOVSIZE WRITE_SIZE, [dstq + dstidxq], m1
            add srcidxq, READ_SIZE
    %if READ_SIZE != WRITE_SIZE
            add dstidxq, WRITE_SIZE
    %endif
            jnz .loop
            add srcq, src_strideq
            add dstq, dst_strideq
            imul srcidxq, bxendq, -READ_SIZE
    %if READ_SIZE != WRITE_SIZE
            imul dstidxq, bxendq, -WRITE_SIZE
    %endif
            dec yendd
            jnz .loop
            RET
%endif
%endmacro

INIT_XMM sse4
DECL_U8_RW_SHUFFLE (RW_SHUFFLE)

INIT_YMM avx2
DECL_U8_RW_SHUFFLE (RW_SHUFFLE)

INIT_ZMM avx512
DECL_U8_RW_SHUFFLE (RW_SHUFFLE)
