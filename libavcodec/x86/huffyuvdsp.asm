;******************************************************************************
;* SIMD-optimized HuffYUV functions
;* Copyright (c) 2008 Loren Merritt
;* Copyright (c) 2014 Christophe Gisquet
;*
;* This file is part of Librempeg.
;*
;* Librempeg is free software; you can redistribute it and/or
;* modify it under the terms of the GNU Lesser General Public
;* License as published by the Free Software Foundation; either
;* version 2.1 of the License, or (at your option) any later version.
;*
;* Librempeg is distributed in the hope that it will be useful,
;* but WITHOUT ANY WARRANTY; without even the implied warranty of
;* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;* Lesser General Public License for more details.
;*
;* You should have received a copy of the GNU Lesser General Public
;* License along with FFmpeg; if not, write to the Free Software
;* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
;******************************************************************************

%include "libavutil/x86/x86util.asm"

SECTION .text

%include "libavcodec/x86/huffyuvdsp_template.asm"

;------------------------------------------------------------------------------
; void (*add_int16)(uint16_t *dst, const uint16_t *src, unsigned mask, int w);
;------------------------------------------------------------------------------

%macro ADD_INT16 0
cglobal add_int16, 4,4,5, dst, src, mask, w, tmp
    test srcq, mmsize-1
    jnz .unaligned
    test dstq, mmsize-1
    jnz .unaligned
    INT16_LOOP a, add
.unaligned:
    INT16_LOOP u, add
%endmacro

INIT_XMM sse2
ADD_INT16

%if HAVE_AVX2_EXTERNAL
INIT_YMM avx2
ADD_INT16
%endif

; void add_hfyu_left_pred_bgr32(uint8_t *dst, const uint8_t *src,
;                               intptr_t w, uint8_t *left)
INIT_XMM sse2
cglobal add_hfyu_left_pred_bgr32, 4,4,3, dst, src, w, left
    shl           wq, 2
    movd          m0, [leftq]
    lea         dstq, [dstq + wq]
    lea         srcq, [srcq + wq]
    LSHIFT        m0, mmsize-4
    neg           wq
.loop:
    movu          m1, [srcq+wq]
    mova          m2, m1
    LSHIFT        m1, 4
    paddb         m1, m2
    pshufd        m0, m0, q3333
    mova          m2, m1
    LSHIFT        m1, 8
    paddb         m1, m2
    paddb         m0, m1
    movu   [dstq+wq], m0
    add           wq, mmsize
    jl         .loop
    movd          m0, [dstq-4]
    movd     [leftq], m0
    RET


; void add_hfyu_median_prediction_mmxext(uint8_t *dst, const uint8_t *top, const uint8_t *diff, int mask, int w, int *left, int *left_top)
INIT_MMX mmxext
cglobal add_hfyu_median_pred_int16, 7,7,0, dst, top, diff, mask, w, left, left_top
    add      wd, wd
    movd    mm6, maskd
    SPLATW  mm6, mm6
    movq    mm0, [topq]
    movq    mm2, mm0
    movd    mm4, [left_topq]
    psllq   mm2, 16
    movq    mm1, mm0
    por     mm4, mm2
    movd    mm3, [leftq]
    psubw   mm0, mm4 ; t-tl
    add    dstq, wq
    add    topq, wq
    add   diffq, wq
    neg      wq
    jmp .skip
.loop:
    movq    mm4, [topq+wq]
    movq    mm0, mm4
    psllq   mm4, 16
    por     mm4, mm1
    movq    mm1, mm0 ; t
    psubw   mm0, mm4 ; t-tl
.skip:
    movq    mm2, [diffq+wq]
%assign i 0
%rep 4
    movq    mm4, mm0
    paddw   mm4, mm3 ; t-tl+l
    pand    mm4, mm6
    movq    mm5, mm3
    pmaxsw  mm3, mm1
    pminsw  mm5, mm1
    pminsw  mm3, mm4
    pmaxsw  mm3, mm5 ; median
    paddw   mm3, mm2 ; +residual
    pand    mm3, mm6
%if i==0
    movq    mm7, mm3
    psllq   mm7, 48
%else
    movq    mm4, mm3
    psrlq   mm7, 16
    psllq   mm4, 48
    por     mm7, mm4
%endif
%if i<3
    psrlq   mm0, 16
    psrlq   mm1, 16
    psrlq   mm2, 16
%endif
%assign i i+1
%endrep
    movq [dstq+wq], mm7
    add      wq, 8
    jl .loop
    movzx   r2d, word [dstq-2]
    mov [leftq], r2d
    movzx   r2d, word [topq-2]
    mov [left_topq], r2d
    RET
