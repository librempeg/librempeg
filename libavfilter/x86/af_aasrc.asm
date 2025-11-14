;*****************************************************************************
;* x86-optimized functions for aasrc filter
;* Copyright (c) 2025 Paul B Mahol
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

SECTION_RODATA 32

imag_negd: times 4 dd 1.0, -1.0
imag_negq: times 2 dq 1.0, -1.0

SECTION .text

%if HAVE_FMA3_EXTERNAL

;------------------------------------------------------------------------------
; float ff_vector_fmul_real(const complex_float *cur,
;                           complex_float *h, const int N)
;------------------------------------------------------------------------------

INIT_YMM fma3
cglobal vector_fmul_real, 3,3,5, cur, h, N
    shl       Nd, 3
    add       curq, Nq
    add       hq, Nq
    neg       Nq
    xorps     m0, m0
.loop:
    movups    m1, [curq + Nq]
    movups    m2, [hq + Nq]
    mulps     m2, m2, [imag_negd]
    vfmadd231ps m0, m1, m2
    add       Nq, mmsize
    jl .loop

    vextractf128 xm1, m0, 0x1
    addps xm1, xm1, xm0
    haddps xm1, xm1, xm1
    haddps xm0, xm1, xm1

    RET

;------------------------------------------------------------------------------
; double ff_vector_dmul_real(const complex_double *cur,
;                            complex_double *h, const int N)
;------------------------------------------------------------------------------

INIT_YMM fma3
cglobal vector_dmul_real, 3,3,5, cur, h, N
    shl       Nd, 4
    add       curq, Nq
    add       hq, Nq
    neg       Nq
    xorps     m0, m0
.loop:
    movupd    m1, [curq + Nq]
    movupd    m2, [hq + Nq]
    mulpd     m2, m2, [imag_negq]
    vfmadd231pd m0, m1, m2
    add       Nq, mmsize
    jl .loop

    vextractf128 xm1, m0, 0x1
    addpd xm1, xm1, xm0
    haddpd xm0, xm1, xm1

    RET

;------------------------------------------------------------------------------
; void ff_vector_fmul_complex(complex_float *x, const complex_float *a,
;                             const complex_float *b, const int N)
;------------------------------------------------------------------------------

INIT_YMM fma3
cglobal vector_fmul_complex, 4,4,4, x, a, b, N
    shl       Nd, 3
    add       aq, Nq
    add       bq, Nq
    add       xq, Nq
    neg       Nq
.loop:
    movups    m0, [aq + Nq]
    movups    m1, [bq + Nq]
    vpermilps m3, m0, 177
    vpermilps m2, m1, 160
    vpermilps m1, m1, 245
    mulps     m1, m1, m3
    vfmaddsub132ps m0, m1, m2
    movups    [xq + Nq], m0
    add       Nq, mmsize
    jl .loop
    RET

;------------------------------------------------------------------------------
; void ff_vector_dmul_complex(complex_double *x, const complex_double *a,
;                             const complex_double *b, const int N)
;------------------------------------------------------------------------------

INIT_YMM fma3
cglobal vector_dmul_complex, 4,4,4, x, a, b, N
    shl       Nd, 4
    add       aq, Nq
    add       bq, Nq
    add       xq, Nq
    neg       Nq
.loop:
    movupd    m0, [aq + Nq]
    movupd    m1, [bq + Nq]
    vpermilpd m2, m1, 0b0101
    mulpd     m3, m0, m1
    mulpd     m1, m0, m2
    hsubpd    m3, m3
    haddpd    m1, m1
    blendpd   m0, m3, m1, 0b1010
    movupd    [xq + Nq], m0
    add       Nq, mmsize
    jl .loop
    RET

;------------------------------------------------------------------------------
; void ff_vector_fmul_complex_add(const float src, const complex_float *fixed,
;                                 const complex_float *in,
;                                 complex_float *out,
;                                 const int N)
;------------------------------------------------------------------------------

INIT_YMM fma3
%if UNIX64
cglobal vector_fmul_complex_add, 4,4,5, fixed, in, out, N
%else
cglobal vector_fmul_complex_add, 5,5,5, src, fixed, in, out, N
%endif
    shl       Nd, 3
    vbroadcastsd m0, xm0
    add       fixedq, Nq
    add       outq, Nq
    add       inq, Nq
    neg       Nq
.loop:
    movups    m1, [fixedq + Nq]
    movups    m2, [inq + Nq]
    vpermilps m4, m1, 177
    vpermilps m3, m2, 160
    vpermilps m2, m2, 245
    mulps     m2, m2, m4
    vfmaddsub132ps m1, m2, m3
    addps     m1, m0
    movups    [outq + Nq], m1
    add       Nq, mmsize
    jl .loop
    RET

;------------------------------------------------------------------------------
; void ff_vector_dmul_complex_add(const double src, const complex_double *fixed,
;                                 const complex_double *in,
;                                 complex_double *out,
;                                 const int N)
;------------------------------------------------------------------------------

INIT_YMM fma3
%if UNIX64
cglobal vector_dmul_complex_add, 4,4,5, fixed, in, out, N
%else
cglobal vector_dmul_complex_add, 5,5,5, src, fixed, in, out, N
%endif
    shl       Nd, 4
    vbroadcastsd m0, xm0
    xorps     m1, m1
    unpcklpd m0, m0, m1
    add       fixedq, Nq
    add       outq, Nq
    add       inq, Nq
    neg       Nq
.loop:
    movupd    m1, [fixedq + Nq]
    movupd    m2, [inq + Nq]
    vpermilpd m3, m2, 0b0101
    mulpd     m4, m1, m2
    mulpd     m2, m1, m3
    hsubpd    m4, m4
    haddpd    m2, m2
    blendpd   m1, m4, m2, 0b1010
    addpd     m1, m0
    movupd    [outq + Nq], m1
    add       Nq, mmsize
    jl .loop
    RET

%endif
