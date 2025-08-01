/*
 * Copyright (c) 2004 Romain Dolbeau <romain@dolbeau.org>
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

#include "config.h"

#include "libavutil/attributes.h"
#include "libavutil/cpu.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem_internal.h"
#include "libavutil/ppc/cpu.h"
#include "libavutil/ppc/util_altivec.h"

#include "libavcodec/h264qpel.h"

#include "hpeldsp_altivec.h"

#if HAVE_ALTIVEC

#define PUT_OP_U8_ALTIVEC(d, s, dst) d = s
#define AVG_OP_U8_ALTIVEC(d, s, dst) d = vec_avg(dst, s)

#define OP_U8_ALTIVEC                          PUT_OP_U8_ALTIVEC
#define PREFIX_h264_qpel16_h_lowpass_altivec   put_h264_qpel16_h_lowpass_altivec
#define PREFIX_h264_qpel16_h_lowpass_num       altivec_put_h264_qpel16_h_lowpass_num
#define PREFIX_h264_qpel16_v_lowpass_altivec   put_h264_qpel16_v_lowpass_altivec
#define PREFIX_h264_qpel16_v_lowpass_num       altivec_put_h264_qpel16_v_lowpass_num
#define PREFIX_h264_qpel16_hv_lowpass_altivec  put_h264_qpel16_hv_lowpass_altivec
#define PREFIX_h264_qpel16_hv_lowpass_num      altivec_put_h264_qpel16_hv_lowpass_num
#include "h264qpel_template.c"
#undef OP_U8_ALTIVEC
#undef PREFIX_h264_qpel16_h_lowpass_altivec
#undef PREFIX_h264_qpel16_h_lowpass_num
#undef PREFIX_h264_qpel16_v_lowpass_altivec
#undef PREFIX_h264_qpel16_v_lowpass_num
#undef PREFIX_h264_qpel16_hv_lowpass_altivec
#undef PREFIX_h264_qpel16_hv_lowpass_num

#define OP_U8_ALTIVEC                          AVG_OP_U8_ALTIVEC
#define PREFIX_h264_qpel16_h_lowpass_altivec   avg_h264_qpel16_h_lowpass_altivec
#define PREFIX_h264_qpel16_h_lowpass_num       altivec_avg_h264_qpel16_h_lowpass_num
#define PREFIX_h264_qpel16_v_lowpass_altivec   avg_h264_qpel16_v_lowpass_altivec
#define PREFIX_h264_qpel16_v_lowpass_num       altivec_avg_h264_qpel16_v_lowpass_num
#define PREFIX_h264_qpel16_hv_lowpass_altivec  avg_h264_qpel16_hv_lowpass_altivec
#define PREFIX_h264_qpel16_hv_lowpass_num      altivec_avg_h264_qpel16_hv_lowpass_num
#include "h264qpel_template.c"
#undef OP_U8_ALTIVEC
#undef PREFIX_h264_qpel16_h_lowpass_altivec
#undef PREFIX_h264_qpel16_h_lowpass_num
#undef PREFIX_h264_qpel16_v_lowpass_altivec
#undef PREFIX_h264_qpel16_v_lowpass_num
#undef PREFIX_h264_qpel16_hv_lowpass_altivec
#undef PREFIX_h264_qpel16_hv_lowpass_num

#define H264_MC(OPNAME, SIZE, CODETYPE) \
static void OPNAME ## h264_qpel ## SIZE ## _mc00_ ## CODETYPE (uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    ff_ ## OPNAME ## pixels ## SIZE ## _ ## CODETYPE(dst, src, stride, SIZE);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc10_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{ \
    DECLARE_ALIGNED(16, uint8_t, half)[SIZE*SIZE];\
    put_h264_qpel ## SIZE ## _h_lowpass_ ## CODETYPE(half, src, SIZE, stride);\
    OPNAME ## pixels ## SIZE ## _l2_ ## CODETYPE(dst, src, half, stride, stride, SIZE);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc20_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    OPNAME ## h264_qpel ## SIZE ## _h_lowpass_ ## CODETYPE(dst, src, stride, stride);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc30_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    DECLARE_ALIGNED(16, uint8_t, half)[SIZE*SIZE];\
    put_h264_qpel ## SIZE ## _h_lowpass_ ## CODETYPE(half, src, SIZE, stride);\
    OPNAME ## pixels ## SIZE ## _l2_ ## CODETYPE(dst, src+1, half, stride, stride, SIZE);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc01_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    DECLARE_ALIGNED(16, uint8_t, half)[SIZE*SIZE];\
    put_h264_qpel ## SIZE ## _v_lowpass_ ## CODETYPE(half, src, SIZE, stride);\
    OPNAME ## pixels ## SIZE ## _l2_ ## CODETYPE(dst, src, half, stride, stride, SIZE);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc02_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    OPNAME ## h264_qpel ## SIZE ## _v_lowpass_ ## CODETYPE(dst, src, stride, stride);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc03_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    DECLARE_ALIGNED(16, uint8_t, half)[SIZE*SIZE];\
    put_h264_qpel ## SIZE ## _v_lowpass_ ## CODETYPE(half, src, SIZE, stride);\
    OPNAME ## pixels ## SIZE ## _l2_ ## CODETYPE(dst, src+stride, half, stride, stride, SIZE);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc11_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    DECLARE_ALIGNED(16, uint8_t, halfH)[SIZE*SIZE];\
    DECLARE_ALIGNED(16, uint8_t, halfV)[SIZE*SIZE];\
    put_h264_qpel ## SIZE ## _h_lowpass_ ## CODETYPE(halfH, src, SIZE, stride);\
    put_h264_qpel ## SIZE ## _v_lowpass_ ## CODETYPE(halfV, src, SIZE, stride);\
    OPNAME ## pixels ## SIZE ## _l2_ ## CODETYPE(dst, halfH, halfV, stride, SIZE, SIZE);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc31_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    DECLARE_ALIGNED(16, uint8_t, halfH)[SIZE*SIZE];\
    DECLARE_ALIGNED(16, uint8_t, halfV)[SIZE*SIZE];\
    put_h264_qpel ## SIZE ## _h_lowpass_ ## CODETYPE(halfH, src, SIZE, stride);\
    put_h264_qpel ## SIZE ## _v_lowpass_ ## CODETYPE(halfV, src+1, SIZE, stride);\
    OPNAME ## pixels ## SIZE ## _l2_ ## CODETYPE(dst, halfH, halfV, stride, SIZE, SIZE);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc13_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    DECLARE_ALIGNED(16, uint8_t, halfH)[SIZE*SIZE];\
    DECLARE_ALIGNED(16, uint8_t, halfV)[SIZE*SIZE];\
    put_h264_qpel ## SIZE ## _h_lowpass_ ## CODETYPE(halfH, src + stride, SIZE, stride);\
    put_h264_qpel ## SIZE ## _v_lowpass_ ## CODETYPE(halfV, src, SIZE, stride);\
    OPNAME ## pixels ## SIZE ## _l2_ ## CODETYPE(dst, halfH, halfV, stride, SIZE, SIZE);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc33_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    DECLARE_ALIGNED(16, uint8_t, halfH)[SIZE*SIZE];\
    DECLARE_ALIGNED(16, uint8_t, halfV)[SIZE*SIZE];\
    put_h264_qpel ## SIZE ## _h_lowpass_ ## CODETYPE(halfH, src + stride, SIZE, stride);\
    put_h264_qpel ## SIZE ## _v_lowpass_ ## CODETYPE(halfV, src+1, SIZE, stride);\
    OPNAME ## pixels ## SIZE ## _l2_ ## CODETYPE(dst, halfH, halfV, stride, SIZE, SIZE);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc22_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    DECLARE_ALIGNED(16, int16_t, tmp)[SIZE*(SIZE+8)];\
    OPNAME ## h264_qpel ## SIZE ## _hv_lowpass_ ## CODETYPE(dst, tmp, src, stride, SIZE, stride);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc21_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    DECLARE_ALIGNED(16, uint8_t, halfH)[SIZE*SIZE];\
    DECLARE_ALIGNED(16, uint8_t, halfHV)[SIZE*SIZE];\
    DECLARE_ALIGNED(16, int16_t, tmp)[SIZE*(SIZE+8)];\
    put_h264_qpel ## SIZE ## _h_lowpass_ ## CODETYPE(halfH, src, SIZE, stride);\
    put_h264_qpel ## SIZE ## _hv_lowpass_ ## CODETYPE(halfHV, tmp, src, SIZE, SIZE, stride);\
    OPNAME ## pixels ## SIZE ## _l2_ ## CODETYPE(dst, halfH, halfHV, stride, SIZE, SIZE);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc23_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    DECLARE_ALIGNED(16, uint8_t, halfH)[SIZE*SIZE];\
    DECLARE_ALIGNED(16, uint8_t, halfHV)[SIZE*SIZE];\
    DECLARE_ALIGNED(16, int16_t, tmp)[SIZE*(SIZE+8)];\
    put_h264_qpel ## SIZE ## _h_lowpass_ ## CODETYPE(halfH, src + stride, SIZE, stride);\
    put_h264_qpel ## SIZE ## _hv_lowpass_ ## CODETYPE(halfHV, tmp, src, SIZE, SIZE, stride);\
    OPNAME ## pixels ## SIZE ## _l2_ ## CODETYPE(dst, halfH, halfHV, stride, SIZE, SIZE);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc12_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    DECLARE_ALIGNED(16, uint8_t, halfV)[SIZE*SIZE];\
    DECLARE_ALIGNED(16, uint8_t, halfHV)[SIZE*SIZE];\
    DECLARE_ALIGNED(16, int16_t, tmp)[SIZE*(SIZE+8)];\
    put_h264_qpel ## SIZE ## _v_lowpass_ ## CODETYPE(halfV, src, SIZE, stride);\
    put_h264_qpel ## SIZE ## _hv_lowpass_ ## CODETYPE(halfHV, tmp, src, SIZE, SIZE, stride);\
    OPNAME ## pixels ## SIZE ## _l2_ ## CODETYPE(dst, halfV, halfHV, stride, SIZE, SIZE);\
}\
\
static void OPNAME ## h264_qpel ## SIZE ## _mc32_ ## CODETYPE(uint8_t *dst, const uint8_t *src, ptrdiff_t stride)\
{\
    DECLARE_ALIGNED(16, uint8_t, halfV)[SIZE*SIZE];\
    DECLARE_ALIGNED(16, uint8_t, halfHV)[SIZE*SIZE];\
    DECLARE_ALIGNED(16, int16_t, tmp)[SIZE*(SIZE+8)];\
    put_h264_qpel ## SIZE ## _v_lowpass_ ## CODETYPE(halfV, src+1, SIZE, stride);\
    put_h264_qpel ## SIZE ## _hv_lowpass_ ## CODETYPE(halfHV, tmp, src, SIZE, SIZE, stride);\
    OPNAME ## pixels ## SIZE ## _l2_ ## CODETYPE(dst, halfV, halfHV, stride, SIZE, SIZE);\
}\

#if HAVE_BIGENDIAN
#define put_unligned_store(s, dest) {    \
    tmp1 = vec_ld(0, dest);              \
    mask = vec_lvsl(0, dest);            \
    tmp2 = vec_ld(15, dest);             \
    edges = vec_perm(tmp2, tmp1, mask);  \
    align = vec_lvsr(0, dest);           \
    tmp2 = vec_perm(s, edges, align);    \
    tmp1 = vec_perm(edges, s, align);    \
    vec_st(tmp2, 15, dest);              \
    vec_st(tmp1, 0 , dest);              \
 }
#else
#define put_unligned_store(s, dest) vec_vsx_st(s, 0, dest);
#endif /* HAVE_BIGENDIAN */

static inline void put_pixels16_l2_altivec( uint8_t * dst, const uint8_t * src1,
                                    const uint8_t * src2, int dst_stride,
                                    int src_stride1, int h)
{
    int i;
    vec_u8 a, b, d, mask_;
#if HAVE_BIGENDIAN
    vec_u8 tmp1, tmp2, mask, edges, align;
    mask_ = vec_lvsl(0, src2);
#endif

    for (i = 0; i < h; i++) {
        a = unaligned_load(i * src_stride1, src1);
        b = load_with_perm_vec(i * 16, src2, mask_);
        d = vec_avg(a, b);
        put_unligned_store(d, dst);
        dst += dst_stride;
    }
}

#if HAVE_BIGENDIAN
#define avg_unligned_store(s, dest){            \
    tmp1 = vec_ld(0, dest);                     \
    mask = vec_lvsl(0, dest);                   \
    tmp2 = vec_ld(15, dest);                    \
    a = vec_avg(vec_perm(tmp1, tmp2, mask), s); \
    edges = vec_perm(tmp2, tmp1, mask);         \
    align = vec_lvsr(0, dest);                  \
    tmp2 = vec_perm(a, edges, align);           \
    tmp1 = vec_perm(edges, a, align);           \
    vec_st(tmp2, 15, dest);                     \
    vec_st(tmp1, 0 , dest);                     \
 }
#else
#define avg_unligned_store(s, dest){            \
    a = vec_avg(vec_vsx_ld(0, dst), s);         \
    vec_vsx_st(a, 0, dst);                      \
 }
#endif /* HAVE_BIGENDIAN */

static inline void avg_pixels16_l2_altivec( uint8_t * dst, const uint8_t * src1,
                                    const uint8_t * src2, int dst_stride,
                                    int src_stride1, int h)
{
    int i;
    vec_u8 a, b, d, mask_;

#if HAVE_BIGENDIAN
    vec_u8 tmp1, tmp2, mask, edges, align;
    mask_ = vec_lvsl(0, src2);
#endif

    for (i = 0; i < h; i++) {
        a = unaligned_load(i * src_stride1, src1);
        b = load_with_perm_vec(i * 16, src2, mask_);
        d = vec_avg(a, b);
        avg_unligned_store(d, dst);
        dst += dst_stride;
    }
}

/* Implemented but could be faster
#define put_pixels16_l2_altivec(d,s1,s2,ds,s1s,h) put_pixels16_l2(d,s1,s2,ds,s1s,16,h)
#define avg_pixels16_l2_altivec(d,s1,s2,ds,s1s,h) avg_pixels16_l2(d,s1,s2,ds,s1s,16,h)
 */

H264_MC(put_, 16, altivec)
H264_MC(avg_, 16, altivec)
#endif /* HAVE_ALTIVEC */

av_cold void ff_h264qpel_init_ppc(H264QpelContext *c, int bit_depth)
{
#if HAVE_ALTIVEC
    const int high_bit_depth = bit_depth > 8;

    if (!PPC_ALTIVEC(av_get_cpu_flags()))
        return;

    if (!high_bit_depth) {
#define dspfunc(PFX, IDX, NUM) \
        c->PFX ## _pixels_tab[IDX][ 0] = PFX ## NUM ## _mc00_altivec; \
        c->PFX ## _pixels_tab[IDX][ 1] = PFX ## NUM ## _mc10_altivec; \
        c->PFX ## _pixels_tab[IDX][ 2] = PFX ## NUM ## _mc20_altivec; \
        c->PFX ## _pixels_tab[IDX][ 3] = PFX ## NUM ## _mc30_altivec; \
        c->PFX ## _pixels_tab[IDX][ 4] = PFX ## NUM ## _mc01_altivec; \
        c->PFX ## _pixels_tab[IDX][ 5] = PFX ## NUM ## _mc11_altivec; \
        c->PFX ## _pixels_tab[IDX][ 6] = PFX ## NUM ## _mc21_altivec; \
        c->PFX ## _pixels_tab[IDX][ 7] = PFX ## NUM ## _mc31_altivec; \
        c->PFX ## _pixels_tab[IDX][ 8] = PFX ## NUM ## _mc02_altivec; \
        c->PFX ## _pixels_tab[IDX][ 9] = PFX ## NUM ## _mc12_altivec; \
        c->PFX ## _pixels_tab[IDX][10] = PFX ## NUM ## _mc22_altivec; \
        c->PFX ## _pixels_tab[IDX][11] = PFX ## NUM ## _mc32_altivec; \
        c->PFX ## _pixels_tab[IDX][12] = PFX ## NUM ## _mc03_altivec; \
        c->PFX ## _pixels_tab[IDX][13] = PFX ## NUM ## _mc13_altivec; \
        c->PFX ## _pixels_tab[IDX][14] = PFX ## NUM ## _mc23_altivec; \
        c->PFX ## _pixels_tab[IDX][15] = PFX ## NUM ## _mc33_altivec

        dspfunc(put_h264_qpel, 0, 16);
        dspfunc(avg_h264_qpel, 0, 16);
#undef dspfunc
    }
#endif /* HAVE_ALTIVEC */
}
