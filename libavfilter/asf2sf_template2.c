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

#include "avfilter.h"
#include "audio.h"

#undef stype
#undef SRC_F
#if SRC_DEPTH == 8
#define stype uint8_t
#define SRC_F u8
#elif SRC_DEPTH == 16
#define stype int16_t
#define SRC_F s16
#elif SRC_DEPTH == 31
#define stype int32_t
#define SRC_F s32
#elif SRC_DEPTH == 32
#define stype float
#define SRC_F flt
#elif SRC_DEPTH == 63
#define stype int64_t
#define SRC_F s64
#else
#define stype double
#define SRC_F dbl
#endif

#define fnc3(a,b,c) a##_##b##_to_##c
#define fnc2(a,b,c) fnc3(a,b,c)
#define fnc(a) fnc2(a, SRC_F, DST_F)

static dtype fnc(convert)(stype src)
{
    dtype dst;
#if   (SRC_DEPTH ==  8) && (DST_DEPTH == 16)
    dst = (src-0x80)<<8;
#elif (SRC_DEPTH ==  8) && (DST_DEPTH == 31)
    dst = (src-0x80)<<24;
#elif (SRC_DEPTH == 16) && (DST_DEPTH ==  8)
    dst = (src>>8)+0x80;
#elif (SRC_DEPTH == 16) && (DST_DEPTH == 31)
    dst = src*(1<<16);
#elif (SRC_DEPTH == 16) && (DST_DEPTH == 32)
    dst = src*(1.f/(1<<15));
#elif (SRC_DEPTH == 16) && (DST_DEPTH == 63)
    dst = src*(1LL<<48);
#elif (SRC_DEPTH == 16) && (DST_DEPTH == 64)
    dst = src*(1.0/(1<<15));
#elif (SRC_DEPTH == 31) && (DST_DEPTH ==  8)
    dst = (src>>24)+0x80;
#elif (SRC_DEPTH == 31) && (DST_DEPTH == 16)
    dst = (src>>16);
#elif (SRC_DEPTH == 31) && (DST_DEPTH == 32)
    dst = src*(1.0/(1U<<31));
#elif (SRC_DEPTH == 31) && (DST_DEPTH == 63)
    dst = src*(1LL<<32);
#elif (SRC_DEPTH == 31) && (DST_DEPTH == 64)
    dst = src*(1.0/(1U<<31));
#elif (SRC_DEPTH == 32) && (DST_DEPTH ==  8)
    int16_t t = lrintf(fminf(fmaxf(src,-1.f),1.f)*(1<<7));
    dst = t - (t == 128) + 0x80;
#elif (SRC_DEPTH == 32) && (DST_DEPTH == 16)
    int32_t t = lrintf(fminf(fmaxf(src,-1.f),1.f)*(1<<15));
    dst = t - (t == 32768);
#elif (SRC_DEPTH == 32) && (DST_DEPTH == 31)
    int64_t t = llrintf(fminf(fmaxf(src,-1.f),1.f)*(1U<<31));
    dst = t - (t == 2147483648LL);
#elif (SRC_DEPTH == 32) && (DST_DEPTH == 63)
    dst = lrintf(fminf(fmaxf(src,-1.f),1.f)*((1ULL<<63)-1));
#elif (SRC_DEPTH == 64) && (DST_DEPTH ==  8)
    int16_t t = lrint(fmin(fmax(src,-1.0),1.0)*(1<<7));
    dst = t - (t == 128) + 0x80;
#elif (SRC_DEPTH == 64) && (DST_DEPTH == 16)
    int32_t t = lrint(fmin(fmax(src,-1.0),1.0)*(1<<15));
    dst = t - (t == 32768);
#elif (SRC_DEPTH == 64) && (DST_DEPTH == 31)
    int64_t t = llrint(fmin(fmax(src,-1.0),1.0)*(1U<<31));
    dst = t - (t == 2147483648LL);
#elif (SRC_DEPTH == 64) && (DST_DEPTH == 63)
    dst = llrint(fmin(fmax(src,-1.0),1.0)*((1ULL<<63)-1));
#elif (SRC_DEPTH == 63) && (DST_DEPTH ==  8)
    dst = (src>>56) + 0x80;
#elif (SRC_DEPTH == 63) && (DST_DEPTH == 16)
    dst = (src>>48);
#elif (SRC_DEPTH == 63) && (DST_DEPTH == 31)
    dst = (src>>32);
#elif (SRC_DEPTH == 63) && (DST_DEPTH == 32)
    dst = (src/(double)(1ULL<<63));
#elif (SRC_DEPTH == 63) && (DST_DEPTH == 64)
    dst = (src/(double)(1ULL<<63));
#else
    dst = src;
#endif
    return dst;
}

#undef spidx
#undef SRC_P
#undef SRC_p
#undef fn_src_ptr
#define spidx m
#define SRC_P packed
#define SRC_p 0
#define fn_src_ptr(x, ch, n) (((const stype *restrict)(x)->data[0]) + ((n)*nb_channels) + (ch))
#include "asf2sf_template3.c"

#undef spidx
#undef SRC_P
#undef SRC_p
#undef fn_src_ptr
#define spidx n
#define SRC_P planar
#define SRC_p 1
#define fn_src_ptr(x, ch, n) ((const stype *restrict)(x)->extended_data[(ch)] + (n))
#include "asf2sf_template3.c"
