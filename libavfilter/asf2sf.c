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

#include "libavutil/samplefmt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"
#include "asf2sf.h"

#define DST_DEPTH 8
#include "asf2sf_dst_depth_template.c"

#undef DST_DEPTH
#define DST_DEPTH 16
#include "asf2sf_dst_depth_template.c"

#undef DST_DEPTH
#define DST_DEPTH 31
#include "asf2sf_dst_depth_template.c"

#undef DST_DEPTH
#define DST_DEPTH 32
#include "asf2sf_dst_depth_template.c"

#undef DST_DEPTH
#define DST_DEPTH 63
#include "asf2sf_dst_depth_template.c"

#undef DST_DEPTH
#define DST_DEPTH 64
#include "asf2sf_dst_depth_template.c"

#undef DST_DEPTH
#define DST_DEPTH 80
#include "asf2sf_dst_depth_template.c"

int ff_asf2sf_setup(const int outlink_format, const int inlink_format,
                   ff_asf2sf_fn *do_sf2sf)
{
    switch (inlink_format) {
    case AV_SAMPLE_FMT_U8P:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_planar_u8_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_planar_u8_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_planar_u8_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_planar_u8_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_planar_u8_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_planar_u8_to_packed_dbl; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_planar_u8_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_planar_u8_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_planar_u8_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_planar_u8_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_planar_u8_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_planar_u8_to_planar_dbl; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_planar_u8_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_S16P:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_planar_s16_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_planar_s16_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_planar_s16_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_planar_s16_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_planar_s16_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_planar_s16_to_packed_dbl; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_planar_s16_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_planar_s16_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_planar_s16_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_planar_s16_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_planar_s16_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_planar_s16_to_planar_dbl; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_planar_s16_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_S32P:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_planar_s32_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_planar_s32_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_planar_s32_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_planar_s32_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_planar_s32_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_planar_s32_to_packed_dbl; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_planar_s32_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_planar_s32_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_planar_s32_to_planar_s16; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_planar_s32_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_planar_s32_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_planar_s32_to_planar_dbl; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_planar_s32_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_FLTP:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_planar_flt_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_planar_flt_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_planar_flt_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_planar_flt_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_planar_flt_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_planar_flt_to_packed_dbl; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_planar_flt_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_planar_flt_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_planar_flt_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_planar_flt_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_planar_flt_to_planar_s64; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_planar_flt_to_planar_dbl; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_planar_flt_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_S64P:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_planar_s64_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_planar_s64_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_planar_s64_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_planar_s64_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_planar_s64_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_planar_s64_to_packed_dbl; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_planar_s64_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_planar_s64_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_planar_s64_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_planar_s64_to_planar_s32; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_planar_s64_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_planar_s64_to_planar_dbl; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_planar_s64_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_DBLP:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_planar_dbl_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_planar_dbl_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_planar_dbl_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_planar_dbl_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_planar_dbl_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_planar_dbl_to_packed_dbl; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_planar_dbl_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_planar_dbl_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_planar_dbl_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_planar_dbl_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_planar_dbl_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_planar_dbl_to_planar_flt; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_planar_dbl_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_LDBLP:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_planar_ldbl_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_planar_ldbl_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_planar_ldbl_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_planar_ldbl_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_planar_ldbl_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_planar_ldbl_to_packed_dbl; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_planar_ldbl_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_planar_ldbl_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_planar_ldbl_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_planar_ldbl_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_planar_ldbl_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_planar_ldbl_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_planar_ldbl_to_planar_dbl; break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_U8:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_packed_u8_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_packed_u8_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_packed_u8_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_packed_u8_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_packed_u8_to_packed_dbl; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_packed_u8_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_packed_u8_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_packed_u8_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_packed_u8_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_packed_u8_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_packed_u8_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_packed_u8_to_planar_dbl; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_packed_u8_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_S16:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_packed_s16_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_packed_s16_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_packed_s16_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_packed_s16_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_packed_s16_to_packed_dbl; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_packed_s16_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_packed_s16_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_packed_s16_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_packed_s16_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_packed_s16_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_packed_s16_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_packed_s16_to_planar_dbl; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_packed_s16_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_S32:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_packed_s32_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_packed_s32_to_packed_s16; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_packed_s32_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_packed_s32_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_packed_s32_to_packed_dbl; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_packed_s32_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_packed_s32_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_packed_s32_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_packed_s32_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_packed_s32_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_packed_s32_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_packed_s32_to_planar_dbl; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_packed_s32_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_FLT:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_packed_flt_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_packed_flt_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_packed_flt_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_packed_flt_to_packed_s64; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_packed_flt_to_packed_dbl; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_packed_flt_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_packed_flt_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_packed_flt_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_packed_flt_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_packed_flt_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_packed_flt_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_packed_flt_to_planar_dbl; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_packed_flt_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_DBL:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_packed_dbl_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_packed_dbl_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_packed_dbl_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_packed_dbl_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_packed_dbl_to_packed_flt; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_packed_dbl_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_packed_dbl_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_packed_dbl_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_packed_dbl_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_packed_dbl_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_packed_dbl_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_packed_dbl_to_planar_dbl; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_packed_dbl_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_LDBL:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_packed_ldbl_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_packed_ldbl_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_packed_ldbl_to_packed_s32; break;
        case AV_SAMPLE_FMT_S64:  do_sf2sf[0] = sf2sf_packed_ldbl_to_packed_s64; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_packed_ldbl_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_packed_ldbl_to_packed_dbl; break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_packed_ldbl_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_packed_ldbl_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_packed_ldbl_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_packed_ldbl_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_packed_ldbl_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_packed_ldbl_to_planar_dbl; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_packed_ldbl_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    case AV_SAMPLE_FMT_S64:
        switch (outlink_format) {
        case AV_SAMPLE_FMT_U8:   do_sf2sf[0] = sf2sf_packed_s64_to_packed_u8;  break;
        case AV_SAMPLE_FMT_S16:  do_sf2sf[0] = sf2sf_packed_s64_to_packed_s16; break;
        case AV_SAMPLE_FMT_S32:  do_sf2sf[0] = sf2sf_packed_s64_to_packed_s32; break;
        case AV_SAMPLE_FMT_FLT:  do_sf2sf[0] = sf2sf_packed_s64_to_packed_flt; break;
        case AV_SAMPLE_FMT_DBL:  do_sf2sf[0] = sf2sf_packed_s64_to_packed_dbl; break;
        case AV_SAMPLE_FMT_LDBL: do_sf2sf[0] = sf2sf_packed_s64_to_packed_ldbl;break;
        case AV_SAMPLE_FMT_U8P:  do_sf2sf[0] = sf2sf_packed_s64_to_planar_u8;  break;
        case AV_SAMPLE_FMT_S16P: do_sf2sf[0] = sf2sf_packed_s64_to_planar_s16; break;
        case AV_SAMPLE_FMT_S32P: do_sf2sf[0] = sf2sf_packed_s64_to_planar_s32; break;
        case AV_SAMPLE_FMT_S64P: do_sf2sf[0] = sf2sf_packed_s64_to_planar_s64; break;
        case AV_SAMPLE_FMT_FLTP: do_sf2sf[0] = sf2sf_packed_s64_to_planar_flt; break;
        case AV_SAMPLE_FMT_DBLP: do_sf2sf[0] = sf2sf_packed_s64_to_planar_dbl; break;
        case AV_SAMPLE_FMT_LDBLP:do_sf2sf[0] = sf2sf_packed_s64_to_planar_ldbl;break;
        default: return AVERROR_BUG;
        }
        break;
    default:
        return AVERROR_BUG;
    }

    return 0;
}
