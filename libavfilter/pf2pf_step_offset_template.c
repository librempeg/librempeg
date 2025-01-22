/*
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

#include "avfilter.h"
#include "video.h"

#undef DST_STEP
#undef DST_OFFSET
#undef DST_OFFSET_NAME

#define DST_STEP        1
#define DST_OFFSET      0
#define DST_OFFSET_NAME 0
#include "pf2pf_special_template.c"

#undef DST_STEP
#undef DST_OFFSET
#undef DST_OFFSET_NAME

#define DST_STEP         2
#define DST_OFFSET      -1
#define DST_OFFSET_NAME  1
#include "pf2pf_special_template.c"

#undef DST_STEP
#undef DST_OFFSET
#undef DST_OFFSET_NAME

#define DST_STEP        2
#define DST_OFFSET      0
#define DST_OFFSET_NAME 2
#include "pf2pf_special_template.c"

#undef DST_STEP
#undef DST_OFFSET
#undef DST_OFFSET_NAME

#define DST_STEP        2
#define DST_OFFSET      1
#define DST_OFFSET_NAME 3
#include "pf2pf_special_template.c"

#undef DST_STEP
#undef DST_OFFSET
#undef DST_OFFSET_NAME

#define DST_STEP        3
#define DST_OFFSET      0
#define DST_OFFSET_NAME 4
#include "pf2pf_special_template.c"

#undef DST_STEP
#undef DST_OFFSET
#undef DST_OFFSET_NAME

#define DST_STEP        3
#define DST_OFFSET      1
#define DST_OFFSET_NAME 5
#include "pf2pf_special_template.c"

#undef DST_STEP
#undef DST_OFFSET
#undef DST_OFFSET_NAME

#define DST_STEP        3
#define DST_OFFSET      2
#define DST_OFFSET_NAME 6
#include "pf2pf_special_template.c"
