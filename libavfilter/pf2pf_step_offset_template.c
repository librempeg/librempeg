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
#include "video.h"

#undef DST_OFFSET_NAME
#define DST_OFFSET_NAME 0
#include "pf2pf_special_template.c"

#undef DST_OFFSET_NAME
#define DST_OFFSET_NAME 1
#include "pf2pf_special_template.c"

#undef DST_OFFSET_NAME
#define DST_OFFSET_NAME 2
#include "pf2pf_special_template.c"

#undef DST_OFFSET_NAME
#define DST_OFFSET_NAME 3
#include "pf2pf_special_template.c"

#undef DST_OFFSET_NAME
#define DST_OFFSET_NAME 4
#include "pf2pf_special_template.c"

#undef DST_OFFSET_NAME
#define DST_OFFSET_NAME 5
#include "pf2pf_special_template.c"

#undef DST_OFFSET_NAME
#define DST_OFFSET_NAME 6
#include "pf2pf_special_template.c"

#undef DST_OFFSET_NAME
#define DST_OFFSET_NAME 7
#include "pf2pf_special_template.c"

#undef DST_OFFSET_NAME
#define DST_OFFSET_NAME 8
#include "pf2pf_special_template.c"

#undef DST_OFFSET_NAME
#define DST_OFFSET_NAME 9
#include "pf2pf_special_template.c"

#undef DST_OFFSET_NAME
#define DST_OFFSET_NAME 10
#include "pf2pf_special_template.c"
