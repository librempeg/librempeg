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

#if (SRC_DEPTH > 8 || SRC_E == 0) && (DST_DEPTH > 8 || DST_E == 0)
#define fns3(a,b,c,d,e,f,g) a##_##b##_##c##_##d##_to_##e##_##f##_##g
#define fns2(a,b,c,d,e,f,g) fns3(a,b,c,d,e,f,g)
#define fns(a)              fns2(a, SRC_F, SRC_E, SRC_OFFSET_NAME, DST_F, DST_E, DST_OFFSET_NAME)

[SRC_F][SRC_E][SRC_OFFSET_NAME][DST_F][DST_E][DST_OFFSET_NAME] = fns(pf2pf_special_loop),

#endif
