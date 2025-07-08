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
#define fng3(a,b,c,d,e) a##_##b##_##c##_to_##d##_##e
#define fng2(a,b,c,d,e) fng3(a,b,c,d,e)
#define fng(a)          fng2(a, SRC_F, SRC_E, DST_F, DST_E)

[SRC_F][SRC_E][DST_F][DST_E] = fng(pf2pf_generic_loop),

#endif
