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

#include "config.h"

#include "libavutil/cpu.h"
#include "libavcodec/aarch64/fdct.h"
#include "libavcodec/aarch64/idct.h"

static const struct algo fdct_tab_arch[] = {
    { "neon", ff_fdct_neon, FF_IDCT_PERM_NONE, AV_CPU_FLAG_NEON },
    { 0 }
};

static const struct algo idct_tab_arch[] = {
    { "SIMPLE-NEON", ff_simple_idct_neon, FF_IDCT_PERM_PARTTRANS, AV_CPU_FLAG_NEON },
    { 0 }
};
