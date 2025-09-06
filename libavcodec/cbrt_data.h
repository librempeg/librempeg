/*
 * Copyright (c) 2016 Reimar Döffinger <Reimar.Doeffinger@gmx.de>
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

#ifndef AVCODEC_CBRT_DATA_H
#define AVCODEC_CBRT_DATA_H

#include <stdint.h>

#define LUT_SIZE     (1 << 13)

#ifndef BUILD_TABLES
#include "config.h"
#define BUILD_TABLES !CONFIG_HARDCODED_TABLES
#endif

#if !BUILD_TABLES
#define ff_cbrt_tableinit_fixed()
#define ff_cbrt_tableinit()
extern const uint32_t ff_cbrt_tab[LUT_SIZE];
extern const uint32_t ff_cbrt_tab_fixed[LUT_SIZE];
#else
void ff_cbrt_tableinit(void);
void ff_cbrt_tableinit_fixed(void);

#define TMP_LUT_SIZE (LUT_SIZE / 2)

extern union CBRT {
    uint32_t cbrt_tab[LUT_SIZE];
    double tmp[TMP_LUT_SIZE];
} ff_cbrt_tab_internal, ff_cbrt_tab_internal_fixed;

#define ff_cbrt_tab       ff_cbrt_tab_internal.cbrt_tab
#define ff_cbrt_tab_fixed ff_cbrt_tab_internal_fixed.cbrt_tab
#endif

#endif
