/*
 * Copyright (c) 2010 Mans Rullgard
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

#ifndef AVCODEC_ARM_ASM_OFFSETS_H
#define AVCODEC_ARM_ASM_OFFSETS_H

/* MpegEncContext */
#define Y_DC_SCALE               0x04
#define C_DC_SCALE               0x08
#define AC_PRED                  0x0c
#define BLOCK_LAST_INDEX         0x10
#define H263_AIC                 0x40
#define INTER_SCANTAB_RASTER_END 0x88

#endif /* AVCODEC_ARM_ASM_OFFSETS_H */
