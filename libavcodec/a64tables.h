/*
 * a64 video encoder - tables used by a64 encoders
 * Copyright (c) 2009 Tobias Bindhammer
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

/**
 * @file
 * a64 video encoder - tables used by a64 encoders
 */

#ifndef AVCODEC_A64TABLES_H
#define AVCODEC_A64TABLES_H

#include <stdint.h>

/**
 * dither patterns used vor rendering the multicolor charset
 */

static const uint8_t multi_dither_patterns[9][4][4] = {
    {
     {0, 0, 0, 0},
     {0, 0, 0, 0},
     {0, 0, 0, 0},
     {0, 0, 0, 0}
     },
    {
     {1, 0, 0, 0},
     {0, 0, 0, 0},
     {0, 0, 1, 0},
     {0, 0, 0, 0}
     },
    {
     {1, 0, 0, 0},
     {0, 0, 1, 0},
     {0, 1, 0, 0},
     {0, 0, 0, 1}
     },
    {
     {1, 0, 0, 0},
     {0, 1, 0, 1},
     {0, 0, 1, 0},
     {0, 1, 0, 1}
     },
    {
     {1, 0, 1, 0},
     {0, 1, 0, 1},
     {1, 0, 1, 0},
     {0, 1, 0, 1}
     },
    {
     {1, 1, 1, 0},
     {0, 1, 0, 1},
     {1, 0, 1, 1},
     {0, 1, 0, 1}
     },
    {
     {0, 1, 1, 1},
     {1, 1, 0, 1},
     {1, 0, 1, 1},
     {1, 1, 1, 0}
     },
    {
     {0, 1, 1, 1},
     {1, 1, 1, 1},
     {1, 1, 0, 1},
     {1, 1, 1, 1}
     },
    {
     {1, 1, 1, 1},
     {1, 1, 1, 1},
     {1, 1, 1, 1},
     {1, 1, 1, 1}
     },
};

static const uint8_t interlaced_dither_patterns[9][8][4] = {
    {
     {0, 0, 0, 0}, {0, 0, 0, 0},
     {0, 0, 0, 0}, {0, 0, 0, 0},
     {0, 0, 0, 0}, {0, 0, 0, 0},
     {0, 0, 0, 0}, {0, 0, 0, 0},
     },
    {
     {1, 0, 1, 0}, {0, 0, 0, 0},
     {0, 0, 0, 0}, {0, 0, 0, 0},
     {1, 0, 1, 0}, {0, 0, 0, 0},
     {0, 0, 0, 0}, {0, 0, 0, 0},
     },
    {
     {1, 0, 1, 0}, {0, 0, 0, 0},
     {0, 0, 0, 0}, {0, 1, 0, 1},
     {1, 0, 1, 0}, {0, 0, 0, 0},
     {0, 0, 0, 0}, {0, 1, 0, 1},
     },
    {
     {1, 0, 1, 0}, {0, 1, 0, 1},
     {0, 1, 0, 1}, {0, 0, 0, 0},
     {1, 0, 1, 0}, {0, 1, 0, 1},
     {0, 1, 0, 1}, {0, 0, 0, 0},
     },
    {
     {1, 0, 1, 0}, {0, 1, 0, 1},
     {0, 1, 0, 1}, {1, 0, 1, 0},
     {1, 0, 1, 0}, {0, 1, 0, 1},
     {0, 1, 0, 1}, {1, 0, 1, 0},
     },
    {
     {1, 0, 1, 0}, {0, 1, 0, 1},
     {1, 1, 1, 1}, {1, 0, 1, 0},
     {1, 0, 1, 0}, {0, 1, 0, 1},
     {1, 1, 1, 1}, {1, 0, 1, 0},
     },
    {
     {1, 0, 1, 0}, {1, 1, 1, 1},
     {1, 1, 1, 1}, {0, 1, 0, 1},
     {1, 0, 1, 0}, {1, 1, 1, 1},
     {1, 1, 1, 1}, {0, 1, 0, 1},
     },
    {
     {1, 1, 1, 1}, {1, 1, 1, 1},
     {1, 1, 1, 1}, {0, 1, 0, 1},
     {1, 1, 1, 1}, {1, 1, 1, 1},
     {1, 1, 1, 1}, {0, 1, 0, 1},
     },
    {
     {1, 1, 1, 1}, {1, 1, 1, 1},
     {1, 1, 1, 1}, {1, 1, 1, 1},
     {1, 1, 1, 1}, {1, 1, 1, 1},
     {1, 1, 1, 1}, {1, 1, 1, 1},
     }
};

#endif /* AVCODEC_A64TABLES_H */
