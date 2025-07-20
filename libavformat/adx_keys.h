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

#ifndef AVFORMAT_ADX_KEYS
#define AVFORMAT_ADX_KEYS

#include <stdint.h>

int ff_adx_find_key(const int type,
                    const uint16_t *prescales, const int nb_prescales,
                    const uint16_t *scales, const int nb_scales,
                    uint16_t *xor_start, uint16_t *xor_mult, uint16_t *xor_add,
                    const uint16_t subkey);

#endif /* AVFORMAT_ADX_KEYS */
