/*
 * WavPack (v1-v3) decoder/encoder common code
 * Copyright (c) 2026 Librempeg
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

#ifndef AVCODEC_WAVPACK_OLD_H
#define AVCODEC_WAVPACK_OLD_H

#include <stdint.h>

typedef struct WavPackV3Header {
    uint16_t version;  //< bitstream version
    uint16_t minbits;  //< minimum bit depth needed for lossy coding
    uint16_t flags;    //< WavPack v3 (de/en)coding flags
    uint16_t shift;    //< WavPack v3 shift number
} WavPackV2Header;

#endif /* AVCODEC_WAVPACK_OLD_H */
