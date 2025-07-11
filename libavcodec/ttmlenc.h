/*
 * TTML subtitle encoder shared functionality
 * Copyright (c) 2020 24i
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

#ifndef AVCODEC_TTMLENC_H
#define AVCODEC_TTMLENC_H

#define TTMLENC_EXTRADATA_SIGNATURE "lavc-ttmlenc"
#define TTMLENC_EXTRADATA_SIGNATURE_SIZE (sizeof(TTMLENC_EXTRADATA_SIGNATURE) - 1)

#define TTML_DEFAULT_NAMESPACING                        \
"  xmlns=\"http://www.w3.org/ns/ttml\"\n"               \
"  xmlns:ttm=\"http://www.w3.org/ns/ttml#metadata\"\n"  \
"  xmlns:tts=\"http://www.w3.org/ns/ttml#styling\"\n"   \
"  xmlns:ttp=\"http://www.w3.org/ns/ttml#parameter\"\n"

#endif /* AVCODEC_TTMLENC_H */
