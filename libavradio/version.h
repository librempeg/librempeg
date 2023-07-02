/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef AVRADIO_VERSION_H
#define AVRADIO_VERSION_H

/**
 * @file
 * @ingroup lavr
 * Libavradio version macros
 */

#include "libavutil/version.h"

#include "version_major.h"

#define LIBAVRADIO_VERSION_MINOR   0
#define LIBAVRADIO_VERSION_MICRO 100

#define LIBAVRADIO_VERSION_INT AV_VERSION_INT(LIBAVRADIO_VERSION_MAJOR, \
                                               LIBAVRADIO_VERSION_MINOR, \
                                               LIBAVRADIO_VERSION_MICRO)
#define LIBAVRADIO_VERSION     AV_VERSION(LIBAVRADIO_VERSION_MAJOR, \
                                           LIBAVRADIO_VERSION_MINOR, \
                                           LIBAVRADIO_VERSION_MICRO)
#define LIBAVRADIO_BUILD       LIBAVRADIO_VERSION_INT

#define LIBAVRADIO_IDENT       "Lavr" AV_STRINGIFY(LIBAVRADIO_VERSION)

#endif /* AVRADIO_VERSION_H */
