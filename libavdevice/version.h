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

#ifndef AVDEVICE_VERSION_H
#define AVDEVICE_VERSION_H

/**
 * @file
 * @ingroup lavd
 * Libavdevice version macros
 */

#include "libavutil/version.h"

#include "version_major.h"

#define LIBAVDEVICE_VERSION_MINOR   0
#define LIBAVDEVICE_VERSION_MICRO 100

#define LIBAVDEVICE_VERSION_INT AV_VERSION_INT(LIBAVDEVICE_VERSION_MAJOR, \
                                               LIBAVDEVICE_VERSION_MINOR, \
                                               LIBAVDEVICE_VERSION_MICRO)
#define LIBAVDEVICE_VERSION     AV_VERSION(LIBAVDEVICE_VERSION_MAJOR, \
                                           LIBAVDEVICE_VERSION_MINOR, \
                                           LIBAVDEVICE_VERSION_MICRO)
#define LIBAVDEVICE_BUILD       LIBAVDEVICE_VERSION_INT

#define LIBAVDEVICE_IDENT       "Lavd" AV_STRINGIFY(LIBAVDEVICE_VERSION)

#endif /* AVDEVICE_VERSION_H */
