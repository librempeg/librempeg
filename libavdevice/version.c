/*
 * Version functions.
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

#include <assert.h>

#include "config.h"

#include "avdevice.h"
#include "version.h"

#include "libavutil/ffversion.h"
const char av_device_ffversion[] = "FFmpeg version " FFMPEG_VERSION;

unsigned avdevice_version(void)
{
    static_assert(LIBAVDEVICE_VERSION_MICRO >= 100, "micro version starts at 100");
    return LIBAVDEVICE_VERSION_INT;
}

const char * avdevice_configuration(void)
{
    return FFMPEG_CONFIGURATION;
}

const char * avdevice_license(void)
{
#define LICENSE_PREFIX "libavdevice license: "
    return &LICENSE_PREFIX FFMPEG_LICENSE[sizeof(LICENSE_PREFIX) - 1];
}
