/*
 * libopus encoder/decoder common code
 * Copyright (c) 2012 Nicolas George
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

#include <opus_defines.h>

#include "libavutil/error.h"
#include "libopus.h"

int ff_opus_error_to_averror(int err)
{
    switch (err) {
    case OPUS_BAD_ARG:
        return AVERROR(EINVAL);
    case OPUS_BUFFER_TOO_SMALL:
        return AVERROR_UNKNOWN;
    case OPUS_INTERNAL_ERROR:
        return AVERROR(EFAULT);
    case OPUS_INVALID_PACKET:
        return AVERROR_INVALIDDATA;
    case OPUS_UNIMPLEMENTED:
        return AVERROR(ENOSYS);
    case OPUS_INVALID_STATE:
        return AVERROR_UNKNOWN;
    case OPUS_ALLOC_FAIL:
        return AVERROR(ENOMEM);
    default:
        return AVERROR(EINVAL);
    }
}
