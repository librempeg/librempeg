/*
 * RAW muxers
 * Copyright (C) 2007  Aurelien Jacobs <aurel@gnuage.org>
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

#ifndef AVFORMAT_RAWENC_H
#define AVFORMAT_RAWENC_H

#include "avformat.h"

int ff_raw_write_packet(AVFormatContext *s, AVPacket *pkt);

#endif /* AVFORMAT_RAWENC_H */
