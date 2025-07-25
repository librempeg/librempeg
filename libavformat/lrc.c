/*
 * LRC lyrics file format common structs
 * Copyright (c) 2014 StarBrilliant <m13253@hotmail.com>
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

#include "metadata.h"
#include "lrc.h"

const AVMetadataConv ff_lrc_metadata_conv[] = {
    {"ti", "title"},
    {"al", "album"},
    {"ar", "artist"},
    {"au", "author"},
    {"by", "creator"},
    {"re", "encoder"},
    {"ve", "encoder_version"},
    {0, 0}
};
