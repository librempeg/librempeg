/*
 * XOpus HexPattern
 * Copyright (c) 2025 smiRaphi
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

#pragma endian little

import std.core;

struct Packet {
    u16 size @ (0x20 + std::core::array_index()*2);
    s64 epos = $ + size;
    if (epos > parent.data_end)
        size = epos - parent.data_end;
    char data[size];
};

struct XOpus {
    char magic[4];
    u8 unk1;
    u8 channels;
    u16 unk2;
    u32 max_packet;
    u32 duration;
    u32 skip;
    u32 entries;
    u32 data_size;
    padding[4];

    u16 packet_sizes[entries];
    u64 data_end = $ + data_size;

    Packet packets[entries];    
};

XOpus Header @ 0;
