/*
 * SADL HexPattern
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

import std.array;

enum SampleRate:u8 {
    _16364,
    _16364,
    _32728,
};
enum Codec:u8 {
    IMA = 0,
    IMA = 7,
    PROCYON = 11,
};
bitfield Flags {
    unk1 : 1;
    SampleRate sample_rate : 2;
    unk2 : 1;
    Codec codec : 4;
};
struct IMAHistStep {
    s16 hist;
    s16 step;
};

struct Header {
    char magic[4];
    padding[4];
    u32 file_size;
    u16 version;
    u16 id;
    u8 unk1[0x10];
    char name[0x10];
    u8 unk2;
    bool loop_flag;
    u8 channels;
    Flags flags;
    u32 unk3;
    u32 unk4;
    padding[4];
    u32 end_offset;
    u32 data_size;
    u32 start_offset;
    u32 unk5;
    u32 num_samples;
    u32 loop_start;
    u32 unk6;
    padding[4];
    u32 unk7;
    padding[4];
    u32 unk8;
    padding[4];

    if (flags.codec == Codec::IMA)
        IMAHistStep ima_hist_step[channels] @ 0x80;
} [[inline]] ;

Header Header @ 0;