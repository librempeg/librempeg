/*
 * BAKA HexPattern
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

#pragma endian big

import std.mem;

struct MARKBlock {
    u16 num;
    u32 val;
    u8 name_length;
    char name[name_length];
    u8 unk1;
};
struct APPLBlock {
    char tag[];
    u8 flags;
    match(tag) {
    ("Sd2a\0"): {
        $ += 0x13A;
    }
    ("Scale\0"|"  secs\0"): {
        u8 unk1;
        u8 unk2;
        u8 unk3;
    }
    ("FREF\0"|"BNDL\0"|"icl4\0"|"ics8\0"): {
        u8 unk1;
        u8 unk2;
    }
    ("ICN#\0"): {
        padding[2];
        u16 unk2;
        u16 unk3;
        u8 unk4;
    }
    ("ics4\0"): {
        u32 samplec;
        u16 unk1[3];
        padding[10];
        u16 unk2[6];
    }
    }
};
struct Block {
    char tag[4];
    u32 size;
    u64 pos = $;

    match(tag) {
    ("BAKA"): {
        char aho[4];
        Block blocks[while(!std::mem::eof())];
    }
    ("PAPA"): {
        u16 block_size;
        u32 samplec;
        padding[1];
        char unk1[5];
        padding[6];
    }
    ("MAMA"): {
        u16 data[size/2] [[hex::visualize("sound", this, 2, 44100)]];
    }
    ("MARK"): {
        u16 sub;
        MARKBlock blocks[sub];
    }
    ("INST"): {
        u32 unk[5];
    }
    ("APPL"): {
        APPLBlock blocks[while(($-pos) < size)];
    }
    }

    $ = pos + size;
};

Block blocks[while(!std::mem::eof())] @ 0;