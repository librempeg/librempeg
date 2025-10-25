/*
 * FSB HexPattern
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

import std.mem;
import std.core;

bitfield Format1 {
    bool format : 1;
    unk1 : 4;
    bool mono : 1;
    bool stereo : 1;
    unk2 : 8;
    unk3 : 8;
    bool adpcm_psx : 1;
    unk4 : 8;
};
struct Entry1 {
    char title[0x20];
    u32 num_samples;
    u32 data_length;
    u32 sample_rate;
    u16 unk1;
    u16 unk2;
    u16 unk3;
    u16 unk4;
    Format1 format;
    u32 loop_start;
    u32 loop_end;
};

struct EntryBasic {
    u32 num_samples;
    u32 data_length;
};

bitfield Format23 {
    bool format : 1;
    padding : 31;
};
bitfield Format3_1 {
    bool format : 1;
    bool basic_headers : 1;
    padding : 30;
};

struct DSPCoefs {
    u16 coefs[16] [[inline]];
    padding[14];
};

bitfield Mode3 {
    bool loop_off : 1;
    bool loop_normal : 1;
    bool loop_bidi : 1;
    bool pcm_8bit : 1;
    bool pcm_16bit : 1;
    bool mono : 1;
    bool stereo : 1;
    bool pcm_unsigned : 1;
    bool pcm_signed : 1;
    bool mpeg : 1;
    bool channelmode_allmono : 1;
    bool channelmode_allstereo : 1;
    bool hw3d : 1;
    bool only_2d : 1;
    bool synpoints_nonames : 1;
    bool duplicate : 1;
    bool channelmode_protools : 1;
    bool mpegaccurate : 1;
    bool mpeg_layer2 : 1;
    bool hw2d : 1;
    bool only_3d : 1;
    bool pcm_32bit : 1;
    bool imaadpcm : 1;
    bool adpcm_psx : 1;
    bool xma : 1;
    bool dsp_adpcm : 1;
    bool multichannel : 1;
    bool ogg_or_celt : 1;
    bool mpeg_layer3 : 1;
    bool load_memory_iop__or__imaadpcm_stereo : 1;
    bool ignore_tags : 1;
    bool synpoints : 1;
};
struct Entry3 {
    u64 pos = $;
    u16 size;
    char name[30];
    u32 num_samples;
    u32 length_data;
    u32 loop_start;
    u32 loop_end;
    Mode3 mode;
    s32 sample_rate;
    u16 volume;
    s16 pan;
    u16 pri;
    u16 channels;

    if (mode.dsp_adpcm) DSPCoefs coefs[channels];

    $ = pos + size;
};
struct Entry3_1 {
    u64 pos = $;
    u16 size;
    char name[30];
    u32 num_samples;
    u32 length_data;
    u32 loop_start;
    u32 loop_end;
    Mode3 mode;
    s32 sample_rate;
    u16 volume;
    s16 pan;
    u16 pri;
    u16 channels;

    float mindistance;
    float maxdistance;
    s32 var_frequency;
    u16 var_volume;
    s16 var_pan;

    if (mode.dsp_adpcm) DSPCoefs coefs[channels];

    $ = pos + size;
};

enum Version34:u32 {
    FSB_3_0 = 0x30000,
    FSB_3_1 = 0x30001,
    FSB_4_0 = 0x40000,
};

struct Entry4 {
    u64 pos = $;
    u16 size;
    char name[30];
    u32 num_samples;
    u32 length_data;
    u32 loop_start;
    u32 loop_end;
    Mode3 mode;
    s32 sample_rate;
    u16 volume;
    s16 pan;
    u16 pri;
    u16 channels;

    float mindistance;
    float maxdistance;
    u32 size_32bits;
    u16 var_volume;
    s16 var_pan;

    if (mode.dsp_adpcm) DSPCoefs coefs[channels];

    $ = pos + size;
};

bitfield Format4 {
    bool format : 1;
    bool basic_headers : 1;
    bool encrypted : 1;
    bool bigendian_pcm : 1;
    bool not_interleaved : 1;
    bool mpeg_padded2 : 1;
    bool mpeg_padded4 : 1;
    padding : 25;
};

enum Codec5:u32 {
    NONE,
    PCM_U8,
    PCM_S16,
    PCM_S24,
    PCM_S32,
    PCM_F32,
    ADPCM_NDSP,
    ADPCM_XBOX,
    ADPCM_PSX,
    ADPCM_HEVAG,
    XMA2,
    MPEG,
    CELT,
    ATRAC9,
    XWMA,
    VORBIS,
    ADPCM_FMOD,
    OPUS,
};
enum SampleRate:u8 {
    _4000,
    _8000,
    _11000,
    _11025,
    _16000,
    _22050,
    _24000,
    _32000,
    _44100,
    _48000,
    _96000,
};
enum Channels:u8 {
    _1,
    _2,
    _6,
    _8
};
bitfield smode {
    bool extra_flags : 1;
    SampleRate sample_rate : 4;
    Channels channels : 2;
    data_offset : 25;
    num_samples : 30;
};

enum XWMAFormat:u16 {
    WMA2 = 0x161,
    WMAPRO = 0x162,
};
struct FreeComment1 {
    u64 pos = $;
    u32 value1;
    char value2[];
    $ = pos + 260;
};
struct VorbisSeekEntry {
    s32 sample;
    u32 offset;
};

enum XFType:u8 {
    channels = 0x1,
    sample_rate,
    loop_info,
    free_comment,
    unknown1,
    xma_seek_table,
    dsp_coefs,
    atrac9_config = 0x9,
    xwma_config,
    vorbis_setup_id_seek_table,
    peak_volume = 0xd,
    vorbis_intra_layers,
    opus_data_size,
};
bitfield xflag {
    bool _continue : 1;
    size : 24;
    XFType type : 7;
};
struct XFlag {
    xflag extraflag;

    s64 pos = $;
    match (extraflag.type) {
        (XFType::channels): u8 channels;
        (XFType::sample_rate): s32 sample_rate;
        (XFType::loop_info): {
            s32 loop_start;
            s32 loop_end;
        }
        (XFType::free_comment): {
            if (!(extraflag.size % 260)) FreeComment1 free_comment[extraflag.size/260];
            else char free_comment[extraflag.size];
        }
        (XFType::unknown1): u32 unknown;
        (XFType::dsp_coefs): DSPCoefs coefs[parent.sample_mode.channels+1]; // not very accurate but... eh
        (XFType::atrac9_config): {
            if (extraflag.size >= 8 && std::mem::read_unsigned($,1,std::mem::Endian::Little) != 0xFE) u32 frame_size;
            u32 config_data[extraflag.size/4-1];
        }
        (XFType::xwma_config): {
            be XWMAFormat format;
            be u16 block_align;
            be u32 bit_rate;
            be u32 dpds_table[(extraflag.size-8)/4];
        }
        (XFType::vorbis_setup_id_seek_table): {
            u32 setup_id;
            u32 table_size;
            VorbisSeekEntry seek_table[table_size/8];
        }
        (XFType::peak_volume): float peak_volume;
        (XFType::vorbis_intra_layers): u32 vorbis_intra_layers;
        (XFType::opus_data_size): u32 opus_data_sizes[extraflag.size/4];
    }
    $ = pos + extraflag.size;
};
bool end_next;
fn check_xflag_end() {
    if (end_next) {
        end_next = 0;
        return 0;
    }
    u32 xflag = std::mem::read_unsigned($,4,std::mem::Endian::Little);
    end_next = !(xflag & 1);
    return 1;
};

struct Entry5 {
    smode sample_mode;
    if (sample_mode.extra_flags) {
        XFlag extraflags[while(check_xflag_end())];
    }
};
struct NameEntry {
    u32 offset;
    char name[] @ (parent.base_header_size + parent.sample_header_size + offset);
};

struct Base {
    char FSB[3];
    char version;

    u32 base_header_size = 0;
    if (version == '1') {
        s32 nb_streams;
        s32 sample_data_size;
        padding[4];

        Entry1 entries[nb_streams];
    } else if (version == '2') {
        s32 nb_streams;
        s32 sample_headers_size;
        s32 sample_data_size;
    } else if (version == '3') {
        s32 nb_streams;
        s32 sample_headers_size;
        s32 sample_data_size;
        Version34 minor_version;
        if (minor_version == Version34::FSB_3_1) {
            Format3_1 flags;
            if (flags.basic_headers) {
                Entry3_1 master_entry;
                EntryBasic entries[nb_streams];
            } else Entry3_1 entries[nb_streams];
        } else {
            Format23 flags;
            Entry3 entries[nb_streams];
        }
    } else if (version == '4') {
        s32 nb_streams;
        s32 sample_headers_size;
        s32 sample_data_size;
        Version34 minor_version;
        Format4 flags;
        char hash[8];
        char guid[16];
        if (flags.basic_headers) {
            Entry4 master_entry;
            EntryBasic entries[nb_streams];
        } else Entry4 entries[nb_streams];
    } else if (version == '5') {
        u32 minor_version;
        base_header_size += 0x3C;
        if (minor_version == 0) base_header_size += 4;
        s32 nb_streams;
        s32 sample_header_size;
        s32 name_table_size;
        s32 sample_data_size;
        Codec5 codec;
        padding[4];
        if (minor_version == 0) padding[8];
        else if (minor_version == 1) padding[4];
        char hash[16];
        char sub_hash[8];

        Entry5 entries[nb_streams];

        if (name_table_size) NameEntry names[nb_streams];
    }
};

Base Base @ 0;