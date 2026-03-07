/*
 * UBI BNM demuxer
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

#include "libavutil/intfloat.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/avstring.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

#define SB_MAX_LAYER_COUNT  16
#define SB_MAX_CHAIN_COUNT 256
#define SB_MAX_SUBSONGS 128000

#define LAYER_HIJACK_GRAW_X360  1
#define LAYER_HIJACK_SCPT_PS2   2

typedef enum ubi_sb_codec {
    UBI_IMA, UBI_ADPCM, RAW_PCM, RAW_PSX, RAW_DSP, RAW_XBOX, FMT_VAG, FMT_AT3, RAW_AT3, FMT_XMA1, RAW_XMA1, FMT_OGG, FMT_CWAV, FMT_APM, FMT_MPDX, UBI_IMA_SCE
} ubi_sb_codec;

typedef enum ubi_sb_platform {
    UBI_PC, UBI_DC, UBI_PS2, UBI_XBOX, UBI_GC, UBI_X360, UBI_PSP, UBI_PS3, UBI_WII, UBI_3DS
} ubi_sb_platform;

typedef enum ubi_sb_type {
    UBI_NONE = 0, UBI_AUDIO, UBI_LAYER, UBI_SEQUENCE, UBI_SILENCE
} ubi_sb_type;

typedef struct ubi_sb_config {
    int map_version;
    int64_t map_entry_size;
    int64_t map_name;
    int64_t section1_entry_size;
    int64_t section2_entry_size;
    int64_t section3_entry_size;
    int64_t resource_name_size;
    int64_t blk_table_size;

    int64_t audio_extra_offset;
    int64_t audio_stream_size;
    int64_t audio_stream_offset;
    int64_t audio_stream_type;
    int64_t audio_software_flag;
    int64_t audio_hwmodule_flag;
    int64_t audio_streamed_flag;
    int64_t audio_cd_streamed_flag;
    int64_t audio_loop_flag;
    int64_t audio_loc_flag;
    int64_t audio_stereo_flag;
    int64_t audio_ram_streamed_flag;
    int64_t audio_internal_flag;
    int64_t audio_num_samples;
    int64_t audio_num_samples2;
    int64_t audio_sample_rate;
    int64_t audio_channels;
    int64_t audio_stream_name;
    int64_t audio_extra_name;
    int64_t audio_xma_offset;
    int64_t audio_pitch;
    int audio_streamed_and;
    int audio_cd_streamed_and;
    int audio_loop_and;
    int audio_software_and;
    int audio_hwmodule_and;
    int audio_loc_and;
    int audio_stereo_and;
    int audio_ram_streamed_and;
    int audio_has_internal_names;
    int audio_interleave;
    int audio_fix_psx_samples;
    int has_rs_files;

    int64_t sequence_extra_offset;
    int64_t sequence_sequence_loop_start;
    int64_t sequence_sequence_num_loops;
    int64_t sequence_sequence_count;
    int64_t sequence_entry_number;
    int64_t sequence_entry_size;

    int64_t layer_extra_offset;
    int64_t layer_layer_count;
    int64_t layer_stream_size;
    int64_t layer_stream_offset;
    int64_t layer_stream_name;
    int64_t layer_extra_name;
    int64_t layer_sample_rate;
    int64_t layer_channels;
    int64_t layer_stream_type;
    int64_t layer_num_samples;
    int64_t layer_pitch;
    int64_t layer_loc_flag;
    int layer_loc_and;
    int64_t layer_entry_size;
    int layer_hijack;

    int64_t silence_duration_int;
    int64_t silence_duration_float;

    int64_t random_extra_offset;
    int64_t random_sequence_count;
    int64_t random_entry_size;
    int random_percent_int;

    int is_padded_section1_offset;
    int is_padded_section2_offset;
    int is_padded_section3_offset;
    int is_padded_sectionX_offset;
    int is_padded_sounds_offset;
    int ignore_layer_error;
} ubi_sb_config;

typedef struct ubi_sb_header {
    ubi_sb_platform platform;
    int is_ps2_old;
    int is_psp_old;
    int big_endian;
    int total_streams;
    int bank_streams;

    ubi_sb_config cfg;

    int64_t map_start;
    uint32_t map_num;

    uint32_t map_type;
    uint32_t map_zero;
    int64_t map_offset;
    int64_t map_size;
    char map_name[0x28];
    uint32_t map_unknown;

    int is_bank;
    int is_map;
    int is_bnm;
    int is_dat;
    int is_ps2_bnm;
    int is_blk;
    int has_numbered_banks;

    int header_init;
    uint32_t version;
    uint32_t version_empty;
    uint32_t section1_num;
    int64_t section1_offset;
    uint32_t section2_num;
    int64_t section2_offset;
    uint32_t section3_num;
    int64_t section3_offset;
    uint32_t section4_num;
    int64_t section4_offset;
    uint32_t sectionX_size;
    int64_t sectionX_offset;
    int64_t bank_size;
    int bank_number;
    int flag1;
    int flag2;

    ubi_sb_type type;
    ubi_sb_codec codec;
    int header_index;
    int64_t header_offset;
    uint32_t header_id;
    uint32_t header_type;
    int64_t extra_offset;
    int64_t stream_offset;
    int64_t stream_size;
    uint32_t stream_type;
    uint32_t subblock_id;
    uint8_t subbank_index;

    int loop_flag;
    int loop_start;
    int num_samples;
    int sample_rate;
    int channels;
    int64_t xma_header_offset;

    int layer_count;
    int layer_channels[SB_MAX_LAYER_COUNT];
    int sequence_count;
    int sequence_chain[SB_MAX_CHAIN_COUNT];
    int sequence_banks[SB_MAX_CHAIN_COUNT];
    int sequence_multibank;
    int sequence_loop_start;
    int sequence_num_loops;

    float duration;

    int is_streamed;
    int is_cd_streamed;
    int is_ram_streamed;
    int is_external;
    int is_localized;
    char resource_name[0x28];

    char readable_name[256];
    int types[16];
    int allowed_types[16];
} ubi_sb_header;

typedef struct UBIBNMDemuxContext {
    ubi_sb_header sb;
    int current_stream;
} UBIBNMDemuxContext;

typedef struct BNMStream {
    int64_t start_offset;
    int64_t stop_offset;
} BNMStream;

static void config_sb_entry(ubi_sb_header *sb,
                            int64_t section1_size_entry,
                            int64_t section2_size_entry)
{
    sb->cfg.section1_entry_size = section1_size_entry;
    sb->cfg.section2_entry_size = section2_size_entry;
    sb->cfg.section3_entry_size = 0x08;
}

static void config_sb_audio_fs(ubi_sb_header *sb, int64_t streamed_flag, int64_t software_flag, int64_t loop_flag)
{
    sb->cfg.audio_streamed_flag = streamed_flag;
    sb->cfg.audio_software_flag = software_flag;
    sb->cfg.audio_loop_flag     = loop_flag;
    sb->cfg.audio_streamed_and  = 1;
    sb->cfg.audio_software_and  = 1;
    sb->cfg.audio_loop_and      = 1;
}

static void config_sb_audio_fb(ubi_sb_header *sb, int64_t flag_bits, int streamed_and, int software_and, int loop_and)
{
    sb->cfg.audio_streamed_flag = flag_bits;
    sb->cfg.audio_software_flag = flag_bits;
    sb->cfg.audio_loop_flag     = flag_bits;
    sb->cfg.audio_streamed_and  = streamed_and;
    sb->cfg.audio_software_and  = software_and;
    sb->cfg.audio_loop_and      = loop_and;
}

static void config_sb_audio_hs(ubi_sb_header *sb, int64_t channels, int64_t sample_rate, int64_t num_samples, int64_t num_samples2, int64_t stream_name, int64_t stream_type)
{
    sb->cfg.audio_channels      = channels;
    sb->cfg.audio_sample_rate   = sample_rate;
    sb->cfg.audio_num_samples   = num_samples;
    sb->cfg.audio_num_samples2  = num_samples2;
    sb->cfg.audio_stream_name   = stream_name;
    sb->cfg.audio_stream_type   = stream_type;
}

static void config_sb_audio_he(ubi_sb_header *sb, int64_t channels, int64_t sample_rate, int64_t num_samples, int64_t num_samples2, int64_t extra_name, int64_t stream_type)
{
    sb->cfg.audio_channels      = channels;
    sb->cfg.audio_sample_rate   = sample_rate;
    sb->cfg.audio_num_samples   = num_samples;
    sb->cfg.audio_num_samples2  = num_samples2;
    sb->cfg.audio_extra_name    = extra_name;
    sb->cfg.audio_stream_type   = stream_type;
}

static void config_sb_audio_fb_ps2(ubi_sb_header *sb, int64_t flag_bits, int streamed_and, int software_and, int loop_and, int hwmodule_and)
{
    sb->cfg.audio_streamed_flag = flag_bits;
    sb->cfg.audio_software_flag = flag_bits;
    sb->cfg.audio_loop_flag     = flag_bits;
    sb->cfg.audio_hwmodule_flag = flag_bits;
    sb->cfg.audio_streamed_and  = streamed_and;
    sb->cfg.audio_software_and  = software_and;
    sb->cfg.audio_loop_and      = loop_and;
    sb->cfg.audio_hwmodule_and  = hwmodule_and;
}

static void config_sb_audio_ps2_bnm(ubi_sb_header *sb, int64_t flag_bits, int streamed_and, int cd_streamed_and, int loop_and, int64_t channels, int64_t sample_rate)
{
    sb->cfg.audio_streamed_flag    = flag_bits;
    sb->cfg.audio_cd_streamed_flag = flag_bits;
    sb->cfg.audio_loop_flag        = flag_bits;
    sb->cfg.audio_streamed_and     = streamed_and;
    sb->cfg.audio_cd_streamed_and  = cd_streamed_and;
    sb->cfg.audio_loop_and         = loop_and;
    sb->cfg.audio_channels         = channels;
    sb->cfg.audio_sample_rate      = sample_rate;
}
static void config_sb_audio_ps2_old(ubi_sb_header *sb, int64_t flag_bits, int streamed_and, int loop_and, int loc_and, int stereo_and, int64_t pitch, int64_t sample_rate)
{
    sb->cfg.audio_streamed_flag = flag_bits;
    sb->cfg.audio_loop_flag     = flag_bits;
    sb->cfg.audio_loc_flag      = flag_bits;
    sb->cfg.audio_stereo_flag   = flag_bits;
    sb->cfg.audio_streamed_and  = streamed_and;
    sb->cfg.audio_loop_and      = loop_and;
    sb->cfg.audio_loc_and       = loc_and;
    sb->cfg.audio_stereo_and    = stereo_and;
    sb->cfg.audio_pitch         = pitch;
    sb->cfg.audio_sample_rate   = sample_rate;
}

static void config_sb_sequence(ubi_sb_header *sb, int64_t sequence_count, int64_t entry_size)
{
    sb->cfg.sequence_sequence_loop_start = sequence_count - 0x10;
    sb->cfg.sequence_sequence_num_loops  = sequence_count - 0x0c;
    sb->cfg.sequence_sequence_count      = sequence_count;
    sb->cfg.sequence_entry_size          = entry_size;
    sb->cfg.sequence_entry_number        = 0x00;
    if (sb->is_bnm || sb->is_dat || sb->is_ps2_bnm) {
        sb->cfg.sequence_sequence_loop_start = sequence_count - 0x0c;
        sb->cfg.sequence_sequence_num_loops  = sequence_count - 0x08;
    } else if (sb->is_blk) {
        sb->cfg.sequence_sequence_loop_start = sequence_count - 0x14;
        sb->cfg.sequence_sequence_num_loops  = sequence_count - 0x0c;
    }
}

static void config_sb_layer_hs(ubi_sb_header *sb, int64_t layer_count, int64_t stream_size, int64_t stream_offset, int64_t stream_name)
{
    sb->cfg.layer_layer_count   = layer_count;
    sb->cfg.layer_stream_size   = stream_size;
    sb->cfg.layer_stream_offset = stream_offset;
    sb->cfg.layer_stream_name   = stream_name;
}

static void config_sb_layer_he(ubi_sb_header *sb, int64_t layer_count, int64_t stream_size, int64_t stream_offset, int64_t extra_name)
{
    sb->cfg.layer_layer_count   = layer_count;
    sb->cfg.layer_stream_size   = stream_size;
    sb->cfg.layer_stream_offset = stream_offset;
    sb->cfg.layer_extra_name    = extra_name;
}

static void config_sb_layer_sh(ubi_sb_header *sb, int64_t entry_size, int64_t sample_rate, int64_t channels, int64_t stream_type, int64_t num_samples)
{
    sb->cfg.layer_entry_size  = entry_size;
    sb->cfg.layer_sample_rate = sample_rate;
    sb->cfg.layer_channels    = channels;
    sb->cfg.layer_stream_type = stream_type;
    sb->cfg.layer_num_samples = num_samples;
}

static void config_sb_layer_ps2_old(ubi_sb_header *sb, int64_t loc_flag, int loc_and, int64_t layer_count, int64_t pitch)
{
    sb->cfg.layer_loc_flag    = loc_flag;
    sb->cfg.layer_loc_and     = loc_and;
    sb->cfg.layer_layer_count = layer_count;
    sb->cfg.layer_pitch       = pitch;
}

static void config_sb_silence_i(ubi_sb_header *sb, int64_t duration)
{
    sb->cfg.silence_duration_int = duration;
}

static void config_sb_silence_f(ubi_sb_header *sb, int64_t duration)
{
    sb->cfg.silence_duration_float = duration;
}

static void config_sb_random_old(ubi_sb_header *sb, int64_t sequence_count, int64_t entry_size)
{
    sb->cfg.random_sequence_count = sequence_count;
    sb->cfg.random_entry_size = entry_size;
    sb->cfg.random_percent_int = 1;
}

typedef unsigned int (*read_u32_t)(AVIOContext *s);

static int test_version_sb_entry(ubi_sb_header *sb, AVFormatContext *s, int64_t offset, int count, uint32_t entry_size)
{
    AVIOContext *pb = s->pb;
    read_u32_t read_u32 = sb->big_endian ? avio_rb32 : avio_rl32;
    uint32_t prev_group = 0;

    for (int i = 0; i < count; i++) {
        uint16_t group, sound;
        uint32_t curr;

        avio_seek(pb, offset, SEEK_SET);
        curr = read_u32(pb);
        if (i > 1 && curr == 0)
            return 0;

        group = (curr >> 16) & 0xFFFF;
        sound = (curr >>  0) & 0xFFFF;
        if (group > 0x1000 || sound > 0x1000)
            return 0;

        if (prev_group && group < prev_group)
            return 0;

        prev_group = group;
        offset += entry_size;
    }

    return 0;
}

static int init_sb_header(ubi_sb_header *sb, AVFormatContext *s)
{
    read_u32_t read_u32 = sb->big_endian ? avio_rb32 : avio_rl32;
    AVIOContext *pb = s->pb;

    if (sb->header_init)
        return 0;

    avio_seek(pb, 0x04, SEEK_SET);
    if (sb->version <= 0x0000000B) {
        sb->section1_num  = read_u32(pb);
        avio_skip(pb, 4);
        sb->section2_num  = read_u32(pb);
        avio_skip(pb, 4);
        sb->section3_num  = read_u32(pb);
        avio_skip(pb, 4);
        sb->sectionX_size = read_u32(pb);

        sb->section1_offset = 0x20;
    } else if (sb->version <= 0x000A0000) {
        sb->section1_num  = read_u32(pb);
        sb->section2_num  = read_u32(pb);
        sb->section3_num  = read_u32(pb);
        sb->sectionX_size = read_u32(pb);
        sb->flag1         = read_u32(pb);

        sb->section1_offset = 0x18;
    } else {
        sb->section1_num  = read_u32(pb);
        sb->section2_num  = read_u32(pb);
        sb->section3_num  = read_u32(pb);
        sb->sectionX_size = read_u32(pb);
        sb->flag1         = read_u32(pb);
        sb->flag2         = read_u32(pb);

        sb->section1_offset = 0x1c;
    }

    if (sb->section1_num > SB_MAX_SUBSONGS || sb->section2_num > SB_MAX_SUBSONGS || sb->section3_num > SB_MAX_SUBSONGS)
        return AVERROR_INVALIDDATA;

    sb->header_init = 1;
    return 0;
}

static int test_version_sb(ubi_sb_header *sb, AVFormatContext *s, uint32_t section1_size_entry, uint32_t section2_size_entry)
{
    int64_t offset;
    int ret;

    ret = init_sb_header(sb, s);
    if (ret < 0)
        return ret;

    if (sb->section2_num == 0)
        return 0;

    offset = sb->section1_offset;
    ret = test_version_sb_entry(sb, s, offset, sb->section1_num, section1_size_entry);
    if (ret < 0)
        return ret;

    offset = sb->section1_offset + sb->section1_num * section1_size_entry;
    ret = test_version_sb_entry(sb, s, offset, sb->section2_num, section2_size_entry);
    if (ret < 0)
        return ret;

    return 0;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const BNMStream *ks1 = s1->priv_data;
    const BNMStream *ks2 = s2->priv_data;

    return FFDIFFSIGN(ks1->start_offset, ks2->start_offset);
}

static int read_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) != 0)
        return 0;
    if (!av_match_ext(p->filename, "bnm"))
        return 0;

    return AVPROBE_SCORE_MAX/3;
}

static int check_project_file(AVFormatContext *s,
                              const char *name,
                              int has_localized_banks)
{
    AVDictionary *tmp = NULL;
    const char *dir_name;
    AVIOContext *pb;
    char *copy_name;
    char *name_path;
    int ret;

    copy_name = av_strdup(s->url);
    if (!copy_name)
        return 0;

    dir_name = av_dirname(copy_name);
    if (!dir_name) {
        av_free(copy_name);
        return 0;
    }
    name_path = av_append_path_component(dir_name, name);
    if (!name_path) {
        av_free(copy_name);
        return 0;
    }
    ret = s->io_open(s, &pb, name_path, AVIO_FLAG_READ, &tmp);
    s->io_close2(s, pb);
    av_free(name_path);
    av_free(copy_name);

    return ret == 0;
}

static int config_sb_version(ubi_sb_header *sb, AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int is_ttse_pc = 0;
    int is_bia_ps2 = 0, is_biadd_psp = 0;
    int is_sc2_ps2_gc = 0;
    int is_sc4_pc_online = 0;
    int is_myst4_pc = 0;

    if (sb->is_bnm || sb->is_dat || sb->is_ps2_bnm)
        sb->has_numbered_banks = 1;

    sb->cfg.resource_name_size = 0x28;

    if (sb->version <= 0x00000007)
        sb->cfg.map_version             = 1;
    else if (sb->version < 0x00150000)
        sb->cfg.map_version             = 2;
    else
        sb->cfg.map_version             = 3;

    sb->cfg.map_entry_size = (sb->cfg.map_version < 2) ? 0x30 : 0x34;
    sb->cfg.map_name = 0x10;
    if (sb->is_blk) {
        sb->cfg.map_entry_size = 0x30;
    }

    if (sb->is_bnm || sb->is_blk || sb->is_dat) {
        sb->cfg.audio_internal_flag     = 0x08;
        sb->cfg.audio_stream_size       = 0x0c;
        sb->cfg.audio_stream_offset     = 0x10;
        sb->cfg.sequence_extra_offset   = 0x10;
        sb->cfg.random_extra_offset     = 0x10;
    } else if (sb->is_ps2_bnm) {
        sb->cfg.audio_stream_size       = 0x2c;
        sb->cfg.audio_stream_offset     = 0x30;
        sb->cfg.sequence_extra_offset   = 0x10;
        sb->cfg.random_extra_offset     = 0x10;
    } else if (sb->version <= 0x00000007) {
        sb->cfg.audio_internal_flag     = 0x08;
        sb->cfg.audio_stream_size       = 0x0c;
        sb->cfg.audio_extra_offset      = 0x10;
        sb->cfg.audio_stream_offset     = 0x14;
        sb->cfg.sequence_extra_offset   = 0x10;
        sb->cfg.layer_extra_offset      = 0x10;
    } else {
        sb->cfg.audio_stream_size       = 0x08;
        sb->cfg.audio_extra_offset      = 0x0c;
        sb->cfg.audio_stream_offset     = 0x10;
        sb->cfg.sequence_extra_offset   = 0x0c;
        sb->cfg.layer_extra_offset      = 0x0c;
    }

    sb->allowed_types[0x01] = 1;
    sb->allowed_types[0x05] = 1;
    sb->allowed_types[0x0c] = 1;
    sb->allowed_types[0x06] = 1;
    sb->allowed_types[0x0d] = 1;
    if (sb->is_bnm || sb->is_dat || sb->is_ps2_bnm)
        sb->allowed_types[0x0b] = 1;

    if (sb->version == 0x00000000 && sb->platform == UBI_PC) {
        if (check_project_file(s, "Dino.lcb", 0))
            sb->version = 0x00000200;
    }

    if ((sb->version == 0xAAAAAAAA && sb->platform == UBI_PC) ||
        (sb->version == 0xCDCDCDCD && sb->platform == UBI_PC)) {
        sb->version = 0x00000000;
    }

    if (sb->is_bnm && sb->version > 0x00000000 && sb->platform == UBI_PC) {
        if (check_project_file(s, "ED_MAIN.LCB", 1)) {
            is_ttse_pc = 1;
            sb->version = 0x00000000;
        }
    }

    if (sb->version == 0x00000000 && sb->platform == UBI_PC && is_ttse_pc) {
        config_sb_entry(sb, 0x20, 0x5c);

        config_sb_audio_fs(sb, 0x2c, 0x00, 0x30);
        config_sb_audio_hs(sb, 0x42, 0x3c, 0x38, 0x38, 0x48, 0x44);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x24, 0x18);
        return 0;
    }

    if ((sb->version == 0x00000000 && sb->platform == UBI_PC) ||
        (sb->version == 0x00000200 && sb->platform == UBI_PC)) {
        config_sb_entry(sb, 0x20, 0x5c);
        if (sb->version == 0x00000200)
            config_sb_entry(sb, 0x20, 0x60);

        config_sb_audio_fs(sb, 0x2c, 0x00, 0x30);
        config_sb_audio_hs(sb, 0x42, 0x3c, 0x34, 0x34, 0x48, 0x44);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x24, 0x18);

        config_sb_random_old(sb, 0x18, 0x0c);

        return 0;
    }

    if (sb->version == 0x00060409 && sb->platform == UBI_PC) {
        config_sb_entry(sb, 0x24, 0x64);

        config_sb_audio_fs(sb, 0x2c, 0x00, 0x30);
        config_sb_audio_hs(sb, 0x4E, 0x48, 0x34, 0x34, 0x54, 0x50);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x2c, 0x1c);

        return 0;
    }

    if (sb->version == 0x00000000 && sb->platform == UBI_DC) {
        if (av_match_ext(s->url, "kat"))
            sb->version = 0x00000200;
    }

    if ((sb->version == 0x00000000 && sb->platform == UBI_DC) ||
        (sb->version == 0x00000200 && sb->platform == UBI_DC)) {
        config_sb_entry(sb, 0x20, 0x64);
        if (sb->version == 0x00000200)
            config_sb_entry(sb, 0x20, 0x68);

        config_sb_audio_fs(sb, 0x2c, 0x00, 0x30);
        config_sb_audio_hs(sb, 0x42, 0x3c, 0x34, 0x34, 0x48, 0x44);

        config_sb_sequence(sb, 0x24, 0x18);

        config_sb_random_old(sb, 0x18, 0x0c);

        return 0;
    }

    if (sb->version == 0x32787370 && sb->platform == UBI_PS2) {
        sb->version = 0x00000000;
        config_sb_entry(sb, 0x1c, 0x44);

        config_sb_audio_ps2_bnm(sb, 0x18, (1 << 5), (1 << 6), (1 << 7), 0x20, 0x22);
        sb->cfg.audio_interleave = 0x400;

        config_sb_sequence(sb, 0x24, 0x14);

        return 0;
    }

    if ((sb->version == 0x00000003 && sb->platform == UBI_PC) ||
        (sb->version == 0x00000003 && sb->platform == UBI_XBOX)) {
        config_sb_entry(sb, 0x40, 0x68);

        config_sb_audio_fs(sb, 0x30, 0x00, 0x34);
        config_sb_audio_hs(sb, 0x52, 0x4c, 0x38, 0x40, 0x58, 0x54);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x2c, 0x1c);

        config_sb_layer_hs(sb, 0x20, 0x4c, 0x44, 0x34);
        config_sb_layer_sh(sb, 0x1c, 0x04, 0x0a, 0x0c, 0x18);
        return 0;
    }

    if (sb->version == 0x00000003 && sb->platform == UBI_PS2 && sb->is_blk) {
        config_sb_entry(sb, 0x20, 0x40);

        config_sb_audio_ps2_old(sb, 0x18, (1 << 4), (1 << 5), (1 << 6), (1 << 7), 0x1c, 0x20);
        sb->cfg.audio_interleave = 0x800;
        sb->is_ps2_old = 1;

        config_sb_sequence(sb, 0x2c, 0x18);

        config_sb_layer_ps2_old(sb, 0x18, (1 << 0), 0x1c, 0x20);
        return 0;
    }

    if (sb->version == 0x00000003 && sb->platform == UBI_PS2) {
        config_sb_entry(sb, 0x30, 0x3c);

        config_sb_audio_ps2_old(sb, 0x1c, (1 << 4), (1 << 5), (1 << 6), (1 << 7), 0x20, 0x24);
        sb->cfg.audio_interleave = 0x800;
        sb->is_ps2_old = 1;

        config_sb_sequence(sb, 0x2c, 0x18);

        config_sb_layer_ps2_old(sb, 0x1c, (1 << 0), 0x20, 0x24);
        return 0;
    }

    if (sb->version == 0x00000003 && sb->platform == UBI_GC) {
        config_sb_entry(sb, 0x40, 0x6c);

        config_sb_audio_fs(sb, 0x30, 0x00, 0x34);
        config_sb_audio_hs(sb, 0x56, 0x50, 0x48, 0x48, 0x5c, 0x58);

        config_sb_sequence(sb, 0x2c, 0x1c);

        config_sb_layer_hs(sb, 0x20, 0x4c, 0x44, 0x34);
        config_sb_layer_sh(sb, 0x1c, 0x04, 0x0a, 0x0c, 0x18);
        return 0;
    }

    if (sb->version == 0x00000004 && sb->platform == UBI_PS2) {
        config_sb_entry(sb, 0x34, 0x70);

        config_sb_audio_fb(sb, 0x1c, (1 << 4), 0, (1 << 5));
        config_sb_audio_hs(sb, 0x24, 0x28, 0x34, 0x3c, 0x44, 0x6c);

        config_sb_sequence(sb, 0x2c, 0x24);
        return 0;
    }

    if (sb->version == 0x00000007 && sb->platform == UBI_PC) {
        config_sb_entry(sb, 0x58, 0x80);

        config_sb_audio_fs(sb, 0x28, 0x00, 0x2c);
        config_sb_audio_hs(sb, 0x4a, 0x44, 0x30, 0x38, 0x50, 0x4c);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x2c, 0x34);

        config_sb_layer_hs(sb, 0x24, 0x64, 0x5c, 0x34);
        config_sb_layer_sh(sb, 0x18, 0x00, 0x06, 0x08, 0x14);
        return 0;
    }

    if (sb->version == 0x00000007 && sb->platform == UBI_XBOX) {
        config_sb_entry(sb, 0x58, 0x78);

        config_sb_audio_fs(sb, 0x28, 0x00, 0x2c);
        config_sb_audio_hs(sb, 0x4a, 0x44, 0x30, 0x38, 0x50, 0x4c);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x2c, 0x34);

        config_sb_layer_hs(sb, 0x24, 0x64, 0x5c, 0x34);
        config_sb_layer_sh(sb, 0x18, 0x00, 0x06, 0x08, 0x14);
        return 0;
    }

    if ((sb->version == 0x00000007 && sb->platform == UBI_PS2) ||
        (sb->version == 0x00000007 && sb->platform == UBI_GC) ) {
        unsigned int (*read_32bit)(AVIOContext *s) = sb->big_endian ? avio_rb32 : avio_rl32;

        avio_seek(pb, 0x08, SEEK_SET);
        is_sc2_ps2_gc = read_32bit(pb) == 0x21;
    }

    if (sb->version == 0x00000007 && sb->platform == UBI_PS2) {
        config_sb_entry(sb, 0x40, 0x70);

        config_sb_audio_fb(sb, 0x1c, (1 << 2), 0, (1 << 3));
        config_sb_audio_hs(sb, 0x24, 0x28, 0x34, 0x3c, 0x44, 0x6c);

        config_sb_sequence(sb, 0x2c, 0x30);

        config_sb_layer_hs(sb, 0x24, 0x64, 0x5c, 0x34);
        config_sb_layer_sh(sb, 0x18, 0x00, 0x06, 0x08, 0x14);

        if (is_sc2_ps2_gc) {
            sb->cfg.map_entry_size = 0x38;
            sb->cfg.map_name = 0x18;
            sb->cfg.has_rs_files = 1;
            sb->cfg.audio_ram_streamed_flag = 0x1c;
            sb->cfg.audio_ram_streamed_and = (1 << 3);
            sb->cfg.audio_loop_and = (1 << 4);
            sb->cfg.layer_hijack = LAYER_HIJACK_SCPT_PS2;
        }
        return 0;
    }

    if (sb->version == 0x00000007 && sb->platform == UBI_GC) {
        config_sb_entry(sb, 0x58, 0x78);

        config_sb_audio_fs(sb, 0x24, 0x00, 0x28);
        config_sb_audio_hs(sb, 0x4a, 0x44, 0x2c, 0x34, 0x50, 0x4c);

        config_sb_sequence(sb, 0x2c, 0x34);

        config_sb_layer_hs(sb, 0x24, 0x64, 0x5c, 0x34);
        config_sb_layer_sh(sb, 0x18, 0x00, 0x06, 0x08, 0x14);

        if (is_sc2_ps2_gc) {
            sb->cfg.map_entry_size = 0x38;
            sb->cfg.map_name = 0x18;
            sb->cfg.audio_streamed_and = 0x01000000;
        }
        return 0;
    }

    if (sb->version == 0x0000000B && sb->platform == UBI_PC) {
        config_sb_entry(sb, 0x5c, 0x7c);

        config_sb_audio_fs(sb, 0x24, 0x00, 0x28);
        config_sb_audio_hs(sb, 0x46, 0x40, 0x2c, 0x34, 0x4c, 0x48);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x28, 0x34);

        config_sb_layer_hs(sb, 0x20, 0x60, 0x58, 0x30);
        config_sb_layer_sh(sb, 0x14, 0x00, 0x06, 0x08, 0x10);
        return 0;
    }

    if (sb->version == 0x0000000D && sb->platform == UBI_XBOX) {
        config_sb_entry(sb, 0x5c, 0x74);

        config_sb_audio_fs(sb, 0x24, 0x00, 0x28);
        config_sb_audio_hs(sb, 0x46, 0x40, 0x2c, 0x34, 0x4c, 0x48);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_layer_hs(sb, 0x20, 0x60, 0x58, 0x30);
        config_sb_layer_sh(sb, 0x14, 0x00, 0x06, 0x08, 0x10);
        return 0;
    }

    if (sb->version == 0x000A0000 && sb->platform == UBI_XBOX) {
        config_sb_entry(sb, 0x64, 0x78);

        config_sb_audio_fs(sb, 0x24, 0x28, 0x2c);
        config_sb_audio_hs(sb, 0x4a, 0x44, 0x30, 0x38, 0x50, 0x4c);

        config_sb_layer_hs(sb, 0x20, 0x60, 0x58, 0x30);
        config_sb_layer_sh(sb, 0x14, 0x00, 0x06, 0x08, 0x10);
        return 0;
    }

    if ((sb->version == 0x000A0002 && sb->platform == UBI_PC) ||
        (sb->version == 0x000A0004 && sb->platform == UBI_PC)) {
        config_sb_entry(sb, 0x64, 0x80);

        config_sb_audio_fs(sb, 0x24, 0x2c, 0x28);
        config_sb_audio_hs(sb, 0x4a, 0x44, 0x30, 0x38, 0x50, 0x4c);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x28, 0x14);

        config_sb_layer_hs(sb, 0x20, 0x60, 0x58, 0x30);
        config_sb_layer_sh(sb, 0x14, 0x00, 0x06, 0x08, 0x10);
        return 0;
    }

    if (sb->version == 0x000A0007 && sb->platform == UBI_PS2) {
        if (check_project_file(s, "BIAAUDIO.SP1", 1))
            is_bia_ps2 = 1;
    }

    if ((sb->version == 0x000A0002 && sb->platform == UBI_PS2) ||
        (sb->version == 0x000A0004 && sb->platform == UBI_PS2) ||
        (sb->version == 0x000A0007 && sb->platform == UBI_PS2 && !is_bia_ps2) ||
        (sb->version == 0x000A0008 && sb->platform == UBI_PS2) ||
        (sb->version == 0x00100000 && sb->platform == UBI_PS2) ||
        (sb->version == 0x00120009 && sb->platform == UBI_PS2) ||
        (sb->version == 0x0012000c && sb->platform == UBI_PS2)) {
        config_sb_entry(sb, 0x48, 0x6c);

        config_sb_audio_fb_ps2(sb, 0x18, (1 << 2), (1 << 3), (1 << 4), (1 << 5));
        config_sb_audio_hs(sb, 0x20, 0x24, 0x30, 0x38, 0x40, 0x68);

        config_sb_sequence(sb, 0x28, 0x10);

        config_sb_layer_hs(sb, 0x20, 0x60, 0x58, 0x30);
        config_sb_layer_sh(sb, 0x14, 0x00, 0x06, 0x08, 0x10);

        config_sb_silence_i(sb, 0x18);
        return 0;
    }

    if (sb->version == 0x000A0007 && sb->platform == UBI_PS2 && is_bia_ps2) {
        config_sb_entry(sb, 0x5c, 0x14c);

        config_sb_audio_fb_ps2(sb, 0x18, (1 << 2), (1 << 3), (1 << 4), (1 << 5));
        config_sb_audio_hs(sb, 0x20, 0x24, 0x30, 0x38, 0x40, 0x148);

        config_sb_sequence(sb, 0x28, 0x10);

        config_sb_layer_hs(sb, 0x20, 0x140, 0x138, 0x30);
        config_sb_layer_sh(sb, 0x14, 0x00, 0x06, 0x08, 0x10);

        sb->cfg.is_padded_section1_offset = 1;
        sb->cfg.is_padded_section2_offset = 1;
        sb->cfg.is_padded_section3_offset = 1;
        sb->cfg.is_padded_sectionX_offset = 1;
        sb->cfg.is_padded_sounds_offset = 1;
        return 0;
    }

    if (sb->version == 0x000A0003 && sb->platform == UBI_XBOX) {
        config_sb_entry(sb, 0x64, 0x80);

        config_sb_audio_fs(sb, 0x24, 0x28, 0x34);
        config_sb_audio_hs(sb, 0x52, 0x4c, 0x38, 0x40, 0x58, 0x54);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x28, 0x14);

        config_sb_layer_hs(sb, 0x20, 0x60, 0x58, 0x30);
        config_sb_layer_sh(sb, 0x14, 0x00, 0x06, 0x08, 0x10);
        return 0;
    }

    if ((sb->version == 0x000A0002 && sb->platform == UBI_XBOX) ||
        (sb->version == 0x000A0004 && sb->platform == UBI_XBOX)) {
        config_sb_entry(sb, 0x64, 0x78);

        config_sb_audio_fs(sb, 0x24, 0x28, 0x2c);
        config_sb_audio_hs(sb, 0x4a, 0x44, 0x30, 0x38, 0x50, 0x4c);

        config_sb_sequence(sb, 0x28, 0x14);

        config_sb_layer_hs(sb, 0x20, 0x60, 0x58, 0x30);
        config_sb_layer_sh(sb, 0x14, 0x00, 0x06, 0x08, 0x10);
        return 0;
    }

    if ((sb->version == 0x000A0002 && sb->platform == UBI_GC) ||
        (sb->version == 0x000A0004 && sb->platform == UBI_GC) ||
        (sb->version == 0x000A0007 && sb->platform == UBI_GC)) {
        config_sb_entry(sb, 0x64, 0x74);

        config_sb_audio_fs(sb, 0x20, 0x24, 0x28);
        config_sb_audio_hs(sb, 0x46, 0x40, 0x2c, 0x34, 0x4c, 0x48);

        config_sb_sequence(sb, 0x28, 0x14);

        config_sb_layer_hs(sb, 0x20, 0x60, 0x58, 0x30);
        config_sb_layer_sh(sb, 0x14, 0x00, 0x06, 0x08, 0x10);

        config_sb_silence_i(sb, 0x18);
        return 0;
    }

    if (sb->version == 0x000A0007 && sb->platform == UBI_XBOX) {
        config_sb_entry(sb, 0x64, 0x8c);

        config_sb_audio_fs(sb, 0x24, 0x28, 0x40);
        config_sb_audio_hs(sb, 0x5e, 0x58, 0x44, 0x4c, 0x64, 0x60);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x28, 0x14);

        config_sb_layer_hs(sb, 0x20, 0x60, 0x58, 0x30);
        config_sb_layer_sh(sb, 0x14, 0x00, 0x06, 0x08, 0x10);

        config_sb_silence_i(sb, 0x18);
        return 0;
    }

    if (sb->version == 0x00100000 && sb->platform == UBI_PC) {
        config_sb_entry(sb, 0x68, 0xa4);

        config_sb_audio_fs(sb, 0x24, 0x2c, 0x28);
        config_sb_audio_hs(sb, 0x4c, 0x44, 0x30, 0x38, 0x54, 0x50);
        sb->cfg.audio_has_internal_names = 1;
        return 0;
    }

    if (sb->version == 0x00100000 && sb->platform == UBI_XBOX) {
        config_sb_entry(sb, 0x68, 0x90);

        config_sb_audio_fs(sb, 0x24, 0x28, 0x40);
        config_sb_audio_hs(sb, 0x60, 0x58, 0x44, 0x4c, 0x68, 0x64);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x28, 0x14);
        return 0;
    }

    if (sb->version == 0x00120006 && sb->platform == UBI_PC) {
        if (test_version_sb(sb, s, 0x6c, 0xa4) || check_project_file(s, "gamesnd_myst4.sp0", 1))
            is_myst4_pc = 1;
    }

    if ((sb->version == 0x00120006 && sb->platform == UBI_PC) ||
        (sb->version == 0x00120009 && sb->platform == UBI_PC)) {
        config_sb_entry(sb, 0x6c, 0x84);
        if (is_myst4_pc)
            config_sb_entry(sb, 0x6c, 0xa4);

        config_sb_audio_fs(sb, 0x24, 0x2c, 0x28);
        config_sb_audio_hs(sb, 0x4c, 0x44, 0x30, 0x38, 0x54, 0x50);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x28, 0x14);
        return 0;
    }

    if (sb->version == 0x00120009 && sb->platform == UBI_XBOX) {
        config_sb_entry(sb, 0x6c, 0x90);

        config_sb_audio_fs(sb, 0x24, 0x28, 0x40);
        config_sb_audio_hs(sb, 0x60, 0x58, 0x44, 0x4c, 0x68, 0x64);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x28, 0x14);
        return 0;
    }

    if (sb->version == 0x00120009 && sb->platform == UBI_GC) {
        config_sb_entry(sb, 0x6c, 0x78);

        config_sb_audio_fs(sb, 0x20, 0x24, 0x28);
        config_sb_audio_hs(sb, 0x48, 0x40, 0x2c, 0x34, 0x50, 0x4c);

        config_sb_sequence(sb, 0x28, 0x14);
        return 0;
    }

    if (sb->version == 0x0012000C && sb->platform == UBI_PSP) {
        if (check_project_file(s, "BIAAUDIO.SP4", 1))
            is_biadd_psp = 1;
    }

    if (sb->version == 0x0012000C && sb->platform == UBI_PSP && !is_biadd_psp) {
        config_sb_entry(sb, 0x68, 0x84);
        if (is_biadd_psp)
            config_sb_entry(sb, 0x80, 0x94);

        config_sb_audio_fs(sb, 0x24, 0x2c, 0x28);
        config_sb_audio_hs(sb, 0x4c, 0x44, 0x30, 0x38, 0x54, 0x50);
        sb->cfg.audio_has_internal_names = 1;

        config_sb_sequence(sb, 0x28, 0x14);

        config_sb_layer_hs(sb, 0x1c, 0x60, 0x64, 0x30);
        config_sb_layer_sh(sb, 0x18, 0x00, 0x08, 0x0c, 0x14);
        return 0;
    }

    if (sb->version == 0x00120012 && sb->platform == UBI_PC) {
        config_sb_entry(sb, 0x68, 0x60);

        config_sb_audio_fs(sb, 0x24, 0x2c, 0x28);
        config_sb_audio_he(sb, 0x4c, 0x44, 0x30, 0x38, 0x54, 0x50);

        config_sb_sequence(sb, 0x28, 0x14);
        return 0;
    }

    if (sb->version == 0x00120012 && sb->platform == UBI_XBOX) {
        config_sb_entry(sb, 0x48, 0x4c);

        config_sb_audio_fb(sb, 0x18, (1 << 3), (1 << 4), (1 << 10));
        config_sb_audio_he(sb, 0x38, 0x30, 0x1c, 0x24, 0x40, 0x3c);

        config_sb_sequence(sb, 0x28, 0x10);
        return 0;
    }

    if (sb->version == 0x00130001 && sb->platform == UBI_PS2) {
        config_sb_entry(sb, 0x48, 0x4c);

        config_sb_audio_fb_ps2(sb, 0x18, (1 << 2), (1 << 3), (1 << 4), (1 << 5));
        config_sb_audio_he(sb, 0x20, 0x24, 0x30, 0x38, 0x40, 0x44);

        config_sb_sequence(sb, 0x28, 0x10);

        return 0;
    }

    if (sb->version == 0x00130001 && sb->platform == UBI_GC) {
        config_sb_entry(sb, 0x68, 0x54);

        config_sb_audio_fs(sb, 0x20, 0x24, 0x28);
        config_sb_audio_he(sb, 0x48, 0x40, 0x2c, 0x34, 0x50, 0x4c);

        config_sb_sequence(sb, 0x28, 0x14);

        config_sb_layer_he(sb, 0x1c, 0x34, 0x3c, 0x40);
        config_sb_layer_sh(sb, 0x18, 0x00, 0x08, 0x0c, 0x14);
        return 0;
    }

    if (sb->version == 0x00130001 && sb->platform == UBI_3DS) {
        config_sb_entry(sb, 0x48, 0x4c);

        config_sb_audio_fb(sb, 0x18, (1 << 2), (1 << 3), (1 << 4));
        config_sb_audio_he(sb, 0x38, 0x30, 0x1c, 0x24, 0x40, 0x3c);

        config_sb_sequence(sb, 0x28, 0x10);

        config_sb_layer_he(sb, 0x1c, 0x28, 0x30, 0x34);
        config_sb_layer_sh(sb, 0x18, 0x00, 0x08, 0x0c, 0x14);
        return 0;
    }

    if (sb->version == 0x00130004 && sb->platform == UBI_PS2) {
        config_sb_entry(sb, 0x48, 0x50);

        config_sb_audio_fb_ps2(sb, 0x18, (1 << 2), (1 << 3), (1 << 4), (1 << 5));
        config_sb_audio_he(sb, 0x20, 0x24, 0x30, 0x38, 0x40, 0x4c);
        sb->cfg.audio_interleave = 0x8000;

        sb->cfg.is_padded_section1_offset = 1;
        sb->cfg.is_padded_sounds_offset = 1;
        return 0;
    }

    if (sb->version == 0x00130004 && sb->platform == UBI_XBOX) {
        config_sb_entry(sb, 0x48, 0x50);

        config_sb_audio_fb(sb, 0x1c, (1 << 3), (1 << 4), (1 << 10));
        config_sb_audio_he(sb, 0x3c, 0x34, 0x20, 0x28, 0x44, 0x40);

        sb->cfg.audio_extra_offset  = 0x10;
        sb->cfg.audio_stream_offset = 0x14;
        return 0;
    }

    if (sb->version == 0x00150000 && sb->platform == UBI_PC) {
        config_sb_entry(sb, 0x68, 0x78);

        config_sb_audio_fs(sb, 0x2c, 0x34, 0x30);
        config_sb_audio_he(sb, 0x5c, 0x54, 0x40, 0x48, 0x64, 0x60);

        config_sb_sequence(sb, 0x2c, 0x14);
        return 0;
    }

    if (sb->version == 0x00150000 && sb->platform == UBI_PS2) {
        config_sb_entry(sb, 0x48, 0x5c);

        config_sb_audio_fb_ps2(sb, 0x20, (1 << 2), (1 << 3), (1 << 4), (1 << 5));
        config_sb_audio_he(sb, 0x2c, 0x30, 0x3c, 0x44, 0x4c, 0x50);

        config_sb_sequence(sb, 0x2c, 0x10);
        return 0;
    }

    if ((sb->version == 0x00150000 && sb->platform == UBI_XBOX) ||
        (sb->version == 0x00160002 && sb->platform == UBI_XBOX) ||
        (sb->version == 0x00170000 && sb->platform == UBI_XBOX)) {
        config_sb_entry(sb, 0x48, 0x58);

        config_sb_audio_fb(sb, 0x20, (1 << 3), (1 << 4), (1 << 10));
        config_sb_audio_he(sb, 0x44, 0x3c, 0x28, 0x30, 0x4c, 0x48);

        config_sb_sequence(sb, 0x2c, 0x10);

        config_sb_layer_he(sb, 0x20, 0x2c, 0x34, 0x3c);
        config_sb_layer_sh(sb, 0x30, 0x00, 0x08, 0x0c, 0x14);
        return 0;
    }

    if ((sb->version == 0x00150000 && sb->platform == UBI_GC) ||
        (sb->version == 0x00160002 && sb->platform == UBI_GC)) {
        config_sb_entry(sb, 0x68, 0x6c);

        config_sb_audio_fs(sb, 0x28, 0x2c, 0x30);
        config_sb_audio_he(sb, 0x58, 0x50, 0x3c, 0x44, 0x60, 0x5c);

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x38, 0x40, 0x48);
        config_sb_layer_sh(sb, 0x30, 0x00, 0x08, 0x0c, 0x14);
        return 0;
    }

    if ((sb->version == 0x00160002 && sb->platform == UBI_PS2) ||
        (sb->version == 0x00180003 && sb->platform == UBI_PS2) ||
        (sb->version == 0x00180003 && sb->platform == UBI_PSP) ||
        (sb->version == 0x00180005 && sb->platform == UBI_PSP) ||
        (sb->version == 0x00180006 && sb->platform == UBI_PSP) ||
        (sb->version == 0x00180007 && sb->platform == UBI_PSP)) {
        config_sb_entry(sb, 0x48, 0x54);

        config_sb_audio_fb_ps2(sb, 0x20, (1 << 2), (1 << 3), (1 << 4), (1 << 5));
        config_sb_audio_he(sb, 0x28, 0x2c, 0x34, 0x3c, 0x44, 0x48);

        config_sb_sequence(sb, 0x2c, 0x10);

        config_sb_layer_he(sb, 0x20, 0x2c, 0x30, 0x38);
        config_sb_layer_sh(sb, 0x34, 0x00, 0x08, 0x0c, 0x14);

        config_sb_silence_f(sb, 0x1c);

        if (sb->version == 0x00180006 && sb->platform == UBI_PSP)
            sb->cfg.ignore_layer_error = 1;
        return 0;
    }

    if (sb->version == 0x00170001 && sb->platform == UBI_X360) {
        config_sb_entry(sb, 0x68, 0x70);

        config_sb_audio_fs(sb, 0x2c, 0x30, 0x34);
        config_sb_audio_he(sb, 0x5c, 0x54, 0x40, 0x48, 0x64, 0x60);
        sb->cfg.audio_xma_offset = 0;

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x38, 0x40, 0x48);
        config_sb_layer_sh(sb, 0x30, 0x00, 0x08, 0x0c, 0x14);
        sb->cfg.layer_hijack = LAYER_HIJACK_GRAW_X360;
        return 0;
    }

    if (sb->version == 0x00180003 && sb->platform == UBI_PC) {
        config_sb_entry(sb, 0x68, 0x78);

        config_sb_audio_fs(sb, 0x2c, 0x34, 0x30);
        config_sb_audio_he(sb, 0x5c, 0x54, 0x40, 0x48, 0x64, 0x60);

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x38, 0x3c, 0x44);
        config_sb_layer_sh(sb, 0x34, 0x00, 0x08, 0x0c, 0x14);

        config_sb_silence_f(sb, 0x1c);
        return 0;
    }

    if (sb->version == 0x00180003 && sb->platform == UBI_XBOX) {
        config_sb_entry(sb, 0x48, 0x58);

        config_sb_audio_fb(sb, 0x20, (1 << 3), (1 << 4), (1 << 10));
        config_sb_audio_he(sb, 0x44, 0x3c, 0x28, 0x30, 0x4c, 0x48);

        config_sb_sequence(sb, 0x2c, 0x10);

        config_sb_layer_he(sb, 0x20, 0x2c, 0x30, 0x38);
        config_sb_layer_sh(sb, 0x34, 0x00, 0x08, 0x0c, 0x14);

        config_sb_silence_f(sb, 0x1c);
        return 0;
    }

    if (sb->version == 0x00180003 && sb->platform == UBI_GC) {
        config_sb_entry(sb, 0x68, 0x6c);

        config_sb_audio_fs(sb, 0x28, 0x2c, 0x30);
        config_sb_audio_he(sb, 0x58, 0x50, 0x3c, 0x44, 0x60, 0x5c);

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x38, 0x3c, 0x44);
        config_sb_layer_sh(sb, 0x34, 0x00, 0x08, 0x0c, 0x14);

        config_sb_silence_f(sb, 0x1c);
        return 0;
    }

    if (sb->version == 0x00180003 && sb->platform == UBI_X360) {
        config_sb_entry(sb, 0x68, 0x74);

        config_sb_audio_fs(sb, 0x2c, 0x30, 0x34);
        config_sb_audio_he(sb, 0x5c, 0x54, 0x40, 0x48, 0x64, 0x60);
        sb->cfg.audio_xma_offset = 0x70;

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x38, 0x3c, 0x44);
        config_sb_layer_sh(sb, 0x34, 0x00, 0x08, 0x0c, 0x14);

        config_sb_silence_f(sb, 0x1c);
        return 0;
    }

    if (sb->version == 0x00180006 && sb->platform == UBI_PC) {
        if (check_project_file(s, "Sc4_online_SoundProject.SP0", 1))
            is_sc4_pc_online = 1;
    }

    if (sb->version == 0x00180006 && sb->platform == UBI_PC) {
        config_sb_entry(sb, 0x68, 0x7c);
        if (is_sc4_pc_online)
            config_sb_entry(sb, 0x68, 0x78);

        config_sb_audio_fs(sb, 0x2c, 0x34, 0x30);
        config_sb_audio_he(sb, 0x5c, 0x54, 0x40, 0x48, 0x64, 0x60);

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x38, 0x3c, 0x44);
        config_sb_layer_sh(sb, 0x34, 0x00, 0x08, 0x0c, 0x14);
        return 0;
    }

    if (sb->version == 0x00180006 && sb->platform == UBI_X360) {
        config_sb_entry(sb, 0x68, 0x78);

        config_sb_audio_fs(sb, 0x2c, 0x30, 0x34);
        config_sb_audio_he(sb, 0x5c, 0x54, 0x40, 0x48, 0x64, 0x60);
        sb->cfg.audio_xma_offset = 0x70;

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x38, 0x3c, 0x44);
        config_sb_layer_sh(sb, 0x34, 0x00, 0x08, 0x0c, 0x14);
        return 0;
    }

    if ((sb->version == 0x00180006 && sb->platform == UBI_WII) ||
        (sb->version == 0x00180007 && sb->platform == UBI_WII) ||
        (sb->version == 0x00180008 && sb->platform == UBI_WII)) {
        config_sb_entry(sb, 0x68, 0x6c);

        config_sb_audio_fs(sb, 0x28, 0x2c, 0x30);
        config_sb_audio_he(sb, 0x58, 0x50, 0x3c, 0x44, 0x60, 0x5c);

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x38, 0x3c, 0x44);
        config_sb_layer_sh(sb, 0x34, 0x00, 0x08, 0x0c, 0x14);
        return 0;
    }

    if (sb->version == 0x0018000b && sb->platform == UBI_X360) {
        config_sb_entry(sb, 0x68, 0x70);

        config_sb_audio_fs(sb, 0x28, 0x2c, 0x30);
        config_sb_audio_he(sb, 0x3c, 0x40, 0x48, 0x50, 0x58, 0x5c);
        sb->cfg.audio_xma_offset = 0x68;

        config_sb_sequence(sb, 0x2c, 0x14);
        return 0;
    }

    if ((sb->version == 0x00190001 && sb->platform == UBI_PSP) ||
        (sb->version == 0x00190005 && sb->platform == UBI_PSP)) {
        config_sb_entry(sb, 0x48, 0x58);

        config_sb_audio_fb(sb, 0x20, (1 << 2), (1 << 3), (1 << 4));
        config_sb_audio_he(sb, 0x28, 0x2c, 0x34, 0x3c, 0x44, 0x48);

        config_sb_sequence(sb, 0x2c, 0x10);

        config_sb_layer_he(sb, 0x20, 0x2c, 0x30, 0x38);
        config_sb_layer_sh(sb, 0x30, 0x00, 0x04, 0x08, 0x10);
        return 0;
    }

    if ((sb->version == 0x00190002 && sb->platform == UBI_PC) ||
        (sb->version == 0x00190005 && sb->platform == UBI_PC)) {
        config_sb_entry(sb, 0x68, 0x74);

        config_sb_audio_fs(sb, 0x28, 0x2c, 0x30);
        config_sb_audio_he(sb, 0x3c, 0x40, 0x48, 0x50, 0x58, 0x5c);

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x34, 0x38, 0x40);
        config_sb_layer_sh(sb, 0x30, 0x00, 0x04, 0x08, 0x10);

        config_sb_silence_f(sb, 0x1c);
        return 0;
    }

    if (sb->version == 0x00190002 && sb->platform == UBI_PS2) {
        config_sb_entry(sb, 0x48, 0x5c);

        config_sb_audio_fb_ps2(sb, 0x20, (1 << 2), (1 << 3), (1 << 4), (1 << 5));
        config_sb_audio_he(sb, 0x28, 0x2c, 0x34, 0x3c, 0x44, 0x48);

        config_sb_sequence(sb, 0x2c, 0x10);

        config_sb_layer_he(sb, 0x20, 0x2c, 0x30, 0x38);
        config_sb_layer_sh(sb, 0x30, 0x00, 0x04, 0x08, 0x10);

        config_sb_silence_f(sb, 0x1c);
        return 0;
    }

    if ((sb->version == 0x00190002 && sb->platform == UBI_GC) ||
        (sb->version == 0x00190005 && sb->platform == UBI_GC)) {
        config_sb_entry(sb, 0x68, 0x6c);

        config_sb_audio_fs(sb, 0x28, 0x2c, 0x30);
        config_sb_audio_he(sb, 0x3c, 0x40, 0x48, 0x50, 0x58, 0x5c);

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x34, 0x38, 0x40);
        config_sb_layer_sh(sb, 0x30, 0x00, 0x04, 0x08, 0x10);

        config_sb_silence_f(sb, 0x1c);
        return 0;
    }

    if ((sb->version == 0x00190002 && sb->platform == UBI_X360) ||
        (sb->version == 0x00190002 && sb->platform == UBI_WII) ||
        (sb->version == 0x00190003 && sb->platform == UBI_WII) ||
        (sb->version == 0x00190005 && sb->platform == UBI_WII) ||
        (sb->version == 0x00190005 && sb->platform == UBI_PS3) ||
        (sb->version == 0x00190005 && sb->platform == UBI_X360)) {
        config_sb_entry(sb, 0x68, 0x70);
        sb->cfg.audio_fix_psx_samples = 1;

        config_sb_audio_fs(sb, 0x28, 0x2c, 0x30);
        config_sb_audio_he(sb, 0x3c, 0x40, 0x48, 0x50, 0x58, 0x5c);
        sb->cfg.audio_xma_offset = 0x6c;
        sb->cfg.audio_interleave = 0x10;

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x34, 0x38, 0x40);
        config_sb_layer_sh(sb, 0x30, 0x00, 0x04, 0x08, 0x10);
        return 0;
    }

    if (sb->version == 0x001A0003 && sb->platform == UBI_PS3) {
        config_sb_entry(sb, 0x6c, 0x78);

        config_sb_audio_fs(sb, 0x30, 0x34, 0x38);
        config_sb_audio_he(sb, 0x40, 0x44, 0x4c, 0x54, 0x5c, 0x60);

        config_sb_sequence(sb, 0x2c, 0x14);

        return 0;
    }

    if (sb->version == 0x001A0003 && sb->platform == UBI_WII) {
        config_sb_entry(sb, 0x6c, 0x78);

        config_sb_audio_fs(sb, 0x2c, 0x30, 0x34);
        config_sb_audio_he(sb, 0x40, 0x44, 0x4c, 0x54, 0x5c, 0x60);
        return 0;
    }

    if ((sb->version == 0x001B0001 && sb->platform == UBI_X360) ||
        (sb->version == 0x001C0000 && sb->platform == UBI_PS3) ||
        (sb->version == 0x001C0000 && sb->platform == UBI_X360)) {
        config_sb_entry(sb, 0x64, 0x7c);

        config_sb_audio_fs(sb, 0x28, 0x30, 0x34);
        config_sb_audio_he(sb, 0x44, 0x48, 0x50, 0x58, 0x60, 0x64);
        sb->cfg.audio_xma_offset = 0x78;
        sb->cfg.audio_interleave = 0x10;
        sb->cfg.audio_fix_psx_samples = 1;

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x44, 0x48, 0x54);
        config_sb_layer_sh(sb, 0x30, 0x00, 0x04, 0x08, 0x10);
        return 0;
    }

    if (sb->version == 0x001D0000 && sb->platform == UBI_PSP) {
        config_sb_entry(sb, 0x40, 0x60);

        config_sb_audio_fb(sb, 0x20, (1 << 2), (1 << 3), (1 << 5));
        config_sb_audio_he(sb, 0x28, 0x30, 0x38, 0x40, 0x48, 0x4c);
        return 0;
    }

    if ((sb->version == 0x001D0000 && sb->platform == UBI_PS3) ||
        (sb->version == 0x001D0000 && sb->platform == UBI_WII)) {
        config_sb_entry(sb, 0x5c, 0x80);
        sb->cfg.audio_interleave = 0x10;
        sb->cfg.audio_fix_psx_samples = 1;

        config_sb_audio_fs(sb, 0x28, 0x30, 0x34);
        config_sb_audio_he(sb, 0x44, 0x4c, 0x54, 0x5c, 0x64, 0x68);

        config_sb_sequence(sb, 0x2c, 0x14);

        config_sb_layer_he(sb, 0x20, 0x44, 0x48, 0x54);
        config_sb_layer_sh(sb, 0x38, 0x00, 0x04, 0x08, 0x10);
        return 0;
    }

    av_log(s, AV_LOG_DEBUG, "unknown SB/SM version+platform %08x\n", sb->version);

    return AVERROR_INVALIDDATA;
}

static int parse_type_audio_ps2_bnm(ubi_sb_header *sb, int64_t offset, AVFormatContext *s)
{
    unsigned int (*read_32bit)(AVIOContext *s) = sb->big_endian ? avio_rb32 : avio_rl32;
    unsigned int (*read_16bit)(AVIOContext *s) = sb->big_endian ? avio_rb16 : avio_rl16;
    AVIOContext *pb = s->pb;

    avio_seek(pb, offset + sb->cfg.audio_stream_size, SEEK_SET);
    sb->stream_size     = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.audio_stream_offset, SEEK_SET);
    sb->stream_offset   = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.audio_channels, SEEK_SET);
    sb->channels        = avio_r8(pb);
    avio_seek(pb, offset + sb->cfg.audio_sample_rate, SEEK_SET);
    sb->sample_rate     = read_16bit(pb);

    if (sb->stream_size == 0) {
        av_log(s, AV_LOG_ERROR, "bad stream size\n");
        goto fail;
    }

    avio_seek(pb, offset + sb->cfg.audio_streamed_flag, SEEK_SET);
    sb->is_streamed     = read_32bit(pb) & sb->cfg.audio_streamed_and;
    avio_seek(pb, offset + sb->cfg.audio_cd_streamed_flag, SEEK_SET);
    sb->is_cd_streamed  = read_32bit(pb) & sb->cfg.audio_cd_streamed_and;
    avio_seek(pb, offset + sb->cfg.audio_loop_flag, SEEK_SET);
    sb->loop_flag       = read_32bit(pb) & sb->cfg.audio_loop_and;

    sb->num_samples = 0;

    if (!sb->is_cd_streamed)
        sb->stream_size *= sb->channels;

    if (sb->is_streamed) {
        if (sb->is_cd_streamed) {
            sprintf(sb->resource_name, "BNK_%d.VSC", sb->bank_number);
        } else {
            sprintf(sb->resource_name, "BNK_%d.VSB", sb->bank_number);
        }
    } else {
        sprintf(sb->resource_name, "BNK_%d.VB", sb->bank_number);
    }

    sb->is_external = 1;

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static uint32_t ubi_ps2_pitch_to_freq(uint32_t pitch)
{
    double sample_rate = (((double)pitch / 65536) * 48000);

    return (uint32_t)ceil(sample_rate);
}

static void blk_get_resource_name(ubi_sb_header *sb)
{
    if (sb->is_streamed) {
        if (sb->is_localized) {
            strcpy(sb->resource_name, "STRLANG.BLK");
        } else {
            strcpy(sb->resource_name, "../STREAMED.BLK");
        }
    } else {
        if (sb->is_localized) {
            strcpy(sb->resource_name, "MAPLANG.BLK");
        } else {
            strcpy(sb->resource_name, "../MAP.BLK");
        }
    }
}

static int parse_type_audio_ps2_old(ubi_sb_header *sb, int64_t offset, AVFormatContext *s)
{
    unsigned int (*read_32bit)(AVIOContext *s) = sb->big_endian ? avio_rb32 : avio_rl32;
    AVIOContext *pb = s->pb;
    av_unused uint32_t test_sample_rate;
    uint32_t pitch;
    int is_stereo;

    avio_seek(pb, offset + sb->cfg.audio_stream_size, SEEK_SET);
    sb->stream_size = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.audio_stream_offset, SEEK_SET);
    sb->stream_offset = read_32bit(pb);

    if (sb->stream_size == 0) {
        av_log(s, AV_LOG_ERROR, "bad stream size\n");
        goto fail;
    }

    avio_seek(pb, offset + sb->cfg.audio_pitch, SEEK_SET);
    pitch               = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.audio_sample_rate, SEEK_SET);
    test_sample_rate    = read_32bit(pb);
    sb->sample_rate     = ubi_ps2_pitch_to_freq(pitch);

    avio_seek(pb, offset + sb->cfg.audio_streamed_flag, SEEK_SET);
    sb->is_streamed     = read_32bit(pb) & sb->cfg.audio_streamed_and;
    avio_seek(pb, offset + sb->cfg.audio_loop_flag, SEEK_SET);
    sb->loop_flag       = read_32bit(pb) & sb->cfg.audio_loop_and;
    avio_seek(pb, offset + sb->cfg.audio_loc_flag, SEEK_SET);
    sb->is_localized    = read_32bit(pb) & sb->cfg.audio_loc_and;
    avio_seek(pb, offset + sb->cfg.audio_stereo_flag, SEEK_SET);
    is_stereo           = read_32bit(pb) & sb->cfg.audio_stereo_and;

    sb->num_samples = 0;
    sb->channels = is_stereo ? 2 : 1;
    sb->stream_size *= sb->channels;
    sb->subblock_id = 0;

    if (sb->is_blk) {
        blk_get_resource_name(sb);
        sb->is_external = 1;
    } else if (sb->is_streamed) {
        strcpy(sb->resource_name, sb->is_localized ? "STRM.LM1" : "STRM.SM1");
        sb->is_external = 1;
    }

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static int parse_type_audio(ubi_sb_header *sb, int64_t offset, AVFormatContext *s)
{
    unsigned int (*read_32bit)(AVIOContext *s) = sb->big_endian ? avio_rb32 : avio_rl32;
    unsigned int (*read_16bit)(AVIOContext *s) = sb->big_endian ? avio_rb16 : avio_rl16;
    AVIOContext *pb = s->pb;

    sb->type = UBI_AUDIO;

    if (sb->is_ps2_bnm)
        return parse_type_audio_ps2_bnm(sb, offset, s);

    if (sb->is_ps2_old)
        return parse_type_audio_ps2_old(sb, offset, s);

    avio_seek(pb, offset + sb->cfg.audio_extra_offset, SEEK_SET);
    sb->extra_offset    = read_32bit(pb) + sb->sectionX_offset;
    avio_seek(pb, offset + sb->cfg.audio_stream_size, SEEK_SET);
    sb->stream_size     = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.audio_stream_offset, SEEK_SET);
    sb->stream_offset   = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.audio_channels, SEEK_SET);
    sb->channels        = (sb->cfg.audio_channels & 3) ? read_16bit(pb) : read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.audio_sample_rate, SEEK_SET);
    sb->sample_rate     = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.audio_stream_type, SEEK_SET);
    sb->stream_type     = read_32bit(pb);

    if (sb->stream_size == 0) {
        av_log(s, AV_LOG_ERROR, "bad stream size\n");
        goto fail;
    }

    avio_seek(pb, offset + sb->cfg.audio_streamed_flag, SEEK_SET);
    sb->is_streamed = read_32bit(pb) & sb->cfg.audio_streamed_and;
    sb->is_external = sb->is_streamed;

    if (sb->cfg.audio_internal_flag && !sb->is_streamed) {
        avio_seek(pb, offset + sb->cfg.audio_internal_flag, SEEK_SET);
        sb->is_external = (int)!(read_32bit(pb));
    }

    if (sb->cfg.audio_software_flag && sb->cfg.audio_software_and) {
        avio_seek(pb, offset + sb->cfg.audio_software_flag, SEEK_SET);
        int software_flag = read_32bit(pb) & sb->cfg.audio_software_and;
        sb->subblock_id = (!software_flag) ? 0 : 1;

        if (sb->platform == UBI_PS2) {
            avio_seek(pb, offset + sb->cfg.audio_hwmodule_flag, SEEK_SET);
            int hwmodule_flag = read_32bit(pb) & sb->cfg.audio_hwmodule_and;
            sb->subblock_id = (!software_flag) ? ((!hwmodule_flag) ? 0 : 3) : 1;
        }

        if (!software_flag && sb->platform != UBI_PS3 && !(sb->platform == UBI_PSP && !sb->is_psp_old))
            sb->stream_type = 0x00;
    } else {
        sb->subblock_id = (sb->stream_type == 0x01) ? 0 : 1;
    }

    if (sb->cfg.has_rs_files && !sb->is_external) {
        avio_seek(pb, offset + sb->cfg.audio_ram_streamed_flag, SEEK_SET);
        sb->is_ram_streamed = read_32bit(pb) & sb->cfg.audio_ram_streamed_and;
        sb->is_external = sb->is_ram_streamed;
    }

    avio_seek(pb, offset + sb->cfg.audio_loop_flag, SEEK_SET);
    sb->loop_flag = read_32bit(pb) & sb->cfg.audio_loop_and;

    if (sb->loop_flag) {
        avio_seek(pb, offset + sb->cfg.audio_num_samples, SEEK_SET);
        sb->loop_start  = read_32bit(pb);
        avio_seek(pb, offset + sb->cfg.audio_num_samples2, SEEK_SET);
        sb->num_samples = read_32bit(pb) + sb->loop_start;

        if (sb->cfg.audio_num_samples == sb->cfg.audio_num_samples2) {
            sb->num_samples = sb->loop_start;
            sb->loop_start = 0;
        }
    } else {
        avio_seek(pb, offset + sb->cfg.audio_num_samples, SEEK_SET);
        sb->num_samples = read_32bit(pb);
    }

    if (sb->cfg.resource_name_size > sizeof(sb->resource_name))
        goto fail;

    if (sb->cfg.audio_stream_name) {
        if (sb->is_dat && !sb->is_external) {
            avio_seek(pb, offset + sb->cfg.audio_stream_name + 0x01, SEEK_SET);
            sb->subbank_index = avio_r8(pb);
        } else if (sb->cfg.has_rs_files && sb->is_ram_streamed) {
            strcpy(sb->resource_name, "MAPS.RS1");
        } else if (sb->is_external || sb->cfg.audio_has_internal_names) {
            avio_seek(pb, offset + sb->cfg.audio_stream_name, SEEK_SET);
            avio_get_str(pb, sb->cfg.resource_name_size, sb->resource_name, sb->cfg.resource_name_size);
        }
    } else {
        avio_seek(pb, offset + sb->cfg.audio_extra_name, SEEK_SET);
        sb->cfg.audio_stream_name = read_32bit(pb);
        if (sb->cfg.audio_stream_name != 0xFFFFFFFF) {
            avio_seek(pb, sb->sectionX_offset + sb->cfg.audio_stream_name, SEEK_SET);
            avio_get_str(pb, sb->cfg.resource_name_size, sb->resource_name, sb->cfg.resource_name_size);
        }
    }

    if (sb->cfg.audio_xma_offset) {
        avio_seek(pb, offset + sb->cfg.audio_xma_offset, SEEK_SET);
        sb->xma_header_offset = read_32bit(pb) + sb->sectionX_offset;
    }

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static void get_ubi_bank_name(ubi_sb_header *sb, int bank_number, char *bank_name)
{
    if (sb->is_bnm) {
        sprintf(bank_name, "Bnk_%d.bnm", bank_number);
    } else if (sb->is_dat) {
        sprintf(bank_name, "BNK_%d.DAT", bank_number);
    } else if (sb->is_ps2_bnm) {
        sprintf(bank_name, "BNK_%d.BNM", bank_number);
    } else {
        strcpy(bank_name, "ERROR");
    }
}

static int is_other_bank(ubi_sb_header *sb, AVFormatContext *s, int bank_number)
{
    char bank_name[255];

    get_ubi_bank_name(sb, bank_number, bank_name);

    return strncmp(s->url, bank_name, 255) != 0;
}

static int parse_type_sequence(ubi_sb_header *sb, int64_t offset, AVFormatContext *s)
{
    unsigned int (*read_32bit)(AVIOContext *s) = sb->big_endian ? avio_rb32 : avio_rl32;
    AVIOContext *pb = s->pb;
    int64_t table_offset;

    sb->type = UBI_SEQUENCE;
    if (sb->cfg.sequence_sequence_count == 0) {
        av_log(s, AV_LOG_ERROR, "sequence not configured at %lx\n", offset);
        goto fail;
    }

    avio_seek(pb, offset + sb->cfg.sequence_extra_offset, SEEK_SET);
    sb->extra_offset        = read_32bit(pb) + sb->sectionX_offset;
    avio_seek(pb, offset + sb->cfg.sequence_sequence_loop_start, SEEK_SET);
    sb->sequence_loop_start = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.sequence_sequence_num_loops, SEEK_SET);
    sb->sequence_num_loops  = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.sequence_sequence_count, SEEK_SET);
    sb->sequence_count      = read_32bit(pb);

    if (sb->sequence_count > SB_MAX_CHAIN_COUNT) {
        av_log(s, AV_LOG_ERROR, "incorrect sequence count %i vs %i\n", sb->sequence_count, SB_MAX_CHAIN_COUNT);
        goto fail;
    }

    table_offset = sb->extra_offset;
    for (int i = 0; i < sb->sequence_count; i++) {
        avio_seek(pb, table_offset + sb->cfg.sequence_entry_number, SEEK_SET);
        uint32_t entry_number = read_32bit(pb);

        if (sb->has_numbered_banks) {
            int16_t bank_number = (entry_number >> 16) & 0xFFFF;
            entry_number        = (entry_number >> 00) & 0xFFFF;

            sb->sequence_banks[i] = bank_number;

            if (!sb->sequence_multibank) {
                sb->sequence_multibank = is_other_bank(sb, s, bank_number);
            }
        } else {
            entry_number = entry_number & 0x3FFFFFFF;
            if (entry_number > sb->section2_num) {
                av_log(s, AV_LOG_ERROR, "chain with wrong entry %i vs %i at %lx\n", entry_number, sb->section2_num, sb->extra_offset);
                goto fail;
            }
        }

        sb->sequence_chain[i] = entry_number;

        table_offset += sb->cfg.sequence_entry_size;
    }

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static int parse_type_layer_ps2_old(ubi_sb_header *sb, int64_t offset, AVFormatContext *s)
{
    unsigned int (*read_32bit)(AVIOContext *s) = sb->big_endian ? avio_rb32 : avio_rl32;
    AVIOContext *pb = s->pb;
    uint32_t pitch;

    avio_seek(pb, offset + sb->cfg.layer_layer_count, SEEK_SET);
    sb->layer_count     = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.audio_stream_size, SEEK_SET);
    sb->stream_size     = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.audio_stream_offset, SEEK_SET);
    sb->stream_offset   = read_32bit(pb);

    if (sb->stream_size == 0) {
        av_log(s, AV_LOG_ERROR, "bad stream size\n");
        goto fail;
    }

    if (sb->layer_count > SB_MAX_LAYER_COUNT) {
        av_log(s, AV_LOG_ERROR, "incorrect layer count\n");
        goto fail;
    }

    avio_seek(pb, offset + sb->cfg.layer_pitch, SEEK_SET);
    pitch               = read_32bit(pb);
    sb->sample_rate     = ubi_ps2_pitch_to_freq(pitch);
    avio_seek(pb, offset + sb->cfg.layer_loc_flag, SEEK_SET);
    sb->is_localized    = read_32bit(pb) & sb->cfg.layer_loc_and;

    sb->num_samples = 0;
    sb->channels = sb->layer_count * 2;
    sb->stream_size *= sb->channels;

    if (sb->is_blk) {
        blk_get_resource_name(sb);
        sb->is_external = 1;
    } else if (sb->is_streamed) {
        strcpy(sb->resource_name, sb->is_localized ? "STRM.LM1" : "STRM.SM1");
        sb->is_external = 1;
    }

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static int parse_type_layer(ubi_sb_header *sb, int64_t offset, AVFormatContext *s)
{
    unsigned int (*read_32bit)(AVIOContext *s) = sb->big_endian ? avio_rb32 : avio_rl32;
    unsigned int (*read_16bit)(AVIOContext *s) = sb->big_endian ? avio_rb16 : avio_rl16;
    AVIOContext *pb = s->pb;
    int64_t table_offset;

    sb->type = UBI_LAYER;
    if (sb->cfg.layer_layer_count == 0) {
        av_log(s, AV_LOG_ERROR, "layers not configured at %lx\n", offset);
        goto fail;
    }

    sb->is_streamed = 1;

    if (sb->is_ps2_old)
        return parse_type_layer_ps2_old(sb, offset, s);

    avio_seek(pb, offset + sb->cfg.layer_extra_offset, SEEK_SET);
    sb->extra_offset    = read_32bit(pb) + sb->sectionX_offset;
    avio_seek(pb, offset + sb->cfg.layer_layer_count, SEEK_SET);
    sb->layer_count     = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.layer_stream_size, SEEK_SET);
    sb->stream_size     = read_32bit(pb);
    avio_seek(pb, offset + sb->cfg.layer_stream_offset, SEEK_SET);
    sb->stream_offset   = read_32bit(pb);

    if (sb->stream_size == 0) {
        av_log(s, AV_LOG_ERROR, "bad stream size\n");
        goto fail;
    }

    if (sb->layer_count > SB_MAX_LAYER_COUNT) {
        av_log(s, AV_LOG_ERROR, "incorrect layer count\n");
        goto fail;
    }

    sb->is_external = sb->is_streamed;

    table_offset = sb->extra_offset;
    avio_seek(pb, table_offset + sb->cfg.layer_sample_rate, SEEK_SET);
    sb->sample_rate = read_32bit(pb);
    avio_seek(pb, table_offset + sb->cfg.layer_stream_type, SEEK_SET);
    sb->stream_type = read_32bit(pb);
    avio_seek(pb, table_offset + sb->cfg.layer_num_samples, SEEK_SET);
    sb->num_samples = read_32bit(pb);

    for (int i = 0; i < sb->layer_count; i++) {
        avio_seek(pb, table_offset + sb->cfg.layer_channels, SEEK_SET);
        int channels = (sb->cfg.layer_channels & 3) ? read_16bit(pb) : read_32bit(pb);
        avio_seek(pb, table_offset + sb->cfg.layer_sample_rate, SEEK_SET);
        int sample_rate = read_32bit(pb);
        avio_seek(pb, table_offset + sb->cfg.layer_stream_type, SEEK_SET);
        int stream_type = read_32bit(pb);
        avio_seek(pb, table_offset + sb->cfg.layer_num_samples, SEEK_SET);
        int num_samples = read_32bit(pb);

        if (sb->sample_rate != sample_rate || sb->stream_type != stream_type) {
            av_log(s, AV_LOG_DEBUG, "%i layer headers don't match at %lx > %lx\n", sb->layer_count, offset, table_offset);
            if (!sb->cfg.ignore_layer_error)
                goto fail;
        }

        sb->layer_channels[i] = channels;

        if (sb->num_samples != num_samples && sb->num_samples + 1 == num_samples) {
            sb->num_samples -= 1;
        }

        table_offset += sb->cfg.layer_entry_size;
    }

    if (sb->cfg.layer_stream_name) {
        avio_seek(pb, offset + sb->cfg.layer_stream_name, SEEK_SET);
        avio_get_str(pb, sb->cfg.resource_name_size, sb->resource_name, sb->cfg.resource_name_size);
    } else if (sb->cfg.layer_extra_name) {
        avio_seek(pb, offset + sb->cfg.layer_extra_name, SEEK_SET);
        sb->cfg.layer_stream_name = read_32bit(pb);
        if (sb->cfg.layer_stream_name != 0xFFFFFFFF) {
            avio_seek(pb, sb->sectionX_offset + sb->cfg.layer_stream_name, SEEK_SET);
            avio_get_str(pb, sb->cfg.resource_name_size, sb->resource_name, sb->cfg.resource_name_size);
        }
    }

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static int parse_type_silence(ubi_sb_header *sb, int64_t offset, AVFormatContext *s)
{
    unsigned int (*read_32bit)(AVIOContext *s) = sb->big_endian ? avio_rb32 : avio_rl32;
    AVIOContext *pb = s->pb;

    sb->type = UBI_SILENCE;
    if (sb->cfg.silence_duration_int == 0 && sb->cfg.silence_duration_float == 0) {
        av_log(s, AV_LOG_ERROR, "silence duration not configured at %x\n", (uint32_t)offset);
        goto fail;
    }

    if (sb->cfg.silence_duration_int) {
        avio_seek(pb, offset + sb->cfg.silence_duration_int, SEEK_SET);
        uint32_t duration_int = read_32bit(pb);
        sb->duration = (float)duration_int / 65536.0f;
    } else if (sb->cfg.silence_duration_float) {
        avio_seek(pb, offset + sb->cfg.silence_duration_float, SEEK_SET);
        sb->duration = av_int2float(read_32bit(pb));
    }

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static int parse_type_random(ubi_sb_header *sb, int64_t offset, AVFormatContext *s)
{
    unsigned int (*read_32bit)(AVIOContext *s) = sb->big_endian ? avio_rb32 : avio_rl32;
    int64_t sb_extra_offset, table_offset;
    AVIOContext *pb = s->pb;
    int sb_sequence_count;

    if (sb->cfg.random_entry_size == 0) {
        av_log(s, AV_LOG_ERROR, "random entry size not configured at %lx\n", offset);
        goto fail;
    }

    avio_seek(pb, offset + sb->cfg.random_extra_offset, SEEK_SET);
    sb_extra_offset    = read_32bit(pb) + sb->sectionX_offset;
    avio_seek(pb, offset + sb->cfg.random_sequence_count, SEEK_SET);
    sb_sequence_count  = read_32bit(pb);

    table_offset = sb_extra_offset;
    for (int i = 0; i < sb_sequence_count; i++) {
        avio_seek(pb, table_offset, SEEK_SET);
        uint32_t entry_number = read_32bit(pb);

        if (sb->has_numbered_banks) {
            int16_t bank_number = (entry_number >> 16) & 0xFFFF;
            entry_number        = (entry_number >> 00) & 0xFFFF;

            av_log(s, AV_LOG_DEBUG, "bnm sequence entry=%i, bank=%i\n", entry_number, bank_number);
            if (is_other_bank(sb, s, bank_number)) {
                av_log(s, AV_LOG_ERROR, "random in other bank\n");
                goto fail;
            }
        } else {
            entry_number = entry_number & 0x3FFFFFFF;
            if (entry_number > sb->section2_num) {
                av_log(s, AV_LOG_ERROR, "random with wrong entry %i vs %i at %lx\n", entry_number, sb->section2_num, sb->extra_offset);
                goto fail;
            }
        }

        {
            int64_t entry_offset = sb->section2_offset + sb->cfg.section2_entry_size * entry_number;
            return parse_type_audio(sb, entry_offset, s);
        }

        table_offset += sb->cfg.random_entry_size;
    }

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static int set_hardware_codec_for_platform(ubi_sb_header *sb, AVFormatContext *s)
{
    switch (sb->platform) {
    case UBI_PC:
        sb->codec = RAW_PCM;
        break;
    case UBI_PS2:
        sb->codec = RAW_PSX;
        break;
    case UBI_PSP:
        if (sb->is_psp_old)
            sb->codec = FMT_VAG;
        else
            sb->codec = RAW_PSX;
        break;
    case UBI_XBOX:
        sb->codec = RAW_XBOX;
        break;
    case UBI_GC:
    case UBI_WII:
        sb->codec = RAW_DSP;
        break;
    case UBI_X360:
        sb->codec = RAW_XMA1;
        break;
    case UBI_3DS:
        sb->codec = FMT_CWAV;
        break;
    default:
        av_log(s, AV_LOG_ERROR, "unknown hardware codec\n");
        return AVERROR_INVALIDDATA;
    }

    return 0;
}

static int parse_stream_codec(ubi_sb_header *sb, AVFormatContext *s)
{
    if (sb->type != UBI_AUDIO && sb->type != UBI_LAYER)
        return 0;

    if (sb->is_dat)
        return 0;

    if (sb->is_ps2_bnm || sb->is_ps2_old) {
        sb->codec = RAW_PSX;
        return 0;
    }

    if (sb->is_bnm || sb->version < 0x00000007) {
        switch (sb->stream_type) {
        case 0x01:
            if (sb->is_streamed)
                sb->codec = RAW_PCM;
            else if (set_hardware_codec_for_platform(sb, s))
                goto fail;
            break;
        case 0x02:
            sb->codec = FMT_MPDX;
            break;
        case 0x04:
            sb->codec = FMT_APM;
            break;
        case 0x06:
            sb->codec = UBI_ADPCM;
            break;
        case 0x08:
            sb->codec = UBI_IMA;
            break;
        default:
            av_log(s, AV_LOG_ERROR, "unknown stream_type %02x for version %08x\n", sb->stream_type, sb->version);
            goto fail;
        }
    } else if (sb->version < 0x000A0000) {
        switch (sb->stream_type) {
        case 0x01:
            if (sb->is_streamed)
                sb->codec = RAW_PCM;
            else if (set_hardware_codec_for_platform(sb, s))
                goto fail;
            break;
        case 0x02:
            sb->codec = UBI_ADPCM;
            break;
        case 0x04:
            sb->codec = UBI_IMA;
            break;
        default:
            av_log(s, AV_LOG_ERROR, "unknown stream_type %02x for version %08x\n", sb->stream_type, sb->version);
            goto fail;
        }
    } else {
        switch (sb->stream_type) {
        case 0x00:
            if (set_hardware_codec_for_platform(sb, s))
                goto fail;
            break;
        case 0x01:
            sb->codec = RAW_PCM;
            break;
        case 0x02:
            switch (sb->platform) {
            case UBI_PC:
            case UBI_XBOX:
                sb->codec = UBI_ADPCM;
                break;
            case UBI_PS3:
                sb->codec = RAW_PSX;
                break;
            case UBI_PSP:
                sb->codec = UBI_IMA_SCE;
                break;
            default:
                av_log(s, AV_LOG_ERROR, "unknown codec for stream_type %02x\n", sb->stream_type);
                goto fail;
            }
            break;
        case 0x03:
            sb->codec = UBI_IMA;
            break;
        case 0x04:
            sb->codec = FMT_OGG;
            break;
        case 0x05:
            switch (sb->platform) {
            case UBI_X360:
                sb->codec = FMT_XMA1;
                break;
            case UBI_PS3:
            case UBI_PSP:
                sb->codec = FMT_AT3;
                break;
            default:
                av_log(s, AV_LOG_ERROR, "unknown codec for stream_type %02x\n", sb->stream_type);
                goto fail;
            }
            break;
        case 0x06:
            sb->codec = RAW_PSX;
            break;
        case 0x07:
            sb->codec = RAW_AT3;
            break;
        case 0x08:
            sb->codec = FMT_AT3;
            break;
        default:
            av_log(s, AV_LOG_ERROR, "unknown stream_type %02x\n", sb->stream_type);
            goto fail;
        }
    }

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static int bnm_parse_offsets(ubi_sb_header *sb, AVFormatContext *s)
{
    unsigned int (*read_32bit)(AVIOContext *s) = sb->big_endian ? avio_rb32 : avio_rl32;
    AVIOContext *pb = s->pb;
    uint32_t block_offset;

    if (sb->is_external)
        return 0;

    if (sb->version == 0x00000000 || sb->version == 0x00000200) {
        switch (sb->stream_type) {
        case 0x01:
            avio_seek(pb, 0x1c, SEEK_SET);
            block_offset = read_32bit(pb);
            break;
        case 0x02:
            avio_seek(pb, 0x14, SEEK_SET);
            block_offset = read_32bit(pb);
            break;
        case 0x04:
            avio_seek(pb, 0x20, SEEK_SET);
            block_offset = read_32bit(pb);
            break;
        default:
            goto fail;
        }
    } else if (sb->version == 0x00060409) {
        switch (sb->stream_type) {
        case 0x01:
            avio_seek(pb, 0x18, SEEK_SET);
            block_offset = read_32bit(pb);
            break;
        case 0x06:
            avio_seek(pb, 0x14, SEEK_SET);
            block_offset = read_32bit(pb);
            break;
        default:
            goto fail;
        }
    } else {
        av_log(s, AV_LOG_ERROR, "unknown subblock offsets for version %08x", sb->version);
        goto fail;
    }

    sb->stream_offset += block_offset;

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static int blk_parse_offsets(ubi_sb_header *sb)
{
    if (sb->is_streamed) {
        sb->stream_offset *= 0x800;
    } else {
        return AVERROR_PATCHWELCOME;
    }

    return 0;
}

static int parse_offsets(ubi_sb_header *sb, AVFormatContext *s)
{
    unsigned int (*read_32bit)(AVIOContext *s) = sb->big_endian ? avio_rb32 : avio_rl32;
    AVIOContext *pb = s->pb;
    uint32_t i, j, k;

    if (sb->type != UBI_AUDIO && sb->type != UBI_LAYER)
        return 0;

    if (sb->is_bnm)
        return bnm_parse_offsets(sb, s);

    if (sb->is_dat)
        return 0;

    if (sb->is_ps2_bnm) {
        if (sb->is_cd_streamed)
            sb->stream_offset *= 0x800;
        return 0;
    }

    if (sb->is_blk)
        return blk_parse_offsets(sb);

    if (sb->is_map) {
        if (sb->is_external && !sb->is_ram_streamed)
            return 0;

        for (i = 0; i < sb->section3_num; i++) {
            int64_t offset = sb->section3_offset + 0x14 * i;
            avio_seek(pb, offset + 0x04, SEEK_SET);
            int64_t table_offset  = read_32bit(pb) + sb->section3_offset;
            uint32_t table_num  = read_32bit(pb);
            int64_t table2_offset = read_32bit(pb) + sb->section3_offset;
            uint32_t table2_num = read_32bit(pb);

            for (j = 0; j < table_num; j++) {
                avio_seek(pb, table_offset + 0x08 * j + 0x00, SEEK_SET);
                int index = read_32bit(pb) & 0x3FFFFFFF;

                if (index == sb->header_index) {
                    avio_seek(pb, table_offset + 0x08 * j + 0x04, SEEK_SET);
                    sb->stream_offset = read_32bit(pb);
                    if (sb->is_ram_streamed)
                        break;

                    for (k = 0; k < table2_num; k++) {
                        avio_seek(pb, table2_offset + 0x10 * k + 0x00, SEEK_SET);
                        uint32_t id = read_32bit(pb);

                        if (id == sb->subblock_id) {
                            avio_seek(pb, table2_offset + 0x10 * k + 0x0c, SEEK_SET);
                            sb->stream_offset += read_32bit(pb);
                            break;
                        }
                    }

                    if (k == table2_num) {
                        av_log(s, AV_LOG_ERROR, "failed to find subblock %d in map %s\n", sb->subblock_id, sb->map_name);
                        goto fail;
                    }
                    break;
                }
            }

            if (sb->stream_offset)
                break;
        }

        if (sb->stream_offset == 0 && !sb->is_external) {
            av_log(s, AV_LOG_ERROR, "failed to find offset for resource %d in subblock %d in map %s\n", sb->header_index, sb->subblock_id, sb->map_name);
            goto fail;
        }
    } else {
        int64_t sounds_offset;

        if (sb->is_external)
            return 0;

        sounds_offset = sb->section3_offset + sb->cfg.section3_entry_size*sb->section3_num;
        if (sb->cfg.is_padded_sounds_offset)
            sounds_offset = FFALIGN(sounds_offset, 0x10);
        sb->stream_offset = sounds_offset + sb->stream_offset;

        for (i = 0; i < sb->section3_num; i++) {
            int64_t offset = sb->section3_offset + sb->cfg.section3_entry_size * i;

            avio_seek(pb, offset, SEEK_SET);
            if (read_32bit(pb) == sb->subblock_id)
                break;

            sb->stream_offset += read_32bit(pb);
        }

        if (i == sb->section3_num) {
            av_log(s, AV_LOG_ERROR, "failed to find subblock %d\n", sb->subblock_id);
            goto fail;
        }
    }

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static int parse_header(ubi_sb_header *sb, AVFormatContext *s, int64_t offset, int index)
{
    unsigned int (*read_32bit)(AVIOContext *pb) = sb->big_endian ? avio_rb32 : avio_rl32;
    int ret = AVERROR_INVALIDDATA;
    AVIOContext *pb = s->pb;

    sb->header_index  = index;
    sb->header_offset = offset;

    avio_seek(pb, offset, SEEK_SET);
    sb->header_id = read_32bit(pb);
    sb->header_type = read_32bit(pb);

    switch (sb->header_type) {
    case 0x01:
        ret = parse_type_audio(sb, offset, s);
        break;
    case 0x05:
    case 0x0b:
    case 0x0c:
        ret = parse_type_sequence(sb, offset, s);
        break;
    case 0x06:
    case 0x0d:
        ret = parse_type_layer(sb, offset, s);
        break;
    case 0x08:
    case 0x0f:
        ret = parse_type_silence(sb, offset, s);
        break;
    case 0x0a:
        ret = parse_type_random(sb, offset, s);
        break;
    case 0x00:
        if (sb->is_dat) {
            sb->type = UBI_SILENCE;
            sb->duration = 1.0f;
            break;
        }
        // fall through
    default:
        av_log(s, AV_LOG_ERROR, "unknown header type %x at %lx\n", sb->header_type, offset);
        goto fail;
    }

    if (ret < 0)
        return ret;

    ret = parse_stream_codec(sb, s);
    if (ret < 0)
        return ret;

    ret = parse_offsets(sb, s);
    if (ret < 0)
        return ret;

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static void build_readable_name(char *buf, int64_t buf_size, ubi_sb_header *sb)
{
    const char *grp_name;
    const char *res_name;
    uint32_t id;
    uint32_t type;
    int index;

    if (sb->is_map) {
        grp_name = sb->map_name;
    } else if (sb->is_bnm || sb->is_ps2_bnm) {
        if (sb->sequence_multibank)
            grp_name = "bnm-multi";
        else
            grp_name = "bnm";
    } else if (sb->is_dat) {
        if (sb->sequence_multibank)
            grp_name = "dat-multi";
        else
            grp_name = "dat";
    } else if (sb->is_blk) {
        grp_name = "blk";
    } else {
        grp_name = "bank";
    }
    id = sb->header_id;
    type = sb->header_type;
    if (sb->is_map)
        index = sb->header_index;
    else
        index = sb->header_index;

    if (sb->type == UBI_SEQUENCE) {
        if (sb->sequence_num_loops) {
            if (sb->sequence_count == 1)
                res_name = "single";
            else
                res_name = "multi";
        } else {
            if (sb->sequence_count == 1)
                res_name = "single-loop";
            else
                res_name = (sb->sequence_loop_start == 0) ? "multi-loop" : "intro-loop";
        }
    } else {
        if (sb->is_external || sb->cfg.audio_has_internal_names)
            res_name = sb->resource_name;
        else
            res_name = NULL;
    }

    if (res_name && res_name[0]) {
        if (index >= 0)
            snprintf(buf, buf_size, "%s/%04d/%02x-%08x/%s", grp_name, index, type, id, res_name);
        else
            snprintf(buf, buf_size, "%s/%02x-%08x/%s", grp_name, type, id, res_name);
    } else {
        if (index >= 0)
            snprintf(buf, buf_size, "%s/%04d/%02x-%08x", grp_name, index, type, id);
        else
            snprintf(buf, buf_size, "%s/%02x-%08x", grp_name, type, id);
    }
}

static int64_t ps_bytes_to_samples(int64_t bytes, int channels)
{
    if (channels <= 0) return 0;
    return bytes / channels / 0x10 * 28;
}

static int parse_sb(ubi_sb_header *sb, AVFormatContext *s)
{
    unsigned int (*read_32bit)(AVIOContext *pb) = sb->big_endian ? avio_rb32 : avio_rl32;
    AVIOContext *pb = s->pb;

    sb->bank_streams = 0;
    for (int i = 0; i < sb->section2_num; i++) {
        int64_t offset = sb->section2_offset + sb->cfg.section2_entry_size*i;
        uint32_t header_type;
        int ret;

        avio_seek(pb, offset, SEEK_SET);
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        read_32bit(pb);
        header_type = read_32bit(pb);

        if (header_type >= 0x10) {
            av_log(s, AV_LOG_ERROR, "unknown type %x at %lx\n", header_type, offset);
            goto fail;
        }

        sb->types[header_type]++;
        if (!sb->allowed_types[header_type])
            continue;

        sb->bank_streams++;
        sb->total_streams++;
        ret = parse_header(sb, s, offset, i);
        if (ret < 0)
            return ret;

        if (sb->type != UBI_AUDIO || sb->is_external) {
            // TODO
            continue;
        }

        if (sb->channels <= 0 || sb->sample_rate <= 0)
            continue;

        build_readable_name(sb->readable_name, sizeof(sb->readable_name)-1, sb);

        {
            BNMStream *bst;
            AVStream *st;
            int codec;

            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            bst = av_mallocz(sizeof(*bst));
            if (!bst)
                return AVERROR(ENOMEM);
            st->priv_data = bst;

            switch (sb->codec) {
            case RAW_PCM:
                codec = (sb->cfg.audio_interleave == 2 || sb->channels == 1) ? AV_CODEC_ID_PCM_S16LE : AV_CODEC_ID_PCM_S16LE_PLANAR;
                break;
            case RAW_PSX:
                if (sb->cfg.has_rs_files && sb->stream_size > 0x30)
                    sb->stream_size -= 0x30;

                if (sb->is_ps2_bnm) {
                    sb->cfg.audio_interleave = sb->is_cd_streamed ?
                                               sb->cfg.audio_interleave :
                                               sb->stream_size / sb->channels;
                } else {
                    sb->cfg.audio_interleave = (sb->cfg.audio_interleave > 0) ?
                                                sb->cfg.audio_interleave :
                                                sb->stream_size / sb->channels;
                }

                if (sb->num_samples == 0)
                    sb->num_samples = ps_bytes_to_samples(sb->stream_size, sb->channels);

                if (sb->cfg.audio_fix_psx_samples)
                    sb->num_samples /= sb->channels;

                codec = AV_CODEC_ID_ADPCM_PSX;
                break;
            case RAW_XBOX:
                codec = AV_CODEC_ID_ADPCM_IMA_XBOX;
                break;
            case RAW_DSP:
                ret = ff_alloc_extradata(st->codecpar, 32 * sb->channels);
                if (ret < 0)
                    return ret;

                avio_seek(pb, sb->extra_offset + 0x10, SEEK_SET);
                for (int ch = 0; ch < sb->channels; ch++) {
                    avio_read(pb, st->codecpar->extradata + 32 * ch, 32);
                    avio_skip(pb, 32);
                }

                codec = AV_CODEC_ID_ADPCM_NDSP;
                break;
            case FMT_VAG:
                if (sb->stream_size > 0x30) {
                    avio_seek(pb, sb->stream_offset, SEEK_SET);
                    if (avio_rb32(pb) == AV_RB32("VAGp")) {
                        sb->stream_offset += 0x30;
                        sb->stream_size  -= 0x30;
                    }
                }

                codec = AV_CODEC_ID_ADPCM_PSX;
                break;
            case FMT_CWAV:
                if (sb->channels > 1)
                    return AVERROR_INVALIDDATA;
                sb->cfg.audio_interleave = 0x08;

                if (sb->stream_size <= 0xe0)
                    return AVERROR_INVALIDDATA;

                ret = ff_alloc_extradata(st->codecpar, 32 * sb->channels);
                if (ret < 0)
                    return ret;

                avio_seek(pb, sb->stream_offset + 0x7c, SEEK_SET);
                for (int ch = 0; ch < sb->channels; ch++) {
                    avio_read(pb, st->codecpar->extradata + 32 * ch, 32);
                    avio_skip(pb, 32);
                }
                sb->stream_offset += 0xe0;
                sb->stream_size -= 0xe0;

                codec = AV_CODEC_ID_ADPCM_NDSP_LE;
                break;
            case FMT_APM:
                sb->cfg.audio_interleave = 0x01;

                if (sb->stream_size <= 0x64)
                    return AVERROR_INVALIDDATA;

                sb->stream_offset += 0x64;
                sb->stream_size -= 0x64;

                codec = AV_CODEC_ID_ADPCM_IMA_DVI;
                break;
            default:
                codec = AV_CODEC_ID_NONE;
                av_log(s, AV_LOG_DEBUG, "unknown codec: %x\n", sb->codec);
                break;
            }

            bst->start_offset = sb->stream_offset;
            bst->stop_offset = bst->start_offset;
            bst->stop_offset += sb->stream_size;

            st->start_time = 0;
            if (sb->num_samples > 0)
                st->duration = sb->num_samples;
            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->ch_layout.nb_channels = sb->channels;
            if (sb->cfg.audio_interleave > 0)
                st->codecpar->block_align = sb->cfg.audio_interleave * sb->channels;
            else
                st->codecpar->block_align = sb->stream_size;
            st->codecpar->sample_rate = sb->sample_rate;

            st->codecpar->codec_id = codec;

            if (sb->readable_name[0] != '\0')
                av_dict_set(&st->metadata, "title", sb->readable_name, 0);

            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
        }
    }

    return 0;
fail:
    return AVERROR_INVALIDDATA;
}

static int read_header(AVFormatContext *s)
{
    UBIBNMDemuxContext *bnm = s->priv_data;
    ubi_sb_header *sb = &bnm->sb;
    AVIOContext *pb = s->pb;
    int64_t offset;
    int ret;

    sb->platform = UBI_PC;
    sb->big_endian = 0;
    sb->is_bnm = 1;

    sb->version = avio_rl32(pb);
    ret = config_sb_version(sb, s);
    if (ret < 0)
        return ret;

    avio_seek(pb, 0x04, SEEK_SET);
    sb->section1_offset = avio_rl32(pb);
    sb->section1_num = avio_rl32(pb);
    sb->section2_offset = avio_rl32(pb);
    sb->section2_num = avio_rl32(pb);
    sb->section3_offset = avio_rl32(pb);
    sb->section3_num = 0;

    sb->sectionX_offset = sb->section2_offset + sb->section2_num * sb->cfg.section2_entry_size;
    sb->sectionX_size = sb->section3_offset - sb->sectionX_offset;

    ret = parse_sb(sb, s);
    if (ret < 0)
        return ret;

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
        if (n == 0) {
            BNMStream *bst = st->priv_data;

            offset = bst->start_offset;
        }
    }

    avio_seek(pb, offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    UBIBNMDemuxContext *bnm = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    BNMStream *bst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (bnm->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[bnm->current_stream];
    bst = st->priv_data;
    if (do_seek)
        avio_seek(pb, bst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= bst->stop_offset) {
        do_seek = 1;
        bnm->current_stream++;
        goto redo;
    }

    {
        const int64_t pos = avio_tell(pb);
        const int block_size = ff_pcm_default_packet_size(st->codecpar);
        const int size = FFMIN(block_size, bst->stop_offset - pos);

        ret = av_get_packet(pb, pkt, size);
        pkt->pos = pos;
    }
    pkt->stream_index = st->index;
    if (ret == AVERROR_EOF) {
        bnm->current_stream++;
        goto redo;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    UBIBNMDemuxContext *bnm = s->priv_data;
    AVIOContext *pb = s->pb;
    BNMStream *bst;
    AVStream *st;
    int64_t pos;

    bnm->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[bnm->current_stream];
    bst = st->priv_data;

    pos = avio_tell(pb);
    if (pos < bst->start_offset) {
        avio_seek(pb, bst->start_offset, SEEK_SET);
        return 0;
    }

    return -1;
}

const FFInputFormat ff_ubibnm_demuxer = {
    .p.name         = "ubibnm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Ubisoft SBx BNM"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "bnm",
    .priv_data_size = sizeof(UBIBNMDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
