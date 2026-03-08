/*
 * UBI BAO demuxer
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

#define BAO_MIN_VERSION 0x1B
#define BAO_MAX_VERSION 0x2B

#define BAO_MAX_LAYER_COUNT 16
#define BAO_MAX_CHAIN_COUNT 128

#define BAO_MAX_TYPES  10
#define BAO_MAX_CODECS 0x10

typedef struct BAOStream {
    int64_t start_offset;
    int64_t stop_offset;

    AVFormatContext *ctx;

    AVIOContext *pb;
} BAOStream;

typedef struct ubi_bao_layer_t {
    int channels;
    int sample_rate;
    int stream_type;
    int num_samples;
    int64_t extradata_offset;
    uint32_t extradata_size;
} ubi_bao_layer_t;

typedef enum { ARCHIVE_NONE = 0, ARCHIVE_ATOMIC, ARCHIVE_PK, ARCHIVE_SPK } ubi_bao_archive_t;
typedef enum { TYPE_NONE = 0, TYPE_AUDIO, TYPE_SEQUENCE, TYPE_LAYER, TYPE_SILENCE, TYPE_IGNORED } ubi_bao_type_t;

typedef enum {
  CODEC_NONE = 0,
  RAW_PCM,
  UBI_IMA, UBI_IMA_seek, UBI_IMA_mark,
  RAW_PSX, RAW_PSX_new,
  RAW_AT3, FMT_AT3,
  RAW_XMA1_mem, RAW_XMA1_str, RAW_XMA2_old, RAW_XMA2_new,
  FMT_OGG,
  RAW_DSP,
  RAW_MP3,
  RAW_AT9,
} ubi_bao_codec_t;

typedef enum { PARSER_1B, PARSER_29 } ubi_bao_parser_t;

typedef struct ubi_bao_config_t {
    ubi_bao_parser_t parser;

    int big_endian;
    int allowed_types[BAO_MAX_TYPES];
    uint32_t version;
    int engine_version;

    ubi_bao_codec_t codec_map[BAO_MAX_CODECS];
    int v1_bao;

    int header_less_le_flag;

    int64_t bao_class;
    int64_t header_id;
    int64_t header_type;
    int64_t header_skip;
    int64_t header_base_size;

    int64_t audio_stream_size;
    int64_t audio_stream_id;
    int64_t audio_stream_flag;
    int64_t audio_loop_flag;
    int64_t audio_channels;
    int64_t audio_sample_rate;
    int64_t audio_num_samples;
    int64_t audio_num_samples2;
    int64_t audio_stream_type;
    int64_t audio_stream_subtype;
    int64_t audio_prefetch_size;
    int64_t audio_cue_count;
    int64_t audio_cue_labels;
    int audio_stream_and;
    int audio_loop_and;
    int audio_ignore_external_size;
    int audio_fix_xma_samples;
    int audio_fix_xma_memory_baos;

    int64_t sequence_sequence_loop;
    int64_t sequence_sequence_single;
    int64_t sequence_sequence_count;
    int64_t sequence_entry_number;
    int64_t sequence_entry_size;

    int64_t layer_layer_count;
    int64_t layer_stream_flag;
    int64_t layer_stream_id;
    int64_t layer_stream_size;
    int64_t layer_prefetch_size;
    int64_t layer_extra_size;
    int64_t layer_cue_labels;
    int64_t layer_cue_count;
    int64_t layer_entry_size;
    int64_t layer_sample_rate;
    int64_t layer_channels;
    int64_t layer_stream_type;
    int64_t layer_num_samples;
    int layer_stream_and;
    int layer_default_subtype;
    int layer_ignore_error;

    int64_t silence_duration_float;
} ubi_bao_config_t;

typedef struct ubi_bao_header_t {
    ubi_bao_archive_t archive;
    ubi_bao_type_t type;
    ubi_bao_codec_t codec;
    int total_streams;

    ubi_bao_config_t cfg;

    int64_t header_offset;      // base BAO offset
    uint32_t header_id;         // current BAO's id
    uint32_t header_type;       // type of audio
    uint32_t header_size;       // normal base size (not counting extra tables)
    uint32_t extra_size;        // extra tables size

    uint32_t stream_id;         // stream or memory BAO's id
    uint32_t stream_size;       // stream or memory BAO's playable data
    uint32_t prefetch_size;

    int64_t memory_skip;       // must skip a bit after base memory BAO offset (class 0x30000000), calculated
    int64_t stream_skip;       // same for stream BAO offset (class 0x50000000)

    int is_stream;             // stream data (external file) is used for audio, memory data (external or internal) otherwise
    int is_prefetch;           // memory data is to be joined as part of the stream
    int is_inline;             // memory data is in the header BAO rather than a separate memory BAO

    int loop_flag;
    int num_samples;
    int loop_start;
    int sample_rate;
    int channels;
    int stream_type;
    int stream_subtype;

    int layer_count;
    ubi_bao_layer_t layer[BAO_MAX_LAYER_COUNT];
    int sequence_count;
    uint32_t sequence_chain[BAO_MAX_CHAIN_COUNT];
    int sequence_loop;
    int sequence_single;

    float silence_duration;

    int64_t inline_offset;
    uint32_t inline_size;

    int64_t extradata_offset;
    uint32_t extradata_size;
} ubi_bao_header_t;

typedef struct UBIBAODemuxContext {
    int current_stream;
    ubi_bao_header_t bao;
} UBIBAODemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (p->buf[0] != 0x01 && p->buf[0] != 0x02)
        return 0;
    if (p->buf[1] < BAO_MIN_VERSION || p->buf[1] > BAO_MAX_VERSION)
        return 0;
    if (!av_match_ext(p->filename, "bao"))
        return 0;

    return AVPROBE_SCORE_MAX*5/11;
}

static int guess_endian32(AVFormatContext *s, int64_t offset)
{
    AVIOContext *pb = s->pb;
    uint8_t buf[4];

    avio_seek(pb, offset, SEEK_SET);
    if (avio_read(pb, buf, 4) != 4)
        return -1;

    return AV_RL32(buf) > AV_RB32(buf) ? 1 : 0;
}

static void config_bao_entry(ubi_bao_config_t *cfg, int64_t header_base_size, int64_t header_skip)
{
    cfg->header_base_size = header_base_size;
    cfg->header_skip = header_skip;
}

static void config_bao_audio_b(ubi_bao_config_t *cfg, int64_t stream_size, int64_t stream_id, int64_t stream_flag, int64_t loop_flag, int stream_and, int loop_and)
{
    cfg->audio_stream_size = stream_size;
    cfg->audio_stream_id = stream_id;
    cfg->audio_stream_flag = stream_flag;
    cfg->audio_loop_flag = loop_flag;
    cfg->audio_stream_and = stream_and;
    cfg->audio_loop_and = loop_and;
}

static void config_bao_audio_m(ubi_bao_config_t *cfg, int64_t channels, int64_t sample_rate, int64_t num_samples, int64_t num_samples2, int64_t stream_type, int64_t prefetch_size)
{
    cfg->audio_channels = channels;
    cfg->audio_sample_rate = sample_rate;
    cfg->audio_num_samples = num_samples;
    cfg->audio_num_samples2 = num_samples2;
    cfg->audio_stream_type = stream_type;
    cfg->audio_prefetch_size = prefetch_size;
}

static void config_bao_audio_c(ubi_bao_config_t *cfg, int64_t cue_count, int64_t cue_labels, int64_t cue_size)
{
    cfg->audio_cue_count = cue_count;
    cfg->audio_cue_labels = cue_labels;
}

static void config_bao_sequence(ubi_bao_config_t *cfg, int64_t sequence_count, int64_t sequence_single, int64_t sequence_loop, int64_t entry_size)
{
    cfg->sequence_sequence_count = sequence_count;
    cfg->sequence_sequence_single = sequence_single;
    cfg->sequence_sequence_loop = sequence_loop;
    cfg->sequence_entry_size = entry_size;
    cfg->sequence_entry_number = 0x00;
}

static void config_bao_layer_m(ubi_bao_config_t *cfg, int64_t stream_id, int64_t layer_count, int64_t stream_flag, int64_t stream_size,
                               int64_t extra_size, int64_t prefetch_size, int64_t cue_count, int64_t cue_labels, int stream_and)
{
    cfg->layer_stream_id = stream_id;
    cfg->layer_layer_count = layer_count;
    cfg->layer_stream_flag = stream_flag;
    cfg->layer_stream_size = stream_size;
    cfg->layer_extra_size = extra_size;
    cfg->layer_prefetch_size = prefetch_size;
    cfg->layer_cue_count = cue_count;
    cfg->layer_cue_labels = cue_labels;
    cfg->layer_stream_and = stream_and;
}

static void config_bao_layer_e(ubi_bao_config_t *cfg, int64_t entry_size, int64_t sample_rate, int64_t channels, int64_t stream_type, int64_t num_samples)
{
    cfg->layer_entry_size = entry_size;
    cfg->layer_sample_rate = sample_rate;
    cfg->layer_channels = channels;
    cfg->layer_stream_type = stream_type;
    cfg->layer_num_samples = num_samples;
}

static void config_bao_silence_f(ubi_bao_config_t *cfg, int64_t duration)
{
    cfg->silence_duration_float = duration;
}

static int ubi_bao_config_version(AVFormatContext *s, ubi_bao_config_t *cfg, uint32_t version)
{
    AVIOContext *pb = s->pb;

    cfg->version = version;

    cfg->allowed_types[0x01] = 1; //sfx
    cfg->allowed_types[0x05] = 1; //sequence
    cfg->allowed_types[0x06] = 1; //layers

    cfg->bao_class      = 0x20; // absolute offset, unlike the rest
    cfg->header_id      = 0x00; // relative
    cfg->header_type    = 0x04; // relative
    cfg->parser = PARSER_1B;

    if ((cfg->version & 0xFFFF0000) >= 0x00290000) {
        cfg->bao_class   = 0x14; // absolute
        cfg->header_id   = 0x04; // relative
        cfg->header_type = 0x2c; // relative
        cfg->header_skip = 0x1c;

        cfg->engine_version = (version >> 8) & 0xFFFF00;
        cfg->parser = PARSER_29;
    }

    if (cfg->version == 0x00220015) {
        int64_t header_size = 0x40 + avio_rl32(pb);

        if (guess_endian32(s, header_size + 0x04))
            version |= 0xF000;
    }

    switch (version) {
    case 0x001B0100:
    case 0x001B0200:
    case 0x001C0000:
        if (cfg->version == 0x001B0100)
            config_bao_entry(cfg, 0xA4, 0x28);
        else
            config_bao_entry(cfg, 0xA0, 0x24);

        config_bao_audio_b(cfg, 0x08, 0x1c, 0x28, 0x34, 1, 1);
        config_bao_audio_m(cfg, 0x44, 0x48, 0x50, 0x58, 0x64, 0x74);

        config_bao_sequence(cfg, 0x2c, 0x20, 0x1c, 0x14);

        config_bao_layer_m(cfg, 0x4c, 0x20, 0x2c, 0x44, 0x00, 0x50, 0x58, 0x5c, 1);
        config_bao_layer_e(cfg, 0x30, 0x00, 0x04, 0x08, 0x10);

        config_bao_silence_f(cfg, 0x1c);

        config_bao_audio_c(cfg, 0x68, 0x6c, 0x78);

        cfg->codec_map[0x00] = RAW_XMA1_mem;
        cfg->codec_map[0x01] = RAW_PCM;
        cfg->codec_map[0x02] = RAW_PSX;
        cfg->codec_map[0x03] = UBI_IMA;
        cfg->codec_map[0x04] = FMT_OGG;
        cfg->codec_map[0x05] = RAW_XMA1_str;
        cfg->codec_map[0x07] = RAW_AT3;

        cfg->audio_stream_subtype = 0x78;

        cfg->v1_bao = 1;
        break;
    case 0x001D0A00:
        config_bao_entry(cfg, 0x84, 0x24);

        config_bao_audio_b(cfg, 0x08, 0x1c, 0x20, 0x20, (1 << 2), (1 << 5));
        config_bao_audio_m(cfg, 0x28, 0x30, 0x38, 0x40, 0x4c, 0x5c);

        config_bao_layer_m(cfg, 0x3c, 0x20, 0x24, 0x34, 0x3c, 0x40, 0x00, 0x00, (1 << 2));
        config_bao_layer_e(cfg, 0x38, 0x00, 0x04, 0x08, 0x10);

        config_bao_silence_f(cfg, 0x1c);

        cfg->codec_map[0x06] = RAW_PSX_new;
        cfg->codec_map[0x07] = FMT_AT3;

        cfg->v1_bao = 1;
        break;
    case 0x001F0008:
    case 0x001F0010:
    case 0x001F0011:
    case 0x0021000C:
    case 0x0022000D:
    case 0x0022F015:
    case 0x00220017:
    case 0x00220018:
    case 0x0022001B:
        config_bao_entry(cfg, 0xA4, 0x28);

        config_bao_audio_b(cfg, 0x08, 0x1c, 0x28, 0x34, 1, 1);
        config_bao_audio_m(cfg, 0x44, 0x4c, 0x54, 0x5c, 0x64, 0x74);

        config_bao_sequence(cfg, 0x2c, 0x20, 0x1c, 0x14);

        config_bao_layer_m(cfg, 0x00, 0x20, 0x2c, 0x44, 0x4c, 0x50, 0x54, 0x58, 1);
        config_bao_layer_e(cfg, 0x28, 0x00, 0x04, 0x08, 0x10);

        config_bao_silence_f(cfg, 0x1c);

        if (cfg->version == 0x0022000D)
            config_bao_audio_c(cfg, 0x68, 0x6c, 0x78);

        cfg->codec_map[0x01] = RAW_PCM;
        cfg->codec_map[0x03] = UBI_IMA;
        cfg->codec_map[0x04] = FMT_OGG;
        cfg->codec_map[0x05] = RAW_XMA1_str;
        cfg->codec_map[0x07] = RAW_AT3;
        cfg->codec_map[0x09] = RAW_DSP;

        if (cfg->version >= 0x00220000)
            cfg->codec_map[0x05] = RAW_XMA2_old;

        if (cfg->version == 0x0022000D)
            cfg->audio_ignore_external_size = 1;

        cfg->audio_stream_subtype = 0x78;
        break;
    case 0x00220015:
    case 0x0022001E:
        config_bao_entry(cfg, 0x84, 0x28);

        config_bao_audio_b(cfg, 0x08, 0x1c, 0x20, 0x20, (1 << 2), (1 << 5)); // (1 << 4): prefetch flag?
        config_bao_audio_m(cfg, 0x28, 0x30, 0x38, 0x40, 0x48, 0x58);

        config_bao_layer_m(cfg, 0x00, 0x20, 0x24, 0x34, 0x3c, 0x40, 0x00, 0x00, (1 << 2)); // 0x1c: bao group id?
        config_bao_layer_e(cfg, 0x28, 0x00, 0x04, 0x08, 0x10);

        cfg->codec_map[0x06] = RAW_PSX_new;
        cfg->codec_map[0x07] = FMT_AT3;
        break;
    case 0x00230008:
        config_bao_entry(cfg, 0xB4, 0x28);

        config_bao_audio_b(cfg, 0x08, 0x24, 0x38, 0x44, 1, 1);
        config_bao_audio_m(cfg, 0x54, 0x5c, 0x64, 0x6c, 0x74, 0x84);

        config_bao_sequence(cfg, 0x34, 0x28, 0x24, 0x14);

        config_bao_layer_m(cfg, 0x00, 0x28, 0x3c, 0x54, 0x5c, 0x00 /*0x60?*/, 0x00, 0x00, 1); // 0x24: id-like
        config_bao_layer_e(cfg, 0x30, 0x00, 0x04, 0x08, 0x18);

        cfg->codec_map[0x01] = RAW_PCM;
        cfg->codec_map[0x02] = UBI_IMA;
        cfg->codec_map[0x03] = FMT_OGG;
        cfg->codec_map[0x04] = RAW_XMA2_old;

        cfg->layer_ignore_error = 1;
        break;
    case 0x00250108:
    case 0x0025010A:
    case 0x00250119:
    case 0x0025011D:
        config_bao_entry(cfg, 0xB4, 0x28);

        config_bao_audio_b(cfg, 0x08, 0x24, 0x2c, 0x38, 1, 1);
        config_bao_audio_m(cfg, 0x48, 0x50, 0x58, 0x60, 0x68, 0x78);

        config_bao_sequence(cfg, 0x34, 0x28, 0x24, 0x14);

        config_bao_layer_m(cfg, 0x00, 0x28, 0x30, 0x48, 0x50, 0x54, 0x58, 0x5c, 1);
        config_bao_layer_e(cfg, 0x30, 0x00, 0x04, 0x08, 0x18);

        config_bao_silence_f(cfg, 0x24);

        cfg->codec_map[0x01] = RAW_PCM;
        cfg->codec_map[0x02] = UBI_IMA;
        cfg->codec_map[0x03] = FMT_OGG;
        cfg->codec_map[0x04] = RAW_XMA2_new;
        cfg->codec_map[0x05] = RAW_PSX;
        cfg->codec_map[0x06] = RAW_AT3;

        if (cfg->version == 0x0025011D)
            cfg->header_less_le_flag = 1;

        cfg->audio_stream_subtype = 0x88;

        cfg->layer_default_subtype = 2;
        if (cfg->version == 0x0025010A)
            cfg->layer_default_subtype = 1;
        break;
    case 0x00260000:
    case 0x00260102:
        config_bao_entry(cfg, 0xB8, 0x28);

        config_bao_audio_b(cfg, 0x08, 0x28, 0x30, 0x3c, 1, 1);
        config_bao_audio_m(cfg, 0x4c, 0x54, 0x5c, 0x64, 0x6c, 0x7c);

        config_bao_sequence(cfg, 0x38, 0x2c, 0x28, 0x14);

        config_bao_layer_m(cfg, 0x00, 0x2c, 0x34,  0x4c, 0x54, 0x58,  0x00, 0x00, 1);
        config_bao_layer_e(cfg, 0x34, 0x00, 0x04, 0x08, 0x1c);

        cfg->codec_map[0x02] = UBI_IMA;
        cfg->codec_map[0x03] = FMT_OGG;
        cfg->codec_map[0x04] = RAW_XMA2_new;
        cfg->codec_map[0x06] = RAW_AT3;

        cfg->audio_ignore_external_size = 1;

        cfg->audio_stream_subtype = 0x8c;
        break;
    case 0x00270102:
    case 0x00280102:
        config_bao_entry(cfg, 0xAC, 0x28);

        config_bao_audio_b(cfg, 0x08, 0x28, 0x2c, 0x38, 1, 1);
        config_bao_audio_m(cfg, 0x44, 0x4c, 0x54, 0x5c, 0x64, 0x70);

        config_bao_sequence(cfg, 0x38, 0x2c, 0x28, 0x14);

        config_bao_layer_m(cfg, 0x00, 0x2c, 0x34, 0x48, 0x50, 0x54, 0x58, 0x5c, 1);
        config_bao_layer_e(cfg, 0x2c, 0x00, 0x04, 0x08, 0x1c);

        config_bao_audio_c(cfg, 0x68, 0x6c, 0x80);

        cfg->codec_map[0x02] = UBI_IMA;
        cfg->codec_map[0x04] = FMT_OGG;
        cfg->codec_map[0x05] = RAW_XMA2_new;

        cfg->audio_fix_xma_memory_baos = 1;
        cfg->layer_ignore_error = 1;
        break;
    case 0x00280303:
    case 0x00280305:
    case 0x00280306:
        config_bao_entry(cfg, 0xBC, 0x28);

        config_bao_audio_b(cfg, 0x08, 0x38, 0x3c, 0x48, 1, 1);
        config_bao_audio_m(cfg, 0x54, 0x5c, 0x64, 0x6c, 0x74, 0x80);

        config_bao_sequence(cfg, 0x48, 0x3c, 0x38, 0x14);

        config_bao_layer_m(cfg, 0x00, 0x3c, 0x44, 0x58, 0x60, 0x64, 0x68, 0x6c, 1);
        config_bao_layer_e(cfg, 0x2c, 0x00, 0x04, 0x08, 0x1c);

        config_bao_silence_f(cfg, 0x38);

        config_bao_audio_c(cfg, 0x78, 0x7c, 0x00);

        cfg->codec_map[0x01] = RAW_PCM;
        cfg->codec_map[0x02] = UBI_IMA;
        cfg->codec_map[0x03] = UBI_IMA_seek;
        cfg->codec_map[0x04] = FMT_OGG;
        cfg->codec_map[0x05] = RAW_XMA2_new;
        cfg->codec_map[0x06] = RAW_PSX_new;
        cfg->codec_map[0x07] = RAW_AT3;

        cfg->audio_fix_xma_samples = 1;
        cfg->layer_ignore_error = 1;
        cfg->audio_stream_subtype = 0x90;
        break;
    case 0x00290106:
    case 0x002A0300:
        cfg->codec_map[0x00] = CODEC_NONE;
        cfg->codec_map[0x01] = RAW_PCM;
        cfg->codec_map[0x02] = UBI_IMA;
        cfg->codec_map[0x03] = UBI_IMA_seek;
        cfg->codec_map[0x04] = FMT_OGG;
        cfg->codec_map[0x05] = RAW_XMA2_new;
        cfg->codec_map[0x07] = RAW_MP3;
        cfg->codec_map[0x09] = RAW_AT9;
        break;
    case 0x002B0000:
    case 0x002B0100:
        cfg->codec_map[0x00] = CODEC_NONE;
        cfg->codec_map[0x01] = RAW_PCM;
        cfg->codec_map[0x02] = UBI_IMA;
        cfg->codec_map[0x03] = UBI_IMA_seek;
        cfg->codec_map[0x04] = UBI_IMA_mark;
        cfg->codec_map[0x05] = FMT_OGG;
        cfg->codec_map[0x06] = RAW_XMA2_new;
        cfg->codec_map[0x08] = RAW_MP3;
        cfg->codec_map[0x0A] = RAW_AT9;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    return 0;
}

typedef uint32_t (*avio_r32)(AVIOContext *s);

static void ubi_bao_config_endian(AVFormatContext *s, ubi_bao_config_t *cfg, int64_t offset)
{
    cfg->big_endian = !guess_endian32(s, offset + cfg->bao_class);
}

static const ubi_bao_type_t type_map[BAO_MAX_TYPES] = {
    TYPE_NONE,
    TYPE_AUDIO,
    TYPE_IGNORED,
    TYPE_IGNORED,
    TYPE_IGNORED,
    TYPE_SEQUENCE,
    TYPE_LAYER,
    TYPE_IGNORED,
    TYPE_SILENCE,
    TYPE_IGNORED,
};

static int parse_type_audio_cfg(AVFormatContext *s, ubi_bao_header_t *bao, int64_t offset)
{
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32 = bao->cfg.big_endian ? avio_rb32 : avio_rl32;

    int64_t h_offset = offset + bao->cfg.header_skip;
    avio_seek(pb, h_offset + bao->cfg.audio_stream_size, SEEK_SET);
    bao->stream_size = avio_r32(pb);
    avio_seek(pb, h_offset + bao->cfg.audio_stream_id, SEEK_SET);
    bao->stream_id = avio_r32(pb);
    avio_seek(pb, h_offset + bao->cfg.audio_stream_flag, SEEK_SET);
    bao->is_stream = avio_r32(pb) & bao->cfg.audio_stream_and;
    avio_seek(pb, h_offset + bao->cfg.audio_loop_flag, SEEK_SET);
    bao->loop_flag = avio_r32(pb) & bao->cfg.audio_loop_and;
    avio_seek(pb, h_offset + bao->cfg.audio_channels, SEEK_SET);
    bao->channels = avio_r32(pb);
    avio_seek(pb, h_offset + bao->cfg.audio_sample_rate, SEEK_SET);
    bao->sample_rate = avio_r32(pb);

    uint32_t cues_size = 0;
    if (bao->cfg.audio_cue_count) {
        avio_seek(pb, h_offset + bao->cfg.audio_cue_count, SEEK_SET);
        cues_size += avio_r32(pb) * 0x08;
    }
    if (bao->cfg.audio_cue_labels) {
        avio_seek(pb, h_offset + bao->cfg.audio_cue_labels, SEEK_SET);
        cues_size += avio_r32(pb);
    }
    bao->extra_size = cues_size;

    if (bao->cfg.audio_prefetch_size) {
        avio_seek(pb, h_offset + bao->cfg.audio_prefetch_size, SEEK_SET);
        bao->prefetch_size = avio_r32(pb);
        bao->is_prefetch = (bao->prefetch_size > 0);
    }

    if (bao->loop_flag) {
        avio_seek(pb, h_offset + bao->cfg.audio_num_samples, SEEK_SET);
        bao->loop_start  = avio_r32(pb);
        avio_seek(pb, h_offset + bao->cfg.audio_num_samples2, SEEK_SET);
        bao->num_samples = avio_r32(pb) + bao->loop_start;
    } else {
        avio_seek(pb, h_offset + bao->cfg.audio_num_samples, SEEK_SET);
        bao->num_samples = avio_r32(pb);
    }

    avio_seek(pb, h_offset + bao->cfg.audio_stream_type, SEEK_SET);
    bao->stream_type = avio_r32(pb);
    if (bao->cfg.audio_stream_subtype) {
        avio_seek(pb, h_offset + bao->cfg.audio_stream_subtype, SEEK_SET);
        bao->stream_subtype = avio_r32(pb);
    }

    return 0;
}

static int parse_type_sequence_cfg(AVFormatContext *s, ubi_bao_header_t *bao, int64_t offset)
{
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32 = bao->cfg.big_endian ? avio_rb32 : avio_rl32;

    if (bao->cfg.sequence_entry_size == 0) {
        av_log(s, AV_LOG_ERROR, "sequence entry size not configured at %lx\n", offset);
        return AVERROR_INVALIDDATA;
    }

    int64_t h_offset = offset + bao->cfg.header_skip;
    avio_seek(pb, h_offset + bao->cfg.sequence_sequence_loop, SEEK_SET);
    bao->sequence_loop   = avio_r32(pb);
    avio_seek(pb, h_offset + bao->cfg.sequence_sequence_single, SEEK_SET);
    bao->sequence_single = avio_r32(pb);
    avio_seek(pb, h_offset + bao->cfg.sequence_sequence_count, SEEK_SET);
    bao->sequence_count  = avio_r32(pb);
    if (bao->sequence_count > BAO_MAX_CHAIN_COUNT) {
        av_log(s, AV_LOG_ERROR, "incorrect sequence count of %i\n", bao->sequence_count);
        return AVERROR_INVALIDDATA;
    }

    int64_t table_offset = offset + bao->header_size;
    for (int i = 0; i < bao->sequence_count; i++) {
        avio_seek(pb, table_offset + bao->cfg.sequence_entry_number, SEEK_SET);
        uint32_t entry_id = avio_r32(pb);

        bao->sequence_chain[i] = entry_id;

        table_offset += bao->cfg.sequence_entry_size;
    }

    return 0;
}

static int parse_type_layer_cfg(AVFormatContext *s, ubi_bao_header_t *bao, int64_t offset)
{
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32 = bao->cfg.big_endian ? avio_rb32 : avio_rl32;

    if (bao->cfg.layer_entry_size == 0) {
        av_log(s, AV_LOG_ERROR, "layer entry size not configured at %lx\n", offset);
        return AVERROR_INVALIDDATA;
    }

    int64_t h_offset = offset + bao->cfg.header_skip;
    avio_seek(pb, h_offset + bao->cfg.layer_layer_count, SEEK_SET);
    bao->layer_count = avio_r32(pb);
    avio_seek(pb, h_offset + bao->cfg.layer_stream_flag, SEEK_SET);
    bao->is_stream = avio_r32(pb) & bao->cfg.layer_stream_and;
    avio_seek(pb, h_offset + bao->cfg.layer_stream_size, SEEK_SET);
    bao->stream_size = avio_r32(pb);
    avio_seek(pb, h_offset + bao->cfg.layer_stream_id, SEEK_SET);
    bao->stream_id = avio_r32(pb);
    if (bao->layer_count > BAO_MAX_LAYER_COUNT) {
        av_log(s, AV_LOG_ERROR, "incorrect layer count of %i\n", bao->layer_count);
        return AVERROR_INVALIDDATA;
    }

    if (bao->cfg.layer_prefetch_size) {
        avio_seek(pb, h_offset + bao->cfg.layer_prefetch_size, SEEK_SET);
        bao->prefetch_size = avio_r32(pb);
        bao->is_prefetch = (bao->prefetch_size > 0);
    }

    uint32_t cues_size = 0;
    if (bao->cfg.layer_cue_labels) {
        avio_seek(pb, h_offset + bao->cfg.layer_cue_labels, SEEK_SET);
        cues_size += avio_r32(pb);
    }
    if (bao->cfg.layer_cue_count) {
        avio_seek(pb, h_offset + bao->cfg.layer_cue_count, SEEK_SET);
        cues_size += avio_r32(pb) * 0x08;
    }

    if (bao->cfg.layer_extra_size) {
        avio_seek(pb, h_offset + bao->cfg.layer_extra_size, SEEK_SET);
        bao->extra_size = avio_r32(pb);
    } else {
        bao->extra_size = cues_size + bao->layer_count * bao->cfg.layer_entry_size + cues_size;
    }

    int64_t table_offset = offset + bao->header_size + cues_size;
    for (int i = 0; i < bao->layer_count; i++) {
        ubi_bao_layer_t *layer = &bao->layer[i];
        avio_seek(pb, table_offset + bao->cfg.layer_channels, SEEK_SET);
        layer->channels = avio_r32(pb);
        avio_seek(pb, table_offset + bao->cfg.layer_sample_rate, SEEK_SET);
        layer->sample_rate = avio_r32(pb);
        avio_seek(pb, table_offset + bao->cfg.layer_stream_type, SEEK_SET);
        layer->stream_type = avio_r32(pb);
        avio_seek(pb, table_offset + bao->cfg.layer_num_samples, SEEK_SET);
        layer->num_samples = avio_r32(pb);

        table_offset += bao->cfg.layer_entry_size;
    }

    return 0;
}

static int parse_type_silence_cfg(AVFormatContext *s, ubi_bao_header_t *bao, int64_t offset)
{
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32 = bao->cfg.big_endian ? avio_rb32 : avio_rl32;

    if (bao->cfg.silence_duration_float == 0) {
        av_log(s, AV_LOG_ERROR, "silence duration not configured at %lx\n", offset);
        return AVERROR_INVALIDDATA;
    }

    int64_t h_offset = offset + bao->cfg.header_skip;
    avio_seek(pb, h_offset + bao->cfg.silence_duration_float, SEEK_SET);
    bao->silence_duration = av_int2float(avio_r32(pb));
    if (bao->silence_duration <= 0.0f) {
        av_log(s, AV_LOG_ERROR, "bad duration %f at %lx\n", bao->silence_duration, offset);
        return AVERROR_INVALIDDATA;
    }

    return 0;
}

static int parse_header_cfg(AVFormatContext *s, ubi_bao_header_t *bao, int64_t offset)
{
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32 = bao->cfg.big_endian ? avio_rb32 : avio_rl32;

    int64_t h_offset = offset + bao->cfg.header_skip;
    avio_seek(pb, h_offset + bao->cfg.header_id, SEEK_SET);
    bao->header_id = avio_r32(pb);
    avio_seek(pb, h_offset + bao->cfg.header_type, SEEK_SET);
    bao->header_type = avio_r32(pb);

    bao->header_size    = bao->cfg.header_base_size;

    if (bao->cfg.header_less_le_flag && !bao->cfg.big_endian) {
        bao->header_size -= 0x04;
    } else if (avio_size(pb) > offset + bao->header_size) {
        avio_seek(pb, offset + bao->header_size, SEEK_SET);
        int32_t end_field = avio_r32(pb);

        if (end_field == -1 || end_field == 0 || end_field == 1)
            bao->header_size += 0x04;
    }

    if (bao->header_type < BAO_MAX_TYPES)
        bao->type = type_map[bao->header_type];

    switch (bao->type) {
    case TYPE_AUDIO: return parse_type_audio_cfg(s, bao, offset); break;
    case TYPE_SEQUENCE: return parse_type_sequence_cfg(s, bao, offset); break;
    case TYPE_LAYER: return parse_type_layer_cfg(s, bao, offset); break;
    case TYPE_SILENCE: return parse_type_silence_cfg(s, bao, offset); break;
    default:
        return AVERROR_INVALIDDATA;
    }

    return AVERROR_INVALIDDATA;
}

static int parse_base_v29(AVFormatContext *s, ubi_bao_header_t *bao)
{
    AVIOContext *pb = s->pb;

    avio_skip(pb, 4); // version
    avio_skip(pb, 0x10); // 128-bit hash
    avio_skip(pb, 4); // bao class
    avio_skip(pb, 4); // fixed 2?

    return 0;
}

static int parse_common_v29(AVFormatContext *s, ubi_bao_header_t *bao)
{
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32 = bao->cfg.big_endian ? avio_rb32 : avio_rl32;

    avio_skip(pb, 4); // fixed? 0x17CE46D9
    bao->header_id = avio_r32(pb);
    avio_skip(pb, 4); // -1
    avio_skip(pb, 4); // value (0xC0xx0000)

    avio_skip(pb, 4); // null
    avio_skip(pb, 4); // null
    avio_skip(pb, 4); // flag 1 (v002b) 0 (v002a)
    avio_skip(pb, 4); // null / -1

    avio_skip(pb, 4); // null
    avio_skip(pb, 4); // null
    avio_skip(pb, 4); // flag 1 / 0 (v0029?)
    bao->header_type = avio_r32(pb); // bao type

    return 0;
}

static int parse_parameters_v2b(AVFormatContext *s, ubi_bao_header_t *bao)
{
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32 = bao->cfg.big_endian ? avio_rb32 : avio_rl32;

    if (bao->cfg.engine_version < 0x2B00)
        return 0;

    int count = avio_r32(pb);
    if (count != 0 && count != 2) {
        av_log(s, AV_LOG_ERROR, "unexpected parameter count %i\n", count);
        return AVERROR_INVALIDDATA;
    }

    for (int i = 0; i < count; i++) {
        uint32_t label_size = avio_r32(pb);
        avio_skip(pb, label_size); // TempoBPM, TempoTimeSig
    }

    int count2 = avio_r32(pb);
    if (count != count2) {
        av_log(s, AV_LOG_ERROR, "unexpected parameter count2 %i\n", count2);
        return AVERROR_INVALIDDATA;
    }

    avio_skip(pb, 4);

    uint32_t points_size = avio_r32(pb);
    if (points_size)
        avio_skip(pb, points_size); // 0x14 x2 (LE fields?)

    return 0;
}

static int parse_type_audio_v29(AVFormatContext *s, ubi_bao_header_t *bao)
{
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32 = bao->cfg.big_endian ? avio_rb32 : avio_rl32;

    avio_skip(pb, 4); // fixed? 0x93B1ECE5
    avio_skip(pb, 4); // flag 1/2
    avio_skip(pb, 4); // bao id?

    parse_parameters_v2b(s, bao);

    bao->loop_flag = avio_r32(pb);

    avio_skip(pb, 4); // flag 1 / 0 (v0029?)
    avio_skip(pb, 4); // null
    avio_skip(pb, 4); // full stream size (ex. header xma chunk + stream size), null if loop_flag not set
    avio_skip(pb, 4); // null

    avio_skip(pb, 4); // null / original rate? (rare)
    avio_skip(pb, 4); // null
    bao->stream_type = avio_r32(pb);
    bao->channels = avio_r32(pb);

    bao->sample_rate = avio_r32(pb);
    avio_skip(pb, 4); // bitrate
    bao->stream_size = avio_r32(pb);
    avio_skip(pb, 4); // null

    avio_skip(pb, 4); // null
    bao->is_stream = avio_r32(pb);

    if (bao->is_stream) {
        bao->is_prefetch = avio_r32(pb);
        bao->prefetch_size = avio_r32(pb);

        avio_skip(pb, 4); // stream offset (always 0x1C = header_skip)
        bao->stream_id = avio_r32(pb);
        uint32_t repeat_id = avio_r32(pb); // prefetch id?
        avio_skip(pb, 4); // -1

        if (bao->stream_id != repeat_id) {
            av_log(s, AV_LOG_ERROR, "audio stream id mismatch %08x != %08x\n", bao->stream_id, repeat_id);
            return AVERROR_INVALIDDATA;
        }
    } else {
        bao->stream_id = avio_r32(pb); // memory id
    }

    int strings_size = avio_r32(pb);
    if (strings_size) {
        strings_size = FFALIGN(strings_size, 0x04);
        avio_skip(pb, strings_size); // strings for cues
    }

    int cues_count = avio_r32(pb);
    if (cues_count) {
        int cues_size = cues_count * 0x04;
        avio_skip(pb, cues_size); // cues with float points
    }

    avio_skip(pb, 4); // -1
    avio_skip(pb, 4); // bao id?
    avio_skip(pb, 4); // low value, atrac9 related? extradata version? (16, 17)
    avio_skip(pb, 4); // flag 1

    bao->extradata_size = avio_r32(pb);
    if (bao->extradata_size) {
        bao->extradata_offset = avio_tell(pb);
        avio_skip(pb, bao->extradata_size); // mp3 info, atrac9 config, xma header in memory BAOs, etc
    }

    int samples1 = avio_r32(pb);
    avio_skip(pb, 4); // samples1 data size

    int samples2 = avio_r32(pb);
    avio_skip(pb, 4); // samples2 data size (0 if not set, sometimes negative in xma?)

    if (bao->cfg.engine_version >= 0x2B00)
        avio_skip(pb, 4); // null

    if (bao->loop_flag) {
        bao->loop_start  = samples1;
        bao->num_samples = samples1 + samples2;
    } else {
        bao->num_samples = samples1;
    }

    return 0;
}

static int parse_type_sequence_v29(AVFormatContext *s, ubi_bao_header_t *bao)
{
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32 = bao->cfg.big_endian ? avio_rb32 : avio_rl32;

    avio_skip(pb, 4); // fixed? 0x3C2E0733
    avio_skip(pb, 4); // null

    parse_parameters_v2b(s, bao);

    avio_skip(pb, 4); // flag 0/1 (loop related) //TODO: sequence single?
    avio_skip(pb, 4); // null

    avio_skip(pb, 4); // null
    avio_skip(pb, 4); // null
    avio_skip(pb, 4); // flag 1
    avio_skip(pb, 4); // null

    avio_skip(pb, 4); // segments?
    avio_skip(pb, 4); // null
    avio_skip(pb, 4); // null
    avio_skip(pb, 4); // null

    avio_skip(pb, 4); // duration f32
    avio_skip(pb, 4); // null
    avio_skip(pb, 4); // null
    avio_skip(pb, 4); // null

    avio_skip(pb, 4); // null
    bao->sequence_count = avio_r32(pb);

    if (bao->sequence_count > BAO_MAX_CHAIN_COUNT) {
        av_log(s, AV_LOG_ERROR, "incorrect sequence count of %i\n", bao->sequence_count);
        return AVERROR_INVALIDDATA;
    }

    for (int i = 0; i < bao->sequence_count; i++) {
        bao->sequence_chain[i] = avio_r32(pb);
        avio_skip(pb, 4); // flag 0/1 (loop related?)
        avio_skip(pb, 4); // flag 0/1 (loop related?)
        avio_skip(pb, 4); // low value
        avio_skip(pb, 4); // chain duration f32
    }

    if (bao->cfg.engine_version >= 0x2B00)
        avio_skip(pb, 4);

    return 0;
}

static int parse_type_layer_v29(AVFormatContext *s, ubi_bao_header_t *bao)
{
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32 = bao->cfg.big_endian ? avio_rb32 : avio_rl32;
    int ret;

    avio_skip(pb, 4); // fixed? 0xF2D3BBD4
    int layer_ids = avio_r32(pb);
    for (int i = 0; i < layer_ids; i++) {
        avio_skip(pb, 4); // bao id (original/internal?)
    }

    ret = parse_parameters_v2b(s, bao);
    if (ret < 0)
        return ret;

    bao->loop_flag = avio_r32(pb); // full loops
    avio_skip(pb, 4); // null
    avio_skip(pb, 4); // layers?
    avio_skip(pb, 4); // null

    avio_skip(pb, 4); // bao id? (shared in multiple BAOs)
    avio_skip(pb, 4); // bao id? low value?
    bao->stream_size = avio_r32(pb); // full size (ex. prefetch + stream)
    avio_skip(pb, 4); // null

    avio_skip(pb, 4); // flag 1 / 0 (less common)
    bao->is_stream = avio_r32(pb);

    if (bao->is_stream) {
        bao->is_prefetch = avio_r32(pb);
        bao->prefetch_size = avio_r32(pb);

        avio_skip(pb, 4); // stream offset (always 0x1C = header_skip)
        avio_skip(pb, 4); // -1
        avio_skip(pb, 4); // -1 or rarely bao id?
        avio_skip(pb, 4); // -1
    } else {
        avio_skip(pb, 4); // -1
    }

    int strings_size = avio_r32(pb);
    if (strings_size) {
        strings_size = FFALIGN(strings_size, 0x04);
        avio_skip(pb, strings_size); // strings for cues
    }

    int cues_count = avio_r32(pb);
    if (cues_count) {
        int cues_size = cues_count * 0x04;
        avio_skip(pb, cues_size); // cues with float points
    }

    avio_skip(pb, 4); // -1
    avio_skip(pb, 4); // bao id? low value? (shared in multiple BAOs)
    if (bao->cfg.engine_version >= 0x2B00) {
        avio_skip(pb, 4); // null
    }
    bao->layer_count = avio_r32(pb);

    if (bao->layer_count > BAO_MAX_LAYER_COUNT) {
        av_log(s, AV_LOG_ERROR, "incorrect layer count of %i\n", bao->layer_count);
        return AVERROR_INVALIDDATA;
    }

    for (int i = 0; i < bao->layer_count; i++) {
        ubi_bao_layer_t* layer = &bao->layer[i];

        layer->sample_rate = avio_r32(pb);
        layer->channels    = avio_r32(pb);
        layer->stream_type = avio_r32(pb);
        avio_skip(pb, 4); // -1

        avio_skip(pb, 4); // null
        avio_skip(pb, 4); // null
        avio_skip(pb, 4); // value / null
        layer->num_samples = avio_r32(pb);

        avio_skip(pb, 4); // related to layer size?
        avio_skip(pb, 4); // null or some config (64000)
        avio_skip(pb, 4); // flags? (00, 01, 02, 06)
        avio_skip(pb, 4); // flag 1

        layer->extradata_size = avio_r32(pb);
        if (layer->extradata_size) {
            layer->extradata_offset = avio_tell(pb);
            avio_skip(pb, layer->extradata_size); // mp3 info, atrac9 config, etc
        }
    }

    if (bao->cfg.engine_version < 0x2A00) {
        bao->is_inline = avio_r32(pb);
        avio_skip(pb, 4); // flag 1
        bao->inline_size = avio_r32(pb);
    } else if (bao->cfg.engine_version < 0x2B00) {
        bao->inline_size = avio_r32(pb);
        bao->is_inline = avio_r32(pb);
        avio_skip(pb, 4); // null
        avio_skip(pb, 4); // null
    } else {
        bao->inline_size = avio_r32(pb);
        avio_skip(pb, 4); // inline_id? (-1 if not set, same as stream_id below)
        bao->is_inline = avio_r32(pb);
        avio_skip(pb, 4); // null
        avio_skip(pb, 4); // null
    }

    if (bao->inline_size) {
        bao->inline_offset = avio_tell(pb);
        avio_skip(pb, bao->inline_size); // codec data
    }

    // footer (stream_id equals header_id and is ignored if stream_flag is not set)
    if (bao->cfg.engine_version <= 0x2900) {
        bao->stream_id = avio_r32(pb);
    } else {
        avio_skip(pb, 4); // null
        avio_skip(pb, 4); // flag 1
        avio_skip(pb, 4); // null
        bao->stream_id = avio_r32(pb);
        avio_skip(pb, 4); // hash?
    }

    if (bao->cfg.engine_version >= 0x2B00)
        avio_skip(pb, 4); // null

    return 0;
}

static int parse_type_silence_v29(AVFormatContext *s, ubi_bao_header_t *bao)
{
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32 = bao->cfg.big_endian ? avio_rb32 : avio_rl32;

    if (bao->cfg.engine_version > 0x2900) {
        av_log(s, AV_LOG_ERROR, "silence_v29 not implemented\n");
        return AVERROR_INVALIDDATA;
    }

    avio_skip(pb, 4); // fixed? 0xFF4F734A
    avio_skip(pb, 4); // flag 1
    avio_skip(pb, 4); // bao id?
    bao->silence_duration = avio_r32(pb);

    avio_skip(pb, 4); // bao id?

    return 0;
}

static int parse_header_v29(AVFormatContext *s, ubi_bao_header_t *bao, int64_t offset)
{
    AVIOContext *pb = s->pb;
    int ret;

    if ((ret = parse_base_v29(s, bao)) < 0)
        return ret;
    if ((ret = parse_common_v29(s, bao)) < 0)
        return ret;

    if (bao->header_type < BAO_MAX_TYPES) {
        bao->type = type_map[bao->header_type];
    }

    ret = AVERROR_INVALIDDATA;
    switch (bao->type) {
    case TYPE_AUDIO:    ret = parse_type_audio_v29(s, bao); break;
    case TYPE_SEQUENCE: ret = parse_type_sequence_v29(s, bao); break;
    case TYPE_LAYER:    ret = parse_type_layer_v29(s, bao); break;
    case TYPE_SILENCE:  ret = parse_type_silence_v29(s, bao); break;
    default: ret = AVERROR_INVALIDDATA;
    }

    if (ret < 0)
        return ret;

    bao->header_size = (uint32_t)(avio_tell(pb) - offset);

    return 0;
}

static int parse_values(AVFormatContext *s, ubi_bao_header_t *bao)
{
    if (bao->type == TYPE_SEQUENCE || bao->type == TYPE_SILENCE)
        return 0;

    if (bao->stream_size == 0) {
        av_log(s, AV_LOG_ERROR, "unknown stream_size at %lx\n", bao->header_offset);
        return AVERROR_INVALIDDATA;
    }

    if (bao->stream_type >= BAO_MAX_CODECS) {
        av_log(s, AV_LOG_ERROR, "unknown stream_type at %lx\n", bao->header_offset);
        return AVERROR_INVALIDDATA;
    }

    if (bao->type == TYPE_LAYER) {
        bao->channels       = bao->layer[0].channels;
        bao->sample_rate    = bao->layer[0].sample_rate;
        bao->stream_type    = bao->layer[0].stream_type;
        bao->num_samples    = bao->layer[0].num_samples;

        // check that layers were parsed correctly
        for (int i = 1; i < bao->layer_count; i++) {
            ubi_bao_layer_t* layer_curr = &bao->layer[i];
            ubi_bao_layer_t* layer_prev = &bao->layer[i-1];

            if (layer_curr->sample_rate != layer_prev->sample_rate || layer_curr->stream_type != layer_prev->stream_type) {
                av_log(s, AV_LOG_ERROR, "layer headers don't match\n");
                if (!bao->cfg.layer_ignore_error)
                    return AVERROR_INVALIDDATA;
            }

            bao->channels += layer_curr->channels;
        }
    }

    bao->codec = bao->cfg.codec_map[bao->stream_type];
    if (bao->codec == CODEC_NONE) {
        av_log(s, AV_LOG_ERROR, "unknown codec %x at %lx\n", bao->stream_type, bao->header_offset);
        return AVERROR_INVALIDDATA;
    }

    if (bao->type == TYPE_LAYER && bao->codec == RAW_AT3) {
        if (bao->cfg.layer_default_subtype)
            bao->stream_subtype = bao->cfg.layer_default_subtype;
        else
            bao->stream_subtype = 1;
    }

    if (bao->type == TYPE_AUDIO && bao->codec == RAW_PSX && bao->cfg.v1_bao && bao->loop_flag) {
        bao->num_samples = bao->num_samples / bao->channels;
    }

    if (bao->stream_id == bao->header_id && (!bao->is_stream || bao->is_prefetch)) { // layers with memory data
        bao->memory_skip = bao->header_size + bao->extra_size;
        bao->stream_skip = bao->cfg.header_skip;
    } else {
        bao->memory_skip = bao->cfg.header_skip;
        bao->stream_skip = bao->cfg.header_skip;
    }

    if (!bao->is_stream && bao->is_prefetch) {
        av_log(s, AV_LOG_ERROR, "unexpected non-streamed prefetch at %lx\n", bao->header_offset);
    }

    if (bao->is_inline && ((bao->is_stream && !bao->is_prefetch) || (!bao->is_stream && bao->is_prefetch))) {
        av_log(s, AV_LOG_ERROR, "unexpected inline stream at %lx\n", bao->header_offset);
        return AVERROR_INVALIDDATA;
    }

    return 0;
}

static int parse_header(AVFormatContext *s, ubi_bao_header_t *bao, int64_t offset)
{
    AVIOContext *pb = s->pb;
    avio_seek(pb, offset, SEEK_SET);
    uint8_t header_format = avio_r8(pb);
    avio_seek(pb, offset, SEEK_SET);
    uint32_t header_version = avio_rb32(pb);

    if ((bao->cfg.version & 0x00FFFF00) != (header_version & 0x00FFFF00) || header_format > 0x03) {
        av_log(s, AV_LOG_ERROR, "mayor version header mismatch at %lx\n", offset);
        return AVERROR_INVALIDDATA;
    }

    bao->header_offset  = offset;

    int ret = 0;
    switch (bao->cfg.parser) {
    case PARSER_1B: ret = parse_header_cfg(s, bao, offset); break;
    case PARSER_29: ret = parse_header_v29(s, bao, offset); break;
    default:
        av_log(s, AV_LOG_ERROR, "unknown parser\n");
        return AVERROR_INVALIDDATA;
    }

    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "failed to parse header at %lx\n", offset);
        return ret;
    }

    ret = parse_values(s, bao);
    if (ret < 0)
        return ret;

    return 0;
}

static int parse_bao(AVFormatContext *s, ubi_bao_header_t *bao, int64_t offset, int target_stream)
{
    AVIOContext *pb = s->pb;
    uint32_t bao_class, header_type;
    avio_seek(pb, offset, SEEK_SET);
    uint32_t bao_version = avio_rb32(pb);
    int ret;

    if (((bao_version >> 24) & 0xFF) > 0x02) {
        av_log(s, AV_LOG_ERROR, "wrong header type %x at %lx\n", bao_version, offset);
        return AVERROR_INVALIDDATA;
    }

    ubi_bao_config_endian(s, &bao->cfg, offset);
    avio_r32 avio_r32 = bao->cfg.big_endian ? avio_rb32 : avio_rl32;

    avio_seek(pb, offset + bao->cfg.bao_class, SEEK_SET);
    bao_class = avio_r32(pb);
    if (bao_class & 0x0FFFFFFF) {
        av_log(s, AV_LOG_ERROR, "unknown class %x at %lx\n", bao_class, offset);
        return AVERROR_INVALIDDATA;
    }

    if (bao_class != 0x20000000)
        return 0;

    int64_t h_offset = offset + bao->cfg.header_skip;
    avio_seek(pb, h_offset + bao->cfg.header_type, SEEK_SET);
    header_type = avio_r32(pb);
    if (header_type > 9) {
        av_log(s, AV_LOG_ERROR, "unknown type %x at %lx\n", header_type, offset);
        return AVERROR_INVALIDDATA;
    }

    if (!bao->cfg.allowed_types[header_type])
        return 0;

    bao->total_streams++;
    if (target_stream != bao->total_streams)
        return 0;

    ret = parse_header(s, bao, offset);
    if (ret < 0)
        return ret;

    return 0;
}

static const char *atomic_memory_baos[] = {
    "%08x.bao", // common
    // .forge names
    "BAO_0x%08x",
    "Common_BAO_0x%08x",
    "BAO_0x%08x.bao", // used?
};

static const int atomic_memory_baos_count = FF_ARRAY_ELEMS(atomic_memory_baos);

static const char* atomic_stream_baos[] = {
    "%08x.sbao", // common
    "%08x.bao", // .fat+bin
    // .forge names (unused)
    "Common_BAO_0x%08x",
    "Common_BAO_0x%08x.sbao", // used?
    // .forge language names
    "English_BAO_0x%08x",
    "French_BAO_0x%08x",
    "Spanish_BAO_0x%08x",
    "Polish_BAO_0x%08x",
    "German_BAO_0x%08x",
    "Chinese_BAO_0x%08x",
    "Hungarian_BAO_0x%08x",
    "Italian_BAO_0x%08x",
    "Japanese_BAO_0x%08x",
    "Czech_BAO_0x%08x",
    "Korean_BAO_0x%08x",
    "Russian_BAO_0x%08x",
    "Dutch_BAO_0x%08x",
    "Danish_BAO_0x%08x",
    "Norwegian_BAO_0x%08x",
    "Swedish_BAO_0x%08x",
};

static const int atomic_stream_baos_count = FF_ARRAY_ELEMS(atomic_stream_baos);

static const uint8_t atrac3_block_types[3] = { 0x60, 0x98, 0xC0 };

static AVStream *open_streamfile_by_filename(AVFormatContext *s, const char *name)
{
    AVDictionary *tmp = NULL;
    AVCodecParameters *par;
    AVIOContext *pb = NULL;
    const char *dir_name;
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

    av_free(name_path);
    av_free(copy_name);

    if (ret == 0) {
        UBIBAODemuxContext *b = s->priv_data;
        ubi_bao_header_t *bao = &b->bao;
        BAOStream *bst;
        AVStream *st;

        st = avformat_new_stream(s, NULL);
        if (!st) {
            s->io_close2(s, pb);
            return NULL;
        }

        bst = av_mallocz(sizeof(*bst));
        if (!bst) {
            s->io_close2(s, pb);
            return NULL;
        }
        st->priv_data = bst;
        bst->pb = pb;
        bst->start_offset = bao->is_stream ? bao->stream_skip : bao->memory_skip;
        bst->stop_offset = bst->start_offset;
        bst->stop_offset += bao->prefetch_size + bao->stream_size;

        par = st->codecpar;
        st->start_time = 0;
        st->duration = bao->num_samples;
        par->codec_type = AVMEDIA_TYPE_AUDIO;
        par->sample_rate = bao->sample_rate;
        par->ch_layout.nb_channels = bao->channels;

        switch (bao->codec) {
        case UBI_IMA:
            par->codec_id = AV_CODEC_ID_PCM_U8;
            par->block_align = 1024;
            break;
        case RAW_PSX:
            par->codec_id = AV_CODEC_ID_ADPCM_PSX;
            par->block_align = 0x10 * bao->channels;
            break;
        case RAW_AT3:
            par->codec_id = AV_CODEC_ID_ATRAC3;

            if (bao->stream_subtype < 1 || bao->stream_subtype > 2)
                return NULL;
            par->block_align = atrac3_block_types[bao->stream_subtype] * bao->channels;

            ret = ff_alloc_extradata(par, 14);
            if (ret < 0)
                return NULL;

            AV_WL16(par->extradata, 1);
            AV_WL16(par->extradata+2, 2048 * bao->channels);
            AV_WL16(par->extradata+4, 0);
            AV_WL16(par->extradata+6, 0);
            AV_WL16(par->extradata+8, 0);
            AV_WL16(par->extradata+10, 1);
            AV_WL16(par->extradata+12, 0);
            break;
        case FMT_AT3:
            if (!(bst->ctx = avformat_alloc_context()))
                return NULL;

            if ((ret = ff_copy_whiteblacklists(bst->ctx, s)) < 0) {
                avformat_free_context(bst->ctx);
                bst->ctx = NULL;

                return NULL;
            }

            bst->ctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
            bst->ctx->probesize = 0;
            bst->ctx->max_analyze_duration = 0;
            bst->ctx->interrupt_callback = s->interrupt_callback;
            bst->ctx->pb = bst->pb;
            bst->ctx->io_open = NULL;
            bst->ctx->skip_initial_bytes = 0;

            avio_seek(pb, bst->start_offset, SEEK_SET);
            ret = avformat_open_input(&bst->ctx, "", NULL, NULL);
            if (ret < 0)
                return NULL;

            st->id = bst->ctx->streams[0]->id;
            st->duration = bst->ctx->streams[0]->duration;
            st->time_base = bst->ctx->streams[0]->time_base;
            st->start_time = bst->ctx->streams[0]->start_time;
            st->pts_wrap_bits = bst->ctx->streams[0]->pts_wrap_bits;

            ret = avcodec_parameters_copy(st->codecpar, bst->ctx->streams[0]->codecpar);
            if (ret < 0)
                return NULL;

            ret = av_dict_copy(&st->metadata, bst->ctx->streams[0]->metadata, 0);
            if (ret < 0)
                return NULL;

            ffstream(st)->request_probe = 0;
            ffstream(st)->need_parsing = ffstream(bst->ctx->streams[0])->need_parsing;

            bst->start_offset += avio_tell(pb);
            break;
        default:
            av_log(s, AV_LOG_ERROR, "unsupported codec %X\n", bao->codec);
            break;
        }

        avpriv_set_pts_info(st, 64, 1, par->sample_rate);

        return st;
    } else {
        return NULL;
    }
}

static AVStream *open_atomic_bao_list(AVFormatContext *s, uint32_t file_id, const char** names, int count, char* buf, int buf_size)
{
    for (int i = 0; i < count; i++) {
        const char *format = names[i];

        snprintf(buf, buf_size, format, file_id);
        AVStream *st = open_streamfile_by_filename(s, buf);
        if (st)
            return st;
    }

    return NULL;
}

static AVStream *open_atomic_bao(AVFormatContext *s, uint32_t file_id, int is_stream)
{
    AVStream *st = NULL;
    char buf[255];
    int64_t buf_size = sizeof(buf);

    const char **names = is_stream ? atomic_stream_baos : atomic_memory_baos;
    int count = is_stream ? atomic_stream_baos_count : atomic_memory_baos_count;

    st = open_atomic_bao_list(s, file_id, names, count, buf, buf_size);
    if (st)
        return st;

    goto fail;
fail:
    av_log(s, AV_LOG_ERROR, "failed opening atomic BAO id %08x\n", file_id);

    return NULL;
}

static AVStream *setup_atomic_bao_common(AVFormatContext *s, uint32_t resource_id, int is_stream, int64_t clamp_offset, uint32_t clamp_size)
{
    AVStream *temp_st = NULL;

    temp_st = open_atomic_bao(s, resource_id, is_stream);
    if (!temp_st)
        goto fail;

    return temp_st;
fail:
    return NULL;
}

static AVStream *open_memory_bao_atomic(AVFormatContext *s, ubi_bao_header_t *bao)
{
    uint32_t memory_offset = bao->memory_skip;
    uint32_t memory_size = bao->is_prefetch ? bao->prefetch_size : bao->stream_size;
    uint32_t internal_id = bao->stream_id;

    if (bao->is_prefetch && bao->cfg.v1_bao)
        internal_id = (internal_id & 0x0FFFFFFF) | 0x30000000;

    return setup_atomic_bao_common(s, internal_id, 0, memory_offset, memory_size);
}

static AVStream *open_stream_bao_atomic(AVFormatContext *s, ubi_bao_header_t *bao)
{
    uint64_t stream_offset = bao->stream_skip;
    uint32_t stream_size = bao->stream_size - bao->prefetch_size;
    uint32_t external_id = bao->stream_id;

    return setup_atomic_bao_common(s, external_id, 1, stream_offset, stream_size);
}

static AVStream *ubi_bao_layer(AVFormatContext *s, ubi_bao_header_t *bao)
{
    uint32_t real_stream_size = bao->stream_size - bao->prefetch_size;
    int load_memory = ((bao->is_prefetch) || (!bao->is_prefetch && !bao->is_stream)) && (!bao->is_inline);
    int load_stream = (bao->is_stream && !bao->is_prefetch) || (bao->is_prefetch && real_stream_size != 0);
    AVStream *st = NULL;

    if (bao->archive == ARCHIVE_ATOMIC) {
        if (load_memory)
            st = open_memory_bao_atomic(s, bao);
        else if (load_stream)
            st = open_stream_bao_atomic(s, bao);
    }

    if (!st)
        goto fail;

    return st;
fail:
    return NULL;
}

static int read_header(AVFormatContext *s)
{
    UBIBAODemuxContext *b = s->priv_data;
    ubi_bao_header_t *bao = &b->bao;
    AVIOContext *pb = s->pb;
    uint32_t version;
    AVStream *st;
    int ret;

    bao->archive = ARCHIVE_ATOMIC;
    version = avio_rb32(pb) & 0x00FFFFFF;
    ret = ubi_bao_config_version(s, &bao->cfg, version);
    if (ret < 0)
        return ret;

    ret = parse_bao(s, bao, 0x00, 1);
    if (ret < 0)
        return ret;

    if (bao->total_streams <= 0) {
        av_log(s, AV_LOG_ERROR, "bank has no streams\n");
        return AVERROR_INVALIDDATA;
    }

    switch (bao->type) {
    case TYPE_LAYER:
    case TYPE_AUDIO:
        st = ubi_bao_layer(s, bao);
        break;
    case TYPE_SEQUENCE:
        //st = ubi_bao_sequence(s, bao);
        //break;
    case TYPE_SILENCE:
        st = NULL;
        break;
    default:
        av_log(s, AV_LOG_ERROR, "stream not found/parsed\n");
        return AVERROR_INVALIDDATA;
    }

    if (s->nb_streams > 0) {
        AVStream *st = s->streams[0];
        BAOStream *bst = st->priv_data;

        avio_seek(bst->pb ? bst->pb : pb, bst->start_offset, SEEK_SET);
    }

    if (st)
        return 0;
    else
        return AVERROR(ENOMEM);
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    UBIBAODemuxContext *bao = s->priv_data;
    AVIOContext *mpb = s->pb;
    AVIOContext *pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    BAOStream *bst;
    AVStream *st;

redo:
    if (bao->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[bao->current_stream];
    bst = st->priv_data;
    pb = bst->pb ? bst->pb : mpb;
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (do_seek)
        avio_seek(pb, bst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= bst->stop_offset) {
        do_seek = 1;
        bao->current_stream++;
        goto redo;
    }

    if (bst->ctx) {
        ret = av_read_frame(bst->ctx, pkt);
    } else {
        const int64_t pos = avio_tell(pb);
        const int block_size = ff_pcm_default_packet_size(st->codecpar);
        const int size = FFMIN(block_size, bst->stop_offset - pos);

        ret = av_get_packet(pb, pkt, size);
        pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
        pkt->pos = pos;
    }
    pkt->stream_index = st->index;
    if (ret == AVERROR_EOF) {
        bao->current_stream++;
        goto redo;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    UBIBAODemuxContext *bao = s->priv_data;
    AVIOContext *mpb = s->pb;
    AVIOContext *pb;
    BAOStream *bst;
    AVStream *st;
    int64_t pos;
    int index;
    int ret;

    bao->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[bao->current_stream];
    bst = st->priv_data;
    pb = bst->pb ? bst->pb : mpb;

    if (bst->ctx) {
        ret = av_seek_frame(bst->ctx, 0, ts, flags);
        avio_skip(pb, bao->bao.is_stream ? bao->bao.stream_skip : bao->bao.memory_skip);
        return ret;
    }

    pos = avio_tell(pb);
    if (pos < bst->start_offset) {
        avio_seek(pb, bst->start_offset, SEEK_SET);
        return 0;
    }

    index = av_index_search_timestamp(st, ts, flags);

    FFStream *const sti = ffstream(st);
    if (index >= 0 && index < sti->nb_index_entries) {
        const AVIndexEntry *ie = &sti->index_entries[index];

        if ((ret = avio_seek(pb, ie->pos, SEEK_SET)) < 0)
            return ret;

        s->io_repositioned = 1;
        avpriv_update_cur_dts(s, st, ie->timestamp);

        return 0;
    }

    return -1;
}

static int read_close(AVFormatContext *s)
{
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        BAOStream *bst = st->priv_data;

        if (bst->ctx)
            avformat_close_input(&bst->ctx);
        else
            s->io_close2(s, bst->pb);
        bst->pb = NULL;
    }

    return 0;
}

const FFInputFormat ff_ubibao_demuxer = {
    .p.name         = "ubibao",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Ubisoft BAO"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "bao",
    .priv_data_size = sizeof(UBIBAODemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
