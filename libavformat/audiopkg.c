/*
 * AUDIOPKG demuxer
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

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

#define MAX_TEMPERATURES 3
#define MAX_CHANNELS 2
#define MAX_NAME 256

enum { PT_NONE, PT_PS2, PT_XBOX, PT_GC, PT_PC };

typedef struct AUDIOPKGStream {
    int64_t start_offset;
    int64_t stop_offset;

    int head_size;
    int head_offset;
    int temperature;
} AUDIOPKGStream;

typedef struct AUDIOPKGContext {
    uint32_t platform;
    int version;
    int big_endian;
    int target_stream;

    int descriptors;
    int identifiers;
    uint32_t descriptors_size;
    uint32_t strings_size;
    uint32_t lipsyncs_size;
    uint32_t musicdata_size;
    uint32_t breakpoints_size;
    int sample_headers[MAX_TEMPERATURES];
    int sample_indices[MAX_TEMPERATURES];
    int sample_sizes[MAX_TEMPERATURES];

    uint64_t strings_offset;
    uint64_t lipsyncs_offset;
    uint64_t breakpoints_offset;
    uint64_t musicdata_offset;
    uint64_t identifiers_index_offset;
    uint64_t descriptors_index_offset;
    uint64_t descriptors_offset;

    uint64_t sample_indices_offset;
    uint64_t sample_headers_offset;
    int sample_indices_count;
    int sample_indices_extras;
    int sample_headers_count;
} AUDIOPKGContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != AV_RB32("v1.5") &&
        AV_RB32(p->buf) != AV_RB32("v1.6") &&
        AV_RB32(p->buf) != AV_RB32("v1.7") &&
        AV_RB32(p->buf) != AV_RB32("v1.8"))
        return 0;

    if (p->buf_size < 64)
        return 0;

    if (memcmp(p->buf+0x10, "Windows", 7) &&
        memcmp(p->buf+0x10, "Xbox", 4) &&
        memcmp(p->buf+0x10, "PlayStation II", 14) &&
        memcmp(p->buf+0x10, "Gamecube", 8))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const AUDIOPKGStream *as1 = s1->priv_data;
    const AUDIOPKGStream *as2 = s2->priv_data;

    return FFDIFFSIGN(as1->start_offset, as2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    AUDIOPKGContext *a = s->priv_data;
    AVIOContext *pb = s->pb;
    uint32_t version, platform;
    int64_t offset = 0x40;

    version = avio_rb32(pb);
    switch (version) {
    case MKBETAG('v', '1', '.', '5'):
        a->version = 5;
        break;
    case MKBETAG('v', '1', '.', '6'):
        a->version = 6;
        break;
    case MKBETAG('v', '1', '.', '7'):
        a->version = 7;
        break;
    case MKBETAG('v', '1', '.', '8'):
        a->version = 8;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    avio_skip(pb, 12);
    platform = avio_rb32(pb);
    switch (platform) {
    case MKBETAG('W','i','n','d'):
        a->platform = PT_PC;
        break;
    case MKBETAG('X','b','o','x'):
        a->platform = PT_XBOX;
        break;
    case MKBETAG('P','l','a','y'):
        a->platform = PT_PS2;
        break;
    case MKBETAG('G','a','m','e'):
        a->platform = PT_GC;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    if (a->platform == PT_PC && a->version == 6)
        a->version = 5;

    a->big_endian = a->platform == PT_GC;

    if (a->version == 5) {
        offset += 0x60;
    } else if (a->version == 6) {
        offset += 0x70;
    } else if (a->version == 7 || a->version == 8) {
        offset += 0x80;
    }

    avio_seek(pb, offset, SEEK_SET);

    unsigned (*avio_r32)(AVIOContext *pb) = a->big_endian ? avio_rb32 : avio_rl32;

    a->descriptors = avio_r32(pb);
    a->identifiers = avio_r32(pb);
    a->descriptors_size  = avio_r32(pb);
    a->strings_size      = avio_r32(pb);
    a->lipsyncs_size     = avio_r32(pb);
    a->musicdata_size    = avio_r32(pb);
    a->breakpoints_size  = avio_r32(pb);
    a->sample_headers[0] = avio_r32(pb);
    a->sample_headers[1] = avio_r32(pb);
    a->sample_headers[2] = avio_r32(pb);
    a->sample_indices[0] = avio_r32(pb);
    a->sample_indices[1] = avio_r32(pb);
    a->sample_indices[2] = avio_r32(pb);
    avio_skip(pb, 12);
    a->sample_sizes[0] = avio_r32(pb);
    a->sample_sizes[1] = avio_r32(pb);
    a->sample_sizes[2] = avio_r32(pb);

    offset += 0x4c;
    if (a->version >= 6)
        offset += 4;

    if (a->sample_indices[1] != 0 || a->sample_headers[1] != 0) {
        av_log(s, AV_LOG_ERROR, "'warm' samples found\n");
        return AVERROR_PATCHWELCOME;
    }

    if ((a->lipsyncs_size && a->musicdata_size) || (a->lipsyncs_size && a->breakpoints_size)) {
        av_log(s, AV_LOG_ERROR, "lipsyncs and musicdata/breakpoints found\n");
        return AVERROR_PATCHWELCOME;
    }

    a->strings_offset = offset;
    offset += a->strings_size;

    a->lipsyncs_offset = offset;
    offset += a->lipsyncs_size;

    a->breakpoints_offset = offset;
    offset += a->breakpoints_size;

    a->musicdata_offset = offset;
    offset += a->musicdata_size;

    a->identifiers_index_offset = offset;
    offset += (a->identifiers * 0x08);

    a->descriptors_index_offset = offset;
    offset += (a->descriptors * 0x04);

    a->descriptors_offset = offset;
    offset += a->descriptors_size;

    a->sample_indices_count = a->sample_indices[0] + a->sample_indices[1] + a->sample_indices[2];
    a->sample_indices_extras = 0;
    for (int i = 0; i < 3; i++) {
        if (!a->sample_indices[i])
            continue;
        a->sample_indices_extras++;
    }
    a->sample_indices_offset = offset;
    uint32_t sample_indices_size = (a->sample_indices_count + a->sample_indices_extras) * 0x02;
    offset += sample_indices_size;

    a->sample_headers_count = a->sample_headers[0] + a->sample_headers[1] + a->sample_headers[2];
    a->sample_headers_offset = offset;
    uint32_t sample_headers_size = 0;
    for (int i = 0; i < 3; i++)
        sample_headers_size += a->sample_headers[i] * a->sample_sizes[i];
    offset += sample_headers_size;

    if (a->sample_indices_count > a->sample_headers_count) {
        av_log(s, AV_LOG_ERROR, "unexpected count\n");
        return AVERROR_INVALIDDATA;
    }

    unsigned (*avio_r16)(AVIOContext *pb) = a->big_endian ? avio_rb16 : avio_rl16;

    offset = a->sample_indices_offset;
    avio_seek(pb, offset, SEEK_SET);

    for (int i = 0; i < 3; i++) {
        if (a->sample_indices[i] <= 0)
            continue;

        int header_index = avio_r16(pb);
        for (int j = 0; j < a->sample_indices[i]; j++) {
            int next_header_index = avio_r16(pb);

            int channels = next_header_index - header_index;

            if (channels == 1 || channels == 2) {
                AUDIOPKGStream *ast;
                AVStream *st;

                st = avformat_new_stream(s, NULL);
                if (!st)
                    return AVERROR(ENOMEM);

                ast = av_mallocz(sizeof(*ast));
                if (!ast)
                    return AVERROR(ENOMEM);
                st->priv_data = ast;

                ast->temperature = i;
                ast->head_size = a->sample_sizes[ast->temperature];
                ast->head_offset = a->sample_headers_offset;

                for (int l = 0; l < 3; l++) {
                    if (!a->sample_headers[l])
                        continue;

                    if (l < ast->temperature) {
                        ast->head_offset += a->sample_headers[l] * a->sample_sizes[l];
                        continue;
                    }

                    ast->head_offset += header_index * a->sample_sizes[l];
                    break;
                }

                st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
                st->codecpar->ch_layout.nb_channels = channels;
            }

            header_index = next_header_index;
        }
    }

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AUDIOPKGStream *ast = st->priv_data;
        int rate, codec, align = 0;
        int64_t duration;

        avio_seek(pb, ast->head_offset + 4, SEEK_SET);

        ast->start_offset = avio_r32(pb);
        ast->stop_offset = ast->start_offset;
        ast->stop_offset += avio_r32(pb);

        avio_skip(pb, 8);
        codec = avio_r32(pb);
        duration = avio_r32(pb);
        rate = avio_r32(pb);
        avio_skip(pb, 8);

        switch (codec) {
        case 0:
            switch (a->platform) {
            case PT_PS2:
                codec = AV_CODEC_ID_ADPCM_PSX;
                align = 0x8000;
                break;
            case PT_GC:
                codec = a->big_endian ? AV_CODEC_ID_ADPCM_NDSP : AV_CODEC_ID_ADPCM_NDSP_LE;
                align = 0x8000;
                break;
            case PT_XBOX:
                codec = AV_CODEC_ID_ADPCM_IMA_XBOX_MONO;
                align = 0x8000;
                break;
            case PT_PC:
                codec = AV_CODEC_ID_ADPCM_IMA_XBOX_MONO;
                align = 0x9000;
                break;
            }
            break;
        case 1:
            codec = a->big_endian ? AV_CODEC_ID_PCM_S16BE : AV_CODEC_ID_PCM_S16LE;
            align = 0x800;
            break;
        case 2:
            codec = AV_CODEC_ID_MP3;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
            align = 0x800;
            break;
        }

        if (align <= 0 || rate <= 0 || align >= INT_MAX/st->codecpar->ch_layout.nb_channels)
            return AVERROR_INVALIDDATA;

        st->start_time = 0;
        st->duration = duration;
        st->codecpar->block_align = align * st->codecpar->ch_layout.nb_channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->codec_id = codec;
    }

    if (s->nb_streams > 1)
        qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
        if (n == 0) {
            AUDIOPKGStream *ast = st->priv_data;

            avio_seek(pb, ast->start_offset, SEEK_SET);
        }
    }

    a->target_stream = 0;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AUDIOPKGContext *a = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (a->target_stream >= s->nb_streams)
        return AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        AUDIOPKGStream *ast = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= ast->start_offset && pos < ast->stop_offset) {
            const int size = FFMIN(par->block_align, ast->stop_offset - pos);

            ret = av_get_packet(pb, pkt, size);

            pkt->pos = pos;
            pkt->stream_index = st->index;

            break;
        } else if (pos >= ast->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            AUDIOPKGStream *ast_next = st_next->priv_data;

            if (ast_next->start_offset > pos)
                avio_skip(pb, ast_next->start_offset - pos);
        }
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t timestamp, int flags)
{
    AUDIOPKGContext *a = s->priv_data;

    a->target_stream = FFMAX(0, stream_index);

    return -1;
}

const FFInputFormat ff_audiopkg_demuxer = {
    .p.name         = "audiopkg",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Inevitable AUDIOPKG"),
    .priv_data_size = sizeof(AUDIOPKGContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "audiopkg",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
