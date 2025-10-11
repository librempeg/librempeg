/*
 * AWC demuxer
 * Copyright (c) 2025 Paul B Mahol
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
#include "libavcodec/bytestream.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct AWCBlock {
    uint32_t start_entry;
    int entries;
    int32_t channel_skip;
    int32_t channel_samples;
    uint32_t channel_size;

    uint32_t frame_size;

    int64_t chunk_start;
    int64_t chunk_size;
} AWCBlock;

typedef struct AWCDemuxContext {
    int is_streamed;
    int codec;
    int channels;
    int big_endian;
    AWCBlock blk[32];
} AWCDemuxContext;

typedef struct AWCStream {
    int64_t start_offset;
    int64_t stop_offset;

    int tag_count;

    uint32_t block_count;
    uint32_t block_chunk;

    int64_t vorbis_offset;
    uint32_t vorbis_size;
} AWCStream;

typedef struct StreamInfo {
    int tag_count;
    uint32_t hash_id;
    uint32_t channel_hash;
    int64_t vorbis_offset;
    uint32_t vorbis_size;
    int64_t duration;
} StreamInfo;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('A','D','A','T') &&
        AV_RL32(p->buf) != MKBETAG('A','D','A','T'))
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const AWCStream *xs1 = s1->priv_data;
    const AWCStream *xs2 = s2->priv_data;

    return FFDIFFSIGN(xs1->start_offset, xs2->start_offset);
}

typedef uint64_t (*avio_r64)(AVIOContext *s);
typedef uint32_t (*avio_r32)(AVIOContext *s);
typedef uint32_t (*avio_r16)(AVIOContext *s);

static int read_block(AVFormatContext *s, const int is_big_endian)
{
    uint32_t channel_entry_size, seek_entry_size, extra_entry_size, header_padding;
    AWCDemuxContext *awc = s->priv_data;
    int64_t block_offset, offset;
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32;
    avio_r16 avio_r16;
    uint32_t sync = 0;

    while (sync != 0xffffffff) {
        sync <<= 8;
        sync |= avio_r8(pb);
        if (avio_feof(pb))
            return AVERROR_EOF;
    }
    avio_seek(pb, -4, SEEK_CUR);
    block_offset = offset = avio_tell(pb);

    if (is_big_endian) {
        avio_r32 = avio_rb32;
        avio_r16 = avio_rb16;
    } else {
        avio_r32 = avio_rl32;
        avio_r16 = avio_rl16;
    }

    switch (awc->codec) {
    case 0x05:
        channel_entry_size = 0x10;
        seek_entry_size = 0x04;
        extra_entry_size = 0x00;
        header_padding = 0x800;
        break;
    case 0x07:
    case 0x08:
        channel_entry_size = 0x18;
        seek_entry_size = 0x04;
        extra_entry_size = 0x00;
        header_padding = 0x800;
        break;
    case 0x0D:
    case 0x0F:
        channel_entry_size = 0x10;
        seek_entry_size = 0x00;
        extra_entry_size = 0x70;
        header_padding = 0x00;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    for (int ch = 0; ch < awc->channels; ch++) {
        avio_seek(pb, offset, SEEK_SET);
        awc->blk[ch].start_entry = avio_r32(pb);
        awc->blk[ch].entries = avio_r32(pb);
        awc->blk[ch].channel_skip = avio_r32(pb);
        awc->blk[ch].channel_samples = avio_r32(pb);
        if (awc->codec == 0x07) {
            avio_skip(pb, 4);
            awc->blk[ch].channel_size = avio_r32(pb);
        }

        offset += channel_entry_size;
    }

    for (int ch = 0; ch < awc->channels; ch++)
        offset += awc->blk[ch].entries * seek_entry_size;

    for (int ch = 0; ch < awc->channels; ch++) {
        switch (awc->codec) {
        case 0x05:
        case 0x08:
            awc->blk[ch].frame_size = 0x800;
            awc->blk[ch].chunk_size = awc->blk[ch].entries * awc->blk[ch].frame_size;
            awc->blk[ch].channel_size = awc->blk[ch].chunk_size;
            break;
        case 0x07:
            awc->blk[ch].frame_size = 0x800;
            awc->blk[ch].chunk_size = FFALIGN(awc->blk[ch].channel_size, 0x10);
            break;
        case 0x0D:
        case 0x0F:
            avio_seek(pb, offset + 4, SEEK_SET);
            awc->blk[ch].frame_size = avio_r16(pb);
            awc->blk[ch].chunk_size = awc->blk[ch].entries * awc->blk[ch].frame_size;
            awc->blk[ch].channel_size = awc->blk[ch].chunk_size;
            break;
        default:
            return AVERROR_INVALIDDATA;
        }

        offset += extra_entry_size;
    }

    if (header_padding == 0x800) {
        int64_t header_size = offset - block_offset;
        offset = block_offset + FFALIGN(header_size, 0x800);
    }

    for (int ch = 0; ch < awc->channels; ch++) {
        awc->blk[ch].chunk_start = offset;
        offset += awc->blk[ch].chunk_size;
    }

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        AWCStream *ast = st->priv_data;

        ast->start_offset = awc->blk[i].chunk_start;
        ast->stop_offset = ast->start_offset + awc->blk[i].chunk_size;
    }

    {
        AVStream *st = s->streams[0];
        AWCStream *ast = st->priv_data;

        avio_seek(pb, ast->start_offset, SEEK_SET);
    }

    if (avio_feof(pb))
        return AVERROR_EOF;

    return 0;
}

static int read_header(AVFormatContext *s)
{
    int entries, is_streamed, streamed_channels = 0;
    int64_t offset, first_start_offset, tags_offset;
    AWCDemuxContext *awc = s->priv_data;
    av_unused int is_alt;
    uint32_t flags;
    AVIOContext *pb = s->pb;
    StreamInfo sinfo[32];
    avio_r64 avio_r64;
    avio_r32 avio_r32;
    avio_r16 avio_r16;

    if (avio_rb32(pb) == MKBETAG('A','D','A','T')) {
        avio_r64 = avio_rl64;
        avio_r32 = avio_rl32;
        avio_r16 = avio_rl16;
        awc->big_endian = 0;
    } else {
        avio_r64 = avio_rb64;
        avio_r32 = avio_rb32;
        avio_r16 = avio_rb16;
        awc->big_endian = 1;
    }

    flags = avio_r32(pb);
    entries = avio_r32(pb);
    if (entries <= 0)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4);

    offset = 0x10;
    if ((flags & 0xFF00FFFF) != 0xFF000001 || (flags & 0x00F00000))
        return AVERROR_INVALIDDATA;

    if (flags & 0x00010000)
        offset += 2 * entries;

    is_alt = !!(flags & 0x00040000);

    if (flags & 0x00080000)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, offset, SEEK_SET);
    is_streamed = (avio_r32(pb) & 0x1FFFFFFF) == 0x00000000;
    if (is_streamed) {
        if (entries > 32)
            return AVERROR_INVALIDDATA;
    }

    avio_seek(pb, offset, SEEK_SET);
    for (int i = 0; i < entries; i++) {
        uint32_t info_header = avio_r32(pb);
        uint32_t hash_id = (info_header >>  0) & 0x1FFFFFFF;
        int entry_count = (info_header >> 29) & 0x7;
        AWCStream *ast;
        AVStream *st;

        if (is_streamed) {
            sinfo[i].hash_id = hash_id;
            sinfo[i].tag_count = entry_count;

            if (i == 0) {
                st = avformat_new_stream(s, NULL);
                if (!st)
                    return AVERROR(ENOMEM);

                ast = av_mallocz(sizeof(*ast));
                if (!ast)
                    return AVERROR(ENOMEM);
                st->priv_data = ast;

                ast->tag_count = entry_count;
            }
        } else {
            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            ast = av_mallocz(sizeof(*ast));
            if (!ast)
                return AVERROR(ENOMEM);
            st->priv_data = ast;

            ast->tag_count = entry_count;
        }

        offset += 0x04;
    }
    tags_offset = offset;

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        AWCStream *ast = st->priv_data;
        int tag_count = ast->tag_count;
        int channels = 0, rate = 0, codec = -1;
        int64_t pos, duration = 0;

        for (int n = 0; n < tag_count; n++) {
            uint64_t tag_header = avio_r64(pb);
            uint8_t  tag_type   = ((tag_header >> 56) & 0xFF);
            uint32_t tag_size   = ((tag_header >> 28) & 0x0FFFFFFF);
            uint32_t tag_offset = ((tag_header >>  0) & 0x0FFFFFFF);

            pos = avio_tell(pb);
            switch (tag_type) {
            case 0x55:
                ast->start_offset = tag_offset;
                ast->stop_offset = tag_offset;
                ast->stop_offset += tag_size;
                break;
            case 0x48:
                if (!is_streamed)
                    return AVERROR_INVALIDDATA;

                avio_seek(pb, tag_offset, SEEK_SET);
                ast->block_count = avio_r32(pb);
                ast->block_chunk = avio_r32(pb);
                streamed_channels = channels = avio_r32(pb);

                if (channels > entries - 1)
                    return AVERROR_INVALIDDATA;

                for (int ch = 0; ch < channels; ch++) {
                    sinfo[ch].channel_hash = avio_r32(pb);
                    sinfo[ch].duration = avio_r32(pb);
                    avio_skip(pb, 2);
                    rate = avio_r16(pb);
                    codec = avio_r8(pb);
                    avio_skip(pb, 3);
                }
                break;
            case 0xFA:
                if (is_streamed)
                    break;

                avio_seek(pb, tag_offset, SEEK_SET);
                duration = avio_r32(pb);
                avio_skip(pb, 4);
                rate = avio_r16(pb);
                avio_skip(pb, 9);
                codec = avio_r8(pb);
                channels = 1;
                break;
            case 0x76:
                if (is_streamed)
                    return AVERROR_INVALIDDATA;

                avio_seek(pb, tag_offset, SEEK_SET);
                duration = avio_r32(pb);
                avio_skip(pb, 4);
                rate = avio_r16(pb);
                avio_skip(pb, 18);
                codec = avio_r8(pb);
                avio_skip(pb, 1);
                if (avio_r16(pb))
                    ast->vorbis_offset = tag_offset + 0x20;
                channels = 1;
                break;
            case 0x7F:
                if (is_streamed)
                    return AVERROR_INVALIDDATA;

                ast->vorbis_offset = tag_offset;
                ast->vorbis_size = tag_size;
                break;
            case 0x68:
                ast->start_offset = tag_offset;
                ast->stop_offset = tag_offset;
                ast->stop_offset += tag_size;
                duration = 48000;
                rate = 48000;
                codec = 0;
                channels = 1;
                break;
            case 0xA3:
            case 0xBD:
            case 0x5C:
            case 0x81:
            case 0x36:
            case 0x2B:
            default:
                break;
            }

            avio_seek(pb, pos, SEEK_SET);
        }

        if (rate <= 0 || channels <= 0)
            return AVERROR_INVALIDDATA;

        awc->channels = channels;
        awc->codec = codec;

        switch (codec) {
        case 0x00:
            codec = (avio_r32 == avio_rl32) ? AV_CODEC_ID_PCM_S16LE : AV_CODEC_ID_PCM_S16BE;
            st->codecpar->block_align = 1024 * channels;
            break;
        case 0x05:
            codec = AV_CODEC_ID_XMA2;
            st->codecpar->block_align = 0x800;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;
            break;
        case 0x07:
            codec = AV_CODEC_ID_MP3;
            st->codecpar->block_align = 1024;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;
            break;
        case 0x08:
            codec = AV_CODEC_ID_VORBIS;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
            break;
        case 0x0D:
            codec = AV_CODEC_ID_OPUS;
            st->codecpar->block_align = 1024;
            break;
        case 0x0F:
            codec = AV_CODEC_ID_ATRAC9;
            st->codecpar->block_align = 1024;
            break;
        case 0x0C:
        case 0x10:
            codec = (avio_r32 == avio_rl32) ? AV_CODEC_ID_ADPCM_NDSP_LE : AV_CODEC_ID_ADPCM_NDSP;
            st->codecpar->block_align = 8 * channels;
            break;
        case 0x11:
            codec = AV_CODEC_ID_ADPCM_MS;
            st->codecpar->block_align = 8 * channels;
            break;
        case 0xFF:
        default:
            codec = AV_CODEC_ID_NONE;
            st->codecpar->block_align = 1024;
            break;
        }

        st->start_time = 0;
        if (!is_streamed && duration > 0)
            st->duration = duration;
        if (is_streamed && sinfo[0].duration > 0)
            st->duration = sinfo[0].duration;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->codec_id = codec;

        if (is_streamed && codec == AV_CODEC_ID_VORBIS) {
            PutByteContext pbc;
            int ret;

            offset = tags_offset;
            offset += 0x08 * sinfo[0].tag_count;

            for (int stream = 1; stream < entries; stream++) {
                for (int tag = 0; tag < sinfo[stream].tag_count; tag++) {
                    avio_seek(pb, offset, SEEK_SET);
                    uint64_t tag_header = avio_r64(pb);
                    uint8_t  tag_type   = ((tag_header >> 56) & 0xFF);
                    uint32_t tag_size   = ((tag_header >> 28) & 0x0FFFFFFF);
                    uint32_t tag_offset = ((tag_header >>  0) & 0x0FFFFFFF);

                    switch (tag_type) {
                    case 0x7f:
                        for (int ch = 0; ch < channels; ch++) {
                            if (sinfo[ch].channel_hash == sinfo[stream].hash_id) {
                                sinfo[ch].vorbis_offset = tag_offset;
                                sinfo[ch].vorbis_size = tag_size;
                                break;
                            }
                        }
                        break;
                    default:
                        break;
                    }

                    offset += 0x08;
                }
            }

            avio_seek(pb, sinfo[i].vorbis_offset, SEEK_SET);
            ret = ff_alloc_extradata(st->codecpar, sinfo[i].vorbis_size);
            if (ret < 0)
                return ret;
            memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
            bytestream2_init_writer(&pbc, st->codecpar->extradata, st->codecpar->extradata_size);
            ret = avio_rl32(pb);
            if (ret <= 0)
                return AVERROR_INVALIDDATA;
            bytestream2_put_be16(&pbc, ret);
            if (bytestream2_get_bytes_left_p(&pbc) < ret)
                return AVERROR_INVALIDDATA;
            avio_read(pb, st->codecpar->extradata+bytestream2_tell_p(&pbc), ret);
            bytestream2_skip_p(&pbc, ret);
            ret = avio_rl32(pb);
            if (ret <= 0)
                return AVERROR_INVALIDDATA;
            bytestream2_put_be16(&pbc, ret);
            if (bytestream2_get_bytes_left_p(&pbc) < ret)
                return AVERROR_INVALIDDATA;
            avio_read(pb, st->codecpar->extradata+bytestream2_tell_p(&pbc), ret);
            bytestream2_skip_p(&pbc, ret);
            ret = avio_rl32(pb);
            if (ret <= 0)
                return AVERROR_INVALIDDATA;
            bytestream2_put_be16(&pbc, ret);
            if (bytestream2_get_bytes_left_p(&pbc) < ret)
                return AVERROR_INVALIDDATA;
            avio_read(pb, st->codecpar->extradata+bytestream2_tell_p(&pbc), ret);
            bytestream2_skip_p(&pbc, ret);
        }
    }

    if (is_streamed) {
        int64_t offset = 0;

        for (int i = 1; i < streamed_channels; i++) {
            AVStream *st0 = s->streams[0];
            PutByteContext pbc;
            AWCStream *ast;
            AVStream *st;
            int ret;

            if (sinfo[i].vorbis_offset <= 0 || sinfo[i].vorbis_size <= 0)
                continue;

            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            ast = av_mallocz(sizeof(*ast));
            if (!ast)
                return AVERROR(ENOMEM);
            st->priv_data = ast;

            st->start_time = 0;
            if (sinfo[i].duration > 0)
                st->duration = sinfo[i].duration;
            st->codecpar->ch_layout.nb_channels = st0->codecpar->ch_layout.nb_channels;
            st->codecpar->sample_rate = st0->codecpar->sample_rate;
            st->codecpar->block_align = st0->codecpar->block_align;
            st->codecpar->codec_type = st0->codecpar->codec_type;
            st->codecpar->codec_id = st0->codecpar->codec_id;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

            avio_seek(pb, sinfo[i].vorbis_offset, SEEK_SET);
            offset = FFMAX(offset, sinfo[i].vorbis_offset + sinfo[i].vorbis_size);
            ret = ff_alloc_extradata(st->codecpar, sinfo[i].vorbis_size);
            if (ret < 0)
                return ret;
            memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
            bytestream2_init_writer(&pbc, st->codecpar->extradata, st->codecpar->extradata_size);
            ret = avio_rl32(pb);
            if (ret <= 0)
                return AVERROR_INVALIDDATA;
            bytestream2_put_be16(&pbc, ret);
            if (bytestream2_get_bytes_left_p(&pbc) < ret)
                return AVERROR_INVALIDDATA;
            avio_read(pb, st->codecpar->extradata+bytestream2_tell_p(&pbc), ret);
            bytestream2_skip_p(&pbc, ret);
            ret = avio_rl32(pb);
            if (ret <= 0)
                return AVERROR_INVALIDDATA;
            bytestream2_put_be16(&pbc, ret);
            if (bytestream2_get_bytes_left_p(&pbc) < ret)
                return AVERROR_INVALIDDATA;
            avio_read(pb, st->codecpar->extradata+bytestream2_tell_p(&pbc), ret);
            bytestream2_skip_p(&pbc, ret);
            ret = avio_rl32(pb);
            if (ret <= 0)
                return AVERROR_INVALIDDATA;
            bytestream2_put_be16(&pbc, ret);
            if (bytestream2_get_bytes_left_p(&pbc) < ret)
                return AVERROR_INVALIDDATA;
            avio_read(pb, st->codecpar->extradata+bytestream2_tell_p(&pbc), ret);
            bytestream2_skip_p(&pbc, ret);
        }

        offset = sinfo[streamed_channels-1].vorbis_offset + sinfo[streamed_channels-1].vorbis_size;
        avio_seek(pb, offset, SEEK_SET);
        if (read_block(s, avio_r32 == avio_rb32) < 0)
            return AVERROR_INVALIDDATA;
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    {
        AVStream *st = s->streams[0];
        AWCStream *ast = st->priv_data;

        first_start_offset = ast->start_offset;
    }

    avio_seek(pb, first_start_offset, SEEK_SET);
    awc->is_streamed = is_streamed;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AWCDemuxContext *awc = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        AWCStream *ast = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= ast->start_offset && pos < ast->stop_offset) {
            int packet_size;

            if (awc->codec == 0x08) {
                packet_size = avio_rl16(pb);
                if (packet_size == 0) {
                    int64_t offset = avio_tell(pb) - ast->start_offset;
                    int skip = 0x800 - (offset % 0x800);

                    avio_skip(pb, skip);
                    packet_size = avio_rl16(pb);
                    if (avio_tell(pb) > ast->stop_offset - packet_size) {
                        avio_seek(pb, ast->stop_offset, SEEK_SET);
                        continue;
                    }
                }
            } else {
                packet_size = par->block_align;
            }
            const int size = FFMIN(packet_size, ast->stop_offset - pos);

            ret = av_get_packet(pb, pkt, size);

            pkt->pos = pos;
            pkt->stream_index = st->index;

            break;
        } else if (pos >= ast->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            AWCStream *pst_next = st_next->priv_data;

            if (pst_next->start_offset > pos)
                avio_seek(pb, pst_next->start_offset, SEEK_SET);
        }
    }

    if (ret == AVERROR_EOF) {
        AVStream *st = s->streams[s->nb_streams-1];
        AWCStream *ast = st->priv_data;

        avio_seek(pb, ast->stop_offset, SEEK_SET);
        if ((ret = read_block(s, awc->big_endian)) < 0)
            return ret;
        return FFERROR_REDO;
    }
    return ret;
}

const FFInputFormat ff_awc_demuxer = {
    .p.name         = "awc",
    .p.long_name    = NULL_IF_CONFIG_SMALL("AWC (Audio Wave Container)"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "awc",
    .priv_data_size = sizeof(AWCDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
