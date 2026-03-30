/*
 * Factor 5 VID1 video demuxer
 * Copyright (c) 2026 Paul B Mahol
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
#define BITSTREAM_READER_LE
#include "libavcodec/get_bits.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"

typedef struct VID1Context {
    int audio_stream_index;
    int video_stream_index;
    int be;
} VID1Context;

static int read_probe(const AVProbeData *p)
{
    uint32_t magic = AV_RB32(p->buf);

    if (magic != MKBETAG('V','I','D','1') &&
        magic != MKTAG('V','I','D','1'))
        return 0;

    return AVPROBE_SCORE_MAX/3*2;
}

static int load_header_packet(AVIOContext *pb, AVCodecParameters *par,
                              int packet_size, int64_t *p_offset, int *e_offset)
{
    if (packet_size + e_offset[0] > par->extradata_size)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, p_offset[0], SEEK_SET);
    if (avio_read(pb, par->extradata + e_offset[0], packet_size) != packet_size)
        return AVERROR_INVALIDDATA;

    p_offset[0] += packet_size;
    e_offset[0] += packet_size;

    return 0;
}

static int get_packet_header(AVIOContext *pb, int64_t *offset, int *size)
{
    GetBitContext gbit;
    GetBitContext *gb = &gbit;
    uint8_t ibuf[4] = {0};
    uint32_t size_bits;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    avio_seek(pb, offset[0], SEEK_SET);
    if (avio_read(pb, ibuf, 4) != 4)
        return AVERROR_INVALIDDATA;

    ret = init_get_bits8(gb, ibuf, 4);
    if (ret < 0)
        return ret;

    size_bits = get_bits(gb, 4);
    size[0] = get_bits_long(gb, size_bits + 1);

    if (size_bits == 0 && size[0] == 0 && ibuf[0] == 128)
        size[0] = 1;

    offset[0] += (get_bits_count(gb)+7)/8;

    return 0;
}

static int read_header(AVFormatContext *s)
{
    int64_t offset, start_offset, header_offset;
    unsigned (*avio_r32)(AVIOContext *pb);
    unsigned (*avio_r16)(AVIOContext *pb);
    VID1Context *vid1 = s->priv_data;
    int codec, rate, channels, ret;
    AVIOContext *pb = s->pb;
    uint32_t magic, chunk;
    AVStream *st;

    magic = avio_rb32(pb);
    switch (magic) {
    case MKBETAG('V','I','D','1'):
        vid1->be = 1;
        avio_r32 = avio_rb32;
        avio_r16 = avio_rb16;
        break;
    case MKTAG('V','I','D','1'):
        vid1->be = 0;
        avio_r32 = avio_rl32;
        avio_r16 = avio_rl16;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    offset = avio_r32(pb);
    avio_seek(pb, offset, SEEK_SET);
    if (avio_r32(pb) != AV_RB32("HEAD"))
        return AVERROR_INVALIDDATA;
    start_offset = offset + avio_r32(pb);
    offset += 12;
    avio_seek(pb, offset, SEEK_SET);
    chunk = avio_r32(pb);
    if (chunk == AV_RB32("VIDH")) {
        offset += avio_r32(pb);

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        avio_skip(pb, 4);
        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        st->codecpar->codec_id = AV_CODEC_ID_MPEG2VIDEO;
        st->codecpar->width = avio_r16(pb);
        st->codecpar->height = avio_r16(pb);

        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

        vid1->video_stream_index = st->index;

        avpriv_set_pts_info(st, 64, 1, 24);
    }

    avio_seek(pb, offset, SEEK_SET);
    chunk = avio_r32(pb);
    if (chunk == AV_RB32("AUDH")) {
        offset += 12;
        avio_seek(pb, offset, SEEK_SET);
        header_offset = offset;
        codec = avio_r32(pb);
        rate = avio_r32(pb);
        channels = avio_r8(pb);
        if (rate <= 0 || channels <= 0)
            return AVERROR_INVALIDDATA;

        if (codec == AV_RB32("PC16")) {
            codec = AV_CODEC_ID_PCM_S16LE;
        } else if (codec == AV_RB32("XAPM")) {
            codec = AV_CODEC_ID_ADPCM_IMA_XBOX;
        } else if (codec == AV_RB32("APCM")) {
            codec = AV_CODEC_ID_ADPCM_NDSP;
        } else if (codec == AV_RB32("VAUD")) {
            codec = AV_CODEC_ID_VORBIS;
        } else {
            avpriv_request_sample(s, "codec %08X", codec);
            return AVERROR_PATCHWELCOME;
        }

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->sample_rate = rate;
        st->codecpar->ch_layout.nb_channels = channels;

        ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;

        vid1->audio_stream_index = st->index;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        if (codec == AV_CODEC_ID_ADPCM_NDSP) {
            avio_seek(pb, header_offset + 10, SEEK_SET);

            ret = ff_get_extradata(s, st->codecpar, pb, 32 * channels);
            if (ret < 0)
                return ret;
        } else if (codec == AV_CODEC_ID_VORBIS) {
            int packet_size = 0, eoffset = 0;
            int64_t offset;
            uint8_t *buf;

            avio_seek(pb, header_offset + 32, SEEK_SET);
            st->duration = avio_r32(pb);

            offset = avio_tell(pb);

            ret = ff_alloc_extradata(st->codecpar, 16384);
            if (ret < 0)
                return ret;
            memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

            ret = get_packet_header(pb, &offset, &packet_size);
            if (ret < 0)
                return ret;

            AV_WB16(st->codecpar->extradata + eoffset, packet_size);
            eoffset += 2;

            ret = load_header_packet(pb, st->codecpar, packet_size, &offset, &eoffset);
            if (ret < 0)
                return ret;

            AV_WB16(st->codecpar->extradata + eoffset, 0x19);
            eoffset += 2;

            buf = st->codecpar->extradata + eoffset;
            bytestream_put_byte(&buf, 0x03);
            bytestream_put_buffer(&buf, "vorbis", 6);
            bytestream_put_le32(&buf, 9);
            bytestream_put_buffer(&buf, "librempeg", 9);
            bytestream_put_le32(&buf, 0);
            bytestream_put_byte(&buf, 1);

            eoffset += 0x19;

            ret = get_packet_header(pb, &offset, &packet_size);
            if (ret < 0)
                return ret;

            AV_WB16(st->codecpar->extradata + eoffset, packet_size);
            eoffset += 2;

            ret = load_header_packet(pb, st->codecpar, packet_size, &offset, &eoffset);
            if (ret < 0)
                return ret;

            st->codecpar->extradata_size = eoffset;
        }
    }

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    unsigned (*avio_r32)(AVIOContext *pb);
    VID1Context *vid1 = s->priv_data;
    int block_size, ret, index, pkt_size;
    AVIOContext *pb = s->pb;
    const int be = vid1->be;
    uint32_t magic;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    avio_r32 = be ? avio_rb32 : avio_rl32;
    pos = avio_tell(pb);
    magic = avio_r32(pb);
    if (magic == AV_RB32("FRAM")) {
        avio_skip(pb, 28);
        magic = avio_r32(pb);
    }

    if (magic == AV_RB32("VIDD")) {
        block_size = avio_r32(pb);
        avio_skip(pb, 2);
        block_size -= 6;
        pkt_size = block_size - 8;
        avio_skip(pb, 4);
        block_size = pkt_size;
        index = vid1->video_stream_index;
    } else if (magic == AV_RB32("AUDD")) {
        block_size = avio_r32(pb);
        avio_skip(pb, 4);
        block_size -= 16;
        pkt_size = avio_r32(pb);
        index = vid1->audio_stream_index;
        if (s->streams[vid1->audio_stream_index]->codecpar->codec_id == AV_CODEC_ID_VORBIS)  {
            int64_t start = avio_tell(pb);
            int64_t offset = start + 4;

            ret = get_packet_header(pb, &offset, &pkt_size);
            if (ret < 0)
                return ret;

            if (avio_feof(pb))
                return AVERROR_EOF;
            block_size = pkt_size;
            avio_seek(pb, offset, SEEK_SET);
        }
    } else {
        if (s->streams[vid1->audio_stream_index]->codecpar->codec_id == AV_CODEC_ID_VORBIS)  {
            int64_t start = avio_tell(pb);
            int64_t offset = start - 4;

            ret = get_packet_header(pb, &offset, &pkt_size);
            if (ret < 0)
                return ret;

            block_size = pkt_size;
            avio_seek(pb, offset, SEEK_SET);
            index = vid1->audio_stream_index;
        } else {
            return AVERROR_INVALIDDATA;
        }
    }

    if (pkt_size <= 0)
        return FFERROR_REDO;

    ret = av_get_packet(pb, pkt, pkt_size);
    if (block_size > pkt_size)
        avio_skip(pb, block_size - pkt_size);
    if (avio_feof(pb))
        return AVERROR_EOF;

    pkt->pos = pos;
    pkt->stream_index = index;
    pkt->flags |= AV_PKT_FLAG_KEY;

    return ret;
}

const FFInputFormat ff_vid1_demuxer = {
    .p.name         = "vid1",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Factor 5 VID1"),
    .p.extensions   = "vid",
    .priv_data_size = sizeof(VID1Context),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
