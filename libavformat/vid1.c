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
#include "libavutil/mem.h"
#include "libavcodec/bytestream.h"
#define BITSTREAM_READER_LE
#include "libavcodec/get_bits.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"

typedef struct VID1Stream {
    AVPacket *pkt;
    int pkt_offset;
} VID1Stream;

typedef struct VID1Context {
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

static int get_packet_header_buf(const uint8_t *src, int offset, int *size)
{
    GetBitContext gbit;
    GetBitContext *gb = &gbit;
    uint8_t ibuf[4] = {0};
    uint32_t size_bits;
    int ret;

    memcpy(ibuf, src + offset, 4);

    ret = init_get_bits8(gb, ibuf, 4);
    if (ret < 0)
        return ret;

    size_bits = get_bits(gb, 4);
    size[0] = get_bits_long(gb, size_bits + 1);

    if (size_bits == 0 && size[0] == 0 && ibuf[0] == 128)
        size[0] = 1;

    return (get_bits_count(gb)+7)/8;
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
    int64_t offset, start_offset, header_offset, chunk_offset, next_chunk_offset;
    int codec, rate, channels, ret, chunk_size;
    unsigned (*avio_r32)(AVIOContext *pb);
    unsigned (*avio_r16)(AVIOContext *pb);
    VID1Context *vid1 = s->priv_data;
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

    while (!avio_feof(pb) && avio_tell(pb) < start_offset) {
        VID1Stream *xst;

        chunk_offset = avio_tell(pb);
        chunk = avio_r32(pb);
        if (chunk == 0)
            break;

        chunk_size = avio_r32(pb);
        if (chunk_size <= 8)
            return AVERROR_INVALIDDATA;

        next_chunk_offset = chunk_offset + chunk_size;

        if (chunk == AV_RB32("VIDH")) {
            AVRational fps;

            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            xst = av_mallocz(sizeof(*xst));
            if (!xst)
                return AVERROR(ENOMEM);
            st->priv_data = xst;

            xst->pkt = av_packet_alloc();
            if (!xst->pkt)
                return AVERROR(ENOMEM);

            avio_skip(pb, 4);
            st->start_time = 0;
            st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
            st->codecpar->codec_id = AV_CODEC_ID_MPEG2VIDEO;
            st->codecpar->width = avio_r16(pb);
            st->codecpar->height = avio_r16(pb);
            st->duration = st->nb_frames = avio_r32(pb);
            avio_skip(pb, 6);
            fps.num = avio_r16(pb);
            fps.den = avio_r16(pb);

            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

            avpriv_set_pts_info(st, 64, fps.den, fps.num);
        } else if (chunk == AV_RB32("AUDH")) {
            offset = chunk_offset;
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

            xst = av_mallocz(sizeof(*xst));
            if (!xst)
                return AVERROR(ENOMEM);
            st->priv_data = xst;

            xst->pkt = av_packet_alloc();
            if (!xst->pkt)
                return AVERROR(ENOMEM);

            st->start_time = 0;
            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->codec_id = codec;
            st->codecpar->sample_rate = rate;
            st->codecpar->ch_layout.nb_channels = channels;

            ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;

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

                xst->pkt_offset = 4;

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

        avio_seek(pb, next_chunk_offset, SEEK_SET);
    }

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int64_t pos, next_pkt_pos = -1, next_pos;
    unsigned (*avio_r32)(AVIOContext *pb);
    VID1Context *vid1 = s->priv_data;
    int frame_size, stream_index = 0;
    int block_size, ret, pkt_size;
    AVIOContext *pb = s->pb;
    const int be = vid1->be;
    uint32_t magic;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        VID1Stream *xst = st->priv_data;

        if (xst->pkt->size > 0 && xst->pkt_offset < xst->pkt->size) {
            if (st->codecpar->codec_id == AV_CODEC_ID_VORBIS) {
                int pkt_size;

                if (xst->pkt_offset + 4 > xst->pkt->size) {
                    av_packet_unref(xst->pkt);
                    xst->pkt_offset = 4;
                    continue;
                }

                ret = get_packet_header_buf(xst->pkt->data, xst->pkt_offset, &pkt_size);
                if (ret < 0)
                    return ret;

                if (pkt_size <= 0)
                    return AVERROR_INVALIDDATA;

                if (ret > xst->pkt->size - xst->pkt_offset)
                    return AVERROR_INVALIDDATA;

                if (xst->pkt_offset == 4)
                    pkt->flags |= AV_PKT_FLAG_KEY;

                xst->pkt_offset += ret;
                if (xst->pkt_offset + pkt_size > xst->pkt->size)
                    return AVERROR_INVALIDDATA;

                if ((ret = av_new_packet(pkt, pkt_size)) < 0)
                    return ret;

                memcpy(pkt->data, xst->pkt->data + xst->pkt_offset, pkt_size);
                xst->pkt_offset += pkt_size;

                pkt->stream_index = xst->pkt->stream_index;
                pkt->pos = xst->pkt->pos;

                if (xst->pkt_offset >= xst->pkt->size) {
                    av_packet_unref(xst->pkt);
                    xst->pkt_offset = 4;
                }
            } else {
                av_packet_move_ref(pkt, xst->pkt);
                pkt->flags |= AV_PKT_FLAG_KEY;
            }
            return 0;
        }
    }

    if (avio_feof(pb))
        return AVERROR_EOF;

    avio_r32 = be ? avio_rb32 : avio_rl32;
    pos = avio_tell(pb);
    magic = avio_r32(pb);
    if (magic == 0)
        return AVERROR_EOF;

    frame_size = avio_r32(pb);
    if (frame_size <= 8)
        return AVERROR_INVALIDDATA;

    if (magic == AV_RB32("FRAM")) {
        avio_skip(pb, 24);
        next_pkt_pos = pos + frame_size;
        frame_size -= 32;
    } else {
        av_log(s, AV_LOG_DEBUG, "magic %08X at %lX\n", magic, pos);
        return AVERROR_INVALIDDATA;
    }

    next_pos = avio_tell(pb);
    while (frame_size > 0) {
        AVStream *st = s->streams[FFMIN(stream_index, s->nb_streams-1)];
        VID1Stream *xst = st->priv_data;
        int64_t frame_pos;

        avio_seek(pb, next_pos, SEEK_SET);

        frame_pos = avio_tell(pb);
        magic = avio_r32(pb);
        block_size = avio_r32(pb);
        if (block_size <= 8)
            return AVERROR_INVALIDDATA;
        next_pos = frame_pos + block_size;

        frame_size -= block_size;
        if (frame_size < 0)
            return AVERROR_INVALIDDATA;

        if (magic == AV_RB32("VIDD")) {
            avio_skip(pb, 2);
            block_size -= 6;
            pkt_size = block_size - 8;
            avio_skip(pb, 4);
            block_size = pkt_size;
            ret = av_get_packet(pb, xst->pkt, pkt_size);
            if (ret < 0)
                return ret;

            xst->pkt->pos = pos;
            xst->pkt->duration = 1;
            xst->pkt->stream_index = stream_index;
            stream_index++;
        } else if (magic == AV_RB32("AUDD")) {
            avio_skip(pb, 4);
            block_size -= 16;
            pkt_size = avio_r32(pb);
            ret = av_get_packet(pb, xst->pkt, pkt_size);
            if (ret < 0)
                return ret;

            xst->pkt->pos = pos;
            xst->pkt->stream_index = stream_index;
            stream_index++;
        } else if (magic == AV_RB32("SUBT")) {
            pkt_size = block_size - 8;
            avio_skip(pb, pkt_size);
            pkt_size = 0;
        } else {
            av_log(s, AV_LOG_DEBUG, "magic %08X at %lX\n", magic, pos);
            pkt_size = block_size - 8;
            avio_skip(pb, pkt_size);
            pkt_size = 0;
        }
    }

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        VID1Stream *xst = st->priv_data;

        if (xst->pkt->size > 0 && xst->pkt_offset < xst->pkt->size) {
            if (st->codecpar->codec_id == AV_CODEC_ID_VORBIS) {
                int pkt_size;

                if (xst->pkt_offset + 4 > xst->pkt->size) {
                    av_packet_unref(xst->pkt);
                    xst->pkt_offset = 4;
                    continue;
                }

                ret = get_packet_header_buf(xst->pkt->data, xst->pkt_offset, &pkt_size);
                if (ret < 0)
                    return ret;

                if (pkt_size <= 0)
                    return AVERROR_INVALIDDATA;

                if (ret > xst->pkt->size - xst->pkt_offset)
                    return AVERROR_INVALIDDATA;

                if (xst->pkt_offset == 4)
                    pkt->flags |= AV_PKT_FLAG_KEY;

                xst->pkt_offset += ret;
                if (xst->pkt_offset + pkt_size > xst->pkt->size)
                    return AVERROR_INVALIDDATA;

                if ((ret = av_new_packet(pkt, pkt_size)) < 0)
                    return ret;

                memcpy(pkt->data, xst->pkt->data + xst->pkt_offset, pkt_size);
                xst->pkt_offset += pkt_size;

                pkt->stream_index = xst->pkt->stream_index;
                pkt->pos = xst->pkt->pos;

                if (xst->pkt_offset >= xst->pkt->size) {
                    av_packet_unref(xst->pkt);
                    xst->pkt_offset = 4;
                }
            } else {
                av_packet_move_ref(pkt, xst->pkt);
                pkt->flags |= AV_PKT_FLAG_KEY;
            }
            break;
        }
    }

    if (next_pkt_pos < 0)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, next_pkt_pos, SEEK_SET);

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t timestamp, int flags)
{
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        VID1Stream *xst = st->priv_data;

        av_packet_unref(xst->pkt);

        if (st->codecpar->codec_id == AV_CODEC_ID_VORBIS)
            xst->pkt_offset = 4;
        else
            xst->pkt_offset = 0;
    }

    return -1;
}

static int read_close(AVFormatContext *s)
{
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        VID1Stream *xst = st->priv_data;

        av_packet_free(&xst->pkt);
    }

    return 0;
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
    .read_seek      = read_seek,
    .read_close     = read_close,
};
