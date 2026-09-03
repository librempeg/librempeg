/*
 * Rockstar AUD demuxer
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
#include "avformat.h"
#include "demux.h"
#include "internal.h"

#define SUBBLOCK_SIZE 0x800
#define MAX_NB_STREAMS 12

typedef struct RAUDContext {
    int be;
    int block_size;
    int header_size;

    int block_subblocks;
    int nb_subblocks[MAX_NB_STREAMS];
    int subblock_offset[MAX_NB_STREAMS];

    int stream_index;
    int subblock_index;
    AVPacket *pkt;
} RAUDContext;

static int read_probe(const AVProbeData *p)
{
    int be = AV_RB32(p->buf) == 0;
    uint64_t offset;

    if (be)
        offset = AV_RB64(p->buf);
    else
        offset = AV_RL64(p->buf);
    if (offset > 0x20000 || offset < 0x1c)
        return 0;

    if (p->buf_size < 20)
        return 0;
    if (AV_RB32(p->buf+16) != 0)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int64_t start, duration, table_offset, offset;
    int nb_streams, rate, be, codec, align = 0;
    unsigned int (*avio_r32)(AVIOContext *pb);
    uint64_t (*avio_r64)(AVIOContext *pb);
    RAUDContext *raud = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st;

    raud->be = be = avio_rb32(pb) == 0;
    avio_r64 = be ? avio_rb64 : avio_rl64;
    avio_r32 = be ? avio_rb32 : avio_rl32;
    avio_seek(pb, 0, SEEK_SET);
    table_offset = avio_r64(pb);
    if (table_offset > 0x20000 || table_offset < 0x1c)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 4);
    raud->block_size = avio_r32(pb);
    if (raud->block_size <= 0)
        return AVERROR_INVALIDDATA;

    nb_streams = avio_r32(pb);
    if (nb_streams < 0)
        return AVERROR_INVALIDDATA;

    if (nb_streams == 0) {
        avio_seek(pb, 20, SEEK_SET);
        offset = avio_r64(pb);
        avio_seek(pb, 36, SEEK_SET);
        nb_streams = avio_r32(pb);
        if (nb_streams <= 0 || nb_streams > FF_ARRAY_ELEMS(raud->nb_subblocks))
            return AVERROR_INVALIDDATA;

        avio_skip(pb, 4);
        start = avio_r32(pb);
        offset += nb_streams * 16LL;
        avio_seek(pb, offset + 16, SEEK_SET);
        duration = avio_r32(pb);
        avio_skip(pb, 8);
        codec = avio_r32(pb);
        avio_seek(pb, table_offset + 4, SEEK_SET);
        rate = avio_r32(pb);
    } else {
        return AVERROR_PATCHWELCOME;
    }

    switch (codec) {
    case 1:
        codec = be ? AV_CODEC_ID_PCM_S16BE : AV_CODEC_ID_PCM_S16LE;
        align = 2;
        break;
    case 0:
        codec = AV_CODEC_ID_XMA1;
        align = 0x800;
        break;
    case 256:
        codec = AV_CODEC_ID_MP3;
        align = 1024;
        break;
    case 1024:
        codec = AV_CODEC_ID_ADPCM_IMA;
        align = 1;
        break;
    default:
        avpriv_request_sample(s, "codec %X", codec);
        return AVERROR_INVALIDDATA;
    }

    if (rate <= 0 || align <= 0)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < nb_streams; n++) {
        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->duration = duration;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->ch_layout.nb_channels = 1;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = align;

        if (codec == AV_CODEC_ID_MP3) {
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        } else if (codec == AV_CODEC_ID_XMA1) {
            int ret = ff_alloc_extradata(st->codecpar, 8 + 20);
            if (ret < 0)
                return ret;
            memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
            st->codecpar->extradata[4] = 1;
            st->codecpar->extradata[8+17] = 1;
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        }

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    avio_seek(pb, start, SEEK_SET);

    if (s->nb_streams > 3)
        raud->header_size = 0x1000;
    else
        raud->header_size = 0x800;

    if (!raud->pkt)
        raud->pkt = av_packet_alloc();
    if (!raud->pkt)
        return AVERROR(ENOMEM);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    RAUDContext *raud = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (raud->pkt->size == 0) {
        const int be = raud->be;

        pos = avio_tell(pb);

        raud->stream_index = 0;
        raud->subblock_index = 0;

        ret = av_get_packet(pb, raud->pkt, raud->block_size);
        if (ret < 0)
            return ret;

        if (raud->pkt->size <= raud->header_size)
            return AVERROR_INVALIDDATA;

        raud->block_subblocks = (raud->pkt->size - raud->header_size) / SUBBLOCK_SIZE;
        if (raud->block_subblocks <= 0)
            return AVERROR_INVALIDDATA;

        int64_t seek_info_offset = be ? AV_RB64(raud->pkt->data) : AV_RL64(raud->pkt->data);
        int64_t seek_info_size = be ? AV_RB64(raud->pkt->data + 8) : AV_RL64(raud->pkt->data + 8);
        int64_t seek_info_entry_size = (seek_info_size - seek_info_offset) / s->nb_streams;

        if (seek_info_offset <= 0 || seek_info_size <= 0)
            return AVERROR_INVALIDDATA;

        if (seek_info_offset >= raud->pkt->size ||
            seek_info_size >= raud->pkt->size)
            return AVERROR_INVALIDDATA;

        memset(raud->nb_subblocks, 0, sizeof(raud->nb_subblocks));
        memset(raud->subblock_offset, 0, sizeof(raud->subblock_offset));

        for (int n = 0; n < s->nb_streams; n++) {
            if (seek_info_offset + seek_info_entry_size * n + 16 > raud->pkt->size)
                return AVERROR_INVALIDDATA;

            const int stream_subblock_offset = be ? AV_RB32(raud->pkt->data + seek_info_offset + seek_info_entry_size * n) : AV_RL32(raud->pkt->data + seek_info_offset + seek_info_entry_size * n);
            const int nb_stream_subblocks = be ? AV_RB32(raud->pkt->data + seek_info_offset + 4 + seek_info_entry_size * n) : AV_RL32(raud->pkt->data + seek_info_offset + 4 + seek_info_entry_size * n);

            raud->subblock_offset[n] = stream_subblock_offset;
            raud->nb_subblocks[n] = nb_stream_subblocks;
        }

        raud->pkt->pos = pos;

        if (raud->subblock_offset[raud->stream_index] + raud->subblock_index >= raud->block_subblocks)
            return AVERROR_INVALIDDATA;

        ret = av_new_packet(pkt, SUBBLOCK_SIZE);
        if (ret < 0)
            return ret;

        memcpy(pkt->data, raud->pkt->data + raud->header_size + SUBBLOCK_SIZE * (raud->subblock_offset[raud->stream_index] + raud->subblock_index), SUBBLOCK_SIZE);
        pkt->stream_index = raud->stream_index;
        raud->subblock_index++;
        if (raud->subblock_index >= raud->nb_subblocks[raud->stream_index]) {
            raud->subblock_index = 0;
            raud->stream_index++;
            if (raud->stream_index >= s->nb_streams) {
                av_packet_unref(raud->pkt);
                raud->subblock_index = 0;
                raud->stream_index = 0;
            }
        }
    } else {
        pos = raud->pkt->pos;

        if (raud->subblock_offset[raud->stream_index] + raud->subblock_index >= raud->block_subblocks)
            return AVERROR_INVALIDDATA;

        ret = av_new_packet(pkt, SUBBLOCK_SIZE);
        if (ret < 0)
            return ret;

        memcpy(pkt->data, raud->pkt->data + raud->header_size + SUBBLOCK_SIZE * (raud->subblock_offset[raud->stream_index] + raud->subblock_index), SUBBLOCK_SIZE);
        pkt->stream_index = raud->stream_index;
        raud->subblock_index++;
        if (raud->subblock_index >= raud->nb_subblocks[raud->stream_index]) {
            raud->subblock_index = 0;
            raud->stream_index++;
            if (raud->stream_index >= s->nb_streams) {
                av_packet_unref(raud->pkt);
                raud->subblock_index = 0;
                raud->stream_index = 0;
            }
        }
    }

    pkt->pos = pos;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t timestamp, int flags)
{
    RAUDContext *raud = s->priv_data;

    raud->stream_index = 0;
    raud->subblock_index = 0;
    av_packet_unref(raud->pkt);

    return -1;
}

static int read_close(AVFormatContext *s)
{
    RAUDContext *raud = s->priv_data;

    av_packet_free(&raud->pkt);

    return 0;
}

const FFInputFormat ff_raud_demuxer = {
    .p.name         = "raud",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Rockstar Games AUD"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(RAUDContext),
    .p.extensions   = "ivaud",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
