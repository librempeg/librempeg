/*
 * NWA demuxer
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
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

typedef struct NWADemuxContext {
    int block_size;
    int current_block;
} NWADemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 32)
        return 0;
    if (AV_RL16(p->buf) != 1 &&
        AV_RL16(p->buf) != 2)
        return 0;
    if (AV_RL16(p->buf+2) != 8 &&
        AV_RL16(p->buf+2) != 16)
        return 0;
    if ((int32_t)AV_RL32(p->buf+4) <= 0)
        return 0;
    if ((int32_t)AV_RL32(p->buf+8) < -1 ||
        (int32_t)AV_RL32(p->buf+8) > 5)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int ret, bps, channels, rate, compression, blocks, bsize;
    NWADemuxContext *nwa = s->priv_data;
    int64_t duration, next_pos;
    AVIOContext *pb = s->pb;
    AVStream *st;

    channels = avio_rl16(pb);
    bps = avio_rl16(pb);
    rate = avio_rl32(pb);
    compression = avio_rl32(pb);
    avio_skip(pb, 4);
    blocks = avio_rl32(pb);
    avio_skip(pb, 8);
    duration = avio_rl32(pb);
    bsize = avio_rl32(pb);
    if ((bps != 8 && bps != 16) || channels == 0 || rate <= 0 ||
        compression < -1 || compression > 5)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_seek(pb, 0x2c, SEEK_SET);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    switch (compression) {
    case -1:
        st->codecpar->codec_id = (bps == 8) ? AV_CODEC_ID_PCM_S8 : AV_CODEC_ID_PCM_S16LE;
        break;
    default:
        st->codecpar->codec_id = AV_CODEC_ID_NWA;
        ret = ff_alloc_extradata(st->codecpar, 8);
        if (ret < 0)
            return ret;
        AV_WL32(st->codecpar->extradata, compression);
        AV_WL32(st->codecpar->extradata+4, bsize);

        nwa->block_size = bsize;
        nwa->current_block = 0;

        next_pos = avio_rl32(pb);
        for (int block = 0; block < blocks; block++) {
            int64_t timestamp = block;
            int64_t pos = next_pos;

            if (avio_feof(pb))
                return AVERROR_INVALIDDATA;

            if (block < blocks-1)
                next_pos = avio_rl32(pb);
            else
                next_pos = avio_size(pb);

            if ((ret = av_add_index_entry(st, pos, timestamp * bsize / (bps/8), next_pos - pos, 0,
                                          AVINDEX_KEYFRAME)) < 0)
                return ret;
        }
        break;
    }

    st->codecpar->ch_layout.nb_channels = channels;
    st->start_time = 0;
    st->duration = duration / channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->bits_per_coded_sample = bps;
    st->codecpar->block_align = 1024 * FFMAX(1, bps/8) * st->codecpar->ch_layout.nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    NWADemuxContext *nwa = s->priv_data;
    AVStream *st = s->streams[0];
    AVIOContext *pb = s->pb;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (st->codecpar->codec_id != AV_CODEC_ID_NWA) {
        ret = av_get_packet(pb, pkt, st->codecpar->block_align);
    } else {
        FFStream *const sti = ffstream(st);
        int pkt_size;

        if (nwa->current_block >= sti->nb_index_entries)
            return AVERROR_EOF;

        pkt_size = sti->index_entries[nwa->current_block].size;
        ret = av_get_packet(pb, pkt, pkt_size);
        pkt->pts = nwa->current_block * nwa->block_size/2;
        pkt->duration = nwa->block_size/2;
        nwa->current_block++;
    }

    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index, int64_t timestamp, int flags)
{
    NWADemuxContext *nwa = s->priv_data;
    AVStream *st = s->streams[0];
    FFStream *const sti = ffstream(st);
    int64_t ret;
    int index;

    if (st->codecpar->codec_id != AV_CODEC_ID_NWA)
        return ff_pcm_read_seek(s, stream_index, timestamp, flags);

    if (timestamp < 0)
        return -1;

    index = av_index_search_timestamp(st, timestamp, flags);
    if (index < 0 || index >= sti->nb_index_entries)
        return -1;

    ret = avio_seek(s->pb, sti->index_entries[index].pos, SEEK_SET);
    if (ret < 0)
        return -1;
    nwa->current_block = FFMAX(0, index);

    return 0;
}

const FFInputFormat ff_nwa_demuxer = {
    .p.name         = "nwa",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Visual Arts NWA"),
    .p.extensions   = "nwa",
    .priv_data_size = sizeof(NWADemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
