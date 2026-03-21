/*
 * NXA1 demuxer
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
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct NXA1DemuxContext {
    int64_t data_end;
} NXA1DemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('N','X','A','1'))
        return 0;

    if (p->buf_size < 20)
        return 0;
    if (AV_RL32(p->buf+4) != 1 && AV_RL32(p->buf+4) != 2)
        return 0;
    if (AV_RL32(p->buf+4) == 2 && AV_RL16(p->buf+18) == 0)
        return 0;
    if ((int)AV_RL32(p->buf+12) <= 0)
        return 0;
    if ((int)AV_RL16(p->buf+16) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int skip, ret, nb_channels, rate, frame_size, type;
    int64_t data_size, duration, loop_start, loop_end;
    NXA1DemuxContext *n = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    type = avio_rl32(pb);
    data_size = avio_rl32(pb);
    rate = avio_rl32(pb);
    nb_channels = avio_rl16(pb);
    frame_size = avio_rl16(pb);
    avio_skip(pb, 2);
    skip = avio_rl16(pb);
    duration = avio_rl32(pb);
    loop_start = avio_rl32(pb);
    loop_end = avio_rl32(pb);
    if (nb_channels <= 0 || rate <= 0 || (type == 2 && frame_size <= 0))
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    n->data_end = data_size;

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_OPUS;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = (type == 2) ? frame_size : 0;

    ret = ff_alloc_extradata(st->codecpar, 19 + (2 + nb_channels) * (nb_channels > 2));
    if (ret < 0)
        return ret;
    memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

    memcpy(st->codecpar->extradata, "OpusHead", 8);
    st->codecpar->extradata[8] = 1;
    st->codecpar->extradata[9] = nb_channels;
    AV_WL16(st->codecpar->extradata + 10, skip);
    AV_WL32(st->codecpar->extradata + 12, 48000);

    ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if (loop_start > 0) {
        av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);
        av_dict_set_int(&st->metadata, "loop_end", loop_end, 0);
    }

    avio_seek(pb, 0x30, SEEK_SET);
    n->data_end += avio_tell(pb);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    NXA1DemuxContext *n = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    uint32_t size;
    int ret;

    if (pos >= n->data_end)
        return AVERROR_EOF;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (s->streams[0]->codecpar->block_align > 0)
        return av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);

    size = avio_rb32(pb);
    if (size < 2)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 4);
    ret = av_get_packet(pb, pkt, size);
    pkt->pos = pos;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_nxa1_demuxer = {
    .p.name         = "nxa1",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Entergram NXA1"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "nxa",
    .priv_data_size = sizeof(NXA1DemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
