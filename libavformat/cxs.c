/*
 * CXS demuxer
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

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('C','X','S',' '))
        return 0;

    if (p->buf_size < 20)
        return 0;
    if ((int)AV_RB32(p->buf + 8) <= 0)
        return 0;
    if ((int)AV_RB32(p->buf + 12) <= 0)
        return 0;
    if (AV_RB32(p->buf + 16) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t start_offset, duration;
    AVIOContext *pb = s->pb;
    int ret, rate, channels;
    AVStream *st;

    avio_skip(pb, 4);
    start_offset = avio_rb32(pb);
    rate = avio_rb32(pb);
    channels = avio_rb32(pb);
    duration = avio_rb32(pb);
    if (rate <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, 0x28, SEEK_SET);
    start_offset += avio_rb32(pb);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_XMA2;
    st->codecpar->block_align = 0x800;
    st->codecpar->sample_rate = rate;

    ret = ff_alloc_extradata(st->codecpar, 34);
    if (ret < 0)
        return ret;
    memset(st->codecpar->extradata, 0, 34);
    AV_WL16(st->codecpar->extradata, 1);
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_cxs_demuxer = {
    .p.name         = "cxs",
    .p.long_name    = NULL_IF_CONFIG_SMALL("tri-Crescendo CXS"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "cxs",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
