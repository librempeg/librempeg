/*
 * PASX demuxer
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
    int offset;

    if (AV_RB32(p->buf) != MKBETAG('P','A','S','X'))
        return 0;

    if (p->buf_size < 20)
        return 0;
    offset = AV_RB32(p->buf+16);

    if (p->buf_size < offset + 8)
        return 0;

    if (AV_RB16(p->buf + offset) != 0x166)
        return 0;

    if (AV_RB16(p->buf + offset+2) == 0)
        return 0;

    if (AV_RB32(p->buf + offset+6) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int64_t offset, start;
    AVStream *st;
    int ret;

    avio_skip(pb, 16);
    offset = avio_rb32(pb);
    avio_skip(pb, 4);
    start = avio_rb32(pb);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_seek(pb, offset, SEEK_SET);

    if (avio_rb16(pb) != 0x166)
        return AVERROR_INVALIDDATA;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_XMA2;
    st->codecpar->ch_layout.nb_channels = avio_rb16(pb);
    if (st->codecpar->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;
    st->codecpar->sample_rate = avio_rb32(pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 16);
    st->duration = avio_rb32(pb);
    st->codecpar->block_align = 0x800;

    ret = ff_alloc_extradata(st->codecpar, 34);
    if (ret < 0)
        return ret;
    memset(st->codecpar->extradata, 0, 34);
    AV_WL16(st->codecpar->extradata, 1);
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

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

const FFInputFormat ff_pasx_demuxer = {
    .p.name         = "pasx",
    .p.long_name    = NULL_IF_CONFIG_SMALL("PASX (Premium Agency Sound)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "past,sgb",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
