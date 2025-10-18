/*
 * ASTB demuxer
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

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) != MKTAG('A','S','T','B'))
        return 0;

    if (AV_RB16(p->buf + 0x30) != 0x165)
        return 0;

    if (AV_RB32(p->buf + 0x10) == 0x0)
        return 0;

    return AVPROBE_SCORE_MAX / 3 * 2;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int64_t start_offset;
    int ret, channels = 0;
    int streams;
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 16);
    start_offset = avio_rb32(pb);
    avio_skip(pb, 32);
    if (start_offset <= avio_tell(pb) + 6)
        return AVERROR_INVALIDDATA;

    ret = ff_get_extradata(s, st->codecpar, pb, start_offset - avio_tell(pb));
    if (ret < 0)
        return ret;

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_XMA1;
    st->codecpar->block_align = 2048;

    streams = AV_RB16(st->codecpar->extradata + 4);
    if (streams <= 0)
        return AVERROR_INVALIDDATA;

    if (st->codecpar->extradata_size < (streams - 1) * 20 + 25)
        return AVERROR_INVALIDDATA;

    for (int i = 0; i < streams; i++)
        channels += st->codecpar->extradata[20 * i + 25];
    st->codecpar->extradata_size = 8 + 20 * streams;

    st->codecpar->ch_layout.nb_channels = channels;
    if (st->codecpar->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;

    st->codecpar->sample_rate = AV_RB32(st->codecpar->extradata + 12);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    AV_WL16(st->codecpar->extradata + 4, streams);
    st->start_time = 0;

    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVStream *st = s->streams[0];
    AVCodecParameters *par = st->codecpar;
    AVIOContext *pb = s->pb;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    ret = av_get_packet(pb, pkt, par->block_align);
    pkt->stream_index = 0;
    return ret;
}

const FFInputFormat ff_astb_demuxer = {
    .p.name         = "astb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Capcom ASTB (Audio Stream)"),
    .p.extensions   = "ast",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
