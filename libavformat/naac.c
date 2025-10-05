/*
 * NAAC demuxer
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
#include "libavutil/channel_layout.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('A','A','C',' '))
        return 0;

    if (AV_RL32(p->buf + 4) != 1)
        return 0;

    if ((int)AV_RL32(p->buf + 8) <= 0)
        return 0;

    if ((int)AV_RL32(p->buf + 12) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int channels, rate;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 8);
    channels = avio_rl32(pb);
    rate = avio_rl32(pb);
    duration = avio_rl32(pb);
    if (rate <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_AAC;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = 2048;
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    avio_seek(pb, 0x1000, SEEK_SET);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, par->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_naac_demuxer = {
    .p.name         = "naac",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Namco 3DS AAC"),
    .p.extensions   = "naac",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
