/*
 * SVAG demuxer
 * Copyright (c) 2015 Paul B Mahol
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

static int svag_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S','v','a','g'))
        return 0;
    if ((int)AV_RL32(p->buf + 8) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 12) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 16) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int svag_read_header(AVFormatContext *s)
{
    unsigned size, align, rate, channels;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    size = avio_rl32(pb);
    rate = avio_rl32(pb);
    channels = avio_rl32(pb);
    align = avio_rl32(pb);
    if (rate <= 0 || channels <= 0 || channels > 8 || align <= 0 || align > INT_MAX/channels)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = channels;
    st->start_time = 0;
    st->duration           = size / (16 * st->codecpar->ch_layout.nb_channels) * 28;
    st->codecpar->block_align = align * st->codecpar->ch_layout.nb_channels;
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

static int svag_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_svag_demuxer = {
    .p.name         = "svag",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Konami PS2 SVAG"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "svag",
    .read_probe     = svag_probe,
    .read_header    = svag_read_header,
    .read_packet    = svag_read_packet,
};
