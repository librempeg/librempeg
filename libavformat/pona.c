/*
 * PONA demuxer
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
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != 0x13020000)
        return 0;

    if (AV_RB16(p->buf+4) < 8)
        return 0;

    return AVPROBE_SCORE_MAX/3;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int64_t start;
    AVStream *st;

    avio_skip(pb, 4);
    start = avio_rb16(pb);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_SDX2_DPCM;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->sample_rate = 22050;
    st->codecpar->block_align = 1024 * st->codecpar->ch_layout.nb_channels;

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

const FFInputFormat ff_pona_demuxer = {
    .p.name         = "pona",
    .p.long_name    = NULL_IF_CONFIG_SMALL("3DO PONA"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "pona",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
