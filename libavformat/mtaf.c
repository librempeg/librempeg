/*
 * MTAF demuxer
 * Copyright (c) 2016 Paul B Mahol
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

static int mtaf_probe(const AVProbeData *p)
{
    if (p->buf_size < 0x800)
        return 0;

    if (AV_RL32(p->buf) != MKTAG('M','T','A','F') ||
        AV_RL32(p->buf + 0x40) != MKTAG('H','E','A','D') ||
        AV_RL32(p->buf + 0x7f8) != MKTAG('D','A','T','A') ||
        AV_RL32(p->buf + 0x7fc) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int mtaf_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int stream_count;
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 0x5c);
    st->duration = avio_rl32(pb);
    avio_skip(pb, 1);
    stream_count = avio_r8(pb);
    if (!stream_count)
        return AVERROR_INVALIDDATA;

    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_MTAF;
    st->codecpar->ch_layout.nb_channels = 2 * stream_count;
    st->codecpar->sample_rate = 48000;
    st->codecpar->block_align = 0x110 * st->codecpar->ch_layout.nb_channels / 2;
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x7f8, SEEK_SET);
    if (avio_rl32(pb) != MKTAG('D','A','T','A'))
        return AVERROR_INVALIDDATA;
    if (avio_rl32(pb) == 0)
        return AVERROR_INVALIDDATA;

    return 0;
}

static int mtaf_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    AVIOContext *pb = s->pb;

    return av_get_packet(pb, pkt, par->block_align);
}

const FFInputFormat ff_mtaf_demuxer = {
    .p.name         = "mtaf",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Konami PS2 MTAF"),
    .p.extensions   = "mtaf",
    .read_probe     = mtaf_probe,
    .read_header    = mtaf_read_header,
    .read_packet    = mtaf_read_packet,
};
