/*
 * PIFF demuxer
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

#include "libavutil/avstring.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 60)
        return 0;

    if (AV_RB32(p->buf)    == AV_RB32("PIFF") &&
        AV_RB32(p->buf+8)  == AV_RB32("TPCM") &&
        AV_RB32(p->buf+12) == AV_RB32("TADH") &&
        AV_RB32(p->buf+56) == AV_RB32("BODY"))
        return 50;
    return 0;
}

static int read_header(AVFormatContext *s)
{
    int rate, nb_channels;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 22);
    nb_channels = avio_rl16(pb);
    rate = avio_rl32(pb);
    avio_skip(pb, 0x38 - avio_tell(pb));
    if (avio_rb32(pb) != AV_RB32("BODY"))
        return AVERROR_INVALIDDATA;
    if (nb_channels <= 0 || rate <= 0 || nb_channels > INT_MAX/16)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_TANTALUS;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 16 * st->codecpar->ch_layout.nb_channels;

    st->duration = (avio_rl32(pb) * 2LL) / st->codecpar->ch_layout.nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret, size, key = avio_tell(s->pb) == 0x40;

    size = ff_pcm_default_packet_size(s->streams[0]->codecpar);
    if (size < 0)
        return size;

    ret = av_get_packet(s->pb, pkt, size);

    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->flags |= AV_PKT_FLAG_KEY * key;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_piff_demuxer = {
    .p.name         = "piff",
    .p.long_name    = NULL_IF_CONFIG_SMALL("PIFF Tantalus"),
    .p.extensions   = "tad",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
