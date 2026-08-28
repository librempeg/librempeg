/*
 * Melbourne House games VS demuxer
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

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != 0xC8000000)
        return 0;

    if (p->buf_size < 12)
        return 0;
    if ((int)AV_RL32(p->buf+4) <= 0)
        return 0;
    if (AV_RL32(p->buf+8) != 0x1000)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    int rate;

    avio_skip(pb, 4);
    rate = avio_rl32(pb);
    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->ch_layout.nb_channels = 2;
    st->codecpar->sample_rate = rate;
    st->codecpar->bit_rate = 16LL * 2 * 8 * rate / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int size, ret;
    int64_t pos;

    pos = avio_tell(pb);
    size = avio_rl32(pb);
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (size <= 0 || size >= INT_MAX/2)
        return AVERROR_INVALIDDATA;

    ret = av_new_packet(pkt, size*2);
    if (ret < 0)
        return ret;

    avio_read(pb, pkt->data, size);
    avio_skip(pb, 4);
    avio_read(pb, pkt->data+size, size);

    pkt->stream_index = 0;
    pkt->pos = pos;

    return ret;
}

const FFInputFormat ff_vs_demuxer = {
    .p.name         = "vs",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Melbourne House VS"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "vs",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
