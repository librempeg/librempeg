/*
 * PCM Success demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('P','C','M',' '))
        return 0;

    if (p->buf_size < 20)
        return 0;

    if (AV_RL32(p->buf+4) != 0x00010000)
        return 0;

    if (AV_RL32(p->buf+8) <= 0)
        return 0;

    if (AV_RL32(p->buf+12) <= 0)
        return 0;

    if (AV_RL32(p->buf+16) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 12);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->sample_rate = avio_rl32(pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    st->codecpar->ch_layout.nb_channels = avio_rl32(pb);
    if (st->codecpar->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;
    st->codecpar->block_align = 0x800 * st->codecpar->ch_layout.nb_channels;
    st->codecpar->bit_rate = (int64_t)st->codecpar->sample_rate * st->codecpar->ch_layout.nb_channels * 16 * 8LL / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x800, SEEK_SET);

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

const FFInputFormat ff_spcm_demuxer = {
    .p.name         = "spcm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Success PCM"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "pcm",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
