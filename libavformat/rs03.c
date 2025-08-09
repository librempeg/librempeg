/*
 * RS03 demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('R','S','\x0','\x3'))
        return 0;
    if (AV_RB32(p->buf+4) < 1 ||
        AV_RB32(p->buf+4) > 2)
        return 0;
    if (p->buf_size < 16)
        return 0;

    if (AV_RB32(p->buf+8) == 0)
        return 0;
    if (AV_RB32(p->buf+12) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    int ret;

    avio_skip(pb, 4);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = avio_rb32(pb);
    if (st->codecpar->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;
    st->start_time = 0;
    st->duration = avio_rb32(pb);
    st->codecpar->sample_rate = avio_rb32(pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    st->codecpar->block_align = 0x8f00 * st->codecpar->ch_layout.nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x20, SEEK_SET);
    if ((ret = ff_get_extradata(s, st->codecpar, pb, 32 * st->codecpar->ch_layout.nb_channels)) < 0)
        return ret;

    avio_seek(pb, 0x60, SEEK_SET);

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

const FFInputFormat ff_rs03_demuxer = {
    .p.name         = "rs03",
    .p.long_name    = NULL_IF_CONFIG_SMALL("RS03 (Metroid Prime GameCube)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "dsp",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
