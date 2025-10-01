/*
 * NWA demuxer
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
    if (p->buf_size < 32)
        return 0;
    if (AV_RL16(p->buf) != 1 &&
        AV_RL16(p->buf) != 2)
        return 0;
    if (AV_RL16(p->buf+2) != 0 &&
        AV_RL16(p->buf+2) != 8 &&
        AV_RL16(p->buf+2) != 16)
        return 0;
    if ((int32_t)AV_RL32(p->buf+4) <= 0)
        return 0;
    if ((int32_t)AV_RL32(p->buf+8) < -1 ||
        (int32_t)AV_RL32(p->buf+8) > 5)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int bps, channels, rate, compression;
    AVIOContext *pb = s->pb;
    int64_t duration;
    AVStream *st;

    channels = avio_rl16(pb);
    bps = avio_rl16(pb);
    rate = avio_rl32(pb);
    compression = avio_rl32(pb);
    avio_skip(pb, 16);
    duration = avio_rl32(pb);
    if ((bps != 0 && bps != 8 && bps != 16) || channels == 0 || rate <= 0 ||
        compression < -1 || compression > 5)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    switch (compression) {
    case -1:
        st->codecpar->codec_id = (bps == 8) ? AV_CODEC_ID_PCM_S8 : AV_CODEC_ID_PCM_S16LE;
        break;
    default:
        break;
    }
    st->codecpar->ch_layout.nb_channels = channels;
    st->start_time = 0;
    st->duration = duration / channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 1024 * FFMAX(1, bps/8) * st->codecpar->ch_layout.nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x2c, SEEK_SET);

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

const FFInputFormat ff_nwa_demuxer = {
    .p.name         = "nwa",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Visual Arts NWA"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "nwa",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
