/*
 * DCNAOMI demuxer
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
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "ADPCM_v0\x0", 9))
        return 0;

    if (p->buf_size < 20)
        return 0;
    if (AV_RL32(p->buf + 16) == 0 ||
        AV_RL32(p->buf + 16) > INT_MAX/0x100)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int64_t data_size;
    AVStream *st;

    avio_skip(pb, 16);
    data_size = avio_rl32(pb);
    if (data_size > INT_MAX/0x100)
        return AVERROR_INVALIDDATA;
    data_size *= 0x100;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_AICA;
    st->codecpar->sample_rate = 44100;
    st->codecpar->block_align = data_size;
    st->codecpar->ch_layout.nb_channels = 2;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 64, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    const int size = s->streams[0]->codecpar->block_align;
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, size);
    if (pkt->size != size)
        return AVERROR_EOF;
    return ret;
}

const FFInputFormat ff_dcnaomi_demuxer = {
    .p.name         = "dcnaomi",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Dreamcast Naomi ADPCM"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
