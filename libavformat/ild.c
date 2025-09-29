/*
 * ILD demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('I','L','D','\x0'))
        return 0;
    if ((int)AV_RL32(p->buf + 4) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 32) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 40) <= 0)
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, align;
    AVIOContext *pb = s->pb;
    int64_t start_offset;
    AVStream *st;

    avio_skip(pb, 4);
    channels = avio_rl32(pb);
    start_offset = avio_rl32(pb);
    avio_skip(pb, 20);
    align = avio_rl32(pb);
    avio_skip(pb, 4);
    rate = avio_rl32(pb);
    if (align <= 0 || rate <= 0 || channels <= 0 || channels > INT_MAX/align)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align * channels;
    st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                    st->codecpar->sample_rate / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

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

const FFInputFormat ff_ild_demuxer = {
    .p.name         = "ild",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Tose ILD"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "ild",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
