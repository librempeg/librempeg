/*
 * MPC3 demuxer
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
    if (p->buf_size < 24)
        return 0;
    if (AV_RB32(p->buf) != MKBETAG('M','P','C','3'))
        return 0;
    if (AV_RB32(p->buf+4) != 0x00011400)
        return 0;
    if ((int32_t)AV_RL32(p->buf+8) <= 0)
        return 0;
    if ((int32_t)AV_RL32(p->buf+12) <= 0)
        return 0;
    if ((int32_t)AV_RL32(p->buf+20) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, align;
    AVIOContext *pb = s->pb;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 8);
    channels = avio_rl32(pb);
    if (channels > INT_MAX/4)
        return AVERROR_INVALIDDATA;
    rate = avio_rl32(pb);
    duration = avio_rl32(pb);
    duration *= 10;
    align = avio_rl32(pb);
    if (align <= 0 || align > (INT_MAX-4)/(4*channels))
        return AVERROR_INVALIDDATA;
    align = align * 4 * channels + 4;
    if (align <= 0 || rate <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_MPC3;
    st->codecpar->ch_layout.nb_channels = channels;
    st->start_time = 0;
    st->duration = duration;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 28, SEEK_SET);

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

const FFInputFormat ff_mpc3_demuxer = {
    .p.name         = "mpc3",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Paradigm Entertainment MPC3"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "mc3",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
