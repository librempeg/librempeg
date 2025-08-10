/*
 * SADL demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('s','a','d','l'))
        return 0;

    if (p->buf_size < 33)
        return 0;

    if (p->buf[32] == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int flags, loop, nb_channels;
    AVIOContext *pb = s->pb;
    int64_t start_offset;
    int64_t loop_start;
    AVStream *st;

    avio_skip(pb, 0x31);
    loop = avio_r8(pb);
    nb_channels = avio_r8(pb);
    flags = avio_r8(pb);
    if (nb_channels == 0)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 20);
    start_offset = avio_rl32(pb);
    avio_skip(pb, 8);
    loop_start = avio_rl32(pb) ;

    switch (flags & 0xf0) {
    case 0xb0:
        break;
    default:
        avpriv_request_sample(s, "flags %d", flags);
        return AVERROR_PATCHWELCOME;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    if (loop)
        av_dict_set_int(&st->metadata, "loop_start", loop_start / nb_channels / 16 * 30, 0);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PROCYON;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = ((flags & 6) == 4) ? 32728 : 16364;
    st->codecpar->block_align = 0x10 * st->codecpar->ch_layout.nb_channels;
    st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                    st->codecpar->sample_rate / 30;


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

const FFInputFormat ff_sadl_demuxer = {
    .p.name         = "sadl",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Procyon DS SADL)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "sad",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
