/*
 * STMA demuxer
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

#include "libavutil/bswap.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S','T','M','A'))
        return 0;

    if (p->buf_size < 24)
        return 0;

    if ((int)AV_RL32(p->buf+8) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf+12) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf+16) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf+20) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int interleave, bps, nb_channels, rate, ret;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 8);
    interleave = avio_rl32(pb);
    rate = avio_rl32(pb);
    bps = avio_rl32(pb);
    nb_channels = avio_rl32(pb);
    if (nb_channels <= 0 || rate <= 0 || bps <= 0 || interleave <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->bits_per_coded_sample = bps;
    switch (bps) {
    case 4:
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_WS;
        st->codecpar->block_align = 0x40 * (1 + (interleave == 0xc000)) * st->codecpar->ch_layout.nb_channels;
        ret = ff_alloc_extradata(st->codecpar, 2);
        if (ret < 0)
            return ret;
        AV_WL16(st->codecpar->extradata, 4);
        break;
    case 16:
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
        st->codecpar->block_align = 1024 * st->codecpar->ch_layout.nb_channels;
        break;
    default:
        avpriv_request_sample(s, "bps 0x%X", bps);
        return AVERROR_PATCHWELCOME;
    }

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

const FFInputFormat ff_stma_demuxer = {
    .p.name         = "stma",
    .p.long_name    = NULL_IF_CONFIG_SMALL("STMA Audio"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "lstm",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
