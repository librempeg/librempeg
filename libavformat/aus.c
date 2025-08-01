/*
 * AUS demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('A','U','S',' '))
        return 0;

    if (p->buf_size < 20)
        return 0;

    if (AV_RL32(p->buf+8) == 0)
        return 0;

    if ((int32_t)AV_RL32(p->buf+12) <= 0)
        return 0;

    if ((int32_t)AV_RL32(p->buf+16) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int codec, nb_channels, rate;
    AVIOContext *pb = s->pb;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 6);
    codec = avio_rl16(pb);
    duration = avio_rl32(pb);
    nb_channels = avio_rl32(pb);
    rate = avio_rl32(pb);

    if (rate <= 0 || nb_channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;
    st->duration = duration;

    switch (codec) {
    case 2:
        st->codecpar->bits_per_coded_sample = 4;
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_XBOX;
        st->codecpar->block_align = 0x24 * st->codecpar->ch_layout.nb_channels;
        break;
    default:
        avpriv_request_sample(s, "codec 0x%X", codec);
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

const FFInputFormat ff_aus_demuxer = {
    .p.name         = "aus",
    .p.long_name    = NULL_IF_CONFIG_SMALL("AUS (Atomic Planet)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "aus",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
