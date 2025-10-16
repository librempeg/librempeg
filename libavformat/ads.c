/*
 * ADS/SS2 demuxer
 * Copyright (c) 2015 Paul B Mahol
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
#include "libavutil/channel_layout.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 36)
        return 0;

    if (memcmp(p->buf, "SShd", 4) ||
        memcmp(p->buf+32, "SSbd", 4))
        return 0;

    if ((int)AV_RL32(p->buf + 12) <= 0)
        return 0;

    if ((int)AV_RL32(p->buf + 16) <= 0)
        return 0;

    if ((int)AV_RL32(p->buf + 20) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int align, codec, rate, channels;
    AVIOContext *pb = s->pb;
    AVStream *st;
    int64_t size;

    avio_skip(pb, 8);
    codec = avio_rl32(pb);
    rate = avio_rl32(pb);
    channels = avio_rl32(pb);
    align = avio_rl32(pb);
    if (rate <= 0 || channels <= 0 || align <= 0 || align > INT_MAX/channels)
        return AVERROR_INVALIDDATA;

    switch (codec) {
    case 1:
        codec = AV_CODEC_ID_PCM_S16LE_PLANAR;
        break;
    case 2:
    case 16:
        codec = AV_CODEC_ID_ADPCM_PSX;
        break;
    default:
        avpriv_request_sample(s, "codec %d", codec);
        return AVERROR_PATCHWELCOME;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = channels * align;
    avio_skip(pb, 12);
    size = avio_rl32(pb);
    if (st->codecpar->codec_id == AV_CODEC_ID_ADPCM_PSX && size >= 0x40)
        st->duration = (size - 0x40) / 16 / st->codecpar->ch_layout.nb_channels * 28;
    st->start_time = 0;
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, par->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_ads_demuxer = {
    .p.name         = "ads",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony PS2 ADS"),
    .p.extensions   = "ads,ss2",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
