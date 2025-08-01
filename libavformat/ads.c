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

#include "libavutil/channel_layout.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int ads_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "SShd", 4) ||
        memcmp(p->buf+32, "SSbd", 4))
        return 0;

    return AVPROBE_SCORE_MAX / 3 * 2;
}

static int ads_read_header(AVFormatContext *s)
{
    int align, codec;
    AVStream *st;
    int64_t size;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(s->pb, 8);
    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    codec                  = avio_rl32(s->pb);
    st->codecpar->sample_rate = avio_rl32(s->pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    st->codecpar->ch_layout.nb_channels = avio_rl32(s->pb);
    if (st->codecpar->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;
    align                  = avio_rl32(s->pb);
    if (align <= 0 || align > INT_MAX / st->codecpar->ch_layout.nb_channels)
        return AVERROR_INVALIDDATA;

    if (codec == 1)
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE_PLANAR;
    else
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;

    st->codecpar->block_align = st->codecpar->ch_layout.nb_channels * align;
    avio_skip(s->pb, 12);
    size = avio_rl32(s->pb);
    if (st->codecpar->codec_id == AV_CODEC_ID_ADPCM_PSX && size >= 0x40)
        st->duration = (size - 0x40) / 16 / st->codecpar->ch_layout.nb_channels * 28;
    st->start_time = 0;
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int ads_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    int ret;

    ret = av_get_packet(s->pb, pkt, par->block_align);
    pkt->stream_index = 0;
    return ret;
}

const FFInputFormat ff_ads_demuxer = {
    .p.name         = "ads",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony PS2 ADS"),
    .p.extensions   = "ads,ss2",
    .read_probe     = ads_probe,
    .read_header    = ads_read_header,
    .read_packet    = ads_read_packet,
};
