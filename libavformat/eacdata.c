/*
 * Electronic Arts .cdata file Demuxer
 * Copyright (c) 2007 Peter Ross
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

/**
 * @file
 * Electronic Arts cdata Format Demuxer
 * by Peter Ross (pross@xvid.org)
 *
 * Technical details here:
 *  http://wiki.multimedia.cx/index.php?title=EA_Command_And_Conquer_3_Audio_Codec
 */

#include "libavutil/channel_layout.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

#include "libavutil/channel_layout.h"

static int cdata_probe(const AVProbeData *p)
{
    const uint8_t *b = p->buf;

    if (b[0] == 0x04 && (b[1] == 0x00 || b[1] == 0x04 || b[1] == 0x0C || b[1] == 0x14))
        return AVPROBE_SCORE_MAX/8;
    return 0;
}

static int cdata_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    unsigned int sample_rate, header;
    AVStream *st;
    AVChannelLayout channel_layout;

    header = avio_rb16(pb);
    switch (header) {
    case 0x0400:
        channel_layout = (AVChannelLayout){ .nb_channels = 1, .order = AV_CHANNEL_ORDER_UNSPEC };
        break;
    case 0x0404:
        channel_layout  = (AVChannelLayout){ .nb_channels = 2, .order = AV_CHANNEL_ORDER_UNSPEC };
        break;
    case 0x040C:
        channel_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_QUAD;         break;
    case 0x0414:
        channel_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_5POINT1_BACK; break;
    default:
        av_log(s, AV_LOG_INFO, "unknown header 0x%04x\n", header);
        return AVERROR_INVALIDDATA;
    };

    sample_rate = avio_rb16(pb);
    avio_skip(pb, (avio_r8(pb) & 0x20) ? 15 : 11);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_tag = 0; /* no fourcc */
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_EA_XAS;
    st->codecpar->ch_layout = channel_layout;
    st->codecpar->sample_rate = sample_rate;
    avpriv_set_pts_info(st, 64, 1, sample_rate);

    return 0;
}

static int cdata_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    const int packet_size = 76 * par->ch_layout.nb_channels;
    int ret = av_get_packet(s->pb, pkt, packet_size);

    if (ret < 0)
        return ret;
    pkt->duration = 128;

    return 0;
}

const FFInputFormat ff_ea_cdata_demuxer = {
    .p.name         = "ea_cdata",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Electronic Arts cdata"),
    .p.extensions   = "cdata",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = cdata_probe,
    .read_header    = cdata_read_header,
    .read_packet    = cdata_read_packet,
};
