/*
 * DC IDVI demuxer
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

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int dcidvi_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('I','D','V','I'))
        return 0;

    if ((int32_t)AV_RL32(p->buf + 4) <= 0)
        return 0;

    if ((int32_t)AV_RL32(p->buf + 8) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int dcidvi_read_header(AVFormatContext *s)
{
    int rate, nb_channels, ret;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    nb_channels = avio_rl32(pb);
    rate = avio_rl32(pb);
    if (nb_channels <= 0 || rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_WS;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 0x400 * st->codecpar->ch_layout.nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    ret = ff_alloc_extradata(st->codecpar, 2);
    if (ret < 0)
        return ret;
    AV_WL16(st->codecpar->extradata, 4);

    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

static int dcidvi_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, par->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_dcidvi_demuxer = {
    .p.name         = "dcidvi",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Capcom DC IDVI"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "idvi",
    .read_probe     = dcidvi_probe,
    .read_header    = dcidvi_read_header,
    .read_packet    = dcidvi_read_packet,
};
