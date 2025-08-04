/*
 * DC KCEY demuxer
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

static int dckcey_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('K','C','E','Y'))
        return 0;

    if (AV_RB32(p->buf+4) != MKBETAG('C','O','M','P'))
        return 0;

    if ((int32_t)AV_RL32(p->buf + 8) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int dckcey_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int nb_channels;
    int64_t start;
    AVStream *st;

    avio_skip(pb, 8);
    nb_channels = avio_rb32(pb);
    if (nb_channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = avio_rb32(pb);
    start = avio_rb32(pb);
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_APC;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = 37800;
    st->codecpar->block_align = 0x200 * st->codecpar->ch_layout.nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

static int dckcey_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, par->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_dckcey_demuxer = {
    .p.name         = "dckcey",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Konami DC KCEY"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "kcey",
    .read_probe     = dckcey_probe,
    .read_header    = dckcey_read_header,
    .read_packet    = dckcey_read_packet,
};
