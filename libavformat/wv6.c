/*
 * WV6 demuxer
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

#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 81)
        return 0;
    if (memcmp(p->buf + 0x2c, "WV6 IMA_ADPCM COMPRESSED 16 BIT AUDIO", 37))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    char title[0x1d] = { 0 };
    int rate, ret;
    AVStream *st;

    avio_skip(pb, 4);
    if ((ret = avio_get_str(pb, 0x1c, title, sizeof(title))) < 0)
        return ret;
    if (title[0] != '\0')
        av_dict_set(&s->metadata, "title", title, 0);

    avio_seek(pb, 0x60, SEEK_SET);
    rate = avio_rl32(pb);
    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_WV6;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 2048;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x8c, SEEK_SET);

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

const FFInputFormat ff_wv6_demuxer = {
    .p.name         = "wv6",
    .p.long_name    = NULL_IF_CONFIG_SMALL("WV6 Audio"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "wv6",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
