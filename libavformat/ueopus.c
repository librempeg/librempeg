/*
 * UEOPUS demuxer
 *
 * Copyright (c) 2026 Paul B Mahol
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
#include "avio_internal.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "UEOPUS\0\0", 8))
        return 0;

    if (p->buf_size < 16)
        return 0;
    if (p->buf[9] == 0)
        return 0;
    if ((int)AV_RL32(p->buf+10) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, rate, nb_channels, skip, skip_start, skip_size;
    AVIOContext *pb = s->pb;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 9);
    nb_channels = avio_r8(pb);
    rate = avio_rl32(pb);
    avio_skip(pb, 4);
    duration = avio_rl32(pb);
    avio_skip(pb, 8);
    skip_start = avio_rl32(pb);
    skip_size = avio_rl32(pb);
    skip = skip_start + skip_size;
    if (rate <= 0 || nb_channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_OPUS;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;

    ret = ff_alloc_extradata(st->codecpar, 19 + 2 + nb_channels);
    if (ret < 0)
        return ret;
    memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

    memcpy(st->codecpar->extradata, "OpusHead", 8);
    st->codecpar->extradata[8] = 1;
    st->codecpar->extradata[9] = nb_channels;
    AV_WL16(st->codecpar->extradata + 10, skip);
    AV_WL32(st->codecpar->extradata + 12, 48000);

    avio_seek(pb, 42, SEEK_SET);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    uint32_t chunk_id;
    int ret, pkt_size;

    if (avio_feof(pb))
        return AVERROR_EOF;

    ret = ffio_ensure_seekback(pb, 4);
    if (ret < 0)
        return ret;

    chunk_id = avio_rb32(pb);
    if (chunk_id == AV_RB32("SEEK")) {
        avio_skip(pb, 7);
        avio_skip(pb, avio_rl32(pb) * 2LL);
    } else {
        avio_seek(pb, -4, SEEK_CUR);
    }
    pkt_size = avio_rl16(pb);
    ret = av_get_packet(pb, pkt, pkt_size);
    pkt->pos = pos;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_ueopus_demuxer = {
    .p.name         = "ueopus",
    .p.long_name    = NULL_IF_CONFIG_SMALL("UE5 Opus"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "ueopus",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
