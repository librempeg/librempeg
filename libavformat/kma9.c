/*
 * KMA9 demuxer
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
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('K','M','A','9'))
        return 0;

    if (p->buf_size < 0x38)
        return 0;
    if ((int)AV_RL32(p->buf + 8) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 16) <= 0)
        return 0;
    if (AV_RL32(p->buf + 20) == 0)
        return 0;
    if (AV_RL16(p->buf + 0x32) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 0x34) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, rate, nb_channels, nb_streams, block_size;
    int64_t start_offset, duration;
    uint32_t config, stream_size;
    AVIOContext *pb = s->pb;
    av_unused int delay;

    avio_skip(pb, 4);
    start_offset = avio_rl32(pb);
    nb_streams = avio_rl32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 4);
    block_size = avio_rl32(pb);
    stream_size = avio_rl32(pb);
    duration = avio_rl32(pb);
    avio_skip(pb, 4);
    delay = avio_rl32(pb);
    avio_seek(pb, 0x32, SEEK_SET);
    nb_channels = avio_rl16(pb);
    rate = avio_rl32(pb);
    avio_seek(pb, 0x5c, SEEK_SET);
    config = avio_rb32(pb);
    if (rate <= 0 || nb_channels <= 0 || stream_size == 0 || block_size <= 0)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < nb_streams; n++) {
        AVStream *st;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->duration = duration;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_ATRAC9;
        st->codecpar->ch_layout.nb_channels = nb_channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = block_size;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        ret = ff_alloc_extradata(st->codecpar, 12);
        if (ret < 0)
            return ret;
        memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
        AV_WB32(st->codecpar->extradata + 4, config);
    }

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    const int64_t offset = ffformatcontext(s)->data_offset;
    AVStream *st = s->streams[0];
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int ret, stream_index;

    if (avio_feof(pb))
        return AVERROR_EOF;

    stream_index = ((pos - offset) / st->codecpar->block_align) % s->nb_streams;
    ret = av_get_packet(pb, pkt, st->codecpar->block_align);
    pkt->pos = pos;
    pkt->stream_index = stream_index;
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->flags |= AV_PKT_FLAG_KEY;

    return ret;
}

const FFInputFormat ff_kma9_demuxer = {
    .p.name         = "kma9",
    .p.long_name    = NULL_IF_CONFIG_SMALL("KMA9 (Koei Tecmo)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "km9",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
