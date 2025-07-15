/*
 * LOPU demuxer
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

#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct LOPUDemuxContext {
    int64_t data_end;
} LOPUDemuxContext;

static int lopu_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) == MKTAG('L','O','P','U') &&
        AV_RL32(p->buf+8) > 0 &&
        AV_RL16(p->buf+12) > 0)
        return AVPROBE_SCORE_MAX/2;

    return 0;
}

static int lopu_read_header(AVFormatContext *s)
{
    LOPUDemuxContext *lc = s->priv_data;
    int ret, rate, nb_channels, skip;
    AVIOContext *pb = s->pb;
    int64_t start;
    AVStream *st;

    avio_skip(pb, 4);
    start = avio_rl32(pb);
    rate = avio_rl32(pb);
    nb_channels = avio_rl16(pb);
    if (rate <= 0 || nb_channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 6);
    st->start_time = 0;
    st->duration = avio_rl32(pb);
    avio_skip(pb, 12);
    skip = avio_rl16(pb);
    avio_skip(pb, 2);
    lc->data_end = start + avio_rl32(pb);
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_OPUS;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    ret = ff_alloc_extradata(st->codecpar, 19 + 2 + nb_channels);
    if (ret < 0)
        return ret;
    memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

    st->codecpar->extradata[9] = nb_channels;
    AV_WL16(st->codecpar->extradata + 10, skip);

    avio_seek(pb, start, SEEK_SET);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int lopu_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    LOPUDemuxContext *lc = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    uint32_t size;
    int ret;

    if (pos >= lc->data_end)
        return AVERROR_EOF;

    if (avio_feof(pb))
        return AVERROR_EOF;

    size = avio_rb32(pb);
    if (size < 2)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 4);
    ret = av_get_packet(pb, pkt, size);
    if (ret < 0)
        return ret;
    pkt->pos = pos;
    pkt->stream_index = 0;

    return 0;
}

const FFInputFormat ff_lopu_demuxer = {
    .p.name         = "lopu",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Switch LOPU"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "lopus",
    .priv_data_size = sizeof(LOPUDemuxContext),
    .read_probe     = lopu_probe,
    .read_header    = lopu_read_header,
    .read_packet    = lopu_read_packet,
};
