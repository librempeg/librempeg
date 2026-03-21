/*
 * NXOF demuxer
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

typedef struct NXOFDemuxContext {
    int64_t data_end;
} NXOFDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKTAG('n','x','o','f'))
        return 0;

    if (p->buf_size < 36)
        return 0;
    if (p->buf[5] == 0)
        return 0;
    if ((int)AV_RL16(p->buf+8) <= 0)
        return 0;
    if (AV_RL16(p->buf+32) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t data_size, duration, loop_start, loop_end, start_offset;
    int skip = 0, ret, nb_channels, rate;
    NXOFDemuxContext *n = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 5);
    nb_channels = avio_r8(pb);
    avio_skip(pb, 2);
    rate = avio_rl32(pb);
    avio_skip(pb, 12);
    start_offset = avio_rl32(pb);
    data_size = avio_rl32(pb);
    duration = avio_rl32(pb);
    avio_skip(pb, 12);
    loop_start = avio_rl32(pb);
    loop_end = avio_rl32(pb);
    if (nb_channels <= 0 || rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    n->data_end = data_size;

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_OPUS;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;

    ret = ff_alloc_extradata(st->codecpar, 19 + (2 + nb_channels) * (nb_channels > 2));
    if (ret < 0)
        return ret;
    memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

    memcpy(st->codecpar->extradata, "OpusHead", 8);
    st->codecpar->extradata[8] = 1;
    st->codecpar->extradata[9] = nb_channels;
    AV_WL16(st->codecpar->extradata + 10, skip);
    AV_WL32(st->codecpar->extradata + 12, 48000);

    ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if (loop_start > 0) {
        av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);
        av_dict_set_int(&st->metadata, "loop_end", loop_end, 0);
    }

    avio_seek(pb, start_offset, SEEK_SET);
    n->data_end += avio_tell(pb);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    NXOFDemuxContext *n = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    uint32_t size;
    int ret;

    if (pos >= n->data_end)
        return AVERROR_EOF;

    if (avio_feof(pb))
        return AVERROR_EOF;

    size = avio_rb32(pb);
    if (size < 2)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 4);
    ret = av_get_packet(pb, pkt, size);
    pkt->pos = pos;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_nxof_demuxer = {
    .p.name         = "nxof",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Nihon Falcom FDK"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "nxopus",
    .priv_data_size = sizeof(NXOFDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
