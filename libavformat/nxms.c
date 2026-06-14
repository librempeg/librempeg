/*
 * NXMS demuxer
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

typedef struct NXMSDemuxContext {
    int segments;
    int64_t data_start[2];
    int64_t data_stop[2];
} NXMSDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('N','X','M','S'))
        return 0;

    if (p->buf_size < 20)
        return 0;
    if ((int)AV_RL16(p->buf+8) <= 0)
        return 0;
    if ((int)AV_RL16(p->buf+8) > 2)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int skip = 0, ret, nb_channels, rate, chunk_size;
    int64_t duration = 0, offset;
    NXMSDemuxContext *n = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 8);
    n->segments = avio_rl16(pb);
    avio_skip(pb, 6);
    if (n->segments <= 0 || n->segments > 2)
        return AVERROR_INVALIDDATA;

    offset = avio_rl32(pb);
    avio_seek(pb, offset, SEEK_SET);
    for (int i = 0; i < n->segments; i++) {
        n->data_start[i] = avio_rl32(pb);
        n->data_stop[i] = avio_rl32(pb);
        n->data_stop[i] += n->data_start[i];
        duration += avio_rl32(pb);
        avio_skip(pb, 36);
    }

    avio_seek(pb, n->data_start[0], SEEK_SET);
    if (avio_rl32(pb) != 0x80000001)
        return AVERROR_INVALIDDATA;
    chunk_size = avio_rl32(pb);
    if (chunk_size <= 0)
        return AVERROR_INVALIDDATA;
    nb_channels = avio_rb16(pb);
    avio_skip(pb, 2);
    rate = avio_rl32(pb);
    n->data_start[0] += chunk_size + 8 + 8;

    if (nb_channels <= 0 || rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

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

    avio_seek(pb, n->data_start[0], SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    NXMSDemuxContext *n = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    uint32_t size;
    int ret;

    if (pos >= n->data_stop[n->segments-1])
        return AVERROR_EOF;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (n->segments > 1 &&
        (pos >= n->data_stop[0] &&
         pos <= n->data_start[1])) {
        avio_seek(pb, n->data_start[1], SEEK_SET);
        if (avio_rl32(pb) != 0x80000001)
            return AVERROR(EIO);

        int chunk_size = avio_rl32(pb);
        if (chunk_size <= 0)
            return AVERROR(EIO);
        avio_skip(pb, chunk_size + 8);
    }

    size = avio_rb32(pb);
    if (size < 2)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 4);
    ret = av_get_packet(pb, pkt, size);
    pkt->pos = pos;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_nxms_demuxer = {
    .p.name         = "nxms",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Arika NXMS"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "nxms",
    .priv_data_size = sizeof(NXMSDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
