/*
 * Nintendo games CWV demuxer
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
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size > 0x100) {
        if ((AV_RL32(p->buf+(p->buf_size-0x100)) & 0xFFFFFFFE) != 2)
            return 0;
        if ((int)AV_RL32(p->buf+(p->buf_size-0x100)+4) <= 0)
            return 0;
        if (AV_RL32(p->buf+(p->buf_size-0x100)+12) != 2)
            return 0;

        return AVPROBE_SCORE_MAX;
    }

    return 0;
}

static int read_header(AVFormatContext *s)
{
    int64_t file_size, duration;
    AVIOContext *pb = s->pb;
    int rate, channels, ret;
    char title[181] = { 0 };
    uint32_t flags;
    AVStream *st;

    file_size = avio_size(pb);
    if (file_size <= 0x100LL)
        return AVERROR_INVALIDDATA;
    avio_seek(pb, file_size - 0x100, SEEK_SET);

    flags = avio_rl32(pb);
    if ((flags & 0xFFFFFFFE) != 2)
        return AVERROR_INVALIDDATA;

    rate = avio_rl32(pb);
    duration = avio_rl32(pb);
    channels = (flags & 1) + 1;
    if (avio_rl32(pb) != 2)
        return AVERROR_INVALIDDATA;

    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 0x3c);
    if ((ret = avio_get_str(pb, 180, title, sizeof(title))) < 0)
        return ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    if (title[0] != '\0')
        av_dict_set(&st->metadata, "title", title, 0);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_CWV_DPCM;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret = AVERROR_EOF, channels;
    AVStream *st = s->streams[0];
    AVIOContext *pb = s->pb;
    int64_t pos, size;

    channels = st->codecpar->ch_layout.nb_channels;

    if (avio_feof(pb))
        return AVERROR_EOF;

    size = avio_size(pb);
    pos = avio_tell(pb);
    if (pos >= FFMAX(0, size - 0x100))
        return AVERROR_EOF;

    ret = av_get_packet(pb, pkt, FFMIN(1024 * channels, (size - 0x100) - pos));
    if (ret < 0)
        return ret;

    pkt->pos = pos;
    pkt->stream_index = 0;

    return 0;
}

const FFInputFormat ff_cwv_demuxer = {
    .p.name         = "cwv",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Nintendo CWV"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "cwv",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
