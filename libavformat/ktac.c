/*
 * KTAC demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('K','T','A','C'))
        return 0;

    if (p->buf_size < 34)
        return 0;
    if (AV_RL32(p->buf + 0x0c) == 0)
        return 0;
    if (AV_RL32(p->buf + 0x10) == 0)
        return 0;
    if ((int)AV_RL32(p->buf + 0x1c) <= 0)
        return 0;
    if (AV_RL16(p->buf + 0x20) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t start_offset, duration, seek_table_offset, pkt_off, ts = 0;
    int ret, rate, nb_channels, nb_entries;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 12);
    start_offset = pkt_off = avio_rl32(pb);
    avio_skip(pb, 8);
    rate = avio_rl32(pb);
    duration = avio_rl32(pb);
    nb_channels = avio_rl16(pb);
    if (rate <= 0 || nb_channels <= 0)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, 0x38, SEEK_SET);
    seek_table_offset = avio_rl32(pb);
    nb_entries = avio_rl32(pb);
    if (nb_entries <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_AAC;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, seek_table_offset, SEEK_SET);
    ts = 0;
    for (int n = 0; n < nb_entries; n++) {
        const int size = avio_rl32(pb);

        if (size <= 0)
            break;

        if ((ret = av_add_index_entry(st, pkt_off, ts, size, 0, AVINDEX_KEYFRAME)) < 0)
            return ret;

        pkt_off += size;
        ts += 1024;
    }

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIndexEntry *e;
    AVStream *st = s->streams[0];
    FFStream *sti = ffstream(st);
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    for (int n = 0; n < sti->nb_index_entries; n++) {
        e = &sti->index_entries[n];
        if (e->pos == pos)
            break;
    }

    if (e->pos != pos)
        return AVERROR_EOF;

    ret = av_get_packet(pb, pkt, e->size);
    pkt->pos = pos;
    pkt->stream_index = 0;
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->flags |= AV_PKT_FLAG_KEY;

    return ret;
}

const FFInputFormat ff_ktac_demuxer = {
    .p.name         = "ktac",
    .p.long_name    = NULL_IF_CONFIG_SMALL("KTAC (Koei Tecmo)"),
    .p.extensions   = "ktac",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
