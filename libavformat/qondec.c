/*
 * QON demuxer
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
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"
#include "libavutil/intreadwrite.h"

static int read_probe(const AVProbeData *p)
{
    if ((p->buf_size < 16) ||
        (AV_RB32(p->buf) != MKBETAG('q','o','n','1')) ||
        ((int)AV_RL32(p->buf + 4) <= 0) ||
        ((int)AV_RL32(p->buf + 8) <= 0) ||
        (p->buf[13] > 1) ||
        (p->buf[12] < 3) ||
        (p->buf[12] > 4))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t frame_duration, index_pos;
    AVIOContext *pb = s->pb;
    AVStream *st;
    int ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 4);
    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id = AV_CODEC_ID_QON;
    st->codecpar->width = avio_rl32(pb);
    st->codecpar->height = avio_rl32(pb);
    if ((ret = ff_get_extradata(s, st->codecpar, pb, 4)) < 0)
        return ret;
    st->nb_frames = avio_rl32(pb);
    frame_duration = avio_rl32(pb);

    avpriv_set_pts_info(st, 64, frame_duration, 1000000);

    index_pos = avio_tell(pb) + st->nb_frames * 8LL;
    for (int n = 0; n < st->nb_frames; n++) {
        uint64_t entry = avio_rl64(pb);
        uint32_t flags = entry >> 48;
        uint64_t offset = entry & ((1LL << 48)-1);

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        if (!(flags & 0x8000))
            av_add_index_entry(st, index_pos + offset, n * frame_duration, 1, 0, AVINDEX_KEYFRAME);
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int size, ret;
    int64_t pos;

    pos = avio_tell(pb);
    size = avio_rl32(pb);
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (size <= 0)
        return AVERROR_INVALIDDATA;

    if ((ret = av_get_packet(pb, pkt, size)) < 0)
        return ret;

    if (ret > 0) {
        AVStream *st = s->streams[0];
        FFStream *sti = ffstream(st);

        for (int n = 0; n < sti->nb_index_entries; n++) {
            const AVIndexEntry *const e = &sti->index_entries[n];

            if (e->pos == pos) {
                pkt->flags |= AV_PKT_FLAG_KEY;
                break;
            } else if (pos < e->pos) {
                break;
            }
        }
    }

    pkt->duration = 1;
    pkt->pos = pos;

    return 0;
}

const FFInputFormat ff_qon_demuxer = {
    .p.name         = "qon",
    .p.long_name    = NULL_IF_CONFIG_SMALL("QON"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "qon",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
