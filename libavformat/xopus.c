/*
 * XOpus demuxer
 * Copyright (c) 2025 smiRaphi
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

typedef struct XOpusDemuxContext {
    int64_t data_end;
} XOpusDemuxContext;

static int xopus_probe(const AVProbeData *p)
{
    int score;
    if (memcmp(p->buf, "XOpu", 4) || p->buf[4] != 1 || p->buf[6] != 0x30 || p->buf[7] != 0)
        return 0;

    if (p->buf_size < (0x20 + 2*AV_RL32(p->buf + 0x14)))
        return 0;

    score = AVPROBE_SCORE_MAX;
    for (int i = 0; i < AV_RL32(p->buf + 0x14); i++) {
        if (AV_RL16(p->buf + 0x20 + 2*i) == 0)
            score -= 2;
    }

    return FFMAX(score, 0);
}

static int xopus_read_header(AVFormatContext *s)
{
    int ret;
    uint8_t channels;
    uint32_t skip, pt_entries;
    int64_t pkt_off, ts;
    XOpusDemuxContext *dc = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 5);
    channels = avio_r8(pb);
    if (channels == 0)
        return AVERROR_INVALIDDATA;
    if (channels > 2) {
        avpriv_request_sample(s, "channels %d", channels);
        return AVERROR_PATCHWELCOME;
    }

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_OPUS;
    st->codecpar->sample_rate = 48000;
    st->codecpar->ch_layout.nb_channels = channels;

    avio_skip(pb, 6);
    st->duration = avio_rl32(pb);
    skip = avio_rl32(pb);
    pt_entries = avio_rl32(pb);
    dc->data_end = 0x20 + 2*pt_entries + avio_rl32(pb);
    avio_skip(pb, 4);

    ts = 0;
    pkt_off = 0x20 + 2*pt_entries;
    for (int i = 0; i < pt_entries; i++) {
        const int size = avio_rl16(pb);

        if ((ret = av_add_index_entry(st, pkt_off, ts, size, 0, AVINDEX_KEYFRAME)) < 0)
            return ret;
        pkt_off += size;
        ts += 960;
    }

    if ((ret = ff_alloc_extradata(st->codecpar, 19)) < 0)
        return ret;
    memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

    memcpy(st->codecpar->extradata, "OpusHead", 8);
    st->codecpar->extradata[8] = 1;
    st->codecpar->extradata[9] = channels;
    AV_WL16(st->codecpar->extradata + 10, skip);
    AV_WL32(st->codecpar->extradata + 12, 48000);

    ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int xopus_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIndexEntry *e;
    AVStream *st = s->streams[0];
    XOpusDemuxContext *dc = s->priv_data;
    FFStream *sti = ffstream(st);
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int ret;

    if (pos >= dc->data_end || avio_feof(pb))
        return AVERROR_EOF;

    for (int n = 0; n < sti->nb_index_entries; n++) {
        e = &sti->index_entries[n];
        if (e->pos == pos)
            break;
    }
    if (e->pos != pos)
        return AVERROR_EOF;

    ret = av_get_packet(pb, pkt, ((e->pos+e->size) > dc->data_end) ? (dc->data_end - e->pos) : e->size);
    pkt->pos = pos;
    pkt->stream_index = 0;
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->flags |= AV_PKT_FLAG_KEY;

    return ret;
}

const FFInputFormat ff_xopus_demuxer = {
    .p.name         = "xopus",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Exient XOpus"),
    .p.extensions   = "xopus",
    .priv_data_size = sizeof(XOpusDemuxContext),
    .read_probe     = xopus_probe,
    .read_header    = xopus_read_header,
    .read_packet    = xopus_read_packet,
};
