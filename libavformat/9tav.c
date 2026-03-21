/*
 * 9TAV demuxer
 * Copyright (C) 2026 Paul B Mahol
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
#include "libavutil/internal.h"
#include "avio_internal.h"
#include "internal.h"
#include "avformat.h"
#include "demux.h"

typedef struct NineTAVContext {
    int padded;
    int last_stream_index;
    int last_count;
} NineTAVContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('9','T','A','V'))
        return 0;

    if (AV_RL16(p->buf+8) == 0)
        return 0;
    if (AV_RL16(p->buf+10) == 0)
        return 0;
    if ((int)AV_RL32(p->buf+12) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, padded, nb_streams, channels, rate;
    NineTAVContext *nine = s->priv_data;
    int64_t duration, start_offset;
    AVIOContext *pb = s->pb;
    uint32_t config;

    avio_skip(pb, 8);
    channels = avio_rl16(pb);
    nb_streams = avio_rl16(pb);
    rate = avio_rl32(pb);
    avio_skip(pb, 8);
    duration = avio_rl32(pb);
    config = avio_rb32(pb);
    if (channels <= 0 || rate <= 0 || nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    if (avio_rb32(pb) == MKBETAG('M','T','A','F')) {
        padded = 1;
        start_offset = 0;
        nine->padded = 1;
    } else {
        padded = 0;
        start_offset = 0x20;
    }

    for (int n = 0; n < nb_streams; n++) {
        AVStream *st = avformat_new_stream(s, NULL);

        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->duration = duration;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_ATRAC9;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;

        ret = ff_alloc_extradata(st->codecpar, 12);
        if (ret < 0)
            return ret;

        AV_WL32(st->codecpar->extradata, 3);
        AV_WB32(st->codecpar->extradata+4, config);
        st->codecpar->block_align = (1 << ((config>>3) & 3)) * (((config >> 5) & 0x7ff) + 1);

        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    avio_seek(pb, padded ? 0xFE4 : start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    NineTAVContext *nine = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret;

    if (nine->padded) {
        uint8_t padding[16] = {0};
        int64_t pos = avio_tell(pb);
        int skip_more = 0;

        ret = ffio_ensure_seekback(pb, 16);
        if (ret < 0)
            return ret;

        while (avio_read(pb, padding, sizeof(padding)) == sizeof(padding)) {
            int i;

            if (avio_feof(pb))
                break;

            for (i = 0; i < sizeof(padding); i++) {
                if (padding[i])
                    break;
            }

            if (skip_more && i >= 12)
                continue;

            if (i >= 12 && (AV_RL32(padding+12) == 0xdeadbeef)) {
                skip_more = 1;
                continue;
            }

            if (i != sizeof(padding)) {
                avio_seek(pb, -16, SEEK_CUR);
                break;
            }
        }

        ret = av_get_packet(pb, pkt, s->streams[nine->last_stream_index]->codecpar->block_align);

        pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
        pkt->stream_index = nine->last_stream_index;
        pkt->pos = pos;
        nine->last_count++;
        if (nine->last_count >= 256) {
            nine->last_stream_index++;
            nine->last_count = 0;
        }
        if (nine->last_stream_index >= s->nb_streams)
            nine->last_stream_index = 0;

        return ret;
    }

    ret = av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    NineTAVContext *nine = s->priv_data;
    const int sti = av_clip(stream_index, 0, s->nb_streams-1);
    AVStream *st = s->streams[sti];
    AVIOContext *pb = s->pb;
    int64_t pos;

    if (ts < 0)
        ts = 0;

    {
        AVIndexEntry *ie;
        int index;

        index = ff_index_search_timestamp(ffstream(st)->index_entries,
                                          ffstream(st)->nb_index_entries, ts, flags);
        if (index < 0) {
            return AVERROR(EINVAL);
        } else {
            ie = &ffstream(st)->index_entries[index];
        }
        ffstream(st)->cur_dts = ie->timestamp;
        pos = ie->pos;

        avio_seek(pb, pos, SEEK_SET);

        nine->last_stream_index = sti;
        nine->last_count = index & 255;
    }

    return 0;
}

const FFInputFormat ff_ninetav_demuxer = {
    .p.name         = "9tav",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Konami 9TAV"),
    .p.extensions   = "9tav",
    .priv_data_size = sizeof(NineTAVContext),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
