/*
 * Sega FILM abridged format demuxer
 *
 * Copyright (c) 2026 Librempeg
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
#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"

typedef struct FilmAbridgedDemuxContext {
    int64_t data_end;
} FilmAbridgedDemuxContext;

static int read_probe(const AVProbeData *p)
{
    int entries;

    if (AV_RB32(p->buf) != MKBETAG('F', 'I', 'L', 'M'))
        return 0;

    if (p->buf_size < 14)
        return 0;

    entries = 1 + AV_RB16(p->buf+12);
    if (p->buf_size < 14 + entries * 3)
        return 0;

    for (int n = 0; n < entries; n++) {
        if ((p->buf[14 + n * 3] & 0x7f) > 1)
            return 0;

        if (AV_RB16(p->buf + 14 + n*3 + 1) == 0)
            return 0;
    }

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int entries, audio_stream_index, video_stream_index;
    FilmAbridgedDemuxContext *ctx = s->priv_data;
    int64_t offset, audio_ts = 0, video_ts = 0;
    AVIOContext *pb = s->pb;
    AVStream *st;

    audio_stream_index = -1;
    video_stream_index = -1;

    avio_skip(pb, 12);
    offset = 0x800;
    entries = avio_rb16(pb) + 1;
    for (int n = 0; n < entries; n++) {
        int entry_type, entry_size, stream_index, ret;
        int64_t ts;

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        entry_type = avio_r8(pb);
        entry_size = avio_rb16(pb);
        if (entry_size == 0)
            break;

        if (entry_type & 0x80)
            entry_type ^= 0x80;

        switch (entry_type) {
        case 0:
            if (video_stream_index == -1) {
                st = avformat_new_stream(s, NULL);
                if (!st)
                    return AVERROR(ENOMEM);
                video_stream_index = st->index;

                st->start_time = 0;
                st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
                st->codecpar->codec_id = AV_CODEC_ID_CINEPAK;
                st->codecpar->width = 320;
                st->codecpar->height = 200;

                avpriv_set_pts_info(st, 64, 1, 10);
            }

            stream_index = video_stream_index;
            ts = video_ts;
            break;
        case 1:
            if (audio_stream_index == -1) {
                st = avformat_new_stream(s, NULL);
                if (!st)
                    return AVERROR(ENOMEM);
                audio_stream_index = st->index;

                st->start_time = 0;
                st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
                st->codecpar->codec_id = AV_CODEC_ID_PCM_SGA;
                av_channel_layout_default(&st->codecpar->ch_layout, 1);
                st->codecpar->sample_rate = 16000;

                avpriv_set_pts_info(st, 64, 1, 16000);
            }

            stream_index = audio_stream_index;
            ts = audio_ts;
            break;
        default:
            avpriv_request_sample(s, "type %X", entry_type);
            return AVERROR_PATCHWELCOME;
        }

        st = s->streams[stream_index];
        if ((ret = av_add_index_entry(st, offset, ts, entry_size, 0, AVINDEX_KEYFRAME)) < 0)
            return ret;

        switch (entry_type) {
        case 0:
            video_ts++;
            break;
        case 1:
            audio_ts += entry_size;
            break;
        }

        offset += entry_size;
    }

    if (video_stream_index >= 0)
        s->streams[video_stream_index]->duration = video_ts;
    if (audio_stream_index >= 0)
        s->streams[audio_stream_index]->duration = audio_ts;

    ctx->data_end = offset;
    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    FilmAbridgedDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    AVIndexEntry *e = NULL;
    int ret, index;
    int64_t pos;

    pos = avio_tell(pb);
    if (pos >= ctx->data_end)
        return AVERROR_EOF;

    if (avio_feof(pb))
        return AVERROR_EOF;

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        FFStream *sti = ffstream(st);

        for (int n = 0; n < sti->nb_index_entries; n++) {
            e = &sti->index_entries[n];
            if (pos == e->pos) {
                break;
            } else {
                e = NULL;
            }
        }

        if (e) {
            index = i;
            break;
        }
    }

    if (!e)
        return AVERROR_EOF;

    avio_seek(pb, e->pos, SEEK_SET);

    ret = av_get_packet(pb, pkt, e->size);
    pkt->pos = e->pos;
    pkt->stream_index = index;
    if (s->streams[index]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO && pkt->size > 0 && !pkt->data[0])
        pkt->flags |= AV_PKT_FLAG_KEY;

    return ret;
}

const FFInputFormat ff_segafilma_demuxer = {
    .p.name         = "filma",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sega FILM (abridged)"),
    .priv_data_size = sizeof(FilmAbridgedDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
