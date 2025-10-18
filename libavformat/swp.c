/*
 * SWP demuxer
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

typedef struct SWPStream {
    int64_t start_offset;
    int64_t end_offset;
} SWPStream;

static int swp_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != 0x65660100 || AV_RL32(p->buf + 4) == 0 || AV_RL8(p->buf + 8) == 0)
        return 0;
    if (AV_RL16(p->buf + 10) != 0 || AV_RL32(p->buf + 12) != 0)
        return 0;
    if (AV_RL8(p->buf + 8) != AV_RL8(p->buf + 9))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const SWPStream *fs1 = s1->priv_data;
    const SWPStream *fs2 = s2->priv_data;

    return FFDIFFSIGN(fs1->start_offset, fs2->start_offset);
}

static int swp_read_header(AVFormatContext *s)
{
    int entry_count1, max_entry_count, real_entry_count, size, channels;
    int64_t table2_offset, data_offset, start_offset;
    uint8_t entry_count2;
    AVIOContext *pb = s->pb;

    avio_skip(pb, 4);
    entry_count1 = avio_rl32(pb);
    if (entry_count1 <= 0)
        return AVERROR_INVALIDDATA;

    entry_count2 = avio_r8(pb);
    if (entry_count2 != avio_r8(pb))
        return AVERROR_INVALIDDATA;

    max_entry_count = entry_count1 * entry_count2;
    avio_skip(pb, 6);

    avio_skip(pb, 0x10LL * (max_entry_count - 1));
    real_entry_count = avio_rl32(pb);
    if (real_entry_count <= 0)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 12);
    table2_offset = avio_tell(pb);
    data_offset = table2_offset + (real_entry_count + 1) * 0x10;

    for (int i = 0; i < real_entry_count; i++) {
        avio_seek(pb, table2_offset + i * 0x10LL, SEEK_SET);

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        start_offset = avio_rl32(pb) + data_offset;
        size = avio_rl32(pb);
        channels = avio_rl16(pb);
        if (channels == 0)
            return AVERROR_INVALIDDATA;

        avio_seek(pb, 0x10 * (i + 1), SEEK_SET);
        avio_skip(pb, 3);
        if (avio_r8(pb) != channels)
            return AVERROR_INVALIDDATA;

        for (int c = 0; c < channels; c++) {
            AVStream *st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);
            SWPStream *sst = av_mallocz(sizeof(*sst));
            if (!sst)
                return AVERROR(ENOMEM);

            st->start_time = 0;
            st->priv_data = sst;
            st->duration = size / channels / 0x10 * 28;
            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
            st->codecpar->ch_layout.nb_channels = channels;
            st->codecpar->sample_rate = 44100;
            st->codecpar->block_align = 0x10 * channels;

            sst->start_offset = start_offset + size * c;
            sst->end_offset = sst->start_offset + size;
        }
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    avio_seek(pb, data_offset, SEEK_SET);

    return 0;
}

static int swp_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int64_t pos, block_size;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        SWPStream *sst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= sst->start_offset && pos < sst->end_offset) {
            block_size = FFMIN(sst->end_offset - pos, st->codecpar->block_align);

            ret = av_get_packet(pb, pkt, block_size);
            pkt->pos = pos;
            pkt->stream_index = st->index;

            break;
        } else if (pos >= sst->end_offset && i+1 < s->nb_streams) {
            AVStream *st_next = s->streams[i+1];
            SWPStream *sst_next = st_next->priv_data;
            if (sst_next->start_offset > pos)
                avio_seek(pb, sst_next->start_offset, SEEK_SET);
        }
    }

    return ret;
}

const FFInputFormat ff_swp_demuxer = {
    .p.name         = "swp",
    .p.long_name    = NULL_IF_CONFIG_SMALL("SWP Bank (Konami/KCE Studio)"),
    .p.extensions   = "swp",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = swp_probe,
    .read_header    = swp_read_header,
    .read_packet    = swp_read_packet,
};
