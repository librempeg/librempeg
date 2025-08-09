/*
 * ZPD demuxer
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

#include "libavutil/bswap.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "\x10ZmAdpCm", 8))
        return 0;

    return AVPROBE_SCORE_MAX;
}

typedef struct ZPDStream {
    int64_t start_offset;
    int64_t stop_offset;
} ZPDStream;

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const ZPDStream *zs1 = s1->priv_data;
    const ZPDStream *zs2 = s2->priv_data;

    return FFDIFFSIGN(zs1->start_offset, zs2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int64_t start_offset;

    avio_skip(pb, 8);

    for (;;) {
        ZPDStream *zst;
        AVStream *st;
        uint16_t note;

        if (avio_feof(pb))
            break;

        note = avio_rb16(pb);
        if (note == 0xFFFF)
            break;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        zst = av_mallocz(sizeof(*zst));
        if (!zst)
            return AVERROR(ENOMEM);
        st->priv_data = zst;

        zst->start_offset = avio_rb32(pb);
        zst->start_offset += avio_tell(pb);
        zst->stop_offset = zst->start_offset + avio_rb32(pb);

        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_ZMUSIC;
        st->codecpar->ch_layout.nb_channels = 1;
        st->start_time = 0;
        st->codecpar->sample_rate = 22050;
        st->codecpar->block_align = 1024;
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        if (n == 0) {
            ZPDStream *zst = st->priv_data;

            start_offset = zst->start_offset;
        }

        st->index = n;
    }

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int64_t pos, block_size;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        ZPDStream *zst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= zst->start_offset && pos < zst->stop_offset) {
            block_size = FFMIN(zst->stop_offset - pos, st->codecpar->block_align);

            ret = av_get_packet(pb, pkt, block_size);
            pkt->pos = pos;
            pkt->stream_index = st->index;

            break;
        } else if (pos >= zst->stop_offset && i+1 < s->nb_streams) {
            AVStream *st_next = s->streams[i+1];
            ZPDStream *zst_next = st_next->priv_data;

            if (zst_next->start_offset > pos)
                avio_seek(pb, zst_next->start_offset, SEEK_SET);
        }
    }

    return ret;
}

const FFInputFormat ff_zpd_demuxer = {
    .p.name         = "zpd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("ZPD (Z-Music)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "zpd",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
