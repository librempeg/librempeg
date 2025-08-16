/*
 * MSX demuxer
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
#include "riff.h"

typedef struct MSXStream {
    int64_t start_offset;
    int64_t end_offset;
} MSXStream;

static int msx_probe(const AVProbeData *p)
{
    uint32_t off;

    if (AV_RL32(p->buf) == 1) {
        if (p->buf_size - 4 < 0x1C)
            return 0;

        off = AV_RL32(p->buf + 0x1C);
    } else if (AV_RL32(p->buf) == 2) {
        if (AV_RL32(p->buf + 8) != AV_RL32(p->buf + 0x10))
            return 0;
        if (p->buf_size < 0x44)
            return 0;

        off = AV_RL32(p->buf + 0x40);
    } else
        return 0;
    if (p->buf_size - 4 < off)
        return 0;

    if (memcmp(p->buf + off, "RIFF", 4) || memcmp(p->buf + off + 8, "WAVEfmt ", 8))
        return 0;

    return AVPROBE_SCORE_MAX / 3 * 2;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const MSXStream *ms1 = s1->priv_data;
    const MSXStream *ms2 = s2->priv_data;

    return FFDIFFSIGN(ms1->start_offset, ms2->start_offset);
}

static int msx_read_header(AVFormatContext *s)
{
    int ret;
    int64_t pos;
    uint32_t version, nb_streams, base_offset, size;
    int32_t relative_offset;
    AVIOContext *pb = s->pb;

    version = avio_rl32(pb);
    nb_streams = avio_rl32(pb) - 1;
    if (version == 1)
        avio_skip(pb, 0x10);
    else if (version == 2) {
        base_offset = avio_rl32(pb);
        avio_skip(pb, 4);
        if (base_offset != avio_rl32(pb))
            return AVERROR_INVALIDDATA;
        relative_offset = avio_rl32(pb);
        avio_skip(pb, 0x18);
    } else {
        avpriv_request_sample(s, "version 0x%X", version);
        return AVERROR_PATCHWELCOME;
    }

    for (int i = 0; i < nb_streams; i++) {
        AVStream *st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);
        MSXStream *mst = av_mallocz(sizeof(*mst));
        if (!mst)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->priv_data = mst;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;

        avio_skip(pb, 4);
        if (version == 2)
            avio_skip(pb, 4);
        mst->start_offset = avio_rl32(pb);
        if (version == 2 && mst->start_offset < relative_offset)
            mst->start_offset += base_offset;
        size = avio_rl32(pb);
        pos = avio_tell(pb);
        if (version == 2)
            pos += 8;

        if (version == 1)
            avio_seek(pb, mst->start_offset, SEEK_SET);
        else if (version == 2)
            avio_seek(pb, avio_rl32(pb), SEEK_SET);
        avio_skip(pb, 0x10);

        ret = ff_get_wav_header(s, pb, st->codecpar, avio_rl32(pb), 0);
        if (ret < 0)
            return ret;

        if (version == 1)
            mst->start_offset = avio_tell(pb) + 8;
        mst->end_offset = mst->start_offset + size;
        avio_seek(pb, pos, SEEK_SET);
    }

    qsort(s->streams, s->nb_streams, sizeof(*s->streams), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        st->index = n;
    }

    MSXStream *first_mst = s->streams[0]->priv_data;
    avio_seek(pb, first_mst->start_offset, SEEK_SET);
    return 0;
}

static int msx_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int64_t pos, block_size;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MSXStream *mst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= mst->start_offset && pos < mst->end_offset) {
            block_size = st->codecpar->block_align;
            ret = av_get_packet(pb, pkt, FFMIN(block_size, mst->end_offset - pos));
            pkt->pos = pos;
            pkt->stream_index = st->index;
            break;
        } else if (pos >= mst->end_offset && i+1 < s->nb_streams) {
            AVStream *st_next = s->streams[i+1];
            MSXStream *mst_next = st_next->priv_data;
            if (mst_next->start_offset > pos)
                avio_seek(pb, mst_next->start_offset, SEEK_SET);
        }
    }

    return ret;
}

const FFInputFormat ff_msx_demuxer = {
    .p.name         = "msx",
    .p.long_name    = NULL_IF_CONFIG_SMALL("MSX (Mortal Kombat XBOX)"),
    .p.extensions   = "msx",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = msx_probe,
    .read_header    = msx_read_header,
    .read_packet    = msx_read_packet,
};
