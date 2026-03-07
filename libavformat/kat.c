/*
 * Dreamcast KAT demuxer
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
#include "pcm.h"

typedef struct KATDemuxContext {
    int current_stream;
} KATDemuxContext;

typedef struct KATStream {
    int64_t start_offset;
    int64_t stop_offset;
} KATStream;

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const KATStream *ks1 = s1->priv_data;
    const KATStream *ks2 = s2->priv_data;

    return FFDIFFSIGN(ks1->start_offset, ks2->start_offset);
}

static int read_probe(const AVProbeData *p)
{
    int nb_streams;

    nb_streams = AV_RL32(p->buf);
    if (nb_streams <= 0)
        return 0;
    if (p->buf_size < nb_streams * 0x2c)
        return 0;
    for (int n = 0; n < nb_streams; n++) {
        int depth;

        if (AV_RL32(p->buf + 4 + n * 0x2c) != 1)
            return 0;
        if ((int)AV_RL32(p->buf + 4 + n * 0x2c + 0x0c) <= 0)
            return 0;
        depth = AV_RL32(p->buf + 4 + n * 0x2c + 0x14);
        if (depth != 4 && depth != 8 && depth != 16)
            return 0;
    }

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int64_t offset;
    int nb_streams;

    nb_streams = avio_rl32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < nb_streams; n++) {
        int64_t start_offset, stop_offset;
        int rate, depth, align, codec;
        KATStream *kst;
        AVStream *st;

        avio_seek(pb, 4 + n * 0x2c, SEEK_SET);

        if (avio_rl32(pb) != 1)
            continue;

        start_offset = avio_rl32(pb);
        stop_offset = start_offset;
        stop_offset += avio_rl32(pb);
        rate = avio_rl32(pb);
        avio_skip(pb, 4);
        depth = avio_rl32(pb);
        if (rate <= 0 || (depth != 4 && depth != 8 && depth != 16))
            continue;

        switch (depth) {
        case 4:
            codec = AV_CODEC_ID_ADPCM_AICA;
            align = 1;
            break;
        case 8:
            codec = AV_CODEC_ID_PCM_S8;
            align = 1;
            break;
        case 16:
            codec = AV_CODEC_ID_PCM_S16LE;
            align = 2;
            break;
        }

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        kst = av_mallocz(sizeof(*kst));
        if (!kst)
            return AVERROR(ENOMEM);
        st->priv_data = kst;
        kst->start_offset = start_offset;
        kst->stop_offset = stop_offset;

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->ch_layout.nb_channels = 1;
        st->codecpar->block_align = align;
        st->codecpar->sample_rate = rate;
        st->codecpar->codec_id = codec;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    if (s->nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
        if (n == 0) {
            KATStream *kst = st->priv_data;

            offset = kst->start_offset;
        }
    }

    avio_seek(pb, offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    KATDemuxContext *kat = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    KATStream *kst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (kat->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[kat->current_stream];
    kst = st->priv_data;
    if (do_seek)
        avio_seek(pb, kst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= kst->stop_offset) {
        do_seek = 1;
        kat->current_stream++;
        goto redo;
    }

    {
        const int64_t pos = avio_tell(pb);
        const int block_size = ff_pcm_default_packet_size(st->codecpar);
        const int size = FFMIN(block_size, kst->stop_offset - pos);

        ret = av_get_packet(pb, pkt, size);
        pkt->pos = pos;
    }
    pkt->stream_index = st->index;
    if (ret == AVERROR_EOF) {
        kat->current_stream++;
        goto redo;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    KATDemuxContext *kat = s->priv_data;
    AVIOContext *pb = s->pb;
    KATStream *kst;
    AVStream *st;
    int64_t pos;

    kat->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[kat->current_stream];
    kst = st->priv_data;

    pos = avio_tell(pb);
    if (pos < kst->start_offset) {
        avio_seek(pb, kst->start_offset, SEEK_SET);
        return 0;
    }

    return -1;
}

const FFInputFormat ff_kat_demuxer = {
    .p.name         = "kat",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sega Dreamcast KAT"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "kat",
    .priv_data_size = sizeof(KATDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
