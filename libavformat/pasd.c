/*
 * PASD demuxer
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

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct PASDStream {
    int64_t header_offset;
    int64_t start_offset;
    int64_t stop_offset;
} PASDStream;

static int read_probe(const AVProbeData *p)
{
    int nb_streams;

    if (AV_RB32(p->buf) != MKBETAG('P','A','S','D'))
        return 0;

    if (p->buf_size < 32)
        return 0;

    nb_streams = AV_RB32(p->buf + 8);
    if (nb_streams <= 0)
        return 0;

    if (AV_RB32(p->buf + 12) < (nb_streams * 32LL + 32LL))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const PASDStream *ps1 = s1->priv_data;
    const PASDStream *ps2 = s2->priv_data;

    return FFDIFFSIGN(ps1->start_offset, ps2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int64_t offset, start;
    int nb_streams;
    int ret;

    avio_skip(pb, 8);
    nb_streams = avio_rb32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;
    offset = avio_rb32(pb);
    start = avio_rb32(pb);
    avio_seek(pb, 0x20, SEEK_SET);

    for (int si = 0; si < nb_streams; si++) {
        PASDStream *pst;
        AVStream *st;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;

        pst = av_mallocz(sizeof(*pst));
        if (!pst)
            return AVERROR(ENOMEM);
        st->priv_data = pst;
        pst->header_offset = avio_rb32(pb);
        if (pst->header_offset < offset)
            return AVERROR_INVALIDDATA;

        pst->start_offset = avio_rb32(pb);
        pst->stop_offset = avio_rb32(pb);
        avio_skip(pb, 20);
    }

    for (int si = 0; si < nb_streams; si++) {
        AVStream *st = s->streams[si];
        PASDStream *pst = st->priv_data;

        avio_seek(pb, pst->header_offset, SEEK_SET);

        if (avio_rb16(pb) != 0x166)
            return AVERROR_INVALIDDATA;

        st->codecpar->codec_id = AV_CODEC_ID_XMA2;
        st->codecpar->ch_layout.nb_channels = avio_rb16(pb);
        if (st->codecpar->ch_layout.nb_channels <= 0)
            return AVERROR_INVALIDDATA;
        st->codecpar->sample_rate = avio_rb32(pb);
        if (st->codecpar->sample_rate <= 0)
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 16);
        st->duration = avio_rb32(pb);
        st->codecpar->block_align = 0x800;

        ret = ff_alloc_extradata(st->codecpar, 34);
        if (ret < 0)
            return ret;
        memset(st->codecpar->extradata, 0, 34);
        AV_WL16(st->codecpar->extradata, 1);
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        PASDStream *pst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= pst->start_offset && pos < pst->stop_offset) {
            const int size = FFMIN(par->block_align, pst->stop_offset - pos);

            ret = av_get_packet(pb, pkt, size);

            pkt->pos = pos;
            pkt->stream_index = st->index;

            break;
        } else if (pos >= pst->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            PASDStream *pst_next = st_next->priv_data;

            if (pst_next->start_offset > pos)
                avio_skip(pb, pst_next->start_offset - pos);
        }
    }

    return ret;
}

const FFInputFormat ff_pasd_demuxer = {
    .p.name         = "pasd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("PASD (Premium Agency Sound Bank)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "sgd",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
