/*
 * WD demuxer
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

typedef struct WDStream {
    int64_t start_offset;
    int64_t stop_offset;
} WDStream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB16(p->buf) != 0x5744)
        return 0;

    if (p->buf_size < 36)
        return 0;
    if (AV_RB32(p->buf+4) == 0)
        return 0;
    if ((int)AV_RB32(p->buf+8) <= 0)
        return 0;
    if ((int)AV_RB32(p->buf+12) <= 0)
        return 0;
    if ((int)AV_RB32(p->buf+8) > (int)AV_RB32(p->buf+12))
        return 0;
    if (AV_RB32(p->buf+32) <= 0x24)
        return 0;

    return AVPROBE_SCORE_MAX/3*2;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const WDStream *ws1 = s1->priv_data;
    const WDStream *ws2 = s2->priv_data;

    return FFDIFFSIGN(ws1->start_offset, ws2->start_offset);
}

static int key_to_rate(int key, int base_rate)
{
    int sample_rate = lrint(base_rate * pow(2.0, key / (double)0x1000000 / 12.0));

    return FFMIN(sample_rate, base_rate);
}

static int read_header(AVFormatContext *s)
{
    int64_t first_entry_offset, data_offset, start_offset;
    int ret, nb_streams, nb_entries;
    AVIOContext *pb = s->pb;

    avio_skip(pb, 0x8);
    nb_entries = avio_rb32(pb);
    nb_streams = avio_rb32(pb);
    if (nb_entries <= 0 || nb_streams <= 0 || nb_entries > nb_streams)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 16);
    first_entry_offset = avio_rb32(pb);
    for (int n = 1; n < nb_entries; n++) {
        int64_t entry_offset = avio_rb32(pb);

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        if (entry_offset == 0)
            continue;

        first_entry_offset = FFMIN(first_entry_offset, entry_offset);
    }

    data_offset = first_entry_offset + nb_streams * 0x60LL;

    for (int si = 0; si < nb_streams; si++) {
        const int64_t single_entry_size = 0x60;
        WDStream *wst;
        AVStream *st;
        int key;

        avio_seek(pb, first_entry_offset + single_entry_size * si, SEEK_SET);

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        wst = av_mallocz(sizeof(*wst));
        if (!wst)
            return AVERROR(ENOMEM);
        st->priv_data = wst;

        avio_skip(pb, 4);
        wst->start_offset = avio_rb32(pb) + data_offset;
        avio_skip(pb, 8);
        wst->stop_offset = avio_rb32(pb);
        wst->stop_offset += wst->start_offset;
        key = avio_rb32(pb);
        avio_skip(pb, 10);

        st->start_time = 0;
        st->duration = (wst->stop_offset - wst->start_offset) / 8 * 14;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->ch_layout.nb_channels = 1;
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
        st->codecpar->sample_rate = key_to_rate(key, 32000);
        st->codecpar->block_align = 512;

        if ((ret = ff_get_extradata(s, st->codecpar, pb, 32)) < 0)
            return ret;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
        if (n == 0) {
            WDStream *wst = st->priv_data;

            start_offset = wst->start_offset;
        }
    }

    avio_seek(pb, start_offset, SEEK_SET);

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
        WDStream *wst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= wst->start_offset && pos < wst->stop_offset) {
            const int size = FFMIN(par->block_align, wst->stop_offset - pos);

            ret = av_get_packet(pb, pkt, size);

            pkt->pos = pos;
            pkt->stream_index = st->index;

            break;
        } else if (pos >= wst->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            WDStream *wst_next = st_next->priv_data;

            if (wst_next->start_offset > pos)
                avio_seek(pb, wst_next->start_offset, SEEK_SET);
        }
    }

    return ret;
}

const FFInputFormat ff_wd_demuxer = {
    .p.name         = "wd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Wave WD"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "wd",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
