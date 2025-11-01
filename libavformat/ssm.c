/*
 * SSM demuxer
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

typedef struct SSMStream {
    int64_t start_offset;
    int64_t stop_offset;
} SSMStream;

static int read_probe(const AVProbeData *p)
{
    int head_size, nb_streams;
    uint32_t data_size;

    if (av_match_ext(p->filename, "ssm") == 0)
        return 0;

    if (p->buf_size < 12)
        return 0;

    head_size = AV_RB32(p->buf);
    data_size = AV_RB32(p->buf+4);
    if (data_size == 0)
        return 0;
    nb_streams = AV_RB32(p->buf+8);
    if (nb_streams <= 0 || nb_streams > INT_MAX/0x48)
        return 0;
    if (head_size < nb_streams*0x48)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const SSMStream *ss1 = s1->priv_data;
    const SSMStream *ss2 = s2->priv_data;

    return FFDIFFSIGN(ss1->start_offset, ss2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    int ret, head_size, nb_streams;
    int64_t start, data_offset;
    AVIOContext *pb = s->pb;
    uint32_t data_size;

    head_size = avio_rb32(pb);
    data_size = avio_rb32(pb);
    nb_streams = avio_rb32(pb);
    if (nb_streams <= 0 || nb_streams > INT_MAX/0x48)
        return AVERROR_INVALIDDATA;
    if (head_size < nb_streams*0x48)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4);

    data_offset = avio_size(pb) - data_size;

    for (int si = 0; si < nb_streams; si++) {
        int64_t stream_offset, stop_offset;
        int channels, rate;
        SSMStream *sst;
        AVStream *st;

        channels = avio_rb32(pb);
        if (channels != 1 && channels != 2)
            return AVERROR_INVALIDDATA;
        rate = avio_rb32(pb);
        if (rate <= 0)
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 8);
        stop_offset = avio_rb32(pb);
        if (stop_offset == 0)
            return AVERROR_INVALIDDATA;
        stream_offset = avio_rb32(pb);
        if (stream_offset == 0)
            return AVERROR_INVALIDDATA;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        sst = av_mallocz(sizeof(*sst));
        if (!sst)
            return AVERROR(ENOMEM);
        st->priv_data = sst;

        st->start_time = 0;
        st->duration = (stream_offset - stop_offset) / channels;

        sst->start_offset = stream_offset / 16 * 8 + data_offset;
        sst->stop_offset = stop_offset / 16 * 8 + data_offset;

        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;

        ret = ff_alloc_extradata(st->codecpar, 32 * channels);
        if (ret < 0)
            return ret;

        avio_read(pb, st->codecpar->extradata, 32);
        avio_skip(pb, 16);
        if (channels >= 2) {
            int64_t align;

            avio_skip(pb, 4);
            align = avio_rb32(pb);
            align -= sst->start_offset;
            st->codecpar->block_align = (align / 16 * 8) * channels;
            if (st->codecpar->block_align <= 0)
                return AVERROR_INVALIDDATA;
            avio_skip(pb, 24);
            avio_read(pb, st->codecpar->extradata+32, 32);
        } else {
            st->codecpar->block_align = 1024;
        }

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        SSMStream *sst = st->priv_data;

        st->index = n;
        if (n == 0)
            start = sst->start_offset;
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
        SSMStream *sst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= sst->start_offset && pos < sst->stop_offset) {
            const int size = FFMIN(par->block_align, sst->stop_offset - pos);

            ret = av_get_packet(pb, pkt, size);

            pkt->pos = pos;
            pkt->stream_index = st->index;

            break;
        } else if (pos >= sst->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            SSMStream *sst_next = st_next->priv_data;

            if (sst_next->start_offset > pos)
                avio_skip(pb, sst_next->start_offset - pos);
        }
    }

    return ret;
}

const FFInputFormat ff_ssm_demuxer = {
    .p.name         = "ssm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Hal Laboratory SSM"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "ssm",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
