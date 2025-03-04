/*
 * SNDB demuxer
 * Copyright (c) 2025 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/bswap.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S','N','D','B'))
        return 0;

    if (AV_RB32(p->buf+32) != MKBETAG('C','S','R',' '))
        return 0;

    return AVPROBE_SCORE_MAX*2/3;
}

typedef struct SNDBStream {
    int64_t start_offset;
    int64_t stop_offset;
} SNDBStream;

static int read_header(AVFormatContext *s)
{
    uint32_t nb_tracks = 0, size;
    AVIOContext *pb = s->pb;
    int64_t start_offset;
    int ret;

    avio_skip(pb, 32);
    if (avio_rb32(pb) != MKBETAG('C','S','R',' '))
        return AVERROR_INVALIDDATA;
    size = avio_rb32(pb);
    if (size <= 8)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, size - 8);
    if (avio_rb32(pb) != MKBETAG('C','S','H',' '))
        return AVERROR_INVALIDDATA;
    size = avio_rb32(pb);
    if (size <= 8)
        return AVERROR_INVALIDDATA;
    start_offset = avio_tell(pb) + size - 8;

    avio_skip(pb, 4);
    nb_tracks = avio_rb32(pb);
    avio_skip(pb, 16);

    for (int n = 0; n < nb_tracks; n++) {
        SNDBStream *sst;
        AVStream *st;

        sst = av_mallocz(sizeof(SNDBStream));
        if (!sst)
            return AVERROR(ENOMEM);
        sst->start_offset = avio_rb32(pb);

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->id = n;
        st->start_time = 0;
        st->priv_data = sst;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_XMA1;
        st->codecpar->block_align = 2048;
    }

    if (start_offset < avio_tell(pb))
        return AVERROR_INVALIDDATA;
    avio_skip(pb, start_offset - avio_tell(pb));

    if (avio_rb32(pb) != MKBETAG('C','S','B',' '))
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 12);

    size = avio_rb32(pb);
    if (size <= 20)
        return AVERROR_INVALIDDATA;
    start_offset = avio_tell(pb) + size - 20;
    avio_skip(pb, 44);

    for (int n = 0; n < nb_tracks; n++) {
        AVStream *st = s->streams[n];
        SNDBStream *sst = st->priv_data;
        int streams, channels = 0;

        if (avio_rb16(pb) != 0x165)
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 2);

        if ((ret = ff_get_extradata(s, st->codecpar, pb, 108)) < 0)
            return ret;

        sst->start_offset += start_offset;

        if (n == nb_tracks - 1) {
            sst->stop_offset = avio_size(pb);
        } else if (n > 0) {
            AVStream *prev_st = s->streams[n-1];
            SNDBStream *prev_sst = prev_st->priv_data;

            prev_sst->stop_offset = sst->start_offset;
        }

        streams = AV_RB16(st->codecpar->extradata + 4);
        AV_WL16(st->codecpar->extradata + 4, streams);
        st->codecpar->sample_rate = AV_RB32(st->codecpar->extradata + 12);
        if (st->codecpar->sample_rate <= 0)
            return AVERROR_INVALIDDATA;

        if (streams <= 0)
            return AVERROR_INVALIDDATA;

        if (st->codecpar->extradata_size < (streams - 1) * 20 + 25)
            return AVERROR_INVALIDDATA;

        for (int i = 0; i < streams; i++)
            channels += st->codecpar->extradata[20 * i + 25];
        st->codecpar->extradata_size = 8 + 20 * streams;

        st->codecpar->ch_layout.nb_channels = channels;
        if (!st->codecpar->ch_layout.nb_channels)
            return AVERROR_INVALIDDATA;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    if (start_offset < avio_tell(pb))
        return AVERROR_INVALIDDATA;
    avio_skip(pb, start_offset - avio_tell(pb));

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        SNDBStream *sst = st->priv_data;
        int64_t pos;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= sst->start_offset && pos < sst->stop_offset) {
            ret = av_get_packet(pb, pkt, par->block_align);
            pkt->stream_index = st->id;
            break;
        } else if (pos >= sst->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            SNDBStream *sst_next = st_next->priv_data;

            if (sst_next->start_offset > pos)
                avio_skip(pb, sst_next->start_offset - pos);
        }
    }
    return ret;
}

const FFInputFormat ff_sndb_demuxer = {
    .p.name         = "sndb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Capcom SNDB (Sound Bank)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "snd",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
