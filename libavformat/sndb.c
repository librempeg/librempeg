/*
 * SNDB demuxer
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

typedef struct SNDBStream {
    int64_t start_offset;
    int64_t stop_offset;
} SNDBStream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S','N','D','B'))
        return 0;

    if (AV_RB32(p->buf+32) != MKBETAG('C','S','R',' '))
        return 0;

    return AVPROBE_SCORE_MAX*2/3;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const SNDBStream *fs1 = s1->priv_data;
    const SNDBStream *fs2 = s2->priv_data;

    return FFDIFFSIGN(fs1->start_offset, fs2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    int64_t start_offset, header_offset, stream_stop, next_offset;
    uint32_t nb_tracks, size;
    AVIOContext *pb = s->pb;
    int ret;

    avio_skip(pb, 32);
    if (avio_rb32(pb) != MKBETAG('C','S','R',' '))
        return AVERROR_INVALIDDATA;
    size = avio_rb32(pb);
    if (size <= 8)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, size - 8);
    header_offset = avio_tell(pb);
    if (avio_rb32(pb) != MKBETAG('C','S','H',' '))
        return AVERROR_INVALIDDATA;
    size = avio_rb32(pb);
    if (size <= 8)
        return AVERROR_INVALIDDATA;
    start_offset = avio_tell(pb) + size - 8;

    avio_skip(pb, 4);
    nb_tracks = avio_rb32(pb);
    if (nb_tracks == 0)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4);
    size = avio_rb32(pb);
    avio_skip(pb, header_offset + size - avio_tell(pb));

    if (start_offset < avio_tell(pb))
        return AVERROR_INVALIDDATA;
    avio_skip(pb, start_offset - avio_tell(pb));

    if (avio_rb32(pb) != MKBETAG('C','S','B',' '))
        return AVERROR_INVALIDDATA;
    stream_stop = avio_rb32(pb) + start_offset;
    nb_tracks = avio_rb32(pb);
    avio_skip(pb, 4);

    size = avio_rb32(pb);
    if (size <= 20)
        return AVERROR_INVALIDDATA;
    start_offset = avio_tell(pb) + size - 20;
    next_offset = start_offset;
    avio_skip(pb, 44);

    for (int si = 0; si < nb_tracks; si++) {
        int streams, channels = 0, codec, skipe = 0, rate;
        SNDBStream *sst;
        AVStream *st;

        codec = avio_rb16(pb);
        if (codec == 0)
            break;
        if (codec != 0x165) {
            avpriv_request_sample(s, "codec %04X", codec);
            return AVERROR_PATCHWELCOME;
        }
        avio_skip(pb, 6);
        streams = avio_rb16(pb);
        if (streams <= 0)
            return AVERROR_INVALIDDATA;

        avio_skip(pb, 1);
        skipe = avio_r8(pb) != 2;
        avio_skip(pb, 4);
        rate = avio_rb32(pb);
        if (rate <= 0)
            return AVERROR_INVALIDDATA;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);
        sst = av_mallocz(sizeof(*sst));
        if (!sst)
            return AVERROR(ENOMEM);
        st->priv_data = sst;

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_XMA1;
        st->codecpar->block_align = 2048;

        if ((ret = ff_alloc_extradata(st->codecpar, 8 + 20 * streams)) < 0)
            return ret;
        memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
        avio_read(pb, st->codecpar->extradata + 16, 8 + 20 * streams - 16);

        AV_WL16(st->codecpar->extradata + 4, streams);
        st->codecpar->sample_rate = rate;

        sst->start_offset = next_offset;
        avio_skip(pb, 4);
        if (skipe)
            avio_skip(pb, 0x30);
        next_offset = start_offset + avio_rb32(pb);

        avio_skip(pb, 0x18);

        for (int i = 0; i < streams; i++)
            channels += st->codecpar->extradata[20 * i + 25];
        st->codecpar->ch_layout.nb_channels = channels;
        if (!st->codecpar->ch_layout.nb_channels)
            return AVERROR_INVALIDDATA;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        SNDBStream *sst = st->priv_data;

        st->index = n;
        if (n + 1 < s->nb_streams) {
            AVStream *next_st = s->streams[n+1];
            SNDBStream *next_sst = next_st->priv_data;

            sst->stop_offset = next_sst->start_offset;
        } else {
            sst->stop_offset = stream_stop;
        }
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
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        SNDBStream *sst = st->priv_data;

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
            SNDBStream *pst_next = st_next->priv_data;

            if (pst_next->start_offset > pos)
                avio_skip(pb, pst_next->start_offset - pos);
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
