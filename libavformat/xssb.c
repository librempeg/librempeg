/*
 * XSSB demuxer
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

typedef struct XSSBDemuxContext {
    int current_stream;
} XSSBDemuxContext;

typedef struct XSSBStream {
    int64_t start_offset;
    int64_t stop_offset;
} XSSBStream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('X','S','S','B'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const XSSBStream *xs1 = s1->priv_data;
    const XSSBStream *xs2 = s2->priv_data;

    return FFDIFFSIGN(xs1->start_offset, xs2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    int64_t first_start_offset, info_start, header_start, data_start, offset;
    AVIOContext *pb = s->pb;
    int entry_size;

    avio_skip(pb, 0x10);
    info_start = avio_rl32(pb);
    header_start = avio_rl32(pb);
    data_start = avio_rl32(pb);
    avio_seek(pb, info_start, SEEK_SET);
    entry_size = avio_rl16(pb);
    offset = header_start;

    while (avio_tell(pb) < data_start) {
        int64_t stream_start, stream_stop;
        int channels, codec, rate, align;
        XSSBStream *xst;
        AVStream *st;

        avio_seek(pb, offset, SEEK_SET);
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        codec = avio_rl16(pb);
        channels = avio_rl16(pb);
        rate = avio_rl16(pb);
        avio_skip(pb, 14);
        stream_start = avio_rl32(pb);
        stream_start += data_start;
        stream_stop = avio_rl32(pb);
        if (channels <= 0 || rate <= 0 || stream_stop <= 0)
            goto skip;

        switch (codec) {
        case 0x01:
            codec = AV_CODEC_ID_PCM_S16LE;
            align = 128;
            break;
        case 0x69:
            codec = AV_CODEC_ID_ADPCM_IMA_XBOX;
            align = 0x24;
            break;
        default:
            avpriv_request_sample(s, "codec %02x", codec);
            return AVERROR_PATCHWELCOME;
        }
        stream_stop += stream_start;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        xst = av_mallocz(sizeof(*xst));
        if (!xst)
            return AVERROR(ENOMEM);
        st->priv_data = xst;

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->sample_rate = rate;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->block_align = align * channels;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        xst->start_offset = stream_start;
        xst->stop_offset = stream_stop;

skip:
        offset += entry_size;
    }

    if (s->nb_streams == 0)
        return AVERROR_INVALIDDATA;

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    {
        AVStream *st = s->streams[0];
        XSSBStream *xst = st->priv_data;

        first_start_offset = xst->start_offset;
    }

    avio_seek(pb, first_start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    XSSBDemuxContext *xssb = s->priv_data;
    int size, n, ret = AVERROR_EOF;
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    int do_seek = 0;
    XSSBStream *xst;
    AVStream *st;
    int64_t pos;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (xssb->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    n = xssb->current_stream;
    st = s->streams[n];
    par = st->codecpar;
    xst = st->priv_data;

    if (do_seek)
        avio_seek(pb, xst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= xst->stop_offset) {
        do_seek = 1;
        xssb->current_stream++;
        goto redo;
    }

    pos = avio_tell(pb);
    size = FFMIN(par->block_align, xst->stop_offset - pos);

    ret = av_get_packet(pb, pkt, size);
    pkt->pos = pos;
    pkt->stream_index = st->index;
    pkt->duration = pkt->size;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    XSSBDemuxContext *xssb = s->priv_data;
    AVIOContext *pb = s->pb;
    XSSBStream *xst;
    AVStream *st;
    int64_t pos;

    xssb->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[xssb->current_stream];
    xst = st->priv_data;

    pos = avio_tell(pb);
    if (pos < xst->start_offset) {
        avio_seek(pb, xst->start_offset, SEEK_SET);
        return 0;
    }

    return -1;
}

const FFInputFormat ff_xssb_demuxer = {
    .p.name         = "xssb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Artoon XSSB"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "bin,lbin",
    .priv_data_size = sizeof(XSSBDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
