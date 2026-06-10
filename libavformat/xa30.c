/*
 * Reflections XA30 demuxer
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

typedef struct XA30Stream {
    int64_t start_offset;
    int64_t stop_offset;
} XA30Stream;

typedef struct XA30DemuxContext {
    int current_stream;
} XA30DemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('X','A','3','0') &&
        AV_RB32(p->buf) != MKBETAG('e','4','x','\x92'))
        return 0;

    if (p->buf_size < 8)
        return 0;
    if (AV_RL32(p->buf + 4) == 0)
        return 0;

    return AVPROBE_SCORE_MAX/3*2;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const XA30Stream *xs1 = s1->priv_data;
    const XA30Stream *xs2 = s2->priv_data;

    return FFDIFFSIGN(xs1->start_offset, xs2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    int64_t start_offset[2], stream_size[2], duration[2];
    int rate, channels, align, codec, nb_streams;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    rate = avio_rl32(pb);
    if (rate > 2) {
        align = avio_rl16(pb);
        channels = avio_rl16(pb);
        start_offset[0] = avio_rl32(pb);
        avio_skip(pb, 4);
        stream_size[0] = avio_rl32(pb);
        codec = AV_CODEC_ID_ADPCM_PSX;
        nb_streams = 1;
        if (channels > 0)
            duration[0] = stream_size[0] / channels / 16 * 28LL;
    } else if (rate >= 1) {
        channels = rate;
        rate = avio_rl32(pb);
        codec = avio_rl32(pb);
        start_offset[0] = avio_rl32(pb);
        start_offset[1] = avio_rl32(pb);
        stream_size[0] = avio_rl32(pb);
        stream_size[1] = avio_rl32(pb);
        nb_streams = 1 + (start_offset[1] > 0 && stream_size[1] > 0);
        avio_skip(pb, 4);
        align = avio_rl32(pb);

        switch (codec) {
        case 0:
            codec = (align > 2) ? AV_CODEC_ID_PCM_S16LE_PLANAR : AV_CODEC_ID_PCM_S16LE;
            align = (align > 2) ? align : 2;
            duration[0] = stream_size[0] / (channels * 2);
            duration[1] = stream_size[1] / (channels * 2);
            break;
        default:
            avpriv_request_sample(s, "codec %02x", codec);
            return AVERROR_PATCHWELCOME;
        }
    } else {
        return AVERROR_INVALIDDATA;
    }

    if (rate <= 0 || align <= 0 || channels <= 0 || channels > INT_MAX/align)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < nb_streams; n++) {
        XA30Stream *xst;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        xst = av_mallocz(sizeof(*xst));
        if (!xst)
            return AVERROR(ENOMEM);
        st->priv_data = xst;

        xst->start_offset = start_offset[n];
        xst->stop_offset = xst->start_offset + stream_size[n];

        st->start_time = 0;
        st->duration = duration[n];
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = align * channels;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    avio_seek(pb, FFMIN(start_offset[0], start_offset[s->nb_streams > 1]), SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    XA30DemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    XA30Stream *xst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (ctx->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[ctx->current_stream];
    xst = st->priv_data;
    if (do_seek)
        avio_seek(pb, xst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= xst->stop_offset) {
        do_seek = 1;
        ctx->current_stream++;
        goto redo;
    }

    {
        const int64_t pos = avio_tell(pb);
        const int block_size = ff_pcm_default_packet_size(st->codecpar);
        const int size = FFMIN(block_size, xst->stop_offset - pos);

        ret = av_get_packet(pb, pkt, size);
        pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
        pkt->pos = pos;
    }

    pkt->stream_index = st->index;
    if (ret == AVERROR_EOF) {
        ctx->current_stream++;
        goto redo;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    XA30DemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    XA30Stream *xst;
    AVStream *st;
    int64_t pos;

    ctx->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[ctx->current_stream];
    xst = st->priv_data;
    pos = avio_tell(pb);
    if (pos < xst->start_offset) {
        avio_seek(pb, xst->start_offset, SEEK_SET);
        return 0;
    }

    return -1;
}

const FFInputFormat ff_xa30_demuxer = {
    .p.name         = "xa30",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Reflections XA30"),
    .p.extensions   = "xa30,xa",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(XA30DemuxContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
