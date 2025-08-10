/*
 * Covox demuxer
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
#include "libavutil/opt.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct CovoxDemuxContext {
    AVClass     *class;
    int         sample_rate;
} CovoxDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB64(p->buf) != 0xFF55FFAAFF55FFAA)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    CovoxDemuxContext *c = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st;
    int codec;

    avio_skip(s->pb, 8);
    codec = avio_r8(pb);
    switch (codec) {
    case 8:
        break;
    default:
        avpriv_request_sample(s, "codec %d", codec);
        return AVERROR_PATCHWELCOME;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_PCM_U8;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->sample_rate = c->sample_rate;
    st->codecpar->block_align = 1024;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    avio_skip(s->pb, 7);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

#define OFFSET(x) offsetof(CovoxDemuxContext, x)
static const AVOption covox_options[] = {
    { "sample_rate", "", OFFSET(sample_rate), AV_OPT_TYPE_INT, { .i64=11025 }, 1, INT_MAX, AV_OPT_FLAG_DECODING_PARAM },
    { NULL },
};

static const AVClass covox_demuxer_class = {
    .class_name = "Covox demuxer",
    .option     = covox_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFInputFormat ff_covox_demuxer = {
    .p.name         = "covox",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Covox Audio"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.priv_class   = &covox_demuxer_class,
    .p.extensions   = "v8,v2s,v3s,v4s",
    .priv_data_size = sizeof(CovoxDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
