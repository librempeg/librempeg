/*
 * ESF demuxer
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

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('E','S','F','\x06'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    uint32_t flags, version;
    int64_t start_offset;
    int rate, bps, codec;
    AVStream *st;

    avio_skip(pb, 3);
    version = avio_r8(pb);
    flags = avio_rl32(pb);

    switch (version) {
    case 3:
        rate = avio_rl32(pb);
        if (rate <= 0)
            return AVERROR_INVALIDDATA;
        start_offset = 0x10;
        codec = AV_CODEC_ID_ADPCM_IMA_WS;
        break;
    case 6:
        bps = (flags & 0x20000000) ? 16 : 8;
        rate = (flags & 0x40000000) ? 22050 : 11025;
        start_offset = 0x8;
        codec = (bps == 8) ? AV_CODEC_ID_PCM_U8 : AV_CODEC_ID_ADPCM_IMA_WS;
        break;
    case 8:
        bps = (flags & 0x10000000) ? 16 : 8;
        rate = (flags & 0x20000000) ? 22050 : 11025;
        start_offset = 0x08;
        codec = (bps == 8) ? AV_CODEC_ID_PCM_U8 : (flags & 0x80000000) ? AV_CODEC_ID_ADPCM_IMA_WS : AV_CODEC_ID_PCM_S16LE;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->block_align = 1024;
    st->codecpar->sample_rate = rate;
    st->codecpar->codec_id = codec;
    if (codec == AV_CODEC_ID_ADPCM_IMA_WS) {
        int ret = ff_alloc_extradata(st->codecpar, 2);
        if (ret < 0)
            return ret;
        AV_WL16(st->codecpar->extradata, 4);
    }

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

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

const FFInputFormat ff_esf_demuxer = {
    .p.name         = "esf",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Eurocom ESF"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "esf",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
