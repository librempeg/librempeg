/*
 * XPCM demuxer
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

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('X','P','C','M'))
        return 0;

    if (p->buf_size < 20)
        return 0;

    if (AV_RB32(p->buf+4) == 0)
        return 0;

    if (AV_RL16(p->buf+14) == 0)
        return 0;

    if (AV_RL32(p->buf+16) == 0)
        return 0;

    if (p->buf[8] > 3)
        return 0;

    return AVPROBE_SCORE_MAX*2/3;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    uint32_t codec;
    AVStream *st;

    avio_skip(pb, 8);
    codec = avio_r8(pb);
    avio_skip(pb, 5);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = avio_rl16(pb);
    if (st->codecpar->ch_layout.nb_channels == 0)
        return AVERROR_INVALIDDATA;

    st->start_time = 0;
    st->codecpar->sample_rate = avio_rl32(pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 8);

    switch (codec) {
    case 0:
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
        st->codecpar->block_align = st->codecpar->ch_layout.nb_channels * 1024;
        break;
    case 2:
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_CIRCUS;
        st->codecpar->block_align = st->codecpar->ch_layout.nb_channels * 256;
        break;
    case 1:
    case 3:
        avpriv_request_sample(s, "codec %d", codec);
        return AVERROR_PATCHWELCOME;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    AVStream *st = s->streams[0];
    AVCodecParameters *par = st->codecpar;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    ret = av_get_packet(pb, pkt, par->block_align);
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_xpcm_demuxer = {
    .p.name         = "xpcm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Circus XPCM"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "xpcm",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
