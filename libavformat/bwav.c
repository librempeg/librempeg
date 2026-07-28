/*
 * BWAV demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('B','W','A','V'))
        return 0;

    if (p->buf_size < 32)
        return 0;
    if (AV_RB16(p->buf+4) != 0xFFFE)
        return 0;
    if (AV_RL16(p->buf+6) != 1)
        return 0;
    if ((int)AV_RL32(p->buf+20) <= 0)
        return 0;
    if (AV_RL32(p->buf+28) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, codec, rate, nb_channels, align;
    int64_t offset, duration;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 14);
    nb_channels = avio_rl16(pb);
    codec = avio_rl16(pb);
    avio_skip(pb, 2);
    rate = avio_rl32(pb);
    avio_skip(pb, 4);
    duration = avio_rl32(pb);

    if (nb_channels > 1) {
        avio_seek(pb, 0x8c, SEEK_SET);
        align = avio_rl32(pb);
    } else {
        align = 1024;
    }

    switch (codec) {
    case 0:
        codec = AV_CODEC_ID_PCM_S16LE_PLANAR;
        break;
    case 1:
        codec = AV_CODEC_ID_ADPCM_NDSP_LE;
        break;
    default:
        avpriv_request_sample(s, "codec %d", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (align <= 0 || rate <= 0 || nb_channels <= 0 || nb_channels > INT_MAX/align)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->block_align = align * nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if (st->codecpar->codec_id == AV_CODEC_ID_ADPCM_NDSP_LE) {
        avio_seek(pb, 0x20, SEEK_SET);
        if ((ret = ff_alloc_extradata(st->codecpar, 32 * st->codecpar->ch_layout.nb_channels)) < 0)
            return ret;

        for (int ch = 0; ch < st->codecpar->ch_layout.nb_channels; ch++) {
            avio_read(pb, st->codecpar->extradata + ch * 32, 32);
            avio_skip(pb, 0x2c);
        }
    }

    avio_seek(pb, 0x44, SEEK_SET);
    offset = avio_rl32(pb);
    avio_seek(pb, offset, SEEK_SET);

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

const FFInputFormat ff_bwav_demuxer = {
    .p.name         = "bwav",
    .p.long_name    = NULL_IF_CONFIG_SMALL("BWAV (NintendoWare BWAV)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "bwav",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
