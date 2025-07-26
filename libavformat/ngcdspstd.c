/*
 * NGC DSP STD demuxer
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
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    int score = 0;

    if (p->buf_size < 0x60)
        return 0;

    if (AV_RB32(p->buf) == 0)
        return 0;
    score++;

    if (AV_RB32(p->buf+4) == 0)
        return 0;
    score++;

    if (AV_RB32(p->buf+8) == 0)
        return 0;
    score++;

    if (AV_RB16(p->buf+0xc) != 0 &&
        AV_RB16(p->buf+0xc) != 1)
        return 0;
    score += 2;

    if (AV_RB16(p->buf+0xe) != 0)
        return 0;
    score++;

    for (int n = 0 ; n < 16; n++) {
        if (AV_RB16(p->buf + 0x1c + n * 2) != 0)
            score++;
    }

    if (AV_RB16(p->buf+0x3c) != 0)
        return 0;
    score++;

    return score;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    int ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->duration = avio_rb32(pb);
    avio_skip(pb, 4);
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_THP;
    st->codecpar->sample_rate = avio_rb32(pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 2);
    if (avio_rb16(pb) != 0)
        return AVERROR_INVALIDDATA;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->block_align = 512;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x1c, SEEK_SET);
    if ((ret = ff_get_extradata(s, st->codecpar, pb, 32 * st->codecpar->ch_layout.nb_channels)) < 0)
        return ret;

    avio_seek(pb, 0x60, SEEK_SET);

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

const FFInputFormat ff_ngcdspstd_demuxer = {
    .p.name         = "ngcdspstd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("NGC (Nintendo DSP Standard)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "dsp",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
