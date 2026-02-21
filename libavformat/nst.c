/*
 * Monster NST demuxer
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
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != 1)
        return 0;
    if (p->buf_size < 0x64)
        return 0;
    if (AV_RB32(p->buf+0x0) != AV_RB32(p->buf+0x54))
        return 0;
    if (AV_RB32(p->buf+0x4) != AV_RB32(p->buf+0x58))
        return 0;
    if (AV_RB32(p->buf+0x8) != AV_RB32(p->buf+0x5C))
        return 0;
    if (AV_RB32(p->buf+0xC) != AV_RB32(p->buf+0x60))
        return 0;
    if ((int)AV_RB32(p->buf+0x14) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int64_t duration;
    int ret, rate;
    AVStream *st;

    avio_skip(pb, 8);
    duration = avio_rb32(pb);
    avio_skip(pb, 8);
    rate = avio_rb32(pb);
    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = 2;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 16 * 2;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x20, SEEK_SET);
    if ((ret = ff_get_extradata(s, st->codecpar, pb, 32 * 2)) < 0)
        return ret;
    memcpy(st->codecpar->extradata+32, st->codecpar->extradata, 32);

    avio_seek(pb, 0xAC, SEEK_SET);

    return 0;
}

const FFInputFormat ff_nst_demuxer = {
    .p.name         = "nst",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Monster GameCube NST"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "nst",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
