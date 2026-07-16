/*
 * UE5XMA demuxer
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

static int read_probe(const AVProbeData *p)
{
    if (AV_RL16(p->buf) != 0x166)
        return 0;
    if (AV_RL16(p->buf+2) != 2)
        return 0;
    if (p->buf_size < 40)
        return 0;
    if ((int)AV_RL32(p->buf+4) <= 0)
        return 0;
    if (AV_RL32(p->buf+36) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, rate, channels;
    AVIOContext *pb = s->pb;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 2);
    channels = avio_rl16(pb);
    rate = avio_rl32(pb);
    avio_skip(pb, 28);
    duration = avio_rl32(pb);
    if (rate <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_XMA2;
    st->codecpar->block_align = 0x800;
    st->codecpar->sample_rate = rate;

    ret = ff_alloc_extradata(st->codecpar, 34);
    if (ret < 0)
        return ret;
    memset(st->codecpar->extradata, 0, 34);
    AV_WL16(st->codecpar->extradata, (channels+1)/2);
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x38, SEEK_SET);

    return 0;
}

const FFInputFormat ff_ue5xma_demuxer = {
    .p.name         = "ue5xma",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Unreal Engine 5 XMA"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "xma",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
