/*
 * ZSD demuxer
 * Copyright (c) 2025 smiRaphi
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
    if (AV_RB32(p->buf) != MKBETAG('Z','S','D','\0'))
        return 0;
    if (p->buf_size < 20)
        return 0;
    if (AV_RL32(p->buf + 4) != 0x1000)
        return 0;
    if (AV_RL32(p->buf + 8) != 0xC)
        return 0;
    if ((int)AV_RL32(p->buf + 12) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 16) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t duration, start_offset;
    AVIOContext *pb = s->pb;
    int channels, rate;
    AVStream *st;

    avio_skip(pb, 12);
    channels = avio_rl32(pb);
    rate = avio_rl32(pb);
    avio_skip(pb, 4);
    duration = avio_rl32(pb);
    avio_skip(pb, 4);
    start_offset = avio_rl32(pb);
    if (channels <= 0 || rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration / channels;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_PCM_S8;
    st->codecpar->block_align = channels;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_zsd_demuxer = {
    .p.name         = "zsd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("ZSD (Dragon Booster DS)"),
    .p.extensions   = "zsd",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
    .read_seek      = ff_pcm_read_seek,
};
