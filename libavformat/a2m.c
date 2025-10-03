/*
 * a2m demuxer
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

static int a2m_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "A2M\0PS2\0", 8) || (int)AV_RB32(p->buf + 0x10) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int a2m_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_seek(pb, 0x10, SEEK_SET);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->ch_layout.nb_channels = 2;
    st->codecpar->block_align = 0x6000 * st->codecpar->ch_layout.nb_channels;
    st->codecpar->sample_rate = avio_rb32(pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                    st->codecpar->sample_rate / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x30, SEEK_SET);
    return 0;
}

const FFInputFormat ff_a2m_demuxer = {
    .p.name         = "a2m",
    .p.long_name    = NULL_IF_CONFIG_SMALL("A2M (Artificial Mind & Movement)"),
    .p.extensions   = "int",
    .read_probe     = a2m_probe,
    .read_header    = a2m_read_header,
    .read_packet    = ff_pcm_read_packet,
};
