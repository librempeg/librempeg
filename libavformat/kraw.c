/*
 * kRAW demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('k','R','A','W'))
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 8);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_PCM_S16BE;
    st->codecpar->ch_layout.nb_channels = 2;
    st->codecpar->sample_rate = 16000;
    st->codecpar->block_align = 4;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_kraw_demuxer = {
    .p.name         = "kraw",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Geometry Wars kRAW"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "kraw",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
