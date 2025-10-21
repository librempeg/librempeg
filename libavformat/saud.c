/*
 * SMUSH SAUD demuxer
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
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S','A','U','D'))
        return 0;

    if (AV_RN32(p->buf+4) == 0)
        return 0;

    if (p->buf_size < 12)
        return 0;

    if (AV_RB32(p->buf+8) != MKBETAG('S','T','R','K'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int64_t offset;
    AVStream *st;

    avio_skip(pb, 12);
    offset = avio_rb32(pb);
    avio_skip(pb, offset);
    if (avio_rb32(pb) != MKBETAG('S','D','A','T'))
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_PCM_U8;
    st->codecpar->ch_layout.nb_channels = 1;
    st->start_time = 0;
    st->codecpar->sample_rate = 22050;
    st->codecpar->block_align = 1;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_saud_demuxer = {
    .p.name         = "saud",
    .p.long_name    = NULL_IF_CONFIG_SMALL("LucasArts SAUD"),
    .p.extensions   = "sad",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
    .read_seek      = ff_pcm_read_seek,
};
