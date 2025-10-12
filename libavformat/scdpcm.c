/*
 * SCDPCM demuxer
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

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB16(p->buf) != 1 &&
        AV_RB16(p->buf) != 2)
        return 0;

    if (p->buf_size < 0x800)
        return 0;

    for (int i = 32; i < 0x800; i++) {
        if (p->buf[i])
            return 0;
    }

    return 3*AVPROBE_SCORE_MAX/4;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int nb_channels;
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    nb_channels = 1 + (avio_rb16(pb) == 1);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_PCM_SGA;
    st->codecpar->sample_rate = 32500;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->block_align = 0x800 * nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

const FFInputFormat ff_scdpcm_demuxer = {
    .p.name         = "scdpcm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sega CD PCM"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "pcm",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
