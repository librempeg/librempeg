/*
 * TRA demuxer
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
    int score = 0;

    for (int i = 0; i+1 < p->buf_size; i += 0x204) {
        if (p->buf[i] == 0xAD)
            score++;
        else
            return 0;
    }

    return FFMIN(AVPROBE_SCORE_MAX, score);
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_TRA;
    st->codecpar->ch_layout.nb_channels = 2;
    st->codecpar->sample_rate = 24000;
    st->codecpar->block_align = 0x204 * 2;
    st->codecpar->bit_rate = 8LL * 0x204 * 2 * 24000 / 0x400;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0, SEEK_SET);

    return 0;
}

const FFInputFormat ff_tra_demuxer = {
    .p.name         = "tra",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Terminal Reality TRA"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "tra",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
