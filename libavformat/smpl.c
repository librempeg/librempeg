/*
 * SMPL demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('S','M','P','L'))
        return 0;
    if ((int)AV_RB32(p->buf+16) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    char title[17];
    int ret, rate;
    AVStream *st;

    avio_skip(pb, 0x10);
    rate = avio_rb32(pb);
    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 16 * st->codecpar->ch_layout.nb_channels;
    st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                    st->codecpar->sample_rate / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x20, SEEK_SET);
    if ((ret = avio_get_str(pb, 16, title, sizeof(title))) < 0)
        return ret;
    if (title[0] != '\0')
        av_dict_set(&s->metadata, "title", title, 0);

    avio_seek(pb, 0x40, SEEK_SET);

    return 0;
}

const FFInputFormat ff_smpl_demuxer = {
    .p.name         = "smpl",
    .p.long_name    = NULL_IF_CONFIG_SMALL("PS2 SMPL"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "v0,v1",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
