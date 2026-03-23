/*
 * Sega GCub demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('G','C','u','b'))
        return 0;

    if ((int)AV_RB32(p->buf+4) <= 0)
        return 0;
    if ((int)AV_RB32(p->buf+8) <= 0)
        return 0;
    if (AV_RB32(p->buf+12) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, ret, align;
    AVIOContext *pb = s->pb;
    int64_t start;
    AVStream *st;

    start = 0x60;
    avio_skip(pb, 4);
    channels = avio_rb32(pb);
    rate = avio_rb32(pb);
    /*data_size = */avio_rb32(pb);
    align = 0x8000;
    if (rate <= 0 || channels <= 0 || align <= 0 || channels >= INT_MAX/align)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align * channels;
    st->codecpar->bit_rate = 8LL * st->codecpar->ch_layout.nb_channels * 8 *
                                   st->codecpar->sample_rate / 14;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x10, SEEK_SET);
    ret = ff_get_extradata(s, st->codecpar, pb, 32 * channels);
    if (ret < 0)
        return ret;

    avio_seek(pb, start, SEEK_SET);
    if (avio_rb32(pb) == MKBETAG('G','C','x','x'))
        avio_skip(pb, 0x24);
    else
        avio_skip(pb, -4);

    return 0;
}

const FFInputFormat ff_gcub_demuxer = {
    .p.name         = "gcub",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sega GCub"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "gcub",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
