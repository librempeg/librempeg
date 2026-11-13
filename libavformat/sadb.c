/*
 * Procyon Studio SADB demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('s','a','d','b'))
        return 0;

    if (p->buf_size < 0x60)
        return 0;
    if (p->buf[50] <= 0)
        return 0;
    if ((int)AV_RB32(p->buf+0x5c) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int rate, channels, ret;
    int64_t start;
    AVStream *st;

    avio_skip(pb, 50);
    channels = avio_r8(pb);
    avio_skip(pb, 21);
    start = avio_rb32(pb);
    avio_skip(pb, 16);
    rate = avio_rb32(pb);
    if (rate <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 16 * channels;
    st->codecpar->bit_rate = 8LL * channels * 8 * rate / 14;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    ret = ff_alloc_extradata(st->codecpar, channels * 32);
    if (ret < 0)
        return ret;

    avio_seek(pb, 156, SEEK_SET);
    for (int ch = 0; ch < channels; ch++) {
        avio_read(pb, st->codecpar->extradata + 32 * ch, 32);
        avio_skip(pb, 64);
    }

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_sadb_demuxer = {
    .p.name         = "sadb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Procyon Studio SADB"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "sad",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
