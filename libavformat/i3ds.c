/*
 * i3DS demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('i','3','D','S'))
        return 0;

    if (p->buf_size < 0x68)
        return 0;
    if ((int)AV_RL32(p->buf+12) < 0)
        return 0;
    if ((int)AV_RL32(p->buf+0x5c) <= 0)
        return 0;
    if (AV_RL32(p->buf+0x64) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, ret, align;
    AVIOContext *pb = s->pb;
    int64_t start, duration;
    AVStream *st;

    avio_skip(pb, 12);
    align = avio_rl32(pb);
    channels = 1 + (align != 0);
    avio_seek(pb, 0x5c, SEEK_SET);
    rate = avio_rl32(pb);
    avio_seek(pb, 0x64, SEEK_SET);
    duration = avio_rl32(pb);
    start = 32 + 192 * channels;
    if (rate <= 0 || align < 0 || align > INT_MAX/channels)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP_LE;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align ? align * 2 : 1024;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x8c, SEEK_SET);
    if ((ret = ff_alloc_extradata(st->codecpar, 32 * channels)) < 0)
        return ret;

    for (int ch = 0; ch < channels; ch++) {
        avio_read(pb, st->codecpar->extradata + ch * 32, 32);
        avio_skip(pb, 160);
    }

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_i3ds_demuxer = {
    .p.name         = "i3ds",
    .p.long_name    = NULL_IF_CONFIG_SMALL("i3DS"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "3ds",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
