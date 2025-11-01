/*
 * ASTL demuxer
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
    if (AV_RL32(p->buf) != MKTAG('A','S','T','L'))
        return 0;

    if (p->buf_size < 0x38)
        return 0;
    if (AV_RL32(p->buf + 0x10) == 0)
        return 0;
    if (AV_RL32(p->buf + 0x20) == 0)
        return 0;
    if (AV_RL16(p->buf + 0x32) == 0)
        return 0;
    if ((int)AV_RL32(p->buf + 0x34) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t start_offset, data_size, duration;
    int channels, codec, rate, align = 0;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 16);
    start_offset = avio_rl32(pb);
    avio_skip(pb, 12);
    data_size = avio_rl32(pb);
    avio_skip(pb, 12);
    codec = avio_rl16(pb);
    channels = avio_rl16(pb);
    rate = avio_rl32(pb);
    if (rate <= 0 || channels == 0 || data_size == 0 || start_offset == 0)
        return AVERROR_INVALIDDATA;

    switch (codec) {
    case 0x0001:
        align = 2;
        codec = AV_CODEC_ID_PCM_S16LE;
        duration = data_size / (2 * channels);
        break;
    default:
        avpriv_request_sample(s, "codec 0x%X", codec);
        return AVERROR_PATCHWELCOME;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = align;
    st->codecpar->sample_rate = rate;
    st->codecpar->codec_id = codec;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_astl_demuxer = {
    .p.name         = "astl",
    .p.long_name    = NULL_IF_CONFIG_SMALL("ASTL Audio"),
    .p.extensions   = "ast",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
