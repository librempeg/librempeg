/*
 * Square Enix BGW demuxer
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
    if (memcmp(p->buf, "BGMStream\0\0\0", 12))
        return 0;

    if (p->buf_size < 0x30)
        return 0;
    if ((int)AV_RL32(p->buf + 0x18) <= 0)
        return 0;
    if (!p->buf[0x2e])
        return 0;
    if (!p->buf[0x2f])
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, codec, align;
    AVIOContext *pb = s->pb;
    int64_t start;
    AVStream *st;

    avio_skip(pb, 12);
    codec = avio_rl32(pb);
    avio_skip(pb, 16);
    rate = avio_rl32(pb);
    rate += avio_rl32(pb);
    rate &= 0x7FFFFFFF;
    start = avio_rl32(pb);
    avio_skip(pb, 2);
    channels = avio_r8(pb);
    align = avio_r8(pb);

    switch (codec) {
    case 0:
        codec = AV_CODEC_ID_ADPCM_PSXC;
        break;
    default:
        avpriv_request_sample(s, "codec %04x", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (rate <= 0 || channels <= 0 || align <= 0 || channels >= INT_MAX/align)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = ((align/2)+1) * channels;
    st->codecpar->bit_rate = 8LL * st->codecpar->ch_layout.nb_channels *
                                   st->codecpar->sample_rate / 2;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_bgw_demuxer = {
    .p.name         = "bgw",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Square Enix BGW"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "bgw",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
