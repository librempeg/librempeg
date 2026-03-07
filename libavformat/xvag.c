/*
 * XVAG demuxer
 * Copyright (c) 2015 Paul B Mahol
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

#include "libavutil/bswap.h"
#include "libavcodec/internal.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int xvag_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "XVAG", 4) ||
        memcmp(p->buf+32, "fmat", 4))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int xvag_read_header(AVFormatContext *s)
{
    unsigned offset, big_endian, codec;
    int nb_channels, rate, align;
    AVIOContext *pb = s->pb;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 4);

    offset     = avio_rl32(pb);
    big_endian = offset > av_bswap32(offset);
    if (big_endian) {
        offset = av_bswap32(offset);
        avio_skip(pb, 28);
        codec = avio_rb32(pb);
        nb_channels = avio_rb32(pb);
        avio_skip(pb, 4);
        duration = avio_rb32(pb);
        avio_skip(pb, 8);
        rate = avio_rb32(pb);
    } else {
        avio_skip(pb, 28);
        codec = avio_rl32(pb);
        nb_channels = avio_rl32(pb);
        avio_skip(pb, 4);
        duration = avio_rl32(pb);
        avio_skip(pb, 8);
        rate = avio_rl32(pb);
    }

    switch (codec) {
    case 0x1c:
        codec = AV_CODEC_ID_ADPCM_PSX;
        if (nb_channels > INT_MAX/16)
            return AVERROR_INVALIDDATA;
        align = 16 * nb_channels;
        break;
    default:
        avpriv_request_sample(s, "codec %X", codec);
        return AVERROR_PATCHWELCOME;
    };

    avio_skip(pb, offset - avio_tell(pb));

    if (avio_rb16(pb) == 0xFFFB) {
        codec = AV_CODEC_ID_MP3;
        align = 0x1000;
    }
    avio_skip(pb, -2);

    if (rate <= 0 || nb_channels <= 0 || align <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->block_align = align;

    if (codec == AV_CODEC_ID_MP3)
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_xvag_demuxer = {
    .p.name         = "xvag",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony PS3 XVAG"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "xvag",
    .read_probe     = xvag_probe,
    .read_header    = xvag_read_header,
    .read_packet    = ff_pcm_read_packet,
};
