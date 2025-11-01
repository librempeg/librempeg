/*
 * DC STR demuxer
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

#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if ((int32_t)AV_RL32(p->buf) <= 0)
        return 0;

    if ((int32_t)AV_RL32(p->buf+4) <= 0)
        return 0;

    if (p->buf_size < 224 || memcmp(p->buf + 213, "Sega Stream", 11))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int mult, align, channels, rate;
    AVIOContext *pb = s->pb;
    int64_t duration = 0;
    unsigned codec;
    AVStream *st;

    channels = avio_rl32(pb);
    rate = avio_rl32(pb);
    codec = avio_rl32(pb);
    align = avio_rl32(pb);
    avio_skip(pb, 4);
    if (codec == 4)
        duration = avio_rl32(pb);
    else
        avio_skip(pb, 4);
    mult = avio_rl32(pb);
    if (channels <= 0 || mult <= 0 || mult > INT_MAX/channels)
        return AVERROR_INVALIDDATA;
    channels *= mult;
    if (rate <= 0 || align <= 0 || align > INT_MAX/channels)
        return AVERROR_INVALIDDATA;

    switch (codec) {
    case  4: codec = AV_CODEC_ID_ADPCM_AICA;       break;
    case  8: codec = AV_CODEC_ID_PCM_U8;           break;
    case 16: codec = AV_CODEC_ID_PCM_S16LE_PLANAR; break;
    default: avpriv_request_sample(s, "codec %X", codec);
             return AVERROR_PATCHWELCOME;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    if (duration)
        st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->codec_id = codec;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align * channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    return av_get_packet(s->pb, pkt, par->block_align);
}

const FFInputFormat ff_dcstr_demuxer = {
    .p.name         = "dcstr",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sega DC STR"),
    .p.extensions   = "str",
    .p.flags        = AVFMT_GENERIC_INDEX | AVFMT_NO_BYTE_SEEK | AVFMT_NOBINSEARCH,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
