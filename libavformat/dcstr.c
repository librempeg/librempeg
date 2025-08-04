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

static int dcstr_probe(const AVProbeData *p)
{
    if ((int32_t)AV_RL32(p->buf) <= 0)
        return 0;

    if ((int32_t)AV_RL32(p->buf+4) <= 0)
        return 0;

    if (p->buf_size < 224 || memcmp(p->buf + 213, "Sega Stream", 11))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int dcstr_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    unsigned codec, align;
    int mult;
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = avio_rl32(pb);
    if (st->codecpar->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;
    st->codecpar->sample_rate = avio_rl32(pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    codec                  = avio_rl32(pb);
    align                  = avio_rl32(pb);
    avio_skip(pb, 4);
    st->start_time = 0;
    st->duration           = avio_rl32(pb);
    mult                   = avio_rl32(pb);
    if (st->codecpar->ch_layout.nb_channels <= 0 || mult <= 0 ||
        mult > INT_MAX / st->codecpar->ch_layout.nb_channels) {
        av_log(s, AV_LOG_ERROR, "invalid number of channels %d x %d\n",
               st->codecpar->ch_layout.nb_channels, mult);
        return AVERROR_INVALIDDATA;
    }
    st->codecpar->ch_layout.nb_channels *= mult;
    if (!align || align > INT_MAX / st->codecpar->ch_layout.nb_channels)
        return AVERROR_INVALIDDATA;
    st->codecpar->block_align = align * st->codecpar->ch_layout.nb_channels;

    switch (codec) {
    case  4: st->codecpar->codec_id = AV_CODEC_ID_ADPCM_AICA;       break;
    case 16: st->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE_PLANAR; break;
    default: avpriv_request_sample(s, "codec %X", codec);
             return AVERROR_PATCHWELCOME;
    }

    avio_skip(pb, 0x800 - avio_tell(pb));
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int dcstr_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par    = s->streams[0]->codecpar;
    return av_get_packet(s->pb, pkt, par->block_align);
}

const FFInputFormat ff_dcstr_demuxer = {
    .p.name         = "dcstr",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sega DC STR"),
    .p.extensions   = "str",
    .p.flags        = AVFMT_GENERIC_INDEX | AVFMT_NO_BYTE_SEEK | AVFMT_NOBINSEARCH,
    .read_probe     = dcstr_probe,
    .read_header    = dcstr_read_header,
    .read_packet    = dcstr_read_packet,
};
