/*
 * AMTS demuxer
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
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('A','M','T','S'))
        return 0;

    if (p->buf_size < 24)
        return 0;

    if ((int)AV_RB32(p->buf+8) <= 0)
        return 0;
    if ((int)AV_RB32(p->buf+12) <= 0)
        return 0;
    if ((int)AV_RB32(p->buf+16) <= 0)
        return 0;
    if ((int)AV_RB32(p->buf+20) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int interleave, bps, nb_channels, rate, ret;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 8);
    interleave = avio_rb32(pb);
    rate = avio_rb32(pb);
    bps = avio_rb32(pb);
    nb_channels = avio_rb32(pb);
    if (nb_channels <= 0 || rate <= 0 || bps <= 0 || interleave <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->bits_per_coded_sample = bps;
    st->codecpar->sample_rate = rate;
    switch (bps) {
    case 4:
        avio_skip(pb, 8);
        st->duration = avio_rb32(pb);
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
        st->codecpar->block_align = interleave * nb_channels;

        ret = ff_alloc_extradata(st->codecpar, 32 * nb_channels);
        if (ret < 0)
            return ret;

        avio_seek(pb, 0x3c, SEEK_SET);
        for (int ch = 0; ch < nb_channels; ch++) {
            avio_read(pb, st->codecpar->extradata + ch * 32, 32);
            avio_skip(pb, 0x40);
        }
        break;
    case 16:
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S16BE;
        st->codecpar->block_align = 1024 * nb_channels;
        break;
    default:
        avpriv_request_sample(s, "bps 0x%X", bps);
        return AVERROR_PATCHWELCOME;
    }

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_amts_demuxer = {
    .p.name         = "amts",
    .p.long_name    = NULL_IF_CONFIG_SMALL("AMTS Audio"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "lstm",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
