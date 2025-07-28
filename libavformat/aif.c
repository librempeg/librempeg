/*
 * AIF demuxer
 * Copyright (c) 2024 Paul B Mahol
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

#include "libavutil/avassert.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int aif_probe(const AVProbeData *p)
{
    if (AV_RL16(p->buf+0x00) != 0x69 ||
        AV_RL16(p->buf+0x02) != 0x02 ||
        AV_RL16(p->buf+0x0e) != 0x04 ||
        AV_RL16(p->buf+0x0c) != 0x48 ||
        AV_RL32(p->buf+0x04) < 8000  ||
        AV_RL32(p->buf+0x04) > 192000)
        return 0;

    return AVPROBE_SCORE_MAX / 2;
}

static int aif_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 2);
    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_IMA_XBOX;
    st->codecpar->ch_layout.nb_channels = avio_rl16(pb);
    if (st->codecpar->ch_layout.nb_channels == 0)
        return AVERROR_INVALIDDATA;

    st->codecpar->sample_rate = avio_rl32(pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;

    st->codecpar->bit_rate = avio_rl32(pb) * 8LL;
    st->codecpar->block_align = avio_rl16(pb);
    if (st->codecpar->block_align == 0)
        return AVERROR_INVALIDDATA;

    st->codecpar->bits_per_coded_sample = avio_rl16(pb);

    avio_seek(s->pb, 20, SEEK_SET);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int aif_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;

    return av_get_packet(s->pb, pkt, par->block_align);
}

const FFInputFormat ff_aif_demuxer = {
    .p.name         = "aif",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Asobo Studio Games AIF"),
    .p.extensions   = "aif,laif",
    .read_probe     = aif_probe,
    .read_header    = aif_read_header,
    .read_packet    = aif_read_packet,
};
