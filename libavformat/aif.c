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
#include "pcm.h"

static int read_probe(const AVProbeData *p)
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

static int read_header(AVFormatContext *s)
{
    int nb_channels, rate, align;
    AVIOContext *pb = s->pb;
    int64_t bit_rate;
    AVStream *st;

    avio_skip(pb, 2);
    nb_channels = avio_rl16(pb);
    rate = avio_rl32(pb);
    bit_rate = avio_rl32(pb) * 8LL;
    align = avio_rl16(pb);
    if (nb_channels <= 0 || rate <= 0 || align <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_XBOX;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->bit_rate = bit_rate;
    st->codecpar->block_align = align;

    st->codecpar->bits_per_coded_sample = avio_rl16(pb);

    avio_seek(pb, 20, SEEK_SET);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_aif_demuxer = {
    .p.name         = "aif",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Asobo Studio Games AIF"),
    .p.extensions   = "aif,laif",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
