/*
 * CPS demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('C','P','S',' '))
        return 0;

    if (p->buf_size < 20)
        return 0;
    if (AV_RB32(p->buf + 4) == 0)
        return 0;
    if ((int)AV_RB32(p->buf + 8) <= 0)
        return 0;
    if ((int)AV_RB32(p->buf + 16) <= 0)
        return 0;
    if (AV_RB32(p->buf + 4) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t start_offset, bit_rate = 0;
    int align, rate, channels, codec;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    start_offset = avio_rb32(pb);
    channels = avio_rb32(pb);
    avio_skip(pb, 4);
    rate = avio_rb32(pb);
    avio_skip(pb, 12);
    codec = avio_rb32(pb);
    switch (codec) {
    case 0:
        codec = AV_CODEC_ID_PCM_S16BE;
        align = 2;
        break;
    default:
        codec = AV_CODEC_ID_ADPCM_PSX;
        bit_rate = 16LL * rate * channels * 8 / 28;
        align = 16;
        break;
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
    st->codecpar->block_align = align * channels;
    st->codecpar->sample_rate = rate;
    if (bit_rate)
        st->codecpar->bit_rate = bit_rate;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_cps_demuxer = {
    .p.name         = "cps",
    .p.long_name    = NULL_IF_CONFIG_SMALL("tri-Crescendo CPS"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "cps",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
