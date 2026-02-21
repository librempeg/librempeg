/*
 * Dreamcast SPSD demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('S','P','S','D'))
        return 0;
    if (p->buf_size < 0x30)
        return 0;
    if (AV_RB32(p->buf+4) != 0x01010004 &&
        AV_RB32(p->buf+4) != 0x00010004)
        return 0;
    if (AV_RL16(p->buf+0x0A) != 0 &&
        AV_RL16(p->buf+0x0A) != 0xd &&
        AV_RL16(p->buf+0x0A) != 0xff)
        return 0;
    if (AV_RL32(p->buf+0x0C) == 0)
        return 0;
    if (AV_RL16(p->buf+0x2A) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int align, rate, codec, flags, channels, index, data_size;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 8);
    codec = avio_r8(pb);
    flags = avio_r8(pb);
    index = avio_rl16(pb);
    data_size = avio_rl32(pb);
    avio_seek(pb, 0x2A, SEEK_SET);
    rate = avio_rl16(pb);
    channels = (flags & 3) ? 2 : 1;
    if (rate <= 0 || data_size <= 0)
        return AVERROR_INVALIDDATA;

    switch (index) {
    case 0:
        if (channels > 1)
            return AVERROR_INVALIDDATA;
        align = 0x1000;
        break;
    case 0xd:
        align = 0x2000;
        break;
    case 0xff:
        align = data_size / channels;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    switch (codec) {
    case 0:
        codec = AV_CODEC_ID_PCM_S16LE;
        break;
    case 1:
        codec = AV_CODEC_ID_PCM_S8;
        break;
    case 3:
        codec = AV_CODEC_ID_ADPCM_AICA;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = align * channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->codec_id = codec;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x40, SEEK_SET);

    return 0;
}

const FFInputFormat ff_spsd_demuxer = {
    .p.name         = "spsd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Dreamcast Naomi SPSD"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "str,spsd",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
