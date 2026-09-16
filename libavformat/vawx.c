/*
 * feelplus VAWX demuxer
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
    if (AV_RB32(p->buf) != MKTAG('X','W','A','V'))
        return 0;

    if (p->buf_size < 0x44)
        return 0;
    if (p->buf[0x39] == 0)
        return 0;
    if (AV_RB32(p->buf + 0x3c) == 0)
        return 0;
    if ((int)AV_RB32(p->buf + 0x40) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, codec, align, ret;
    AVIOContext *pb = s->pb;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 48);
    avio_skip(pb, 6);
    codec = avio_r8(pb);
    avio_skip(pb, 2);
    channels = avio_r8(pb);
    avio_skip(pb, 2);
    duration = avio_rb32(pb);
    rate = avio_rb32(pb);

    switch (codec) {
    case 1:
        align = 0x800;
        codec = AV_CODEC_ID_XMA2;
        break;
    case 2:
        align = 16 * channels;
        codec = AV_CODEC_ID_ADPCM_PSX;
        break;
    case 6:
    case 7:
    case 8:
        rate = 48000;
        align = (codec == 8 ? 0xC0 : codec == 0x07 ? 0x98 : 0x60) * channels;
        codec = AV_CODEC_ID_ATRAC3;
        break;
    default:
        avpriv_request_sample(s, "codec %X", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (rate <= 0 || channels <= 0 || align <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    if (codec == AV_CODEC_ID_XMA2) {
        ret = ff_alloc_extradata(st->codecpar, 34);
        if (ret < 0)
            return ret;

        memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
        AV_WL16(st->codecpar->extradata, (channels+1)/2);
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;
    } else if (codec == AV_CODEC_ID_ATRAC3) {
        ret = ff_alloc_extradata(st->codecpar, 14);
        if (ret < 0)
            return ret;

        AV_WL16(st->codecpar->extradata, 1);
        AV_WL16(st->codecpar->extradata+2, 2048 * channels);
        AV_WL16(st->codecpar->extradata+4, 0);
        AV_WL16(st->codecpar->extradata+6, (align == 0x60 * channels) ? 1 : 0);
        AV_WL16(st->codecpar->extradata+8, (align == 0x60 * channels) ? 1 : 0);
        AV_WL16(st->codecpar->extradata+10, 1);
        AV_WL16(st->codecpar->extradata+12, 0);
    }

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->codec_id = codec;
    st->codecpar->block_align = align;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

const FFInputFormat ff_vawx_demuxer = {
    .p.name         = "vawx",
    .p.long_name    = NULL_IF_CONFIG_SMALL("feelplus VAWX"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "xwv",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
