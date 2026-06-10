/*
 * Valve XWV demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('X','W','V',' '))
        return 0;

    if (p->buf_size < 0x2c)
        return 0;
    if (AV_RB32(p->buf+4) != 4)
        return 0;
    if (p->buf[0x2a] > 2)
        return 0;
    if (p->buf[0x2b] == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, ret, align, codec, freq;
    int64_t start_offset, duration;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 16);
    start_offset = avio_rb32(pb);
    /* data_size = */avio_rb32(pb);
    duration = avio_rb32(pb);
    avio_skip(pb, 12);
    codec = avio_r8(pb);
    avio_skip(pb, 1);
    freq = avio_r8(pb);
    channels = avio_r8(pb);

    switch (freq) {
    case 0x00: rate = 11025; break;
    case 0x01: rate = 22050; break;
    case 0x02: rate = 44100; break;
    default:
        return AVERROR_INVALIDDATA;
    }

    switch (codec) {
    case 0:
        codec = AV_CODEC_ID_PCM_S16BE;
        align = 2 * channels;
        break;
    case 1:
        codec = AV_CODEC_ID_XMA2;
        align = 0x800;
        break;
    case 3:
        codec = AV_CODEC_ID_MP3;
        align = 1024;
        break;
    default:
        avpriv_request_sample(s, "codec %02x", codec);
        break;
    }

    if (rate <= 0 || channels <= 0 || align <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    if (codec = AV_CODEC_ID_XMA2) {
        ret = ff_alloc_extradata(st->codecpar, 34);
        if (ret < 0)
            return ret;

        memset(st->codecpar->extradata, 0, 34);
        AV_WL16(st->codecpar->extradata, (channels+1)/2);
    }

    if (codec == AV_CODEC_ID_XMA2 || codec == AV_CODEC_ID_MP3)
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = align;
    st->codecpar->sample_rate = rate;
    st->codecpar->codec_id = codec;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_xwv_demuxer = {
    .p.name         = "xwv",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Valve XWV"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "wav,lwav",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
