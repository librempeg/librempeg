/*
 * feelplus XWAV demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('X','W','A','V'))
        return 0;

    if (p->buf[0x26] != 2 && p->buf[0x26] != 4)
        return 0;
    if (p->buf[0x27] == 0)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, codec, align, tracks, ret;
    AVIOContext *pb = s->pb;
    int64_t start, duration;
    AVStream *st;

    avio_skip(pb, 16);
    start = avio_rl16(pb) * 16LL;
    avio_skip(pb, 20);
    codec = avio_r8(pb);
    tracks = avio_r8(pb);

    switch (codec) {
    case 2:
        avio_skip(pb, 8);
        duration = avio_rl32(pb);
        rate = avio_rl16(pb);
        avio_skip(pb, 1);
        channels = avio_r8(pb);
        align = 16 * channels;
        if (tracks > 1)
            return AVERROR_INVALIDDATA;
        codec = AV_CODEC_ID_ADPCM_PSX;
        break;
    case 4:
        avio_skip(pb, 4);
        duration = avio_rl32(pb);
        avio_skip(pb, 4);
        rate = avio_rl16(pb);
        align = 0x800;
        channels = 0;
        for (int i = 0; i < tracks; i++) {
            avio_seek(pb, 64 + 16 * i + 7, SEEK_SET);
            channels += avio_r8(pb);
        }
        codec = AV_CODEC_ID_XMA2;
        break;
    default:
        avpriv_request_sample(s, "codec %X", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (rate <= 0 || channels <= 0 || align <= 0 || channels >= INT_MAX/align)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    if (codec == AV_CODEC_ID_XMA2) {
        ret = ff_alloc_extradata(st->codecpar, 34);
        if (ret < 0)
            return ret;

        memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
        AV_WL16(st->codecpar->extradata, tracks);
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;
    }

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->codec_id = codec;
    st->codecpar->block_align = align;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_xwav_demuxer = {
    .p.name         = "xwav",
    .p.long_name    = NULL_IF_CONFIG_SMALL("feelplus XWAV"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "xwv",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
