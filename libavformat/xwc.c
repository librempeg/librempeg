/*
 * XWC demuxer
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
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 16)
        return 0;

    if (AV_RB32(p->buf) != 0x00030000 &&
        AV_RB32(p->buf) != 0x00040000)
        return 0;

    if (AV_RB32(p->buf+4) != 0x00900000)
        return 0;

    if ((int32_t)AV_RL32(p->buf+12) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX * 2 / 3;
}

static int read_header(AVFormatContext *s)
{
    int64_t start, extra_offset, chunk_offset, duration;
    uint32_t version, codec, seek_size, chunk_size;
    AVIOContext *pb = s->pb;
    int nb_channels, rate;
    AVStream *st;
    int ret;

    version = avio_rb32(pb);
    avio_skip(pb, 8);
    nb_channels = avio_rl32(pb);
    avio_skip(pb, 12 + 8 * (version == 0x00040000));
    codec = avio_rb32(pb);
    duration = avio_rl32(pb);
    switch (codec) {
    case MKBETAG('X','M','A','\0'):
        codec = AV_CODEC_ID_XMA2;
        extra_offset = 0x28;
        avio_seek(pb, extra_offset, SEEK_SET);
        seek_size = avio_rl32(pb);
        avio_seek(pb, extra_offset + seek_size + 4, SEEK_SET);
        chunk_size = avio_rl32(pb);
        chunk_offset = extra_offset + 4 + seek_size + 4;
        start = chunk_offset + 4 + chunk_size;
        start = FFALIGN(start, 0x800);

        switch (chunk_size) {
        case 0x2c:
            chunk_offset = extra_offset + 4 + seek_size + 16;
            avio_seek(pb, chunk_offset, SEEK_SET);
            rate = avio_rb32(pb);
            break;
        case 0x34:
            chunk_offset = extra_offset + 4 + seek_size + 8;
            avio_seek(pb, chunk_offset, SEEK_SET);
            rate = avio_rl32(pb);
            break;
        default:
            rate = 0;
            break;
        }
        break;
    case MKBETAG('M','P','E','G'):
        codec = AV_CODEC_ID_MP3;
        start = 0x800;
        rate = 44100;
        break;
    default:
        avpriv_request_sample(s, "codec %08X", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (nb_channels <= 0 || rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 0x800;

    switch (codec) {
    case AV_CODEC_ID_XMA2:
        ret = ff_alloc_extradata(st->codecpar, 34);
        if (ret < 0)
            return ret;
        memset(st->codecpar->extradata, 0, 34);
        AV_WL16(st->codecpar->extradata, 1);
        break;
    }
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_xwc_demuxer = {
    .p.name         = "xwc",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Starbreeze XWC"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "xwc",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
