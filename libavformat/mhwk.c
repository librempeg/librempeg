/*
 * MHWK demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('M','H','W','K'))
        return 0;

    if (p->buf_size < 12)
        return 0;
    if (AV_RB32(p->buf+8) != MKBETAG('W','A','V','E'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t start_offset, duration, data_offset = 0;
    int rate, channels, align, codec;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 12);
    while (!avio_feof(pb)) {
        uint32_t chunk = avio_rb32(pb);
        int64_t size = avio_rb32(pb);

        switch (chunk) {
        case MKBETAG('D','a','t','a'):
            data_offset = avio_tell(pb);
            break;
        case MKBETAG('C','u','e','#'):
        case MKBETAG('A','D','P','C'):
            avio_skip(pb, size);
            break;
        default:
            return AVERROR_INVALIDDATA;
        }

        if (data_offset)
            break;
    }

    if (data_offset == 0)
        return AVERROR_INVALIDDATA;

    rate = avio_rb16(pb);
    duration = avio_rb32(pb);
    avio_skip(pb, 1);
    channels = avio_r8(pb);
    codec = avio_rb16(pb);
    start_offset = data_offset + 20;

    switch (codec) {
    case 0:
        codec = AV_CODEC_ID_PCM_U8;
        align = channels;
        break;
    case 1:
        codec = AV_CODEC_ID_ADPCM_IMA_WS;
        align = 1;
        break;
    case 2:
        codec = AV_CODEC_ID_MP3;
        align = 1024;
        break;
    default:
        avpriv_request_sample(s, "codec %02x", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (rate <= 0 || channels <= 0 || align <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    if (codec == AV_CODEC_ID_MP3)
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
    if (codec == AV_CODEC_ID_ADPCM_IMA_WS)
        st->codecpar->profile = 4;

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

const FFInputFormat ff_mhwk_demuxer = {
    .p.name         = "mhwk",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Broderbund's Mohawk"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "mhk",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
