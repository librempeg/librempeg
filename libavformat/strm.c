/*
 * STRM demuxer
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
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S','T','R','M'))
        return 0;

    if (AV_RB32(p->buf+4) != 0xFFFE0001 &&
        AV_RB32(p->buf+4) != 0xFEFF0001)
        return 0;

    if (p->buf_size < 40)
        return 0;

    if (AV_RB32(p->buf+16) != MKBETAG('H','E','A','D'))
        return 0;

    if (AV_RB8(p->buf+26) == 0)
        return 0;

    if (AV_RL16(p->buf+28) == 0)
        return 0;

    if (AV_RL32(p->buf+36) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int codec, rate, nb_channels, align;
    AVIOContext *pb = s->pb;
    int64_t start, duration;
    AVStream *st;

    avio_skip(pb, 20);
    start = avio_rl32(pb) + 24LL;
    codec = avio_r8(pb);
    avio_skip(pb, 1);
    nb_channels = avio_r8(pb);
    avio_skip(pb, 1);
    rate = avio_rl16(pb);
    avio_skip(pb, 6);
    duration = avio_rl32(pb);

    switch (codec) {
    case 0:
        codec = AV_CODEC_ID_PCM_S8;
        align = 1;
        break;
    case 1:
        codec = AV_CODEC_ID_PCM_S16LE;
        align = 2;
        break;
    case 2:
        codec = AV_CODEC_ID_ADPCM_IMA_NDS;
        avio_skip(pb, 0x30 - avio_tell(pb));
        align = avio_rl32(pb);
        break;
    default:
        avpriv_request_sample(s, "codec 0x%X", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (rate <= 0 || nb_channels <= 0 || align <= 0 || nb_channels >= INT_MAX/align)
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
    st->codecpar->block_align = align * nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_strm_demuxer = {
    .p.name         = "strm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("STRM (Nintendo DS Stream)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "strm",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
