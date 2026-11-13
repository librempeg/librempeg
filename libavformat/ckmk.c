/*
 * Cricket CKMK demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('c','k','m','k'))
        return 0;

    if (p->buf_size < 20)
        return 0;
    if (AV_RL32(p->buf+8) != 0)
        return 0;
    if (AV_RL32(p->buf+12) != 2)
        return 0;
    if (p->buf[17] <= 0)
        return 0;
    if (AV_RL16(p->buf+18) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, codec, align;
    AVIOContext *pb = s->pb;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 8);
    if (avio_rl32(pb) != 0)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4);
    codec = avio_r8(pb);
    channels = avio_r8(pb);
    rate = avio_rl16(pb);
    duration = avio_rl32(pb);
    align = avio_rl16(pb);
    duration *= avio_rl16(pb);

    switch (codec) {
    case 0:
        codec = AV_CODEC_ID_PCM_S16LE;
        break;
    case 1:
        codec = AV_CODEC_ID_PCM_S8;
        break;
    case 2:
        codec = AV_CODEC_ID_ADPCM_CKMK;
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

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 44, SEEK_SET);

    return 0;
}

const FFInputFormat ff_ckmk_demuxer = {
    .p.name         = "ckmk",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Cricket Audio CKMK"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "cks",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
