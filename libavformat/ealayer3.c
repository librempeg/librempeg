/*
 * Electronic Arts Layer 3 file demuxer
 * Copyright (c) 2007 Peter Ross
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

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavcodec/mpegaudiodecheader.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int ealayer3_probe(const AVProbeData *p)
{
    int score = 0, i = 0;

    while (i + 10 < p->buf_size) {
        unsigned int nb_samples = AV_RB32(p->buf + i + 4);
        unsigned int size = AV_RB32(p->buf + i);
        MPADecodeHeader c;

        if (size < 10 || size > 0x10000 ||
            !nb_samples || nb_samples > 0x10000 ||
            (p->buf[i + 8] && p->buf[i + 8] != 0xEE) ||
            avpriv_ealayer3_decode_header(&c, p->buf[i + 9]) < 0)
            break;

        score += 3;
        if (score >= AVPROBE_SCORE_MAX)
            break;

        i += size;
    }

    return FFMIN(AVPROBE_SCORE_MAX, score);
}

static int ealayer3_read_header(AVFormatContext *s)
{
    MPADecodeHeader c;
    AVStream *st;

    avio_skip(s->pb, 9);
    avpriv_ealayer3_decode_header(&c, avio_r8(s->pb));
    avio_seek(s->pb, 0, SEEK_SET);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_EALAYER3;
    st->codecpar->ch_layout.nb_channels = c.nb_channels;
    st->codecpar->sample_rate = c.sample_rate;

    avpriv_set_pts_info(st, 64, 1, c.sample_rate);

    return 0;
}

static int ealayer3_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int64_t pos = avio_tell(s->pb);
    int size, duration, ret;

    size = avio_rb32(s->pb);
    if (size < 10)
        return AVERROR_INVALIDDATA;
    duration = avio_rb32(s->pb);

    ret = av_get_packet(s->pb, pkt, size - 8);
    if (ret < 0)
        return ret;

    pkt->pos = pos;
    pkt->stream_index = 0;
    pkt->duration = duration;

    return 0;
}

const FFInputFormat ff_ealayer3_demuxer = {
    .p.name         = "ealayer3",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Electronic Arts Layer 3"),
    .p.extensions   = "cdata",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = ealayer3_probe,
    .read_header    = ealayer3_read_header,
    .read_packet    = ealayer3_read_packet,
};
