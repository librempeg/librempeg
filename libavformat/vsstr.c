/*
 * VS STR demuxer
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

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S','T','R','L') &&
        AV_RB32(p->buf) != MKBETAG('S','T','R','R') &&
        AV_RB32(p->buf) != MKBETAG('S','T','R','M'))
        return 0;
    if ((int)AV_RL32(p->buf+4) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int channels;
    AVStream *st;

    channels = 1 + (avio_rb32(pb) != AV_RB32("STRM"));

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = 44100;
    st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                    st->codecpar->sample_rate / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *avpkt)
{
    const int channels = s->streams[0]->codecpar->ch_layout.nb_channels;
    AVIOContext *pb = s->pb;
    uint32_t size;
    int64_t pos;
    int ret;

    pos = avio_tell(pb);
    avio_skip(pb, 4);
    size = avio_rl32(pb);
    if (size > 0x7e0)
        return AVERROR_INVALIDDATA;
    ret = av_new_packet(avpkt, size * channels);
    if (ret < 0)
        return ret;
    avio_skip(pb, 24);
    ret = avio_read(pb, avpkt->data, size);
    if (ret < size)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 0x7e0 - size);
    if (channels > 1) {
        avio_skip(pb, 32);
        ret = avio_read(pb, avpkt->data + size, size);
        if (ret < size)
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 0x7e0 - size);
    }
    avpkt->pos = pos;

    return ret;
}

const FFInputFormat ff_vsstr_demuxer = {
    .p.name         = "vsstr",
    .p.long_name    = NULL_IF_CONFIG_SMALL("VS STR"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "vs,str",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
