/*
 * MTA2 demuxer
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

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int mta2_probe(uint8_t *buf, const int offset, const int size)
{
    if (size - offset < 0x800)
        return 0;

    if (AV_RB32(buf + offset) != MKBETAG('M','T','A','2') ||
        AV_RB32(buf + offset + 0x40) != MKBETAG('H','E','A','D') ||
        AV_RB32(buf + offset + 0x7f8) != MKBETAG('D','A','T','A') ||
        AV_RB32(buf + offset + 0x7fc) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_probe(const AVProbeData *p)
{
    uint32_t chunk;

    chunk = AV_RB32(p->buf);
    if (chunk == MKBETAG('D','L','B','M'))
        return mta2_probe(p->buf, 0x820, p->buf_size);
    else if (chunk == 0x10)
        return mta2_probe(p->buf, 32, p->buf_size);
    else
        return mta2_probe(p->buf, 0, p->buf_size);

}

static int read_header(AVFormatContext *s)
{
    int64_t duration, start = 0x800;
    AVIOContext *pb = s->pb;
    int nb_channels, rate;
    unsigned chunk;
    AVStream *st;

    chunk = avio_rb32(pb);
    if (chunk == 0x10) {
        avio_skip(pb, 32);
        start += 32;
    } else if (chunk == MKBETAG('D','L','B','M')) {
        avio_skip(pb, 0x820);
        start += 0x820;
    }
    avio_skip(pb, 0x52);
    nb_channels = avio_rb16(pb);
    avio_skip(pb, 4);
    duration = avio_rb32(pb);
    avio_skip(pb, 28);
    rate = av_int2float(avio_rb32(pb));
    if (rate == 0)
        rate = 48000;

    if (nb_channels <= 0 || rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_MTA2;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int size, ret, track;
    int64_t pos;

    pos = avio_tell(pb);
    avio_skip(pb, 4);
    if (avio_feof(pb))
        return AVERROR_EOF;

    size = avio_rb32(pb);
    if (size == 0xffffffff)
        return AVERROR_EOF;

    if (size <= 16)
        return FFERROR_REDO;

    avio_skip(pb, 4);
    track = avio_rb32(pb);
    if (track == 0) {
        avio_skip(pb, size - 16);
        return FFERROR_REDO;
    }

    ret = av_get_packet(pb, pkt, size - 16);
    if (ret < 0)
        return ret;

    pkt->pos = pos;
    pkt->stream_index = 0;

    return 0;
}

const FFInputFormat ff_mta2_demuxer = {
    .p.name         = "mta2",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Konami PS3 MTA2"),
    .p.extensions   = "mta2",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
