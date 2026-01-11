/*
 * STHD demuxer
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

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S','T','H','D'))
        return 0;

    if (AV_RL16(p->buf+4) != 0x0800)
        return 0;
    if (AV_RL16(p->buf+6) == 0 ||
        AV_RL16(p->buf+6) > 8)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int build_date, start_offset, nb_channels, rate;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    start_offset = avio_rl16(pb);
    nb_channels = avio_rl16(pb);
    build_date = avio_rl32(pb);
    avio_skip(pb, 20);
    rate = avio_rl32(pb);

    if (start_offset != 0x0800 || nb_channels > 8 || rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = build_date >= 0x20170000 ? AV_CODEC_ID_PCM_S16LE : AV_CODEC_ID_ADPCM_IMA_XBOX_MONO;
    st->codecpar->bits_per_coded_sample = build_date >= 0x20170000 ? 16 : 4;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = rate;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int data_offset, channel_size, nb_channels, tag, ret;
    const int block_size = 0x800;
    AVIOContext *pb = s->pb;
    int64_t pos;

    pos = avio_tell(pb);
    tag = avio_rb32(pb);
    data_offset = avio_rl16(pb);
    nb_channels = avio_rl16(pb);
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (tag != MKBETAG('S','T','H','D'))
        return AVERROR_INVALIDDATA;

    if (data_offset >= block_size || data_offset < 24 || nb_channels == 0)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 14);
    channel_size = avio_rl16(pb);
    avio_skip(pb, data_offset - 24);

    if (channel_size == 0) {
        avio_skip(pb, block_size - data_offset);
        return FFERROR_REDO;
    }

    ret = av_get_packet(pb, pkt, channel_size * nb_channels);
    pkt->stream_index = 0;
    pkt->pos = pos;

    avio_skip(pb, pos + block_size - avio_tell(pb));

    return ret;
}

const FFInputFormat ff_sthd_demuxer = {
    .p.name         = "sthd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Dream Factory STHD"),
    .p.extensions   = "stx",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
