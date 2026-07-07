/*
 * tri-Crescendo CAF demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('C','A','F',' '))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    int ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = 2;
    st->codecpar->sample_rate = 32000;
    st->codecpar->bit_rate = 8LL * st->codecpar->ch_layout.nb_channels * 8 *
                                   st->codecpar->sample_rate / 14;

    avio_seek(pb, 0x34, SEEK_SET);
    if ((ret = ff_alloc_extradata(st->codecpar, 32 * 2)) < 0)
        return ret;

    for (int ch = 0; ch < 2; ch++) {
        avio_read(pb, st->codecpar->extradata + ch * 32, 32);
        avio_skip(pb, 12);
    }

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret, block_size, ch_offset, ch_size, ch_offset2, ach_size;
    AVIOContext *pb = s->pb;
    int64_t pos;

    pos = avio_tell(pb);
    avio_skip(pb, 4);
    if (avio_feof(pb))
        return AVERROR_EOF;

    block_size = avio_rb32(pb);
    if (block_size <= 128)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 8);
    ch_offset = avio_rb32(pb);
    ch_size = avio_rb32(pb);
    ach_size = FFALIGN(ch_size, 8);
    if (ch_offset <= 128 || ch_offset >= block_size)
        return AVERROR_INVALIDDATA;
    if (ch_size <= 0 || ch_size > block_size - ch_offset)
        return AVERROR_INVALIDDATA;
    ch_offset2 = avio_rb32(pb);
    if ((ch_offset2 <= 128) || (ch_offset2 >= block_size) || (ch_size > block_size - ch_offset2))
        return AVERROR_INVALIDDATA;

    ret = av_new_packet(pkt, ach_size * 2);
    if (ret < 0)
        return ret;

    avio_seek(pb, pos + ch_offset, SEEK_SET);
    avio_read(pb, pkt->data, ch_size);
    avio_seek(pb, pos + ch_offset2, SEEK_SET);
    avio_read(pb, pkt->data + ach_size, ch_size);

    avio_seek(pb, pos + block_size, SEEK_SET);

    pkt->pos = pos;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_cfn_demuxer = {
    .p.name         = "cfn",
    .p.long_name    = NULL_IF_CONFIG_SMALL("tri-Crescendo CAF"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "cfn",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
