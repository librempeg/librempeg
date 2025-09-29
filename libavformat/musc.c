/*
 * MUSC demuxer
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

static int musc_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('M','U','S','C'))
        return 0;
    if (p->buf_size < 32)
        return 0;
    if (AV_RL16(p->buf + 6) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 24) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int musc_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int64_t start_offset;
    int rate, block_align;
    AVStream *st;

    avio_skip(pb, 6);
    rate = avio_rl16(pb);
    avio_skip(pb, 8);
    start_offset = avio_rl32(pb);
    avio_skip(pb, 4);
    block_align = avio_rl32(pb);

    if (rate <= 0 || block_align <= 0 || block_align > INT_MAX/2)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = 2;
    st->codecpar->block_align = block_align;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                    st->codecpar->sample_rate / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int musc_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_musc_demuxer = {
    .p.name         = "musc",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Krome MUSC"),
    .p.extensions   = "musc,mus",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = musc_probe,
    .read_header    = musc_read_header,
    .read_packet    = musc_read_packet,
};
