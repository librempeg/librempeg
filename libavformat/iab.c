/*
 * IAB demuxer
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

static int read_probe(const AVProbeData *pd)
{
    if (AV_RB32(pd->buf) != 0x10000000)
        return 0;
    if (pd->buf_size < 0x44)
        return 0;
    if ((int)AV_RL32(pd->buf+4) <= 0)
        return 0;
    if (AV_RB32(pd->buf+0x40) != 0x48124812)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    AVStream *st;
    int rate;

    avio_skip(pb, 4);
    rate = avio_rl32(pb);
    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    par              = st->codecpar;
    par->codec_type  = AVMEDIA_TYPE_AUDIO;
    par->codec_id    = AV_CODEC_ID_ADPCM_PSX;
    st->start_time   = 0;
    par->sample_rate = rate;
    par->ch_layout.nb_channels = 2;
    par->bit_rate = 16LL * par->ch_layout.nb_channels * 8 *
                           par->sample_rate / 28;

    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    avio_seek(pb, 0x40, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret, block_size, pkt_size;
    AVIOContext *pb = s->pb;
    uint32_t type;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    type = avio_rb32(pb);
    if (type == 0x55aa55aa)
        return AVERROR_EOF;
    if (type != 0x48124812)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4);
    pkt_size = avio_rl32(pb);
    if (pkt_size <= 0)
        return AVERROR_INVALIDDATA;
    block_size = avio_rl32(pb);

    ret = av_get_packet(pb, pkt, pkt_size);
    pkt->pos = pos;
    if (block_size > pkt_size + 16)
        avio_skip(pb, block_size - pkt_size);

    return ret;
}

const FFInputFormat ff_iab_demuxer = {
    .p.name         = "iab",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Runtime IAB Audio"),
    .p.extensions   = "iab",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
