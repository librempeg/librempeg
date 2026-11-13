/*
 * Next Level IDSP demuxer
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
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    int score = 0;

    if (AV_RB32(p->buf) != MKBETAG('I','D','S','P'))
        return 0;

    if (p->buf_size < 256)
        return 0;
    if ((int)AV_RB32(p->buf + 4) <= 0)
        return 0;
    if (AV_RB32(p->buf + 12) == 0)
        return 0;
    if (AV_RB32(p->buf + 108) == 0)
        return 0;
    if ((int)AV_RB32(p->buf + 20) <= 0)
        return 0;
    if (AV_RB32(p->buf + 20) != AV_RB32(p->buf + 116))
        return 0;
    for (int ch = 0; ch < 2; ch++) {
        for (int n = 0; n < 16; n++)
            score += 3 * (AV_RN16(p->buf + 40 + n * 2 + 64*ch) != 0);
    }

    return score;
}

static int read_header(AVFormatContext *s)
{
    int ret, rate, block_align;
    AVIOContext *pb = s->pb;
    int64_t duration, start;
    AVStream *st;

    avio_skip(pb, 4);
    block_align = avio_rb32(pb);
    avio_skip(pb, 4);
    duration = avio_rb32(pb);
    avio_skip(pb, 4);
    rate = avio_rb32(pb);
    start = 12 + 0x60 * 2;
    if (rate <= 0 || block_align <= 0 || block_align > INT_MAX/2)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = 2;
    st->codecpar->block_align = block_align * 2;

    ret = ff_alloc_extradata(st->codecpar, 32 * st->codecpar->ch_layout.nb_channels);
    if (ret < 0)
        return ret;

    avio_seek(pb, 40, SEEK_SET);
    for (int ch = 0; ch < 2; ch++) {
        uint8_t *dst = st->codecpar->extradata + 32*ch;

        avio_read(pb, dst, 32);
        avio_skip(pb, 64);
    }

    avio_seek(pb, start, SEEK_SET);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_nlidsp_demuxer = {
    .p.name         = "nlidsp",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Next Level IDSP"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "idsp",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
