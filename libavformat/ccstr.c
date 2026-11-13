/*
 * Cauldron STR demuxer
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
    int score = 0;

    if (AV_RB32(p->buf) != 0xFAAF0001)
        return 0;

    if (p->buf_size < 80)
        return 0;

    if ((int)AV_RB32(p->buf+4) <= 0)
        return 0;
    if (AV_RB32(p->buf+8) == 0)
        return 0;
    if ((int)AV_RB32(p->buf+12) <= 0)
        return 0;

    for (int ch = 0; ch < 2; ch++) {
        for (int n = 0; n < 16; n++)
            score += 3 * (AV_RN16(p->buf + 16 + n * 2 + 32 * ch) != 0);
    }

    return score;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int align, rate, ret;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 4);
    rate = avio_rb32(pb);
    duration = avio_rb32(pb);
    align = avio_rb32(pb);
    if (rate <= 0 || align <= 0 || align >= INT_MAX/2)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = 2;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align * 2;
    st->codecpar->bit_rate = 8LL * 2 * 8 * rate / 14;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 16, SEEK_SET);
    ret = ff_get_extradata(s, st->codecpar, pb, 32 * 2);
    if (ret < 0)
        return ret;

    avio_seek(pb, 0x60, SEEK_SET);

    return 0;
}

const FFInputFormat ff_ccstr_demuxer = {
    .p.name         = "ccstr",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Cauldron STR"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "str",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
