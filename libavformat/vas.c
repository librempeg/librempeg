/*
 * Rockstar Games VAS demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('V','A','G','s') &&
        AV_RB32(p->buf) != MKBETAG('2','A','G','s'))
        return 0;

    if (p->buf_size < 12)
        return 0;
    if ((int)AV_RL16(p->buf+8) <= 0)
        return 0;
    if (p->buf[11] == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, type;
    AVIOContext *pb = s->pb;
    int64_t start = 0xc;
    AVStream *st;

    type = avio_r8(pb);
    avio_skip(pb, 7);
    rate = avio_rl16(pb);
    avio_skip(pb, 1);
    channels = avio_r8(pb);
    if (rate <= 0 || channels <= 0 || channels >= INT_MAX/0x40)
        return AVERROR_INVALIDDATA;

    if (type == '2')
        start += 0x20;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 0x40 * channels;
    st->codecpar->bit_rate = 16LL * channels * 8 * rate / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_vas_demuxer = {
    .p.name         = "vas",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Rockstar Games VAS"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "vas",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
