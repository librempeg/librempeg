/*
 * MCG demuxer
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
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    int offset;

    if (AV_RB32(p->buf) != MKBETAG('M','C','G','\0'))
        return 0;

    if (p->buf_size < 24)
        return 0;
    offset = AV_RL32(p->buf+4);
    if (offset < 4 || p->buf_size-20 < offset)
        return 0;
    if (AV_RB32(p->buf+offset) != MKBETAG('V','A','G','p'))
        return 0;
    if (AV_RB32(p->buf+offset+16) <= 0)
        return 0;
    offset = AV_RL32(p->buf+8);
    if (offset < 4 || p->buf_size-4 < offset)
        return 0;
    if (AV_RB32(p->buf+offset) != MKBETAG('V','A','G','p'))
        return 0;
    if (AV_RN32(p->buf+10) == 0)
        return 0;
    if ((int)AV_RL32(p->buf+20) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int rate, align, channels;
    AVIOContext *pb = s->pb;
    int64_t start, offset;
    AVStream *st;

    avio_skip(pb, 4);
    offset = avio_rl32(pb);
    avio_skip(pb, 4);
    start = avio_rl32(pb);
    avio_skip(pb, 4);
    align = avio_rl32(pb);
    avio_seek(pb, offset + 16, SEEK_SET);
    rate = avio_rb32(pb);

    if (rate <= 0 || align <= 0)
        return AVERROR_INVALIDDATA;

    channels = 1;
    while (!avio_feof(pb)) {
        int found = 0;

        avio_seek(pb, start + align * channels, SEEK_SET);

        for (int i = 0; i < 16; i++) {
            if (avio_r8(pb)) {
                found = 1;
                break;
            }
        }

        if (found)
            break;

        channels++;
        if (align > INT_MAX/channels)
            return AVERROR_INVALIDDATA;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = align * channels;
    st->codecpar->bit_rate = rate * 8LL * 16LL * channels / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_mcg_demuxer = {
    .p.name         = "mcg",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Namco MCG"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "gcm",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
