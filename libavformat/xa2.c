/*
 * Acclaim XA2 demuxer
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

static int psx_probe(uint8_t *buf, int offset, int size)
{
    int score = 0;

    for (int i = offset; i < size - 2; i += 16) {
        int predictor = (buf[i+0] >> 4) & 15;
        int flags = buf[i+1];

        if (predictor > 5 || flags > 7)
            return 0;

        score++;
    }

    return score;
}

static int read_probe(const AVProbeData *p)
{
    if ((int)AV_RL32(p->buf) <= 0)
        return 0;

    if (p->buf_size < 0x1000)
        return 0;
    return FFMIN(AVPROBE_SCORE_MAX, psx_probe(p->buf, 0x800, p->buf_size));
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int channels, align;
    AVStream *st;

    channels = avio_rl32(pb);
    align = avio_rl32(pb);
    if (align > 0x1000)
        align = (channels > 2) ? 0x400 : 0x1000;

    if (channels <= 0 || align < 16 || channels >= INT_MAX/align)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = 44100;
    st->codecpar->block_align = align * channels;
    st->codecpar->bit_rate = 16LL * channels * 8 * 44100 / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

const FFInputFormat ff_xa2_demuxer = {
    .p.name         = "xa2",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Acclaim XA2"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "xa2",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
