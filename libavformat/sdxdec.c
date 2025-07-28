/*
 * SDX demuxer
 * Copyright (c) 2017 Paul B Mahol
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

#include "libavutil/avstring.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int sdx_probe(const AVProbeData *p)
{
    int n = 0;

    if (AV_RB32(p->buf) != AV_RB32("SDX:"))
        return 0;

    for (n = 4; n < p->buf_size-1; n++) {
        if (p->buf[n] == 0x1a) {
            n++;
            if (p->buf[n] != 1)
                return 0;
            n++;
            break;
        }
    }

    if (n >= p->buf_size)
        return 0;

    n += p->buf[n] + 5;
    if (n > p->buf_size - 5)
        return 0;

    if (p->buf[n] != 8 &&
        p->buf[n] != 16 &&
        p->buf[n] != 24 &&
        p->buf[n] != 32)
        return 0;

    if (AV_RL32(p->buf + n + 1) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int sdx_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    int depth, length;

    avio_skip(pb, 4);
    while (!avio_feof(pb)) {
        if (avio_r8(pb) == 0x1a)
            break;
    }
    if (avio_r8(pb) != 1)
        return AVERROR_INVALIDDATA;
    length = avio_r8(pb);
    avio_skip(pb, length);
    avio_skip(pb, 4);
    depth = avio_r8(pb);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->sample_rate = avio_rl32(pb);
    switch (depth) {
    case 8:
        st->codecpar->codec_id = AV_CODEC_ID_PCM_U8;
        break;
    case 16:
        st->codecpar->codec_id = AV_CODEC_ID_PCM_U16LE;
        break;
    case 24:
        st->codecpar->codec_id = AV_CODEC_ID_PCM_U24LE;
        break;
    case 32:
        st->codecpar->codec_id = AV_CODEC_ID_PCM_U32LE;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }
    avio_skip(pb, 16);
    st->codecpar->block_align = depth / 8;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_sdx_demuxer = {
    .p.name         = "sdx",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sample Dump eXchange"),
    .p.extensions   = "sdx",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = sdx_probe,
    .read_header    = sdx_read_header,
    .read_packet    = ff_pcm_read_packet,
    .read_seek      = ff_pcm_read_seek,
};
