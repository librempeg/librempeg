/*
 * Her Interactive Games HIS demuxer
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
    if (p->buf_size < 22)
        return 0;
    if (memcmp(p->buf, "Her Interactive Sound\x1a", 22))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int codec, rate, channels, bps;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 22);
    channels = avio_rl16(pb);
    rate = avio_rl32(pb);
    avio_skip(pb, 6);
    bps = avio_rl16(pb);
    if (rate <= 0 || channels <= 0 || channels >= INT_MAX/((bps+7)/8))
        return AVERROR_INVALIDDATA;

    switch (bps) {
    case 8:
        codec = AV_CODEC_ID_PCM_U8;
        break;
    case 16:
        codec = AV_CODEC_ID_PCM_S16LE;
        break;
    default:
        avpriv_request_sample(s, "bps %d", bps);
        return AVERROR_PATCHWELCOME;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = channels * (bps+7) / 8;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 44, SEEK_SET);

    return 0;
}

const FFInputFormat ff_his_demuxer = {
    .p.name         = "his",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Her Interactive Sound"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "his",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
