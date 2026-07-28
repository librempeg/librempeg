/*
 * 208 demuxer
 * Copyright (c) 2025 smiRaphi
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
    if (p->buf_size < 0xF0)
        return 0;

    if (AV_RL32(p->buf) < 0xF0 || AV_RL32(p->buf + 8) > 0 ||
       (AV_RL64(p->buf + 0x10) + AV_RL64(p->buf + 0x18) + AV_RL64(p->buf + 0x20) + AV_RL32(p->buf + 0x28)) > 0)
        return 0;

    if (AV_RL32(p->buf + 4) == 0x00 && AV_RB32(p->buf + 0xCC) == 0x1F7D984D)
        return AVPROBE_SCORE_MAX;
    else if (AV_RL32(p->buf + 4) == 0xF0 && AV_RB32(p->buf + 0xCC) == 0x00000000)
        return AVPROBE_SCORE_MAX/3*2;

    return 0;
}

static int read_header(AVFormatContext *s)
{
    int64_t start_offset, nb_frames;
    AVIOContext *pb = s->pb;
    int rate, nb_channels;
    uint32_t codec;
    AVStream *st;

    start_offset = avio_rl32(pb);
    avio_skip(pb, 8);
    nb_frames = avio_rl32(pb);
    avio_skip(pb, 0x24);
    rate = avio_rl32(pb);
    codec = avio_rl32(pb);
    nb_channels = avio_rl32(pb);

    switch (codec) {
    case 8:
        codec = AV_CODEC_ID_PCM_U8;
        break;
    default:
        avpriv_request_sample(s, "codec 0x%X", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (rate <= 0 || nb_channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->nb_frames = nb_frames;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->block_align = nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_two08_demuxer = {
    .p.name         = "208",
    .p.long_name    = NULL_IF_CONFIG_SMALL("208 (Ocean Games)"),
    .p.extensions   = "208",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
    .read_seek      = ff_pcm_read_seek,
};
