/*
 * VIG demuxer
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

#define PSB2SMP(s, c) (s / c / 0x10 * 28)

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != 0x01006408)
        return 0;

    if (p->buf_size < 36)
        return 0;
    if (AV_RL32(p->buf + 4) != 0 || AV_RL32(p->buf + 0x20) != 0)
        return 0;
    if ((AV_RL32(p->buf + 8) + AV_RL32(p->buf + 12)) > p->buf_size)
        return 0;
    if ((int)AV_RL32(p->buf + 0x18) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf + 0x1C) <= 0)
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    uint32_t data_size, loop_start, loop_end;
    int rate, nb_channels, align;
    AVIOContext *pb = s->pb;
    int64_t start_offset;
    AVStream *st;

    avio_skip(pb, 8);
    start_offset = avio_rl32(pb);
    data_size = avio_rl32(pb);
    loop_start = avio_rl32(pb);
    loop_end = avio_rl32(pb);
    rate = avio_rl32(pb);
    nb_channels = avio_rl32(pb);
    avio_skip(pb, 4);
    align = avio_rl32(pb);
    if (align == 0)
        align = 0x800;
    if (rate <= 0 || nb_channels <= 0 || align <= 0 || align > INT_MAX/nb_channels)
        return AVERROR_INVALIDDATA;

    if (loop_end > 0) {
        av_dict_set_int(&s->metadata, "loop_start", PSB2SMP(loop_start, nb_channels), 0);
        av_dict_set_int(&s->metadata, "loop_end", PSB2SMP(loop_end + loop_start, nb_channels), 0);
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = PSB2SMP(data_size, nb_channels);
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->block_align = align * nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_vig_demuxer = {
    .p.name         = "vig",
    .p.long_name    = NULL_IF_CONFIG_SMALL("VIG (Konami/KCE Studio)"),
    .p.extensions   = "vig",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
