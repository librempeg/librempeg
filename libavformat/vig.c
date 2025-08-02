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

#define PSB2SMP(s, c) (s / c / 0x10 * 28)

static int vig_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != 0x01006408 || AV_RL32(p->buf + 4) != 0 || AV_RL32(p->buf + 0x20) != 0)
        return 0;
    if ((AV_RL32(p->buf + 8) + AV_RL32(p->buf + 12)) > p->buf_size)
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int vig_read_header(AVFormatContext *s)
{
    uint32_t data_offset, data_size, loop_start, loop_end, block_size;
    AVIOContext *pb = s->pb;

    AVStream *st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;

    avio_skip(pb, 8);
    data_offset = avio_rl32(pb);
    data_size = avio_rl32(pb);

    loop_start = avio_rl32(pb);
    loop_end = avio_rl32(pb);

    st->codecpar->sample_rate = avio_rl32(pb);
    st->codecpar->ch_layout.nb_channels = avio_rl32(pb);
    avio_skip(pb, 4);

    st->duration = PSB2SMP(data_size, st->codecpar->ch_layout.nb_channels);
    if (loop_end > 0) {
        av_dict_set_int(&s->metadata, "loop_start", PSB2SMP(loop_start, st->codecpar->ch_layout.nb_channels), 0);
        av_dict_set_int(&s->metadata, "loop_end", PSB2SMP(loop_end + loop_start, st->codecpar->ch_layout.nb_channels), 0);
    }

    block_size = avio_rl32(pb);
    if (block_size == 0 || st->codecpar->ch_layout.nb_channels == 1)
        block_size = 0x800;
    st->codecpar->block_align = block_size * st->codecpar->ch_layout.nb_channels;
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, data_offset, SEEK_SET);
    return 0;
}

static int vig_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    AVIOContext *pb = s->pb;

    return av_get_packet(pb, pkt, par->block_align);
}

const FFInputFormat ff_vig_demuxer = {
    .p.name         = "vig",
    .p.long_name    = NULL_IF_CONFIG_SMALL("VIG (Konami/KCE Studio)"),
    .p.extensions   = "vig",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = vig_probe,
    .read_header    = vig_read_header,
    .read_packet    = vig_read_packet,
};
