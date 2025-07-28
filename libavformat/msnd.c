/*
 * MSND demuxer
 * Copyright (c) 2025 Paul B Mahol
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

static int msnd_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "\x00\x08MSND", 6))
        return 0;

    if (p->buf_size < 14)
        return 0;

    if (AV_RL16(p->buf + 8) == 0)
        return 0;

    if (AV_RL16(p->buf + 12) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int msnd_read_header(AVFormatContext *s)
{
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(s->pb, 6);
    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id    = AV_CODEC_ID_ADPCM_IMA_MAGIX;
    st->codecpar->ch_layout.nb_channels = 2;
    avio_skip(s->pb, 2);
    st->codecpar->sample_rate = avio_rl16(s->pb);
    if (st->codecpar->sample_rate == 0)
        return AVERROR_INVALIDDATA;
    avio_skip(s->pb, 2);
    st->codecpar->block_align = avio_rl16(s->pb);
    if (st->codecpar->block_align == 0)
        return AVERROR_INVALIDDATA;
    st->start_time = 0;
    avio_seek(s->pb, 0x800, SEEK_SET);
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int msnd_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;

    return av_get_packet(s->pb, pkt, par->block_align);
}

const FFInputFormat ff_msnd_demuxer = {
    .p.name         = "msnd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("PS2 MSND"),
    .p.extensions   = "snd",
    .read_probe     = msnd_probe,
    .read_header    = msnd_read_header,
    .read_packet    = msnd_read_packet,
};
