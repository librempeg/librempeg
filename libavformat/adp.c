/*
 * ADP demuxer
 * Copyright (c) 2013 James Almer
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

static int adp_probe(const AVProbeData *p)
{
    int i, changes = 0;
    uint8_t last = 0;

    if (p->buf_size < 32)
        return 0;

    for (i = 0; i < p->buf_size - 3; i+=32) {
        if (p->buf[i] != p->buf[i+2] || p->buf[i+1] != p->buf[i+3])
            return 0;
        if (p->buf[i] != last)
            changes++;
        last = p->buf[i];
    }
    if (changes <= 1)
        return 0;

    return p->buf_size < 260 ? 1 : AVPROBE_SCORE_MAX / 4;
}

static int adp_read_header(AVFormatContext *s)
{
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_DTK;
    st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
    st->codecpar->sample_rate = 48000;
    st->start_time = 0;
    st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                    st->codecpar->sample_rate / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int adp_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, 1024);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_adp_demuxer = {
    .p.name         = "adp",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Nintendo GC DTK"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "adp,dtk",
    .read_probe     = adp_probe,
    .read_header    = adp_read_header,
    .read_packet    = adp_read_packet,
};
