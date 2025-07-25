/*
 * CRYO APC audio format demuxer
 * Copyright (c) 2007 Anssi Hannula <anssi.hannula@gmail.com>
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

#include <string.h>

#include "libavutil/channel_layout.h"
#include "avformat.h"
#include "demux.h"

static int apc_probe(const AVProbeData *p)
{
    if (!strncmp(p->buf, "CRYO_APC", 8))
        return AVPROBE_SCORE_MAX;

    return 0;
}

static int apc_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    int ret;
    int channels;

    avio_rl32(pb); /* CRYO */
    avio_rl32(pb); /* _APC */
    avio_rl32(pb); /* 1.20 */

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_APC;

    avio_rl32(pb); /* number of samples */
    st->codecpar->sample_rate = avio_rl32(pb);

    /* initial predictor values for adpcm decoder */
    if ((ret = ff_get_extradata(s, st->codecpar, pb, 2 * 4)) < 0)
        return ret;

    channels = !!avio_rl32(pb) + 1;
    av_channel_layout_default(&st->codecpar->ch_layout, channels);

    st->codecpar->bits_per_coded_sample = 4;
    st->codecpar->bit_rate = (int64_t)st->codecpar->bits_per_coded_sample * channels
                          * st->codecpar->sample_rate;
    st->codecpar->block_align = 1;

    return 0;
}

#define MAX_READ_SIZE 4096

static int apc_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    if (av_get_packet(s->pb, pkt, MAX_READ_SIZE) <= 0)
        return AVERROR(EIO);
    pkt->stream_index = 0;
    return 0;
}

const FFInputFormat ff_apc_demuxer = {
    .p.name         = "apc",
    .p.long_name    = NULL_IF_CONFIG_SMALL("CRYO APC"),
    .read_probe     = apc_probe,
    .read_header    = apc_read_header,
    .read_packet    = apc_read_packet,
};
