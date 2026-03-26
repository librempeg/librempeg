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

#include "libavutil/intreadwrite.h"
#include "libavutil/channel_layout.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "CRYO_APC", 8))
        return 0;

    if (p->buf_size < 20)
        return 0;
    if ((int)AV_RL32(p->buf + 16) <= 0)
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, channels, rate;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_rb64(pb);
    avio_rl32(pb); /* 1.20 */
    avio_rl32(pb); /* number of samples */
    rate = avio_rl32(pb);
    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_APC;
    st->codecpar->sample_rate = rate;

    /* initial predictor values for adpcm decoder */
    if ((ret = ff_get_extradata(s, st->codecpar, pb, 2 * 4)) < 0)
        return ret;

    channels = !!avio_rl32(pb) + 1;
    av_channel_layout_default(&st->codecpar->ch_layout, channels);

    st->codecpar->bits_per_coded_sample = 4;
    st->codecpar->bit_rate = (int64_t)st->codecpar->bits_per_coded_sample * channels
                          * st->codecpar->sample_rate;
    st->codecpar->block_align = 1;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_apc_demuxer = {
    .p.name         = "apc",
    .p.long_name    = NULL_IF_CONFIG_SMALL("CRYO APC"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
