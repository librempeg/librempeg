/*
 * Ocean Games DSF demuxer
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

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "OCEAN DSA\0\0\0", 12))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int rate, channels, ret;
    char title[0x21];
    int64_t start;
    AVStream *st;

    avio_skip(pb, 0x1a);
    start = avio_rl32(pb);
    rate = avio_rl32(pb);
    channels = avio_rl32(pb) + 1;
    if (rate <= 0 || channels <= 0 || channels >= INT_MAX/8)
        return AVERROR_INVALIDDATA;

    if ((ret = avio_get_str(pb, 0x21, title, sizeof(title))) < 0)
        return ret;
    if (title[0] != '\0')
        av_dict_set(&s->metadata, "title", title, 0);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_DSA;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 8 * channels;
    st->codecpar->bit_rate = 8LL * st->codecpar->ch_layout.nb_channels * 8 *
                                   st->codecpar->sample_rate / 14;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_oceandsf_demuxer = {
    .p.name         = "oceandsf",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Ocean Games DSF"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "dsf",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
