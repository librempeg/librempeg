/*
 * WiiBGM demuxer
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
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "WiiBGM\0\0", 8))
        return 0;

    if (p->buf_size < 40)
        return 0;

    if ((int32_t)AV_RB32(p->buf + 32) <= 0)
        return 0;

    if ((int32_t)AV_RB32(p->buf + 36) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int ret, channels, rate;
    uint32_t loop_start;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 0x10);
    duration = avio_rb32(pb);
    loop_start = avio_rb32(pb);
    avio_skip(pb, 8);
    channels = avio_rb32(pb);
    rate = avio_rb32(pb);
    if (channels <= 0 || rate <= 0 || channels > INT_MAX/8)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP_SI1;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 8 * channels;

    if (loop_start > 0)
        av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    ret = ff_alloc_extradata(st->codecpar, 32 * channels);
    if (ret < 0)
        return ret;

    avio_skip(pb, 0x34);
    for (int c = 0; c < channels; c++) {
        avio_read(pb, st->codecpar->extradata + 0x20 * c, 0x20);
        avio_skip(pb, 0x40);
    }

    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

const FFInputFormat ff_wiibgm_demuxer = {
    .p.name         = "wiibgm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("WiiBGM (Koei Tecmo)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "dsp,wiibgm",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
