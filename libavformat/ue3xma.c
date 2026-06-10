/*
 * Unreal Engine 3 XMA demuxer
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
    if ((int)AV_RB32(p->buf+0) <= 0 ||
        (int)AV_RB32(p->buf+4) <= 0 ||
        (int)AV_RB32(p->buf+8) <= 0)
        return 0;

    if (!av_match_ext(p->filename, "xma") &&
        !av_match_ext(p->filename, "x360audio"))
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int seek_size, data_size, rate, channels, ret, extradata_size;
    int64_t start_offset, duration;
    AVIOContext *pb = s->pb;
    AVStream *st;

    extradata_size = avio_rb32(pb);
    seek_size = avio_rb32(pb);
    data_size = avio_rb32(pb);
    if (extradata_size <= 0 || seek_size <= 0 || data_size <= 0)
        return AVERROR_INVALIDDATA;

    if (extradata_size != 0x34) {
        int xma2_chunk_version = avio_r8(pb);
        int num_streams = avio_r8(pb);
        avio_skip(pb, 10);
        rate = avio_rb32(pb);

        avio_skip(pb, (xma2_chunk_version == 3) ? 4 : 12);
        duration = avio_rb32(pb);

        avio_skip(pb, 8);
        channels = 0;
        for (int i = 0; i < num_streams; i++) {
            channels += avio_r8(pb);
            avio_skip(pb, 3);
        }
    } else {
        avio_skip(pb, 2);
        channels = avio_rb16(pb);
        rate = avio_rb32(pb);
    }

    if (rate <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_XMA2;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = 2048;

    avio_seek(pb, 12, SEEK_SET);
    if ((ret = ff_get_extradata(s, st->codecpar, pb, extradata_size) < 0))
        return ret;

    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    start_offset = 12LL + extradata_size + seek_size;

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_ue3xma_demuxer = {
    .p.name         = "ue3xma",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Unreal Engine 3 XMA"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "xma,x360audio",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
