/*
 * CompressWave demuxer
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
#include "avformat.h"
#include "demux.h"
#include "rawdec.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "CmpWave\0", 8))
        return 0;

    if (p->buf_size < 16)
        return 0;

    if ((int)AV_RL32(p->buf+8) < 1 ||
        (int)AV_RL32(p->buf+8) > 2)
        return 0;

    if ((int)AV_RL32(p->buf+12) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    int ret;

    avio_skip(pb, 8);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id   = AV_CODEC_ID_COMPRESSWAVE;

    ret = ff_get_extradata(s, st->codecpar, pb, 0x940);
    if (ret < 0)
        return ret;

    st->start_time = 0;
    st->duration = AV_RL64(st->codecpar->extradata + 0x410) / 4;
    st->codecpar->ch_layout.nb_channels = AV_RL32(st->codecpar->extradata);
    st->codecpar->sample_rate = AV_RL32(st->codecpar->extradata + 4);

    if (st->codecpar->ch_layout.nb_channels < 1 ||
        st->codecpar->ch_layout.nb_channels > 2)
        return AVERROR_INVALIDDATA;

    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;

    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_cmpwave_demuxer = {
    .p.name         = "cmpwave",
    .p.long_name    = NULL_IF_CONFIG_SMALL("CompressWave CWAV"),
    .p.flags        = AVFMT_NOBINSEARCH | AVFMT_NOGENSEARCH | AVFMT_NO_BYTE_SEEK | AVFMT_NOTIMESTAMPS,
    .p.extensions   = "cwav",
    .p.priv_class   = &ff_raw_demuxer_class,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_raw_read_partial_packet,
    .raw_codec_id   = AV_CODEC_ID_COMPRESSWAVE,
    .priv_data_size = sizeof(FFRawDemuxerContext),
};
