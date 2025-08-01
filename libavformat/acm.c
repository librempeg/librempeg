/*
 * ACM demuxer
 * Copyright (c) 2015 Paul B Mahol
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

static int acm_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != 0x97280301)
        return 0;

    return AVPROBE_SCORE_MAX / 3 * 2;
}

static int acm_read_header(AVFormatContext *s)
{
    AVStream *st;
    int ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id   = AV_CODEC_ID_INTERPLAY_ACM;

    ret = ff_get_extradata(s, st->codecpar, s->pb, 14);
    if (ret < 0)
        return ret;

    st->codecpar->ch_layout.nb_channels = AV_RL16(st->codecpar->extradata +  8);
    st->codecpar->sample_rate = AV_RL16(st->codecpar->extradata + 10);
    if (st->codecpar->ch_layout.nb_channels <= 0 || st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    st->start_time         = 0;
    st->duration           = AV_RL32(st->codecpar->extradata +  4) / st->codecpar->ch_layout.nb_channels;
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_acm_demuxer = {
    .p.name         = "acm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Interplay ACM"),
    .p.flags        = AVFMT_NOBINSEARCH | AVFMT_NOGENSEARCH | AVFMT_NO_BYTE_SEEK | AVFMT_NOTIMESTAMPS,
    .p.extensions   = "acm",
    .p.priv_class   = &ff_raw_demuxer_class,
    .read_probe     = acm_probe,
    .read_header    = acm_read_header,
    .read_packet    = ff_raw_read_partial_packet,
    .raw_codec_id   = AV_CODEC_ID_INTERPLAY_ACM,
    .priv_data_size = sizeof(FFRawDemuxerContext),
};
