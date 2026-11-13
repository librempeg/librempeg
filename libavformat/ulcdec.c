/*
 * ULC demuxer
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
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"
#include "rawdec.h"

static int ulc_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('U','L','C','2'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int ulc_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    int ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 4);

    ret = ff_get_extradata(s, st->codecpar, pb, 20);
    if (ret < 0)
        return ret;

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ULC;
    st->codecpar->sample_rate = AV_RL32(st->codecpar->extradata + 8);
    st->codecpar->ch_layout.nb_channels = AV_RL16(st->codecpar->extradata + 12);
    st->codecpar->bit_rate = AV_RL16(st->codecpar->extradata + 14) * 1000LL;

    if (st->codecpar->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;

    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, AV_RL32(st->codecpar->extradata + 16), SEEK_SET);

    return 0;
}

const FFInputFormat ff_ulc_demuxer = {
    .p.name         = "ulc",
    .p.long_name    = NULL_IF_CONFIG_SMALL("ULC"),
    .p.extensions   = "ulc",
    .p.priv_class   = &ff_raw_demuxer_class,
    .priv_data_size = sizeof(FFRawDemuxerContext),
    .read_probe     = ulc_probe,
    .read_header    = ulc_read_header,
    .read_packet    = ff_raw_read_partial_packet,
    .raw_codec_id   = AV_CODEC_ID_ULC,
};
