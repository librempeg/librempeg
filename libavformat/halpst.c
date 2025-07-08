/*
 * HALPST demuxer
 * Copyright (c) 2024 Paul B Mahol
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

#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int halpst_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, " HALPST\0", 8))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int halpst_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    AVStream *st;
    int ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 8);
    par = st->codecpar;

    par->codec_type = AVMEDIA_TYPE_AUDIO;
    par->codec_id   = AV_CODEC_ID_ADPCM_THP;
    par->sample_rate = avio_rb32(pb);
    par->ch_layout.nb_channels = avio_rb32(pb);
    if (par->sample_rate <= 0 ||
        par->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;

    ret = ff_alloc_extradata(par, par->ch_layout.nb_channels * 32);
    if (ret < 0)
        return ret;

    for (int ch = 0; ch < par->ch_layout.nb_channels; ch++) {
        if (ch == 0) {
            avio_skip(pb, 8);
            st->duration = avio_rb32(pb);
            avio_skip(pb, 4);
        } else {
            avio_skip(pb, 16);
        }
        avio_read(pb, par->extradata + 32 * ch, 32);
        avio_skip(pb, 8);
    }

    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    return 0;
}

static int halpst_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int64_t pos;
    int size, ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    size = avio_rb32(pb);
    avio_skip(pb, 28);
    if (size <= 0 || size > INT_MAX)
        return AVERROR_INVALIDDATA;

    ret = av_get_packet(pb, pkt, size);
    pkt->pos = pos;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_halpst_demuxer = {
    .p.name         = "halpst",
    .p.long_name    = NULL_IF_CONFIG_SMALL("HAL Labs"),
    .p.extensions   = "hps",
    .read_probe     = halpst_probe,
    .read_header    = halpst_read_header,
    .read_packet    = halpst_read_packet,
    .p.flags        = AVFMT_GENERIC_INDEX,
};
