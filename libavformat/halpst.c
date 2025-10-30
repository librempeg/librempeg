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

#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, " HALPST\0", 8))
        return 0;

    if (p->buf_size < 16)
        return 0;

    if ((int)AV_RB32(p->buf + 8) <= 0)
        return 0;

    if ((int)AV_RB32(p->buf + 12) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, channels, rate;
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    AVStream *st;

    avio_skip(pb, 8);
    rate = avio_rb32(pb);
    channels = avio_rb32(pb);
    if (rate <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    par = st->codecpar;
    par->codec_type = AVMEDIA_TYPE_AUDIO;
    par->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    par->ch_layout.nb_channels = channels;
    par->sample_rate = rate;

    ret = ff_alloc_extradata(par, par->ch_layout.nb_channels * 32);
    if (ret < 0)
        return ret;

    for (int ch = 0; ch < par->ch_layout.nb_channels; ch++) {
        if (ch == 0) {
            int64_t nibbles;

            avio_skip(pb, 8);
            nibbles = avio_rb32(pb);
            st->duration = nibbles/16*14 + (nibbles%16)-1;
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

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int64_t pos;
    int size, ret;

    pos = avio_tell(pb);
    size = avio_rb32(pb);
    avio_skip(pb, 28);
    if (avio_feof(pb))
        return AVERROR_EOF;

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
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
