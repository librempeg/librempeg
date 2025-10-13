/*
 * Namco IDSP demuxer
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

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('I','D','S','P'))
        return 0;

    if (p->buf_size < 44)
        return 0;
    if (AV_RB32(p->buf + 4) != 0)
        return 0;
    if ((int)AV_RB32(p->buf + 8) <= 0)
        return 0;
    if ((int)AV_RB32(p->buf + 12) <= 0)
        return 0;
    if ((int)AV_RB32(p->buf + 28) <= 0)
        return 0;
    if (AV_RB32(p->buf + 16) == 0)
        return 0;
    if (AV_RB32(p->buf + 40) <= 44)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t duration, start_offset, align, header, spacing;
    int ret, channels, rate;
    AVIOContext *pb = s->pb;
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 8);
    channels = avio_rb32(pb);
    rate = avio_rb32(pb);
    duration = avio_rb32(pb);
    avio_skip(pb, 8);
    align = avio_rb32(pb);
    header = avio_rb32(pb);
    spacing = avio_rb32(pb);
    start_offset = avio_rb32(pb);
    if (align == 0)
        align = avio_rb32(pb);
    if (spacing < 32 || channels <= 0 || align <= 0 || align > INT_MAX/channels)
        return AVERROR_INVALIDDATA;

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = align * channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    ret = ff_alloc_extradata(st->codecpar, 32 * st->codecpar->ch_layout.nb_channels);
    if (ret < 0)
        return ret;

    avio_seek(pb, header + 0x1c, SEEK_SET);
    for (int ch = 0; ch < st->codecpar->ch_layout.nb_channels; ch++) {
        uint8_t *dst = st->codecpar->extradata + 32*ch;

        ret = avio_read(pb, dst, 32);
        if (ret != 32)
            return AVERROR_INVALIDDATA;
        avio_skip(pb, spacing-32);
    }

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    AVCodecParameters *par = s->streams[0]->codecpar;

    return av_get_packet(pb, pkt, par->block_align);
}

const FFInputFormat ff_namcoidsp_demuxer = {
    .p.name         = "namcoidsp",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Namco IDSP"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "idsp",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
