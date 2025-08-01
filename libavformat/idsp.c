/*
 * IDSP demuxer
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

static int idsp_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('I','D','S','P'))
        return 0;

    if (AV_RB32(p->buf + 4) <= 0)
        return 0;

    if (AV_RB32(p->buf + 8) <= 0)
        return 0;

    if (AV_RB32(p->buf + 12) <= 0)
        return 0;

    if (AV_RB32(p->buf + 16) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int idsp_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    int ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 4);
    st->start_time = 0;
    st->duration = avio_rb32(pb);
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP_LE;
    st->codecpar->sample_rate = avio_rb32(pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    st->codecpar->ch_layout.nb_channels = avio_rb32(pb);
    if (st->codecpar->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;
    st->codecpar->block_align = avio_rb32(pb) * st->codecpar->ch_layout.nb_channels;
    if (st->codecpar->block_align <= 0)
        return AVERROR_INVALIDDATA;

    ret = ff_alloc_extradata(st->codecpar, 32 * st->codecpar->ch_layout.nb_channels);
    if (ret < 0)
        return ret;

    for (int ch = 0; ch < st->codecpar->ch_layout.nb_channels; ch++) {
        int16_t *dst = (int16_t *)(st->codecpar->extradata + 32*ch);

        for (int n = 0; n < 16; n++)
            dst[n] = avio_rb16(pb);
        avio_skip(pb, 14);
    }

    avio_seek(pb, 0x70, SEEK_SET);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int idsp_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    AVCodecParameters *par = s->streams[0]->codecpar;

    return av_get_packet(pb, pkt, par->block_align);
}

const FFInputFormat ff_idsp_demuxer = {
    .p.name         = "idsp",
    .p.long_name    = NULL_IF_CONFIG_SMALL("IDSP (Inevitable Entertainment)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "idsp",
    .read_probe     = idsp_probe,
    .read_header    = idsp_read_header,
    .read_packet    = idsp_read_packet,
};
