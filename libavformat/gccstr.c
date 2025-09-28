/*
 * GC CSTR demuxer
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

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('C','s','t','r'))
        return 0;
    if (p->buf_size < 48)
        return 0;
    if (AV_RB16(p->buf+6) == 0)
        return 0;
    if (p->buf[27] == 0)
        return 0;
    if ((int32_t)AV_RB32(p->buf+40) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, align, channels, rate;
    AVIOContext *pb = s->pb;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 6);
    align = avio_rb16(pb);
    avio_skip(pb, 19);
    channels = avio_r8(pb);
    avio_skip(pb, 4);
    duration = avio_rb32(pb);
    avio_skip(pb, 4);
    rate = avio_rb32(pb);
    if (align == 0 || channels == 0 || rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = channels;
    st->start_time = 0;
    st->duration = duration;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align * st->codecpar->ch_layout.nb_channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x3c, SEEK_SET);
    if ((ret = ff_alloc_extradata(st->codecpar, 32 * st->codecpar->ch_layout.nb_channels)) < 0)
        return ret;
    for (int ch = 0; ch < channels; ch++) {
        avio_read(pb, st->codecpar->extradata + ch * 32, 32);
        avio_skip(pb, 64);
    }

    avio_seek(pb, 0x20 + 0x60 * channels, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_gccstr_demuxer = {
    .p.name         = "gccstr",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Namco NuSound GameCube CSTR"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "dsp",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
