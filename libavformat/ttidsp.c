/*
 * TT IDSP demuxer
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
#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('I','D','S','P'))
        return 0;
    if (AV_RB32(p->buf+4) < 1 || AV_RB32(p->buf+4) > 3)
        return 0;
    if (AV_RB32(p->buf+8) != 0xc8 &&
        AV_RB32(p->buf+8) != 0xd2 &&
        AV_RB32(p->buf+8) != 0x12c)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t header_offset, start_offset;
    uint32_t version_main, version_sub;
    int ret, channels, align, rate;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    version_main = avio_rb32(pb);
    version_sub = avio_rb32(pb);
    align = avio_rb32(pb);
    if (align <= 0)
        return AVERROR_INVALIDDATA;

    if (version_main == 0x01 && version_sub == 0xc8) {
        channels = 2;
        rate = 44100;
        header_offset = 0x10;
    } else if (version_main == 0x02 && version_sub == 0xd2) {
        channels = 2;
        rate = 44100;
        header_offset = 0x20;
    } else if (version_main == 0x03 && version_sub == 0x12c) {
        header_offset = 0x20;
        channels = avio_rb32(pb);
        rate = avio_rb32(pb);
        if (rate <= 0 || channels <= 0 || channels > INT_MAX/align)
            return AVERROR_INVALIDDATA;
    } else {
        return AVERROR_INVALIDDATA;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align * channels;
    st->codecpar->bit_rate = 8LL * channels * 8 *
                                    rate / 14;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if ((ret = ff_alloc_extradata(st->codecpar, 32 * channels)) < 0)
        return ret;

    avio_seek(pb, header_offset + 0x1c, SEEK_SET);
    for (int ch = 0; ch < channels; ch++) {
        avio_read(pb, st->codecpar->extradata + 32*ch, 32);
        avio_skip(pb, 0x40);
    }

    start_offset = header_offset + 0x60 * channels;

    avio_seek(pb, start_offset, SEEK_SET);

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

const FFInputFormat ff_ttidsp_demuxer = {
    .p.name         = "ttidsp",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Traveller's Tales IDSP"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "gcm,dsp,wua",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
