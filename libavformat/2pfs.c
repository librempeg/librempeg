/*
 * 2PFS demuxer
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
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('2','P','F','S'))
        return 0;
    if (p->buf_size < 0x48)
        return 0;
    if (AV_RL16(p->buf + 4) < 1 ||
        AV_RL16(p->buf + 4) > 2)
        return 0;
    if (p->buf[16] < 1 || p->buf[16] > 3)
        return 0;
    if (p->buf[0x40] == 0)
        return 0;
    if (AV_RL16(p->buf + 4) == 1)
        if ((int)AV_RL32(p->buf + 0x44) <= 0)
            return 0;
    else if ((int)AV_RL32(p->buf + 0x48) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int align, channels, rate, version, type, nb_streams;
    int64_t stream_offset, offset;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    version = avio_rl16(pb);
    avio_skip(pb, 10);
    type = avio_r8(pb);
    switch (type) {
    case 1:
        avio_skip(pb, 39);
        stream_offset = avio_rl32(pb) + 0x40;
        nb_streams = avio_rl32(pb);
        if (nb_streams != 1)
            return AVERROR_PATCHWELCOME;
        channels = avio_r8(pb);
        avio_skip(pb, 1);
        if (version == 1) {
            avio_skip(pb, 2);
            rate = avio_rl32(pb);
        } else if (version == 2) {
            avio_skip(pb, 6);
            rate = avio_rl32(pb);
        }
        align = 0x1000;
        break;
    case 2:
    case 3:
        avio_skip(pb, 56);
        stream_offset = avio_rl32(pb) + 0x50;
        avio_skip(pb, 8);
        nb_streams = avio_rl32(pb);
        offset = 0x60 + 0x20 * 0;
        avio_seek(pb, offset, SEEK_SET);
        channels = avio_r8(pb);
        avio_skip(pb, 7);
        rate = avio_rl32(pb);
        rate = av_rescale_rnd(rate, 48000, 4096, AV_ROUND_UP);
        stream_offset = avio_rl32(pb);
        /*stream_size = avio_rl32(pb);*/
        align = 0x1000;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    if (align <= 0 || channels <= 0 || rate <= 0 || channels > INT_MAX/align)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = align * channels;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                    st->codecpar->sample_rate / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, stream_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_twopfs_demuxer = {
    .p.name         = "2pfs",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Konami PS2 2PFS"),
    .p.extensions   = "sap,iap",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
