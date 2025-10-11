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
    int channels, rate, version;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    version = avio_rl16(pb);
    avio_skip(pb, 0x3a);
    channels = avio_r8(pb);
    avio_skip(pb, 3);
    if (version == 2)
        avio_skip(pb, 4);
    rate = avio_rl32(pb);
    if (channels == 0 || rate <= 0 || channels > INT_MAX/0x1000)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = 0x1000 * channels;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                    st->codecpar->sample_rate / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

const FFInputFormat ff_twopfs_demuxer = {
    .p.name         = "2pfs",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Konami PS2 2PFS"),
    .p.extensions   = "sap",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
