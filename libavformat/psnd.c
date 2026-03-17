/*
 * PSND demuxer
 * Copyright (c) 2026 Paul B Mahol
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
    if (AV_RB32(p->buf) != MKBETAG('P','S','N','D'))
        return 0;

    if (p->buf_size < 14)
        return 0;
    if (AV_RL32(p->buf + 8) != 0x0030006 &&
        AV_RL32(p->buf + 8) != 0x0000004)
        return 0;
    if (AV_RL16(p->buf + 12) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int type, rate, channels, codec, align;
    AVIOContext *pb = s->pb;
    int64_t start_offset;
    AVStream *st;

    avio_skip(pb, 8);
    type = avio_rl32(pb);
    rate = avio_rl16(pb);

    switch (type) {
    case 0x0030006:
        channels = avio_r8(pb);
        start_offset = 16;
        align = 2;
        codec = AV_CODEC_ID_PCM_S16LE;
        break;
    case 0x0000004:
        channels = 1;
        start_offset = 15;
        align = 4;
        codec = AV_CODEC_ID_ADPCM_IMA_DVI;
        break;
    default:
        avpriv_request_sample(s, "type %X", type);
        return AVERROR_PATCHWELCOME;
    }

    if (rate <= 0 || channels <= 0 || align <= 0 || align > INT_MAX/channels)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align * channels;
    st->codecpar->ch_layout.nb_channels = channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_psnd_demuxer = {
    .p.name         = "psnd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Polarbit PSND"),
    .p.extensions   = "psn",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
