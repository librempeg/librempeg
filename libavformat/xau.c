/*
 * XPEC XAU demuxer
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
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('X','A','U','\0'))
        return 0;
    if (AV_RL32(p->buf+8) != 0x40)
        return 0;
    if (p->buf[24] == 0)
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int find_chunk(AVIOContext *pb, uint32_t chunk_id)
{
    while (!avio_feof(pb)) {
        uint32_t chunk_type = avio_rl32(pb);
        uint32_t chunk_size = avio_rl32(pb);

        if (chunk_type == 0xFFFFFFFF || chunk_size == 0xFFFFFFFF)
            return AVERROR_INVALIDDATA;

        if (chunk_type == chunk_id)
            return 0;

        avio_skip(pb, chunk_size);
    }

    return AVERROR_INVALIDDATA;
}

static int read_header(AVFormatContext *s)
{
    uint32_t codec, loop_start, loop_stop;
    int64_t start_offset, bit_rate;
    int rate, channels, align;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 12);
    codec = avio_rb32(pb);
    loop_start = avio_rl32(pb);
    loop_stop = avio_rl32(pb);
    channels = avio_r8(pb);

    switch (codec) {
    case MKBETAG('P','S','2','\x0'):
        avio_seek(pb, 0x50, SEEK_SET);
        rate = avio_rb32(pb);
        codec = AV_CODEC_ID_ADPCM_PSX;
        align = 0x8000;
        start_offset = 0x800;
        bit_rate = 16LL * channels * 8 * rate / 28;
        break;
    case MKBETAG('X','B','\x0','\x0'):
        avio_seek(pb, 0x40, SEEK_SET);
        if (avio_rb32(pb) != MKBETAG('R','I','F','F'))
            return AVERROR_INVALIDDATA;

        avio_seek(pb, 0x58, SEEK_SET);
        rate = avio_rl32(pb);

        avio_seek(pb, 0x4c, SEEK_SET);
        if (!find_chunk(pb, AV_RL32("data")))
            return AVERROR_INVALIDDATA;
        codec = AV_CODEC_ID_ADPCM_IMA_XBOX;
        align = 0x24;
        start_offset = avio_tell(pb);
        bit_rate = 36LL * channels * 8 * rate / 64;
        break;
    default:
        avpriv_request_sample(s, "codec %08X", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (rate <= 0 || channels <= 0 || channels >= INT_MAX/8)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->codec_id = codec;
    st->codecpar->block_align = align * channels;
    st->codecpar->bit_rate = bit_rate;

    if (loop_start > 0)
        av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);
    if (loop_stop > 0)
        av_dict_set_int(&st->metadata, "loop_stop", loop_stop, 0);

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_xau_demuxer = {
    .p.name         = "xau",
    .p.long_name    = NULL_IF_CONFIG_SMALL("XPEC Entertainment XAU"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "xau",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
