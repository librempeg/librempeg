/*
 * Vicious Cycle Games MUS demuxer
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
    if (AV_RB32(p->buf) != 0xFBBFFBBF &&
        AV_RL32(p->buf) != 0xFBBFFBBF)
        return 0;

    if (p->buf_size < 0x30)
        return 0;
    if (AV_RB32(p->buf+0x08) != 0xBBBBBBBB ||
        AV_RB32(p->buf+0x14) != 0xBBBBBBBB ||
        AV_RB32(p->buf+0x2c) != 0xBEBEBEBE)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int rate, be, channels, codec, ret, align;
    unsigned (*avio_r32)(AVIOContext *pb);
    AVIOContext *pb = s->pb;
    int64_t start, bitrate;
    AVStream *st;

    be = avio_rb32(pb) == 0xFBBFFBBF;
    avio_r32 = be ? avio_rb32 : avio_rl32;
    codec = avio_r32(pb);
    avio_seek(pb, 0x54, SEEK_SET);
    channels = avio_r32(pb);
    rate = avio_r32(pb);

    switch (codec) {
    case 1:
        codec = AV_CODEC_ID_ADPCM_NDSP;
        bitrate = 8LL * channels * 8 * rate / 14;
        align = 8;
        start = 0xB8;
        break;
    case 2:
        codec = AV_CODEC_ID_ADPCM_IMA_XBOX;
        bitrate = 36LL * channels * 8 * rate / 64;
        align = 36;
        start = 0x9e;
        break;
    default:
        avpriv_request_sample(s, "codec %X", codec);
        return AVERROR_INVALIDDATA;
    }

    if (align <= 0 || rate <= 0 || channels <= 0 || channels >= INT_MAX/align)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align * channels;
    st->codecpar->bit_rate = bitrate;

    if (codec = AV_CODEC_ID_ADPCM_NDSP) {
        avio_seek(pb, 0x88, SEEK_SET);
        ret = ff_get_extradata(s, st->codecpar, pb, 32);
        if (ret < 0)
            return ret;
    }

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_vcmus_demuxer = {
    .p.name         = "vcmus",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Vicious Cycle MUS"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "mus",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
