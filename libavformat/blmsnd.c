/*
 * BLM! snd! demuxer
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

typedef struct TBFDHeader {
    uint32_t type;
    int32_t count;
    int32_t size;
} TBFDHeader;

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 0x28)
        return 0;

    if (AV_RB32(p->buf + 0x24) != MKTAG('s','n','d','!'))
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int align, rate, codec, channels, path_length = 0;
    int64_t start, body_start, body_size, bit_rate;
    AVIOContext *pb = s->pb;
    TBFDHeader tbfd = { 0 };
    uint32_t tag;
    AVStream *st;

    avio_skip(pb, 0x24);
    tag = avio_rb32(pb);
    if (tag != MKTAG('s','n','d','!'))
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4+4+8+2+2+4);

    tag = avio_rb32(pb);
    if (tag != MKTAG('t','b','f','d'))
        return AVERROR_INVALIDDATA;
    tbfd.type = avio_rl32(pb);
    tbfd.count = avio_rl32(pb);
    tbfd.size = body_size = avio_rl32(pb);
    if (tbfd.size <= 0 || tbfd.count <= 0)
        return AVERROR_INVALIDDATA;
    body_start = avio_tell(pb);
    if (tbfd.size > 0xAC) {
        avio_skip(pb, 4);
        avio_skip(pb, 4);
        path_length = avio_rl32(pb);
        if (path_length < 0)
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 4);
    }
    avio_skip(pb, 4+1);
    rate = avio_r8(pb);
    switch (rate) {
    case 0:
        rate = 22050;
        break;
    case 1:
        rate = 44100;
        break;
    case 2:
        rate = 32000;
        break;
    case 3:
        rate = 48000;
        break;
    default:
        return AVERROR_PATCHWELCOME;
    }
    avio_skip(pb, 2+56+20+2);
    channels = avio_r8(pb);
    switch (channels) {
    case 0:
        channels = 1;
        break;
    case 1:
    case 2:
        channels = 2;
        break;
    case 3:
        channels = 4;
        break;
    default:
        return AVERROR_PATCHWELCOME;
    }
    codec = avio_r8(pb);
    switch (codec) {
    case 1:
        codec = AV_CODEC_ID_ADPCM_IMA_XBOX_SBR;
        align = 36;
        bit_rate = 36LL * channels * 8 * rate / 64;
        break;
    case 3:
        codec = AV_CODEC_ID_PCM_S16LE;
        align = 2;
        bit_rate = 2LL * channels * 8 * rate;
        break;
    default:
        return AVERROR_PATCHWELCOME;
    }
    avio_skip(pb, 12+12+12+4+4+4);
    avio_skip(pb, 4*3);
    avio_skip(pb, 12);
    avio_skip(pb, 4*3);

    avio_seek(pb, body_start + body_size, SEEK_SET);
    if (body_size > 0xAC && path_length > 0)
        avio_skip(pb, path_length);

    tag = avio_rb32(pb);
    while (tag != MKTAG('t','b','f','d')) {
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        switch (tag) {
        case MKTAG('s','n','p','l'):
        case MKTAG('s','n','s','c'):
        case MKTAG('s','n','p','r'):
            avio_skip(pb, 12);
            tag = avio_rb32(pb);
            break;
        default:
            return AVERROR_INVALIDDATA;
        }
    }

    int subtype, name_size;
    do {
        int extra = 0;

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        if (tag != MKTAG('t','b','f','d'))
            return AVERROR_INVALIDDATA;

        tbfd.type = avio_rl32(pb);
        tbfd.count = avio_rl32(pb);
        tbfd.size = avio_rl32(pb);
        if (tbfd.size <= 4 || tbfd.count <= 0)
            return AVERROR_INVALIDDATA;

        for (int i = 0; i < tbfd.count; i++) {
            if (avio_feof(pb))
                return AVERROR_INVALIDDATA;

            subtype = avio_rl16(pb);
            name_size = avio_rb16(pb);
            extra += name_size;
            avio_skip(pb, tbfd.size - 4);
        }

        avio_skip(pb, extra);
        if (subtype)
            tag = avio_rb32(pb);
    } while (subtype);
    start = avio_tell(pb);

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
    st->codecpar->bit_rate = bit_rate;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_blmsnd_demuxer = {
    .p.name         = "blmsnd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Guerilla BLM! snd!"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "sound",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
