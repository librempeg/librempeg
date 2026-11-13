/*
 * RFRM demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('R','F','R','M'))
        return 0;

    if (AV_RB32(p->buf+20) != MKBETAG('C','S','M','P'))
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int ret, align, rate, version, be, channels;
    unsigned int (*avio_r32)(AVIOContext *pb);
    unsigned int (*avio_r16)(AVIOContext *pb);
    uint64_t (*avio_r64)(AVIOContext *pb);
    int64_t duration, start;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 24);
    version = avio_rb32(pb);
    if (version > 255)
        version = av_bswap32(version);

    switch (version) {
    case 0x0a:
    case 0x11:
    case 0x12:
    case 0x1f:
    case 0x2e:
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    be = version == 0x0a;
    avio_r32 = be ? avio_rb32 : avio_rl32;
    avio_r16 = be ? avio_rb16 : avio_rl16;
    avio_r64 = (version < 0x1f) ? avio_rb64 : avio_rl64;

    int64_t fmta_offset = 0, ras3_offset = 0, data_offset = 0;
    int64_t data_size = 0, chunk_size;
    int64_t chunk_offset = 0x20;
    uint32_t chunk_type;

    while (!avio_feof(pb)) {
        avio_seek(pb, chunk_offset, SEEK_SET);
        chunk_type = avio_rb32(pb);
        chunk_size = avio_r64(pb);

        switch (chunk_type) {
        case MKBETAG('F','M','T','A'):
            fmta_offset = chunk_offset + 0x18;
            break;
        case MKBETAG('R','A','S','3'):
            ras3_offset = chunk_offset + 0x18;
            break;
        case MKBETAG('D','A','T','A'):
            data_offset = chunk_offset + 0x18;
            data_size = chunk_size;
            break;
        default:
            break;
        }

        if (data_size > 0)
            break;

        chunk_offset += 0x18 + chunk_size;
    }

    if (!fmta_offset || !data_offset || !data_size)
        return AVERROR_INVALIDDATA;

    av_unused int channel_layout = 0;
    uint32_t coef_spacing = 0;

    avio_seek(pb, fmta_offset, SEEK_SET);
    if (version < 0x1f) {
        channels = avio_r8(pb);
        avio_skip(pb, 3);
        channel_layout = avio_r8(pb);
        if (channels == 1)
            channel_layout = 0;
    } else if (version < 0x2e) {
        channels = avio_r8(pb);
        channel_layout = avio_r16(pb);
    } else {
        channel_layout = avio_r16(pb);
        channels = avio_r8(pb);
    }

    if (channels <= 0)
        return AVERROR_INVALIDDATA;

    uint32_t header_offset = data_offset;
    if (version == 0x0a) {
        header_offset += 3;
        data_size -= 3;
    }

    avio_seek(pb, header_offset, SEEK_SET);
    duration = avio_r32(pb);
    avio_skip(pb, 4);
    rate = avio_r32(pb);

    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    if (ras3_offset) {
        avio_seek(pb, ras3_offset, SEEK_SET);
        int block_size = avio_r32(pb);
        if (block_size < channels)
            return AVERROR_INVALIDDATA;

        align = block_size / channels;
    } else {
        if (data_size > INT_MAX/channels)
            return AVERROR_INVALIDDATA;

        align = data_size / channels;
    }

    if (version < 0x1f) {
        coef_spacing = align;
        start = header_offset + 0x60;
    } else {
        coef_spacing = 0x80;
        start = header_offset + coef_spacing * channels;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = be ? AV_CODEC_ID_ADPCM_NDSP : AV_CODEC_ID_ADPCM_NDSP_LE;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align * channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if ((ret = ff_alloc_extradata(st->codecpar, 32 * channels)) < 0)
        return ret;

    avio_seek(pb, header_offset + 0x1c, SEEK_SET);
    for (int ch = 0; ch < channels; ch++) {
        avio_read(pb, st->codecpar->extradata + ch*32, 32);
        avio_skip(pb, coef_spacing - 32);
    }

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_rfrm_demuxer = {
    .p.name         = "rfrm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Retro Studios RFRM"),
    .p.extensions   = "csmp",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
    .read_seek      = ff_pcm_read_seek,
};
