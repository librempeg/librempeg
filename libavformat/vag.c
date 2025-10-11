/*
 * VAG demuxer
 * Copyright (c) 2015 Paul B Mahol
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
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "VAG", 3))
        return 0;
    if (p->buf_size < 20)
        return 0;
    if (p->buf[3] != '1' &&
        p->buf[3] != '2' &&
        p->buf[3] != 'i' &&
        p->buf[3] != 'p')
        return 0;
    if ((int)AV_RB32(p->buf+16) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int avio_seek_r8(AVIOContext *pb, int64_t offset)
{
    avio_seek(pb, offset, SEEK_SET);
    return avio_r8(pb);
}

static uint32_t avio_seek_rl32(AVIOContext *pb, int64_t offset)
{
    avio_seek(pb, offset, SEEK_SET);
    return avio_rl32(pb);
}

static int read_header(AVFormatContext *s)
{
    int ret, rate, channels = 0, stream_name_size = 16, align = 0, codec;
    uint32_t type, version, reserved;
    int64_t start_offset, duration = 0;
    AVIOContext *pb = s->pb;
    char title[33];
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    type = avio_rb32(pb);
    version = avio_rb32(pb);
    reserved = avio_rb32(pb);
    avio_rb32(pb);
    if (version == 0 && type == MKBETAG('V','A','G','p')) {
        avio_skip(pb, 2);
        rate = avio_rb16(pb);
    } else {
        rate = avio_rb32(pb);
    }
    codec = AV_CODEC_ID_ADPCM_PSX;
    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    switch (type) {
    case MKBETAG('V','A','G','1'):
        start_offset = 0x40;
        align = 0x10;
        channels = avio_seek_r8(pb, 0x1e);
        if (channels == 0)
            channels = 1;
        break;
    case MKBETAG('V','A','G','2'):
        start_offset = 0x40;
        align = 0x800;
        channels = 2;
        break;
    case MKBETAG('V','A','G','i'):
        start_offset = 0x800;
        align = avio_seek_rl32(pb, 0x08);
        channels = 2;
        break;
    case MKBETAG('V','A','G','p'):
        switch (version) {
        case 0x00020001:
        case 0x00030000:
            codec = AV_CODEC_ID_ADPCM_HEVAG;
            start_offset = 0x30;
            align = 0x10;
            channels = avio_seek_r8(pb, 0x1e);
            if (channels == 0)
                channels = 1;
            break;
        case 0x02000000:
        case 0x40000000:
            start_offset = 0x40;
            align = 0x10;
            channels = 1;
            break;
        case 0x00000020:
            if (reserved == 0x01010101) {
                start_offset = 0x800;
                align = 0x400;
                channels = 2;
                break;
            }
        default:
            start_offset = 0x30;
            align = 0x10;
            channels = 1;
            break;
        }
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    if (channels <= 0 || align <= 0 || align > INT_MAX/channels)
        return AVERROR_INVALIDDATA;

    st->start_time = 0;
    if (duration > 0)
        st->duration = duration;
    st->codecpar->sample_rate = rate;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->block_align = align * channels;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->bit_rate = 16LL * st->codecpar->ch_layout.nb_channels * 8 *
                                    st->codecpar->sample_rate / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x20, SEEK_SET);
    if ((ret = avio_get_str(pb, stream_name_size, title, sizeof(title))) < 0)
        return ret;
    if (title[0] != '\0')
        av_dict_set(&s->metadata, "title", title, 0);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_vag_demuxer = {
    .p.name         = "vag",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony PS2 VAG"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "vag",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
