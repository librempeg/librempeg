/*
 * PSF demuxer
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
    if (p->buf_size < 44)
        return 0;

    if ((AV_RB32(p->buf) & 0xFFFFFF00) != MKBETAG('P','S','F','\0'))
        return 0;

    if (((AV_RL32(p->buf) >> 20) & 0xFFF) == 0)
        return 0;

    switch (p->buf[3]) {
    case 0xC0:
    case 0x40:
    case 0xA1:
    case 0x21:
    case 0x80:
    case 0x81:
    case 0x01:
    case 0xD1:
        break;
    default:
        return 0;
    }

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    int align, ret, codec, rate, channels, flags;
    AVIOContext *pb = s->pb;
    int64_t start, bit_rate;
    AVStream *st;

    avio_skip(pb, 3);
    flags = avio_r8(pb);
    rate = avio_rl32(pb);

    rate = (rate >> 20) & 0xFFF;
    switch (rate) {
    case 3763: rate = 44100; break;
    case 1365: rate = 16000; break;
    case 940:  rate = 11050; break;
    case 460:  rate = 5000;  break;
    default:
        av_log(s, AV_LOG_WARNING, "unknown rate value %x\n", rate);
        rate = rate * 11.72;
        break;
    }

    switch (flags) {
    case 0xC0:
    case 0x40:
    case 0xA1:
    case 0x21:
        codec = AV_CODEC_ID_ADPCM_PSX;
        channels = (flags == 0x21 || flags == 0x40) ? 1 : 2;
        align = 16;
        start = 8;
        bit_rate = 16LL * channels * 8 * rate / 28;
        break;
    case 0x80:
    case 0x81:
    case 0x01:
        codec = AV_CODEC_ID_ADPCM_PSXC;
        channels = (flags == 0x01) ? 1 : 2;
        align = 16;
        start = 8;
        bit_rate = 16LL * channels * 8 * rate / 30;
        break;
    case 0xD1:
        codec = AV_CODEC_ID_ADPCM_NDSP;
        channels = 2;
        align = 8;
        start  = 8 + 0x60 * channels;
        bit_rate = 8LL * channels * 8 * rate / 14;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    if (rate <= 0 || channels <= 0 || align <= 0)
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

    if (codec == AV_CODEC_ID_ADPCM_NDSP) {
        ret = ff_alloc_extradata(st->codecpar, 32 * channels);
        if (ret < 0)
            return ret;

        avio_seek(pb, 8 + 0x1c, SEEK_SET);
        for (int ch = 0; ch < channels; ch++) {
            avio_read(pb, st->codecpar->extradata + 32 * ch, 32);
            avio_skip(pb, 0x40);
        }
    }

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_psf_demuxer = {
    .p.name         = "psf",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Pivotal PSF"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "psf",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
