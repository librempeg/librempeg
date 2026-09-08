/*
 * KNON demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('K','N','O','N'))
        return 0;
    if (AV_RB32(p->buf+8) != MKBETAG('W','I','I',' '))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int codec, channels, align, rate, ret;
    int64_t bitrate;
    AVIOContext *pb = s->pb;
    AVStream *st;

    channels = 2;
    avio_skip(pb, 0x20);
    codec = avio_rb32(pb);
    avio_skip(pb, 0x18);
    avio_skip(pb, 4);
    rate = avio_rb32(pb);

    switch (codec) {
    case MKBETAG('K','A','S','T'):
        codec = AV_CODEC_ID_ADPCM_NDSP;
        align = 16;
        bitrate = 8LL * channels * 8 * rate / 14;
        break;
    case MKBETAG('K','P','S','T'):
        codec = AV_CODEC_ID_PCM_S16BE_PLANAR;
        align = 16;
        bitrate = 2LL * channels * 8 * rate;
        break;
    default:
        avpriv_request_sample(s, "codec %X", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (rate <= 0 || align <= 0 || align >= INT_MAX/2)
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

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if (codec == AV_CODEC_ID_ADPCM_NDSP) {
        avio_seek(pb, 0x8c, SEEK_SET);

        ret = ff_alloc_extradata(st->codecpar, 32 * 2);
        if (ret < 0)
            return ret;

        for (int ch = 0; ch < channels; ch++) {
            avio_read(pb, st->codecpar->extradata + ch * 32, 32);
            avio_skip(pb, 0x40);
        }
    }

    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

const FFInputFormat ff_knon_demuxer = {
    .p.name         = "knon",
    .p.long_name    = NULL_IF_CONFIG_SMALL("KNON"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "str,asr",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
