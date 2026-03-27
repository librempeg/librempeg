/*
 * WAVE demuxer
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

#include "libavutil/intfloat.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('V','A','W','3') &&
        AV_RB32(p->buf) != MKBETAG('W','W','A','V'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, rate, channels, align, codec, be, profile = 0;
    uint32_t magic, version, loop_start, loop_stop;
    int64_t start_offset, duration, extra_offset;
    AVIOContext *pb = s->pb;
    AVStream *st;

    magic = avio_rb32(pb);
    be = magic == 0xE5B7ECFE || magic == AV_RB32("WWAV");
    unsigned (*avio_r32)(AVIOContext *pb) = be ? avio_rb32 : avio_rl32;

    version = avio_r32(pb);
    avio_skip(pb, 4);
    rate = av_int2float(avio_r32(pb));
    duration = avio_r32(pb);
    loop_start = avio_r32(pb);
    loop_stop = avio_r32(pb);
    codec = avio_r8(pb);
    channels = avio_r8(pb);
    avio_skip(pb, 2);
    start_offset = avio_r32(pb);
    align = avio_r32(pb);
    extra_offset = avio_r32(pb);

    if (codec == 0 && version == 0x00050000 && start_offset > 0x40) {
        codec = 2;
    } else if (codec == 2 && start_offset <= 0x40) {
        codec = 3;
    }

    switch (codec) {
    case 0:
        codec = AV_CODEC_ID_PCM_U8;
        break;
    case 1:
        codec = AV_CODEC_ID_PCM_S16BE;
        break;
    case 2:
        codec = be ? AV_CODEC_ID_ADPCM_NDSP : AV_CODEC_ID_ADPCM_NDSP_LE;
        break;
    case 3:
        codec = AV_CODEC_ID_ADPCM_IMA_WS;
        profile = 4;
        break;
    default:
        avpriv_request_sample(s, "codec %02x", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (rate <= 0 || channels <= 0 || align <= 0 || align > INT_MAX/channels)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    if (codec == AV_CODEC_ID_MP3)
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
    if (codec == AV_CODEC_ID_ADPCM_IMA_WS)
        st->codecpar->profile = 4;

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = (channels > 1) ?  align * channels : FFMIN(align, 1024);
    st->codecpar->sample_rate = rate;
    st->codecpar->codec_id = codec;
    st->codecpar->profile = profile;

    if (loop_start > 0)
        av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);
    if (loop_stop > 0)
        av_dict_set_int(&st->metadata, "loop_stop", loop_stop, 0);

    if (codec == AV_CODEC_ID_ADPCM_NDSP ||
        codec == AV_CODEC_ID_ADPCM_NDSP_LE) {
        avio_seek(pb, extra_offset, SEEK_SET);

        ret = ff_alloc_extradata(st->codecpar, 32 * channels);
        if (ret < 0)
            return ret;

        for (int ch = 0; ch < channels; ch++) {
            uint8_t *ptr = st->codecpar->extradata + ch * 32;

            avio_read(pb, ptr, 32);
            avio_skip(pb, 12 + 2 * (version == 0x00050000));
        }
    }

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

const FFInputFormat ff_wave_demuxer = {
    .p.name         = "wave",
    .p.long_name    = NULL_IF_CONFIG_SMALL("EngineBlack WayForward"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "wave",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
