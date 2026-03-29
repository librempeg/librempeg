/*
 * Stormfront SEG demuxer
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
    uint32_t codec;

    if (AV_RB32(p->buf) != MKBETAG('s','e','g','\0'))
        return 0;

    codec = AV_RB32(p->buf+4);
    switch (codec) {
    case MKBETAG('p','s','2','\x0'):
    case MKBETAG('x','b','x','\x0'):
    case MKBETAG('w','i','i','\x0'):
    case MKBETAG('p','c','_','\x0'):
    case MKBETAG('x','b','3','\x0'):
        break;
    default:
        return 0;
    }

    return AVPROBE_SCORE_MAX;
}

static int guess_endian32(AVIOContext *pb)
{
    uint8_t buf[4];

    if (avio_read(pb, buf, 4) != 4)
        return -1;

    return AV_RL32(buf) > AV_RB32(buf) ? 1 : 0;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels, ret, codec, be, align, profile = 0;
    int64_t start_offset = 0x4000, duration;
    unsigned (*avio_r32)(AVIOContext *pb);
    AVIOContext *pb = s->pb;
    char title[0x22] = { 0 };
    AVStream *st;

    avio_skip(pb, 4);
    codec = avio_rb32(pb);
    be = guess_endian32(pb);
    avio_r32 = be ? avio_rb32 : avio_rl32;
    avio_skip(pb, 12);
    rate = avio_r32(pb);
    duration = avio_r32(pb);
    avio_skip(pb, 4);
    channels = avio_r32(pb);
    if (rate <= 0 || channels <= 0 || channels >= INT_MAX/0x2000)
        return AVERROR_INVALIDDATA;

    switch (codec) {
    case MKBETAG('p','s','2','\x0'):
        codec = AV_CODEC_ID_ADPCM_PSX;
        align = 0x2000 * channels;
        break;
    case MKBETAG('x','b','x','\x0'):
        codec = AV_CODEC_ID_ADPCM_IMA_XBOX;
        align = 36 * channels;
        break;
    case MKBETAG('w','i','i','\x0'):
        codec = AV_CODEC_ID_ADPCM_NDSP;
        align = 0x2000 * channels;
        break;
    case MKBETAG('p','c','_','\x0'):
        codec = AV_CODEC_ID_ADPCM_IMA_WS;
        profile = 3;
        align = 0x2000 * channels;
        break;
    case MKBETAG('x','b','3','\x0'):
        codec = AV_CODEC_ID_XMA2;
        align = 0x800;
        break;
    default:
        avpriv_request_sample(s, "codec %08x", codec);
        return AVERROR_PATCHWELCOME;
    }

    avio_seek(pb, 0x38, SEEK_SET);
    if ((ret = avio_get_str(pb, INT_MAX, title, sizeof(title))) < 0)
        return ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    if (title[0] != '\0')
        av_dict_set(&st->metadata, "title", title, 0);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->codec_id = codec;
    st->codecpar->profile = profile;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    switch (codec) {
    case AV_CODEC_ID_XMA2:
        if (avio_rb32(pb) == 0) {
            st->codecpar->codec_id = AV_CODEC_ID_MP3;
            start_offset += 0x20A;
        } else {
            ret = ff_alloc_extradata(st->codecpar, 34);
            if (ret < 0)
                return ret;

            memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
            AV_WL16(st->codecpar->extradata, (channels+1)/2);
        }

        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

        avio_seek(pb, start_offset, SEEK_SET);
        break;
    case AV_CODEC_ID_ADPCM_NDSP:
        avio_seek(pb, start_offset + 0x1c, SEEK_SET);
        ret = ff_alloc_extradata(st->codecpar, 32 * channels + 1);
        if (ret < 0)
            return ret;

        for (int ch = 0; ch < channels; ch++) {
            avio_read(pb, st->codecpar->extradata+32*ch, 32);
            avio_skip(pb, 0x2000-32);
        }

        st->codecpar->extradata[32 * channels] = 0x60;
        avio_seek(pb, start_offset, SEEK_SET);
        break;
    }

    return 0;
}

const FFInputFormat ff_seg_demuxer = {
    .p.name         = "seg",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Stormfront SEG"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "seg",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
