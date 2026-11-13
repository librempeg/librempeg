/*
 * Radical P3D demuxer
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
    if (AV_RB32(p->buf) != MKBETAG('P','3','D',0xFF) &&
        AV_RL32(p->buf) != MKBETAG('P','3','D',0xFF))
        return 0;

    return AVPROBE_SCORE_MAX/2;
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
    int64_t start = 0, bitrate = 0, offset, name_offset = 0, duration = 0;
    unsigned int (*avio_r32)(AVIOContext *pb);
    int rate, be, channels, codec, align;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    be = guess_endian32(pb);
    avio_r32 = be ? avio_rb32 : avio_rl32;
    avio_seek(pb, 4, SEEK_SET);
    if (avio_r32(pb) != 12)
        return AVERROR_INVALIDDATA;
    offset = 12;

    avio_seek(pb, 20, SEEK_SET);
    if (avio_rb64(pb) == AV_RB64("AudioDia")) {
        avio_seek(pb, 16, SEEK_SET);
        offset += avio_r32(pb);
    }

    avio_seek(pb, offset, SEEK_SET);
    if (avio_r32(pb) != 0xFE000000)
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 8);
    if (avio_r32(pb) != 0x0000000A)
        return AVERROR_INVALIDDATA;
    if (avio_r32(pb) != 9)
        return AVERROR_INVALIDDATA;
    offset += 20;

    avio_seek(pb, offset, SEEK_SET);
    if (avio_rb64(pb) != AV_RB64("AudioFil"))
        return AVERROR_INVALIDDATA;
    if (avio_rb16(pb) != 0x6500)
        return AVERROR_INVALIDDATA;
    offset += 10;

    avio_seek(pb, offset, SEEK_SET);
    int name_count = avio_r32(pb);
    if (name_count != 2 && name_count != 3)
        return AVERROR_INVALIDDATA;
    offset += 4;

    for (int i = 0; i < 2; i++) {
        if (!name_offset)
            name_offset = offset + 4;
        avio_seek(pb, offset, SEEK_SET);
        int64_t text_len = avio_r32(pb) + 1LL;
        offset += 4 + text_len;
    }

    avio_seek(pb, offset, SEEK_SET);
    int64_t unk_count = avio_r32(pb);
    if (unk_count != 0 && unk_count != 1)
        return AVERROR_INVALIDDATA;
    offset += 4;

    avio_seek(pb, offset, SEEK_SET);
    int64_t text_len = avio_r32(pb);
    codec = avio_rb32(pb);
    offset += 4LL + text_len + 1LL;

    avio_seek(pb, offset, SEEK_SET);
    if (name_count >= 3) {
        text_len = avio_r32(pb) + 1LL;
        offset += 4 + text_len;
        avio_seek(pb, offset, SEEK_SET);
    }

    switch (codec) {
    case MKBETAG('r','a','d','p'):
        codec = AV_CODEC_ID_ADPCM_IMA_RAD_MONO;
        avio_skip(pb, 4);
        channels = avio_r32(pb);
        rate = avio_r32(pb);
        align = 20 * channels;
        start = offset + 20;
        bitrate = 20LL * channels * 8 * rate / 32;
        break;
    case MKBETAG('m','p','3','\0'):
        codec = AV_CODEC_ID_MP3;
        offset += 3;
        avio_skip(pb, 3);
        rate = avio_rl32(pb);
        duration = avio_rl32(pb);
        avio_skip(pb, 4);
        channels = FFMAX(1, avio_rl32(pb));
        align = 1024;
        duration /= channels;
        start = offset + 24;
        break;
    case MKBETAG('x','m','a','\0'):
        codec = AV_CODEC_ID_XMA2;
        avio_skip(pb, 4);
        start = offset;
        start += avio_rb32(pb);
        start += avio_rb32(pb);
        avio_skip(pb, 8);
        align = 0x800;
        start += 20;
        rate = 48000;
        channels = 2;
        break;
    default:
        avpriv_request_sample(s, "codec %X", codec);
        return AVERROR_INVALIDDATA;
    }

    if (align <= 0 || rate <= 0 || channels <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    if (codec == AV_CODEC_ID_MP3) {
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL;
    } else if (codec == AV_CODEC_ID_XMA2) {
        int type, num_streams;

        type = avio_r8(pb);
        num_streams = avio_r8(pb);

        avio_skip(pb, 10);
        rate = avio_rb32(pb);
        avio_skip(pb, 4);
        if (type != 3)
            avio_skip(pb, 8);
        duration = avio_rb32(pb);
        avio_skip(pb, 8);
        channels = 0;
        for (int i = 0; i < num_streams; i++) {
            channels += avio_r8(pb);
            avio_skip(pb, 3);
        }

        if (rate <= 0 || channels <= 0)
            return AVERROR_INVALIDDATA;

        int ret = ff_alloc_extradata(st->codecpar, 34);
        if (ret < 0)
            return ret;
        memset(st->codecpar->extradata, 0, 34);
        AV_WL16(st->codecpar->extradata, (channels+1)/2);
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
    }

    st->start_time = 0;
    if (duration > 0)
        st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align;
    if (bitrate > 0)
        st->codecpar->bit_rate = bitrate;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_p3d_demuxer = {
    .p.name         = "p3d",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Radical P3D"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "p3d",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
