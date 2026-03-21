/*
 * NGC DSP STD demuxer
 * Copyright (c) 2025 Paul B Mahol
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

#define READ_PROBE(score, R32, R16)             \
do {                                            \
    if (p->buf_size < 0x60) {                   \
        score = 0;                              \
        break;                                  \
    }                                           \
                                                \
    if (R32(p->buf) == 0) {                     \
        score = 0;                              \
        break;                                  \
    }                                           \
    score++;                                    \
                                                \
    if (R32(p->buf+4) == 0) {                   \
        score = 0;                              \
        break;                                  \
    }                                           \
    score++;                                    \
                                                \
    if (((int)R32(p->buf+8)) <= 0) {            \
        score = 0;                              \
        break;                                  \
    }                                           \
    score += 4;                                 \
                                                \
    if (R16(p->buf+0xc) != 0 &&                 \
        R16(p->buf+0xc) != 1) {                 \
        score = 0;                              \
        break;                                  \
    }                                           \
    score += 4;                                 \
                                                \
    if (R16(p->buf+0xe) != 0) {                 \
        score = 0;                              \
        break;                                  \
    }                                           \
    score += 2;                                 \
                                                \
    for (int n = 0 ; n < 16; n++) {             \
        if (R16(p->buf + 0x1c + n * 2) != 0)    \
            score += 2;                         \
    }                                           \
                                                \
    if (R16(p->buf+0x3c) != 0) {                \
        score = 0;                              \
        break;                                  \
    }                                           \
    score += 2;                                 \
} while (0);

static int read_probe(const AVProbeData *p)
{
    int score_le = 0, score_be = 0;

    READ_PROBE(score_le, AV_RL32, AV_RL16)
    READ_PROBE(score_be, AV_RB32, AV_RB16)

    return FFMAX(score_le, score_be);
}

typedef unsigned (*avio_r32)(AVIOContext *s);
typedef unsigned (*avio_r16)(AVIOContext *s);

static int read_header_e(AVFormatContext *s,
                         int codec,
                         avio_r32 r32,
                         avio_r16 r16)
{
    AVIOContext *pb = s->pb;
    int64_t duration;
    int ret, rate;
    AVStream *st;

    duration = r32(pb);
    avio_skip(pb, 4);
    rate = r32(pb);
    avio_skip(pb, 2);
    if (r16(pb) != 0)
        return AVERROR_INVALIDDATA;
    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->sample_rate = rate;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->block_align = 8;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0x1c, SEEK_SET);
    if ((ret = ff_get_extradata(s, st->codecpar, pb, 32 * st->codecpar->ch_layout.nb_channels)) < 0)
        return ret;

    avio_seek(pb, 0x60, SEEK_SET);

    return 0;
}

static int read_header(AVFormatContext *s)
{
    int ret;

    ret = read_header_e(s, AV_CODEC_ID_ADPCM_NDSP, avio_rb32, avio_rb16);
    if (ret == AVERROR_INVALIDDATA) {
        avio_seek(s->pb, 0, SEEK_SET);
        ret = read_header_e(s, AV_CODEC_ID_ADPCM_NDSP_LE, avio_rl32, avio_rl16);
    }

    return ret;
}

const FFInputFormat ff_ngcdspstd_demuxer = {
    .p.name         = "ngcdspstd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("NGC (Nintendo DSP Standard)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "dsp",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
