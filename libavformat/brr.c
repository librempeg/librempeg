/*
 * BRR demuxer
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

#include "avformat.h"
#include "demux.h"
#include "internal.h"

static int read_probe(const AVProbeData *p)
{
    int score = 0;

    if (av_match_ext(p->filename, "brr") == 0)
        return 0;

    for (int n = 0; n < p->buf_size - 9; n += 9) {
        if ((p->buf[n] >> 4) > 12)
            return 0;

        if (score < AVPROBE_SCORE_MAX)
            score += !!(p->buf[n] >> 4);
    }

    return score;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    unsigned valid = 0;
    int64_t start = 0;
    AVStream *st;

    while (start < 3) {
        while (!avio_feof(pb)) {
            if  ((avio_r8(pb) >> 4) > 12) {
                start += 2;
                valid = 0;
                avio_seek(pb, start, SEEK_SET);
            } else {
                avio_skip(pb, 8);
                valid++;
                if (valid > 13)
                    break;
            }
        }

        break;
    }

    if (start >= 3)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_BRR;
    st->codecpar->ch_layout.nb_channels = 1;
    st->codecpar->sample_rate = 32000;
    st->codecpar->block_align = 9 * 256;
    st->codecpar->bit_rate = (int64_t)st->codecpar->sample_rate * st->codecpar->ch_layout.nb_channels * 32 * 8LL / 9;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret;

    ret = av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_brr_demuxer = {
    .p.name         = "brr",
    .p.long_name    = NULL_IF_CONFIG_SMALL("BRR (Bit Rate Reduction)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "brr",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
