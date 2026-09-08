/*
 * Raw PSX demuxer
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
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int psx_probe(uint8_t *buf, int offset, int size)
{
    int score = 0;

    for (int i = offset; i < size - 2; i += 16) {
        int predictor = (buf[i+0] >> 4) & 15;
        int flags = buf[i+1];

        if (predictor > 5 || flags > 7)
            return 0;

        for (int j = i + 2; j < FFMIN(i + 16, size); j++) {
            if (buf[j]) {
                score++;
                break;
            }
        }
    }

    return score;
}

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 0x2000)
        return 0;

    return FFMIN(AVPROBE_SCORE_MAX, psx_probe(p->buf, 0, p->buf_size));
}

static int read_header(AVFormatContext *s)
{
    int channels = 1, align = 0, ret;
    AVIOContext *pb = s->pb;
    uint8_t start[0x10];
    uint8_t tmp[0x10];
    AVStream *st;

    ret = ffio_read_size(pb, start, sizeof(start));
    if (ret < 0)
        return ret;

    while (!avio_feof(pb)) {
        ret = ffio_read_size(pb, tmp, sizeof(tmp));
        if (ret < 0)
            return ret;

        if (!memcmp(start, tmp, sizeof(start))) {
            align = avio_tell(pb) - 16;
            channels++;
        }

        if (align > 0)
            break;
    }

    if (align <= 0)
        return AVERROR_INVALIDDATA;

    while (!avio_feof(pb)) {
        avio_seek(pb, align * channels, SEEK_SET);

        ret = ffio_read_size(pb, tmp, sizeof(tmp));
        if (ret < 0)
            return ret;

        if (!memcmp(start, tmp, sizeof(start)))
            channels++;
        else
            break;

        if (channels >= INT_MAX/align)
            return AVERROR_INVALIDDATA;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = 44100;
    st->codecpar->block_align = align * channels;
    st->codecpar->bit_rate = 16LL * channels * 8 * 44100 / 28;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0, SEEK_SET);

    return 0;
}

const FFInputFormat ff_rawpsx_demuxer = {
    .p.name         = "rawpsx",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Raw PSX"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
