/*
 * WebP demuxer
 * Copyright (c) 2020 Pexeso Inc.
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * WebP demuxer.
 */

#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "rawdec.h"

static int webp_probe(const AVProbeData *p)
{
    const uint8_t *b = p->buf;
    int i = 12;

    if (AV_RB32(b)     != MKBETAG('R', 'I', 'F', 'F') ||
        AV_RB32(b + 8) != MKBETAG('W', 'E', 'B', 'P'))
        return 0;

    while (i + 8 < p->buf_size) {
        uint32_t chunk, size;

        chunk = AV_RL32(b+i);
        size = AV_RL32(b+i+4);
        size += size & 1;

        if (chunk == MKTAG('A', 'N', 'I', 'M'))
            return AVPROBE_SCORE_MAX;
        i += size + 8;
    }

    return 0;
}

static int webp_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    int64_t nb_frames = 0, duration = 0;
    int n, width = 0, height = 0;

    avio_skip(pb, 12);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->width  = 0;
    st->codecpar->height = 0;

    while (!avio_feof(pb)) {
        uint32_t chunk_type = avio_rl32(pb);
        uint32_t chunk_size = avio_rl32(pb);

        if (chunk_size == UINT32_MAX)
            return AVERROR_INVALIDDATA;
        chunk_size += chunk_size & 1;

        if (avio_feof(pb))
            break;

        switch (chunk_type) {
        case MKTAG('V', 'P', '8', 'X'):
            avio_skip(pb, 4);
            width  = avio_rl24(pb) + 1;
            height = avio_rl24(pb) + 1;
            break;
        case MKTAG('V', 'P', '8', ' '):
            avio_skip(pb, 6);
            width  = avio_rl16(pb) & 0x3fff;
            height = avio_rl16(pb) & 0x3fff;
            nb_frames++;
            avio_skip(pb, chunk_size - 10);
            break;
        case MKTAG('V', 'P', '8', 'L'):
            avio_skip(pb, 1);
            n = avio_rl32(pb);
            width  = (n & 0x3fff) + 1;          /* first 14 bits */
            height = ((n >> 14) & 0x3fff) + 1;  /* next 14 bits */
            nb_frames++;
            avio_skip(pb, chunk_size - 5);
            break;
        case MKTAG('A', 'N', 'M', 'F'):
            avio_skip(pb, 6);
            width  = avio_rl24(pb) + 1;
            height = avio_rl24(pb) + 1;
            duration += avio_rl24(pb);
            nb_frames++;
            avio_skip(pb, chunk_size - 15);
            break;
        default:
            avio_skip(pb, chunk_size);
        }

        if (st->codecpar->width == 0 && width > 0)
            st->codecpar->width = width;
        if (st->codecpar->height == 0 && height > 0)
            st->codecpar->height = height;
    }

    /* WebP format operates with time in "milliseconds",
     * therefore timebase is 1/1000 */
    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id   = AV_CODEC_ID_AWEBP;
    st->start_time           = 0;
    st->nb_frames            = nb_frames;
    st->duration             = duration;
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

    avpriv_set_pts_info(st, 64, 1, 1000);

    avio_seek(pb, 0, SEEK_SET);

    return 0;
}

const FFInputFormat ff_webp_demuxer = {
    .p.name         = "webp",
    .p.long_name    = NULL_IF_CONFIG_SMALL("WebP Animation"),
    .p.extensions   = "webp",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.priv_class   = &ff_raw_demuxer_class,
    .priv_data_size = sizeof(FFRawDemuxerContext),
    .read_probe     = webp_probe,
    .read_header    = webp_read_header,
    .read_packet    = ff_raw_read_partial_packet,
    .raw_codec_id   = AV_CODEC_ID_AWEBP,
};
