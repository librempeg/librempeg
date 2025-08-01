/*
 * MIPU video demuxer
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
#include "avio_internal.h"
#include "rawdec.h"

#include "libavutil/intreadwrite.h"

static int mipu_read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('M', 'I', 'P', 'U'))
        return 0;

    if (p->buf_size < 20)
        return 0;

    if (AV_RL16(p->buf + 6) == 0)
        return 0;

    if (AV_RL16(p->buf + 8) == 0)
        return 0;

    if (AV_RL16(p->buf + 10) == 0)
        return 0;

    if (AV_RL32(p->buf + 16) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int mipu_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st = avformat_new_stream(s, NULL);
    int64_t start_offset;

    if (!st)
        return AVERROR(ENOMEM);
    avio_skip(pb, 6);
    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id = AV_CODEC_ID_IPU;
    st->codecpar->width = avio_rl16(pb);
    st->codecpar->height = avio_rl16(pb);
    st->start_time = 0;

    avio_skip(pb, 6);
    start_offset = avio_rl32(pb);
    st->duration =
    st->nb_frames = (start_offset - avio_tell(pb)) / 4;
    avio_skip(pb, start_offset - avio_tell(pb));
    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
    avpriv_set_pts_info(st, 64, 1, 25);

    return 0;
}

const FFInputFormat ff_mipu_demuxer = {
    .p.name         = "mipu",
    .p.long_name    = NULL_IF_CONFIG_SMALL("MIPU Video"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.priv_class   = &ff_raw_demuxer_class,
    .read_probe     = mipu_read_probe,
    .read_header    = mipu_read_header,
    .read_packet    = ff_raw_read_partial_packet,
    .raw_codec_id   = AV_CODEC_ID_IPU,
    .priv_data_size = sizeof(FFRawDemuxerContext),
};
