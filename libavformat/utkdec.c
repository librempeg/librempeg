/*
 * Maxis UTK demuxer
 * Copyright (c) 2017 Peter Ross
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
#include "riff.h"

static int utk_probe(const AVProbeData *p)
{
    if (p->buf_size < 32
       || memcmp(p->buf, "UTM0", 4)
       || !AV_RL32(p->buf + 4) || (AV_RL32(p->buf + 4) & 1) /* dwOutSize */
       || AV_RL32(p->buf + 8) < 14 /* dwWfxSize */
       || AV_RL16(p->buf + 12) != 1 /* wFormatTag) */
       ) return 0;
    return AVPROBE_SCORE_MAX;
}

static int utk_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st;
    unsigned int size;
    int ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 4);
    st->duration = avio_rl32(pb) * 2LL;
    size = avio_rl32(pb);
    if ((ret = ff_alloc_extradata(st->codecpar, 1)) < 0)
        return ret;
    st->codecpar->extradata[0] = 0;

    if ((ret = ff_get_wav_header(s, pb, st->codecpar, size, 0)) < 0)
        return ret;
    st->codecpar->codec_id = AV_CODEC_ID_UTK;
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    return 0;
}

static int utk_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int64_t pos = avio_tell(s->pb);
    int ret;

    if (avio_feof(s->pb))
        return AVERROR_EOF;

    if ((ret = av_get_packet(s->pb, pkt, 256)) < 0)
        return ret;

    pkt->stream_index = 0;
    if (pos == 32)
        pkt->flags |= AV_PKT_FLAG_KEY;

    return 0;
}

const FFInputFormat ff_utk_demuxer = {
    .p.name         = "utk",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Maxis UTK"),
    .p.extensions   = "utk",
    .p.flags        = AVFMT_NOGENSEARCH | AVFMT_NO_BYTE_SEEK | AVFMT_NOBINSEARCH | AVFMT_NOTIMESTAMPS,
    .read_probe     = utk_probe,
    .read_header    = utk_read_header,
    .read_packet    = utk_read_packet,
    .raw_codec_id   = AV_CODEC_ID_UTK,
};
