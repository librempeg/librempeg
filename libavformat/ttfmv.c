/*
 * Traveller's Tales FMV demuxer
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
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('F','M','V','!'))
        return 0;

    if (p->buf_size < 36)
        return 0;
    if ((int)AV_RL16(p->buf+8) <= 0)
        return 0;
    if ((int)AV_RL16(p->buf+10) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf+32) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int rate, fps;
    AVStream *vst = avformat_new_stream(s, NULL);
    if (!vst)
        return AVERROR(ENOMEM);

    avio_skip(pb, 8);
    vst->start_time = 0;
    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_id = AV_CODEC_ID_CINEPAK;
    vst->codecpar->width = avio_rl16(pb);
    vst->codecpar->height = avio_rl16(pb);

    avio_skip(pb, 9);
    fps = avio_r8(pb);
    avpriv_set_pts_info(vst, 64, 1, fps);
    avio_skip(pb, 10);
    rate = avio_rl32(pb);
    if (rate <= 0)
        return AVERROR_INVALIDDATA;

    AVStream *ast = avformat_new_stream(s, NULL);
    if (!ast)
        return AVERROR(ENOMEM);

    ast->start_time = 0;
    ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    ast->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_SSI;
    ast->codecpar->sample_rate = rate;
    ast->codecpar->ch_layout.nb_channels = 2;

    avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);

    avio_seek(pb, 0x28, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret, index, size, key = 0;
    AVIOContext *pb = s->pb;
    uint32_t type;
    int64_t pos;

    pos = avio_tell(pb);
    type = avio_rb32(pb);
    size = avio_rl32(pb);
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (size < 0)
        return AVERROR_INVALIDDATA;

    if (size == 0)
        return FFERROR_REDO;

    switch (type) {
    case MKBETAG('F','M','A','\0'):
        index = 1;
        key = 1;
        break;
    case MKBETAG('F','M','V','k'):
        key = 1;
        av_fallthrough;
    case MKBETAG('F','M','V','d'):
    case MKBETAG('F','M','V','n'):
        index = 0;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    ret = av_get_packet(pb, pkt, size);
    if (ret < 0)
        return ret;
    pkt->pos = pos;
    pkt->stream_index = index;
    if (key)
        pkt->flags |= AV_PKT_FLAG_KEY;

    return ret;
}

const FFInputFormat ff_ttfmv_demuxer = {
    .p.name         = "ttfmv",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Traveller's Tales FMV"),
    .p.extensions   = "fmv",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
