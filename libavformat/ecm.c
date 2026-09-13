/*
 * ECM demuxer
 *
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
#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/internal.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('E','C','M','\2'))
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *ast, *vst;

    ast = avformat_new_stream(s, NULL);
    if (!ast)
        return AVERROR(ENOMEM);

    ast->start_time = 0;
    ast->codecpar->block_align = 1;
    ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    ast->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_DVI;
    ast->codecpar->ch_layout.nb_channels = 1;
    ast->codecpar->sample_rate = 22050;

    avpriv_set_pts_info(ast, 64, 1, ast->codecpar->sample_rate);

    vst = avformat_new_stream(s, NULL);
    if (!vst)
        return AVERROR(ENOMEM);

    vst->start_time = 0;
    vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    vst->codecpar->codec_id = AV_CODEC_ID_CINEPAK;
    vst->codecpar->width = 320;
    vst->codecpar->height = 240;

    avpriv_set_pts_info(vst, 64, 1, 15);

    avio_seek(pb, 0xe18, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int64_t pos, duration;
    int ret, size, index;
    uint32_t type;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    type = avio_rb32(pb);
    switch (type) {
    case MKBETAG('S','E','C','\0'):
        size = 11025;
        ret = av_get_packet(pb, pkt, size);
        if (ret < 0)
            return ret;
        avio_skip(pb, 3);
        index = 0;
        duration = size;
        break;
    case MKBETAG('E','D','L','\1'):
        size = avio_rl32(pb);
        if (size <= 8)
            return AVERROR_INVALIDDATA;

        ret = av_get_packet(pb, pkt, size - 8);
        if (ret < 0)
            return ret;
        index = 1;
        duration = 1;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    pkt->pos = pos;
    pkt->duration = duration;
    pkt->stream_index = index;

    return ret;
}

const FFInputFormat ff_ecm_demuxer = {
    .p.name         = "ecm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Eurocom ECM"),
    .p.extensions   = "ecm",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
