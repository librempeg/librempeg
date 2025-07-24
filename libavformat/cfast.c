/*
 * CFAST demuxer
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

typedef struct CFASTDemuxContext {
    int nb_bit_planes;
} CFASTDemuxContext;

static int cfast_read_probe(const AVProbeData *p)
{
    int offset = 4;

    if (AV_RB32(p->buf) != MKBETAG('G','U','C','F') &&
        AV_RB32(p->buf) != MKBETAG('S','T','D','Y'))
        return 0;

    if (AV_RB32(p->buf) == MKBETAG('S','T','D','Y'))
        offset += 2 + AV_RB16(p->buf+4);

    if (p->buf_size < offset + 20)
        return 0;

    if (AV_RB32(p->buf+offset+  0) == 0 ||
        AV_RB32(p->buf+offset+  4) == 0 ||
        AV_RB32(p->buf+offset+ 16) == 0)
        return 0;

    return 2*AVPROBE_SCORE_MAX/3;
}

static int cfast_read_header(AVFormatContext *s)
{
    CFASTDemuxContext *cfast = s->priv_data;
    int ret, n, width, height, nb_frames;
    AVIOContext *pb = s->pb;
    uint8_t hdr[8];
    uint32_t tag;
    AVStream *st;

    tag = avio_rb32(pb);
    if (tag == MKBETAG('S','T','D','Y')) {
        n = avio_rb16(pb);
        avio_skip(pb, n);
    }
    width = avio_rb32(pb);
    height = avio_rb32(pb);
    avio_skip(pb, 8);
    if (width <= 0 || height <= 0)
        return AVERROR_INVALIDDATA;

    ret = avio_read(pb, hdr, sizeof(hdr));
    if (ret != sizeof(hdr))
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    if ((ret = ff_alloc_extradata(st->codecpar, 8 + hdr[7])) < 0)
        return ret;

    memcpy(st->codecpar->extradata, hdr, sizeof(hdr));
    if (hdr[7] > 0)
        avio_read(pb, st->codecpar->extradata + 8, hdr[7]);

    cfast->nb_bit_planes = hdr[0];
    nb_frames = avio_rb32(pb);

    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id = AV_CODEC_ID_CFAST;
    st->codecpar->width = width;
    st->codecpar->height = height;
    st->nb_frames = st->duration = nb_frames;

    avpriv_set_pts_info(st, 64, 1, 15);

    return 0;
}

static int cfast_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    CFASTDemuxContext *cfast = s->priv_data;
    const int nb_bit_planes = cfast->nb_bit_planes;
    AVIOContext *pb = s->pb;
    int ret, offset = 0;
    int32_t size;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);

    for (int n = 0; n < nb_bit_planes; n++) {
        size = avio_rb32(pb);

        if (size < 0)
            return AVERROR_INVALIDDATA;

        ret = av_grow_packet(pkt, size + 4);
        if (ret < 0)
            return ret;

        AV_WB32(pkt->data + offset, size);
        offset += 4;
        ret = avio_read(pb, pkt->data + offset, size);
        if (ret != size)
            return AVERROR_INVALIDDATA;
        offset += size;
    }

    size = avio_r8(pb);
    ret = av_grow_packet(pkt, 3 * size + 1);
    if (ret < 0)
        return ret;

    AV_WB8(pkt->data + offset, size);
    offset++;

    ret = avio_read(pb, pkt->data + offset, 3 * size);
    if (ret != 3 * size)
        return AVERROR_INVALIDDATA;
    offset += 3 * size;

    size = avio_r8(pb);
    ret = av_grow_packet(pkt, size * 44 + 1);
    if (ret < 0)
        return ret;

    AV_WB8(pkt->data + offset, size);
    offset++;

    ret = avio_read(pb, pkt->data + offset, 44 * size);
    if (ret != 44 * size)
        return AVERROR_INVALIDDATA;
    offset += 44 * size;

    pkt->stream_index = 0;
    pkt->duration = 1;
    pkt->pos = pos;

    return ret;
}

const FFInputFormat ff_cfast_demuxer = {
    .p.name         = "cfast",
    .p.long_name    = NULL_IF_CONFIG_SMALL("CFAST (Disney Animation Studio)"),
    .p.extensions   = "cft",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(CFASTDemuxContext),
    .read_probe     = cfast_read_probe,
    .read_header    = cfast_read_header,
    .read_packet    = cfast_read_packet,
};
