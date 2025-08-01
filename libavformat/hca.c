/*
 * HCA demuxer
 * Copyright (c) 2020 Paul B Mahol
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

#include "libavutil/opt.h"
#include "libavutil/intreadwrite.h"
#include "libavcodec/bytestream.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"

#define HCA_MASK 0x7f7f7f7f

typedef struct HCADemuxContext {
    AVClass *class;
    int64_t keyl;
    int64_t keyh;
    int subkey;
} HCADemuxContext;

static int hca_probe(const AVProbeData *p)
{
    if ((AV_RL32(p->buf) & HCA_MASK) != MKTAG('H', 'C', 'A', 0))
        return 0;

    if ((AV_RL32(p->buf + 8) & HCA_MASK) != MKTAG('f', 'm', 't', 0))
        return 0;

    return AVPROBE_SCORE_MAX / 3;
}

static int hca_read_header(AVFormatContext *s)
{
    HCADemuxContext *hca = s->priv_data;
    AVCodecParameters *par;
    GetByteContext gb;
    AVIOContext *pb = s->pb;
    AVStream *st;
    uint32_t chunk;
    uint16_t version;
    uint32_t block_count;
    uint16_t block_size, data_offset;
    int ret;

    avio_skip(pb, 4);
    version = avio_rb16(pb);

    data_offset = avio_rb16(pb);
    if (data_offset <= 8)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    par = st->codecpar;
    ret = ff_alloc_extradata(par, data_offset + 10);
    if (ret < 0)
        return ret;

    ret = avio_read(pb, par->extradata + 8, par->extradata_size - 8 - 10);
    if (ret < par->extradata_size - 8 - 10)
        return AVERROR(EIO);
    AV_WL32(par->extradata, MKTAG('H', 'C', 'A', 0));
    AV_WB16(par->extradata + 4, version);
    AV_WB16(par->extradata + 6, data_offset);
    AV_WB32(par->extradata + par->extradata_size - 10, hca->keyh);
    AV_WB32(par->extradata + par->extradata_size -  6, hca->keyl);
    AV_WB16(par->extradata + par->extradata_size -  2, hca->subkey);

    bytestream2_init(&gb, par->extradata + 8, par->extradata_size - 8);

    if ((bytestream2_get_le32(&gb) & HCA_MASK) != MKTAG('f', 'm', 't', 0))
        return AVERROR_INVALIDDATA;

    par->codec_type  = AVMEDIA_TYPE_AUDIO;
    par->codec_id    = AV_CODEC_ID_HCA;
    par->codec_tag   = 0;
    st->codecpar->ch_layout.nb_channels = bytestream2_get_byte(&gb);
    par->sample_rate = bytestream2_get_be24(&gb);
    block_count      = bytestream2_get_be32(&gb);
    bytestream2_skip(&gb, 4);
    chunk = bytestream2_get_le32(&gb) & HCA_MASK;
    if (chunk == MKTAG('c', 'o', 'm', 'p')) {
        block_size = bytestream2_get_be16(&gb);
    } else if (chunk == MKTAG('d', 'e', 'c', 0)) {
        block_size = bytestream2_get_be16(&gb);
    } else {
        return AVERROR_INVALIDDATA;
    }

    if (block_size < 8)
        return AVERROR_INVALIDDATA;
    par->block_align = block_size;
    st->duration = 1024 * block_count;
    st->start_time = 0;

    avio_seek(pb, data_offset, SEEK_SET);
    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    return 0;
}

static int hca_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    int ret;

    ret = av_get_packet(s->pb, pkt, par->block_align);
    pkt->duration = 1024;
    return ret;
}

#define OFFSET(x) offsetof(HCADemuxContext, x)
static const AVOption hca_options[] = {
    { "hca_lowkey",
        "Low key used for handling CRI HCA files", OFFSET(keyl),
        AV_OPT_TYPE_INT64, {.i64=0}, .min = 0, .max = UINT32_MAX, .flags = AV_OPT_FLAG_DECODING_PARAM, },
    { "hca_highkey",
        "High key used for handling CRI HCA files", OFFSET(keyh),
        AV_OPT_TYPE_INT64, {.i64=0}, .min = 0, .max = UINT32_MAX, .flags = AV_OPT_FLAG_DECODING_PARAM, },
    { "hca_subkey",
        "Subkey used for handling CRI HCA files", OFFSET(subkey),
        AV_OPT_TYPE_INT, {.i64=0}, .min = 0, .max = UINT16_MAX, .flags = AV_OPT_FLAG_DECODING_PARAM },
    { NULL },
};

static const AVClass hca_class = {
    .class_name = "hca",
    .option     = hca_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFInputFormat ff_hca_demuxer = {
    .p.name         = "hca",
    .p.long_name    = NULL_IF_CONFIG_SMALL("CRI HCA"),
    .p.priv_class   = &hca_class,
    .p.extensions   = "hca",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(HCADemuxContext),
    .read_probe     = hca_probe,
    .read_header    = hca_read_header,
    .read_packet    = hca_read_packet,
};
