/*
 * BKHD demuxer
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

#include "libavutil/bswap.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"

typedef struct BKHDDemuxContext {
    int current_stream;
} BKHDDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('B','K','H','D'))
        return 0;
    if (p->buf_size < 32)
        return 0;

    return AVPROBE_SCORE_MAX;
}

typedef struct BKHDStream {
    int64_t start_offset;
    int64_t stop_offset;
    int64_t data_offset;

    AVFormatContext *xctx;
    AVFormatContext *parent;
    FFIOContext apb;
} BKHDStream;

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const BKHDStream *bs1 = s1->priv_data;
    const BKHDStream *bs2 = s2->priv_data;

    return FFDIFFSIGN(bs1->start_offset, bs2->start_offset);
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    BKHDStream *bst = opaque;
    AVFormatContext *s = bst->parent;
    AVIOContext *pb = s->pb;

    return avio_read(pb, buf, buf_size);
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    BKHDStream *bst = opaque;
    AVFormatContext *s = bst->parent;
    AVIOContext *pb = s->pb;

    return avio_seek(pb, offset + bst->start_offset, whence);
}

typedef uint32_t (*avio_r32)(AVIOContext *s);

static int read_header(AVFormatContext *s)
{
    int64_t first_start_offset, data_offset;
    int ret, nb_streams, version;
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32;
    uint32_t offset;

    avio_skip(pb, 4);
    offset = avio_rb32(pb);
    if (offset > av_bswap32(offset)) {
        avio_r32 = avio_rl32;
        offset = av_bswap32(offset);
    } else {
        avio_r32 = avio_rb32;
    }
    version = avio_r32(pb);
    if (version == 0 || version == 1) {
        avio_skip(pb, 4);
        version = avio_r32(pb);
    }

    if (version <= 26) {
        return AVERROR_INVALIDDATA;
    } else {
        uint32_t chunk, size;

        avio_skip(pb, offset + 8 - avio_tell(pb));

        chunk = avio_rb32(pb);
        size = avio_r32(pb);
        if (chunk != MKBETAG('D','I','D','X'))
            return AVERROR_INVALIDDATA;
        nb_streams = size / 12;
        if (nb_streams <= 0)
            return AVERROR_INVALIDDATA;

        for (int n = 0; n < nb_streams; n++) {
            BKHDStream *bst;
            AVStream *st;

            if (avio_feof(pb))
                return AVERROR_INVALIDDATA;

            bst = av_mallocz(sizeof(*bst));
            if (!bst)
                return AVERROR(ENOMEM);

            avio_skip(pb, 4);
            bst->start_offset = avio_r32(pb);
            bst->stop_offset = avio_r32(pb);
            bst->stop_offset += bst->start_offset;

            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);

            st->priv_data = bst;
            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        }

        chunk = avio_rb32(pb);
        avio_skip(pb, 4);
        if (chunk != MKBETAG('D','A','T','A'))
            return AVERROR_INVALIDDATA;

        data_offset = avio_tell(pb);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        BKHDStream *bst = st->priv_data;

        st->index = n;
        bst->start_offset += data_offset;
        bst->stop_offset += data_offset;
    }

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        BKHDStream *bst = st->priv_data;

        if (!(bst->xctx = avformat_alloc_context()))
            return AVERROR(ENOMEM);

        if ((ret = ff_copy_whiteblacklists(bst->xctx, s)) < 0) {
            avformat_free_context(bst->xctx);
            bst->xctx = NULL;

            return ret;
        }

        ffio_init_context(&bst->apb, NULL, 0, 0, bst,
                          read_data, NULL, seek_data);

        bst->xctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
        bst->xctx->probesize = 0;
        bst->xctx->max_analyze_duration = 0;
        bst->xctx->interrupt_callback = s->interrupt_callback;
        bst->xctx->pb = &bst->apb.pub;
        bst->xctx->io_open = NULL;
        bst->xctx->skip_initial_bytes = 0;
        bst->parent = s;

        avio_seek(pb, bst->start_offset, SEEK_SET);
        ret = avformat_open_input(&bst->xctx, "", NULL, NULL);
        if (ret < 0)
            return ret;

        st->id = bst->xctx->streams[0]->id;
        st->duration = bst->xctx->streams[0]->duration;
        st->time_base = bst->xctx->streams[0]->time_base;
        st->start_time = bst->xctx->streams[0]->start_time;
        st->pts_wrap_bits = bst->xctx->streams[0]->pts_wrap_bits;
        st->codecpar->level = bst->xctx->streams[0]->codecpar->level = 1;
        st->codecpar->codec_id = bst->xctx->streams[0]->codecpar->codec_id;
        st->codecpar->sample_rate = bst->xctx->streams[0]->codecpar->sample_rate;
        st->codecpar->block_align = bst->xctx->streams[0]->codecpar->block_align;
        ret = av_channel_layout_copy(&st->codecpar->ch_layout, &bst->xctx->streams[0]->codecpar->ch_layout);
        if (ret < 0)
            return ret;

        ret = ff_alloc_extradata(st->codecpar, bst->xctx->streams[0]->codecpar->extradata_size);
        if (ret < 0)
            return ret;
        memcpy(st->codecpar->extradata, bst->xctx->streams[0]->codecpar->extradata,
               bst->xctx->streams[0]->codecpar->extradata_size);

        ret = av_dict_copy(&st->metadata, bst->xctx->streams[0]->metadata, 0);
        if (ret < 0)
            return ret;

        ffstream(st)->request_probe = 0;
        ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;

        bst->data_offset = avio_tell(pb);
    }

    {
        AVStream *st = s->streams[0];
        BKHDStream *bst = st->priv_data;

        first_start_offset = bst->data_offset;
    }

    avio_seek(pb, first_start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    BKHDDemuxContext *g = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    BKHDStream *bst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (g->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[g->current_stream];
    bst = st->priv_data;
    if (do_seek)
        avio_seek(pb, bst->data_offset, SEEK_SET);

    if (avio_tell(pb) >= bst->stop_offset) {
        do_seek = 1;
        g->current_stream++;
        goto redo;
    }

    ret = av_read_frame(bst->xctx, pkt);
    pkt->stream_index = st->index;
    if (ret == AVERROR_EOF) {
        g->current_stream++;
        goto redo;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    BKHDDemuxContext *bkhd = s->priv_data;
    BKHDStream *bst;
    AVStream *st;

    bkhd->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[bkhd->current_stream];
    bst = st->priv_data;

    return av_seek_frame(bst->xctx, 0, ts, flags);
}

static int read_close(AVFormatContext *s)
{
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        BKHDStream *bst = st->priv_data;

        avformat_close_input(&bst->xctx);
    }

    return 0;
}

const FFInputFormat ff_bkhd_demuxer = {
    .p.name         = "bkhd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Wwise soundbank container BKHD"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "bnk",
    .priv_data_size = sizeof(BKHDDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
