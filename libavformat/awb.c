/*
 * AWB demuxer
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

typedef struct AWBDemuxContext {
    int current_stream;
} AWBDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('A','F','S','2'))
        return 0;
    if (p->buf[5] != 2 && p->buf[5] != 4)
        return 0;
    if ((int)AV_RL32(p->buf + 8) <= 0)
        return 0;
    if (AV_RL16(p->buf + 12) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

typedef struct AWBStream {
    int64_t start_offset;
    int64_t stop_offset;
    int64_t data_offset;

    AVFormatContext *xctx;
    AVFormatContext *parent;
    FFIOContext apb;
} AWBStream;

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const AWBStream *bs1 = s1->priv_data;
    const AWBStream *bs2 = s2->priv_data;

    return FFDIFFSIGN(bs1->start_offset, bs2->start_offset);
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    AWBStream *ast = opaque;
    AVFormatContext *s = ast->parent;
    AVIOContext *pb = s->pb;
    int ret;

    ret = avio_read(pb, buf, buf_size);
    return ret;
}

static int read_header(AVFormatContext *s)
{
    int offset_size, ret, waveid_alignment, nb_streams, offset_alignment;
    int64_t first_start_offset, next_offset;
    AVIOContext *pb = s->pb;

    avio_skip(pb, 5);
    offset_size = avio_r8(pb);
    waveid_alignment = avio_rl16(pb);
    nb_streams = avio_rl32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    offset_alignment = avio_rl16(pb);
    /*subkey = */avio_rl16(pb);

    avio_skip(pb, nb_streams * waveid_alignment);

    switch (offset_size) {
    case 4:
        next_offset = avio_rl32(pb);
        break;
    case 2:
        next_offset = avio_rl16(pb);
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    next_offset += (next_offset % offset_alignment) ?
        offset_alignment - (next_offset % offset_alignment) : 0;

    for (int n = 0; n < nb_streams; n++) {
        int64_t stream_offset = next_offset;
        AWBStream *ast;
        AVStream *st;

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        ast = av_mallocz(sizeof(*ast));
        if (!ast)
            return AVERROR(ENOMEM);

        switch (offset_size) {
        case 4:
            next_offset = avio_rl32(pb);
            break;
        case 2:
            next_offset = avio_rl16(pb);
            break;
        default:
            return AVERROR_INVALIDDATA;
        }

        next_offset += (next_offset % offset_alignment) ?
            offset_alignment - (next_offset % offset_alignment) : 0;
        ast->start_offset = stream_offset;
        ast->stop_offset = next_offset;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->priv_data = ast;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        AWBStream *ast = st->priv_data;

        if (!(ast->xctx = avformat_alloc_context()))
            return AVERROR(ENOMEM);

        if ((ret = ff_copy_whiteblacklists(ast->xctx, s)) < 0) {
            avformat_free_context(ast->xctx);
            ast->xctx = NULL;

            return ret;
        }

        ffio_init_context(&ast->apb, NULL, 0, 0, ast,
                          read_data, NULL, NULL);

        ast->xctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
        ast->xctx->ctx_flags |= AVFMTCTX_UNSEEKABLE;
        ast->xctx->probesize = 0;
        ast->xctx->max_analyze_duration = 0;
        ast->xctx->interrupt_callback = s->interrupt_callback;
        ast->xctx->pb = &ast->apb.pub;
        ast->xctx->io_open = NULL;
        ast->xctx->skip_initial_bytes = 0;
        ast->parent = s;

        avio_seek(pb, ast->start_offset, SEEK_SET);
        ret = avformat_open_input(&ast->xctx, "", NULL, NULL);
        if (ret < 0)
            return ret;

        st->id = ast->xctx->streams[0]->id;
        st->duration = ast->xctx->streams[0]->duration;
        st->start_time = ast->xctx->streams[0]->start_time;
        st->codecpar->codec_id = ast->xctx->streams[0]->codecpar->codec_id;
        st->codecpar->sample_rate = ast->xctx->streams[0]->codecpar->sample_rate;
        st->codecpar->block_align = ast->xctx->streams[0]->codecpar->block_align;
        ret = av_channel_layout_copy(&st->codecpar->ch_layout, &ast->xctx->streams[0]->codecpar->ch_layout);
        if (ret < 0)
            return ret;

        ret = ff_alloc_extradata(st->codecpar, ast->xctx->streams[0]->codecpar->extradata_size);
        if (ret < 0)
            return ret;
        memcpy(st->codecpar->extradata, ast->xctx->streams[0]->codecpar->extradata,
               ast->xctx->streams[0]->codecpar->extradata_size);

        ret = av_dict_copy(&st->metadata, ast->xctx->streams[0]->metadata, 0);
        if (ret < 0)
            return ret;

        ffstream(st)->request_probe = 0;
        ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        ast->data_offset = avio_tell(pb);
    }

    {
        AVStream *st = s->streams[0];
        AWBStream *ast = st->priv_data;

        first_start_offset = ast->data_offset;
    }

    avio_seek(pb, first_start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AWBDemuxContext *g = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    AWBStream *ast;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (g->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[g->current_stream];
    ast = st->priv_data;
    if (do_seek)
        avio_seek(pb, ast->data_offset, SEEK_SET);

    if (avio_tell(pb) >= ast->stop_offset) {
        do_seek = 1;
        g->current_stream++;
        goto redo;
    }

    ret = av_read_frame(ast->xctx, pkt);
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
    AWBDemuxContext *awb = s->priv_data;
    AWBStream *ast;
    AVStream *st;

    awb->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[awb->current_stream];
    ast = st->priv_data;

    return av_seek_frame(ast->xctx, 0, ts, flags);
}

static int read_close(AVFormatContext *s)
{
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        AWBStream *ast = st->priv_data;

        avformat_close_input(&ast->xctx);
    }

    return 0;
}

const FFInputFormat ff_awb_demuxer = {
    .p.name         = "awb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("AWB (Atom Wave Bank)"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "awb",
    .priv_data_size = sizeof(AWBDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
