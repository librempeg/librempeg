/*
 * PPST demuxer
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

typedef struct PPSTDemuxContext {
    int stream_index;
    int interleave_size;
} PPSTDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('P','P','S','T'))
        return 0;

    if (p->buf_size < 0xA8)
        return 0;
    if (AV_RL32(p->buf+0xA4) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

typedef struct PPSTStream {
    AVFormatContext *ctx;
    AVFormatContext *parent;
    FFIOContext pb;
    int stream_index;
    int64_t start_offset;
    int64_t stream_size;
    int64_t stream_pos;
} PPSTStream;

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    PPSTStream *p = opaque;
    AVFormatContext *s = p->parent;
    PPSTDemuxContext *ppst = s->priv_data;
    const int interleave_size = ppst->interleave_size;
    AVIOContext *pb = s->pb;
    const int index = p->stream_index;
    int64_t stream_pos = p->stream_pos;
    int ret, offset = 0;
    int64_t new_pos;

    if (stream_pos >= p->stream_size)
        return AVERROR_EOF;

    new_pos = (stream_pos % 0x4000) + (stream_pos / 0x4000) * interleave_size + 0x4000 * index + 0x800LL;
    avio_seek(pb, new_pos, SEEK_SET);

    while (offset < buf_size) {
        int to_read = FFMIN(buf_size - offset, 0x4000 - (p->stream_pos % 0x4000));

        ret = avio_read(pb, buf + offset, to_read);
        if (ret <= 0)
            break;

        offset += ret;
        p->stream_pos += ret;

        if ((p->stream_pos % 0x4000) == 0)
            avio_skip(pb, interleave_size - 0x4000);
    }


    return offset;
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    PPSTStream *p = opaque;
    AVFormatContext *s = p->parent;
    PPSTDemuxContext *ppst = s->priv_data;
    const int interleave_size = ppst->interleave_size;
    AVIOContext *pb = s->pb;
    const int index = p->stream_index;
    int64_t new_offset;

    if (whence & AVSEEK_SIZE)
        return p->stream_size;
    p->stream_pos = offset;

    new_offset = (offset % 0x4000) + (offset / 0x4000) * interleave_size + 0x4000 * index + 0x800LL;

    avio_seek(pb, new_offset, whence);

    return offset;
}

static int read_header(AVFormatContext *s)
{
    PPSTDemuxContext *ppst = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret;

    avio_skip(pb, 0xa0);

    while (!avio_feof(pb)) {
        int64_t stream_size;
        PPSTStream *pst;
        FFStream *sti;
        AVStream *st;
        int index;

        avio_seek(pb, 0xA0 + 8 * s->nb_streams, SEEK_SET);
        avio_skip(pb, 4);
        stream_size = avio_rl32(pb);
        if (!stream_size)
            break;
        index = s->nb_streams;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        pst = av_mallocz(sizeof(*pst));
        if (!pst)
            return AVERROR(ENOMEM);

        st->priv_data = pst;
        pst->start_offset = 0x800 + 0x4000 * index;
        pst->stream_size = stream_size;
        pst->stream_index = index;

        if (!(pst->ctx = avformat_alloc_context()))
            return AVERROR(ENOMEM);

        if ((ret = ff_copy_whiteblacklists(pst->ctx, s)) < 0) {
            avformat_free_context(pst->ctx);
            pst->ctx = NULL;

            return ret;
        }

        ffio_init_context(&pst->pb, NULL, 0, 0, pst,
                          read_data, NULL, seek_data);

        pst->ctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
        pst->ctx->probesize = 0;
        pst->ctx->max_analyze_duration = 0;
        pst->ctx->interrupt_callback = s->interrupt_callback;
        pst->ctx->pb = &pst->pb.pub;
        pst->ctx->io_open = NULL;
        pst->ctx->skip_initial_bytes = 0;
        pst->parent = s;

        avio_seek(pb, pst->start_offset, SEEK_SET);
        ret = avformat_open_input(&pst->ctx, "", NULL, NULL);
        if (ret < 0)
            return ret;

        st->id = pst->ctx->streams[0]->id;
        st->duration = pst->ctx->streams[0]->duration;
        st->time_base = pst->ctx->streams[0]->time_base;
        st->start_time = pst->ctx->streams[0]->start_time;
        st->pts_wrap_bits = pst->ctx->streams[0]->pts_wrap_bits;

        ret = avcodec_parameters_copy(st->codecpar, pst->ctx->streams[0]->codecpar);
        if (ret < 0)
            return ret;

        ret = av_dict_copy(&st->metadata, pst->ctx->streams[0]->metadata, 0);
        if (ret < 0)
            return ret;

        sti = ffstream(st);
        sti->request_probe = 0;
        sti->need_parsing = ffstream(pst->ctx->streams[0])->need_parsing;
    }

    if (s->nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    ppst->interleave_size = 0x4000 * s->nb_streams;

    avio_seek(pb, 0x800, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    PPSTDemuxContext *ppst = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st = s->streams[ppst->stream_index];
    PPSTStream *pst = st->priv_data;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    ret = av_read_frame(pst->ctx, pkt);
    pkt->stream_index = pst->stream_index;
    ppst->stream_index++;
    if (ppst->stream_index >= s->nb_streams)
        ppst->stream_index = 0;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    PPSTDemuxContext *ppst = s->priv_data;
    PPSTStream *pst;
    AVStream *st;

    ppst->stream_index = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[ppst->stream_index];
    pst = st->priv_data;

    return av_seek_frame(pst->ctx, 0, ts, flags);
}

static int read_close(AVFormatContext *s)
{
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        PPSTStream *ppst = st->priv_data;

        avformat_close_input(&ppst->ctx);
    }

    return 0;
}

const FFInputFormat ff_ppst_demuxer = {
    .p.name         = "ppst",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Epics PPST"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "sng",
    .priv_data_size = sizeof(PPSTDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
