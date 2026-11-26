/*
 * WMSF demuxer
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

typedef struct WMSFDemuxContext {
    AVFormatContext *ctx;
    AVFormatContext *parent;
    FFIOContext pb;
    int64_t offset;
} WMSFDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 20)
        return 0;

    if (AV_RB32(p->buf) != MKBETAG('W','M','S','F'))
        return 0;
    if ((AV_RB32(p->buf+16) & 0xFFFFFF00) != MKBETAG('M','S','F','\0'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    WMSFDemuxContext *w = opaque;
    AVFormatContext *s = w->parent;
    AVIOContext *pb = s->pb;

    return avio_read(pb, buf, buf_size);
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    WMSFDemuxContext *w = opaque;
    AVFormatContext *s = w->parent;
    AVIOContext *pb = s->pb;

    return avio_seek(pb, offset + w->offset, whence);
}

static int read_header(AVFormatContext *s)
{
    WMSFDemuxContext *w = s->priv_data;
    AVIOContext *pb = s->pb;
    FFStream *sti;
    AVStream *st;
    int ret;

    w->offset = 16;
    if (!(w->ctx = avformat_alloc_context()))
        return AVERROR(ENOMEM);

    if ((ret = ff_copy_whiteblacklists(w->ctx, s)) < 0) {
        avformat_free_context(w->ctx);
        w->ctx = NULL;

        return ret;
    }

    ffio_init_context(&w->pb, NULL, 0, 0, w,
                      read_data, NULL, seek_data);

    w->ctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
    w->ctx->probesize = 0;
    w->ctx->max_analyze_duration = 0;
    w->ctx->interrupt_callback = s->interrupt_callback;
    w->ctx->pb = &w->pb.pub;
    w->ctx->io_open = NULL;
    w->ctx->skip_initial_bytes = 0;
    w->parent = s;

    avio_seek(pb, w->offset, SEEK_SET);
    ret = avformat_open_input(&w->ctx, "", NULL, NULL);
    if (ret < 0)
        return ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->id = w->ctx->streams[0]->id;
    st->duration = w->ctx->streams[0]->duration;
    st->time_base = w->ctx->streams[0]->time_base;
    st->start_time = w->ctx->streams[0]->start_time;
    st->pts_wrap_bits = w->ctx->streams[0]->pts_wrap_bits;

    ret = avcodec_parameters_copy(st->codecpar, w->ctx->streams[0]->codecpar);
    if (ret < 0)
        return ret;

    ret = av_dict_copy(&st->metadata, w->ctx->streams[0]->metadata, 0);
    if (ret < 0)
        return ret;

    sti = ffstream(st);
    sti->request_probe = 0;
    sti->need_parsing = ffstream(w->ctx->streams[0])->need_parsing;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    WMSFDemuxContext *w = s->priv_data;
    int ret;

    ret = av_read_frame(w->ctx, pkt);
    pkt->stream_index = 0;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    WMSFDemuxContext *w = s->priv_data;

    return av_seek_frame(w->ctx, 0, ts, flags);
}

static int read_close(AVFormatContext *s)
{
    WMSFDemuxContext *w = s->priv_data;

    avformat_close_input(&w->ctx);

    return 0;
}

const FFInputFormat ff_wmsf_demuxer = {
    .p.name         = "wmsf",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Banpresto MSF"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "msf",
    .priv_data_size = sizeof(WMSFDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
