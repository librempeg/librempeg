/*
 * MOGG demuxer
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

typedef struct MOGGDemuxContext {
    AVFormatContext *ogg_ctx;
} MOGGDemuxContext;

static int read_probe(const AVProbeData *p)
{
    int64_t offset;

    if (AV_RL32(p->buf) != 0x0a)
        return 0;

    offset = AV_RL32(p->buf + 4);
    if (offset <= 8)
        return 0;

    if (p->buf_size < offset + 4)
        return 0;

    if (memcmp(p->buf + offset, "OggS", 4))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    extern const FFInputFormat ff_ogg_demuxer;
    MOGGDemuxContext *m = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t offset;
    FFStream *sti;
    AVStream *st;
    int ret;

    avio_skip(pb, 4);
    offset = avio_rl32(pb);
    avio_seek(pb, offset, SEEK_SET);

    if (!(m->ogg_ctx = avformat_alloc_context()))
        return AVERROR(ENOMEM);

    if ((ret = ff_copy_whiteblacklists(m->ogg_ctx, s)) < 0) {
        avformat_free_context(m->ogg_ctx);
        m->ogg_ctx = NULL;

        return ret;
    }

    m->ogg_ctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
    m->ogg_ctx->ctx_flags |= AVFMTCTX_UNSEEKABLE;
    m->ogg_ctx->probesize = 0;
    m->ogg_ctx->max_analyze_duration = 0;
    m->ogg_ctx->interrupt_callback = s->interrupt_callback;
    m->ogg_ctx->pb = pb;
    m->ogg_ctx->io_open = NULL;

    ret = avformat_open_input(&m->ogg_ctx, "", &ff_ogg_demuxer.p, NULL);
    if (ret < 0)
        return ret;

    ret = avformat_find_stream_info(m->ogg_ctx, NULL);
    if (ret < 0)
        return ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->id = m->ogg_ctx->streams[0]->id;
    st->duration = m->ogg_ctx->streams[0]->duration;
    st->time_base = m->ogg_ctx->streams[0]->time_base;
    st->start_time = m->ogg_ctx->streams[0]->start_time;
    st->pts_wrap_bits = m->ogg_ctx->streams[0]->pts_wrap_bits;

    ret = avcodec_parameters_copy(st->codecpar, m->ogg_ctx->streams[0]->codecpar);
    if (ret < 0)
        return ret;

    ret = av_dict_copy(&st->metadata, m->ogg_ctx->streams[0]->metadata, 0);
    if (ret < 0)
        return ret;

    sti = ffstream(st);
    sti->request_probe = 0;
    sti->need_parsing = ffstream(m->ogg_ctx->streams[0])->need_parsing;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    MOGGDemuxContext *m = s->priv_data;
    int ret;

    ret = av_read_frame(m->ogg_ctx, pkt);
    pkt->stream_index = 0;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    MOGGDemuxContext *m = s->priv_data;

    return av_seek_frame(m->ogg_ctx, 0, ts, flags);
}

static int read_close(AVFormatContext *s)
{
    MOGGDemuxContext *m = s->priv_data;

    avformat_close_input(&m->ogg_ctx);

    return 0;
}

const FFInputFormat ff_mogg_demuxer = {
    .p.name         = "mogg",
    .p.long_name    = NULL_IF_CONFIG_SMALL("MOGG Audio"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "mogg",
    .priv_data_size = sizeof(MOGGDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
