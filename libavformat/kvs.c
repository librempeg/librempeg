/*
 * KVS demuxer
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
#include "libavutil/mem.h"
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"

typedef struct KVSDemuxContext {
    AVFormatContext *ogg_ctx;
    FFIOContext ogg_pb;
} KVSDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('K','O','V','S'))
        return 0;

    if (AV_RB32(p->buf + 4) == 0)
        return 0;

    if (p->buf_size < 36)
        return 0;

    if (AV_RB32(p->buf+32) != MKBETAG('O','f','e','P'))
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    AVFormatContext *s = opaque;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb) - 32 - s->skip_initial_bytes;
    int ret;

    ret = avio_read(pb, buf, buf_size);

    if (pos >= 0 && pos < 256) {
        const int maxi = FFMIN(256, buf_size);

        for (int i = pos; i < maxi; i++)
            buf[i-pos] ^= i;
    }

    return ret;
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    AVFormatContext *s = opaque;
    AVIOContext *pb = s->pb;

    return avio_seek(pb, offset, whence);
}

static int read_header(AVFormatContext *s)
{
    KVSDemuxContext *n = s->priv_data;
    AVIOContext *pb = s->pb;
    FFStream *sti;
    AVStream *st;
    int ret;

    avio_skip(pb, 0x20);

    if (!(n->ogg_ctx = avformat_alloc_context()))
        return AVERROR(ENOMEM);

    if ((ret = ff_copy_whiteblacklists(n->ogg_ctx, s)) < 0) {
        avformat_free_context(n->ogg_ctx);
        n->ogg_ctx = NULL;

        return ret;
    }

    ffio_init_context(&n->ogg_pb, NULL, 0, 0, s,
                      read_data, NULL, seek_data);

    n->ogg_ctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
    n->ogg_ctx->ctx_flags |= AVFMTCTX_UNSEEKABLE;
    n->ogg_ctx->probesize = 0;
    n->ogg_ctx->max_analyze_duration = 0;
    n->ogg_ctx->interrupt_callback = s->interrupt_callback;
    n->ogg_ctx->pb = &n->ogg_pb.pub;
    n->ogg_ctx->io_open = NULL;

    ret = avformat_open_input(&n->ogg_ctx, "", NULL, NULL);
    if (ret < 0)
        return ret;

    ret = avformat_find_stream_info(n->ogg_ctx, NULL);
    if (ret < 0)
        return ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->id = n->ogg_ctx->streams[0]->id;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->duration = n->ogg_ctx->streams[0]->duration;
    st->time_base = n->ogg_ctx->streams[0]->time_base;
    st->start_time = n->ogg_ctx->streams[0]->start_time;
    st->pts_wrap_bits = n->ogg_ctx->streams[0]->pts_wrap_bits;

    ret = avcodec_parameters_copy(st->codecpar, n->ogg_ctx->streams[0]->codecpar);
    if (ret < 0)
        return ret;

    ret = av_dict_copy(&st->metadata, n->ogg_ctx->streams[0]->metadata, 0);
    if (ret < 0)
        return ret;

    sti = ffstream(st);
    sti->request_probe = 0;
    sti->need_parsing = ffstream(n->ogg_ctx->streams[0])->need_parsing;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    KVSDemuxContext *n = s->priv_data;
    int ret;

    ret = av_read_frame(n->ogg_ctx, pkt);
    pkt->stream_index = 0;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    KVSDemuxContext *n = s->priv_data;

    return av_seek_frame(n->ogg_ctx, 0, ts, flags);
}

static int read_close(AVFormatContext *s)
{
    KVSDemuxContext *n = s->priv_data;

    avformat_close_input(&n->ogg_ctx);

    return 0;
}

const FFInputFormat ff_kvs_demuxer = {
    .p.name         = "kvs",
    .p.long_name    = NULL_IF_CONFIG_SMALL("KVS Audio"),
    .p.extensions   = "kvs",
    .priv_data_size = sizeof(KVSDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
