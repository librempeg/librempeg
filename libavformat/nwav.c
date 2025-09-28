/*
 * NWAV demuxer
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
#include "demux.h"
#include "internal.h"

typedef struct NWAVDemuxContext {
    AVFormatContext *ogg_ctx;
} NWAVDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('N','W','A','V'))
        return 0;

    if (p->buf_size < 32)
        return 0;

    if (AV_RL16(p->buf + 0x1a) <= 0x24)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_header(AVFormatContext *s)
{
    extern const FFInputFormat ff_ogg_demuxer;
    NWAVDemuxContext *n = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret, offset;
    FFStream *sti;
    AVStream *st;

    avio_skip(pb, 0x1a);
    offset = avio_rl16(pb);
    avio_seek(pb, offset, SEEK_SET);

    if (!(n->ogg_ctx = avformat_alloc_context()))
        return AVERROR(ENOMEM);

    if ((ret = ff_copy_whiteblacklists(n->ogg_ctx, s)) < 0) {
        avformat_free_context(n->ogg_ctx);
        n->ogg_ctx = NULL;

        return ret;
    }

    n->ogg_ctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
    n->ogg_ctx->ctx_flags |= AVFMTCTX_UNSEEKABLE;
    n->ogg_ctx->probesize = 0;
    n->ogg_ctx->max_analyze_duration = 0;
    n->ogg_ctx->interrupt_callback = s->interrupt_callback;
    n->ogg_ctx->pb = pb;
    n->ogg_ctx->io_open = NULL;

    ret = avformat_open_input(&n->ogg_ctx, "", &ff_ogg_demuxer.p, NULL);
    if (ret < 0)
        return ret;

    ret = avformat_find_stream_info(n->ogg_ctx, NULL);
    if (ret < 0)
        return ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->id = n->ogg_ctx->streams[0]->id;
    st->start_time = n->ogg_ctx->streams[0]->start_time;
    st->duration = n->ogg_ctx->streams[0]->duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = n->ogg_ctx->streams[0]->codecpar->codec_id;
    st->codecpar->sample_rate = n->ogg_ctx->streams[0]->codecpar->sample_rate;
    ret = av_channel_layout_copy(&st->codecpar->ch_layout, &n->ogg_ctx->streams[0]->codecpar->ch_layout);
    if (ret < 0)
        return ret;

    ret = ff_alloc_extradata(st->codecpar, n->ogg_ctx->streams[0]->codecpar->extradata_size);
    if (ret < 0)
        return ret;
    memcpy(st->codecpar->extradata, n->ogg_ctx->streams[0]->codecpar->extradata,
           n->ogg_ctx->streams[0]->codecpar->extradata_size);

    sti = ffstream(st);
    sti->request_probe = 0;
    sti->need_parsing = AVSTREAM_PARSE_HEADERS;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    NWAVDemuxContext *n = s->priv_data;
    int ret;

    ret = av_read_frame(n->ogg_ctx, pkt);
    pkt->stream_index = 0;

    return ret;
}

static int read_close(AVFormatContext *s)
{
    NWAVDemuxContext *n = s->priv_data;

    avformat_close_input(&n->ogg_ctx);

    return 0;
}

const FFInputFormat ff_nwav_demuxer = {
    .p.name         = "nwav",
    .p.long_name    = NULL_IF_CONFIG_SMALL("NWAV Audio"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "nwav",
    .priv_data_size = sizeof(NWAVDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_close     = read_close,
};
