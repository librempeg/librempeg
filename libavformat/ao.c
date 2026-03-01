/*
 * AO demuxer
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
#include "demux.h"
#include "internal.h"

typedef struct AODemuxContext {
    AVFormatContext *ogg_ctx;
} AODemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "ALPHAOGG", 8))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    extern const FFInputFormat ff_ogg_demuxer;
    AODemuxContext *ao = s->priv_data;
    AVIOContext *pb = s->pb;
    FFStream *sti;
    AVStream *st;
    int ret;

    avio_skip(pb, 0xc8);

    if (!(ao->ogg_ctx = avformat_alloc_context()))
        return AVERROR(ENOMEM);

    if ((ret = ff_copy_whiteblacklists(ao->ogg_ctx, s)) < 0) {
        avformat_free_context(ao->ogg_ctx);
        ao->ogg_ctx = NULL;

        return ret;
    }

    ao->ogg_ctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
    ao->ogg_ctx->ctx_flags |= AVFMTCTX_UNSEEKABLE;
    ao->ogg_ctx->probesize = 0;
    ao->ogg_ctx->max_analyze_duration = 0;
    ao->ogg_ctx->interrupt_callback = s->interrupt_callback;
    ao->ogg_ctx->pb = pb;
    ao->ogg_ctx->io_open = NULL;

    ret = avformat_open_input(&ao->ogg_ctx, "", &ff_ogg_demuxer.p, NULL);
    if (ret < 0)
        return ret;

    ret = avformat_find_stream_info(ao->ogg_ctx, NULL);
    if (ret < 0)
        return ret;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->id = ao->ogg_ctx->streams[0]->id;
    st->duration = ao->ogg_ctx->streams[0]->duration;
    st->time_base = ao->ogg_ctx->streams[0]->time_base;
    st->start_time = ao->ogg_ctx->streams[0]->start_time;
    st->pts_wrap_bits = ao->ogg_ctx->streams[0]->pts_wrap_bits;

    ret = avcodec_parameters_copy(st->codecpar, ao->ogg_ctx->streams[0]->codecpar);
    if (ret < 0)
        return ret;

    ret = av_dict_copy(&st->metadata, ao->ogg_ctx->streams[0]->metadata, 0);
    if (ret < 0)
        return ret;

    sti = ffstream(st);
    sti->request_probe = 0;
    sti->need_parsing = ffstream(ao->ogg_ctx->streams[0])->need_parsing;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AODemuxContext *ao = s->priv_data;
    int ret;

    ret = av_read_frame(ao->ogg_ctx, pkt);
    pkt->stream_index = 0;

    return ret;
}

static int read_seek(AVFormatContext *s, int index,
                     int64_t ts, int flags)
{
    AODemuxContext *ao = s->priv_data;

    return av_seek_frame(ao->ogg_ctx, 0, ts, flags);
}

static int read_close(AVFormatContext *s)
{
    AODemuxContext *ao = s->priv_data;

    avformat_close_input(&ao->ogg_ctx);

    return 0;
}

const FFInputFormat ff_ao_demuxer = {
    .p.name         = "ao",
    .p.long_name    = NULL_IF_CONFIG_SMALL("AlphaOgg"),
    .p.extensions   = "ao",
    .priv_data_size = sizeof(AODemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
