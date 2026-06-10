/*
 * Her Interactive Games HIS0 demuxer
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
#include "demux.h"
#include "internal.h"
#include "pcm.h"

typedef struct HIS0DemuxContext {
    AVFormatContext *ogg_ctx;
} HIS0DemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('H','I','S','\x0'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, version, codec, rate, channels, bps;
    extern const FFInputFormat ff_ogg_demuxer;
    HIS0DemuxContext *his = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t start_offset;
    AVStream *st;

    avio_skip(pb, 4);
    version = avio_rl32(pb);
    avio_skip(pb, 2);
    channels = avio_rl16(pb);
    rate = avio_rl32(pb);
    avio_skip(pb, 6);
    bps = avio_rl16(pb);
    if (rate <= 0 || channels <= 0 || channels >= INT_MAX/((bps+7)/8))
        return AVERROR_INVALIDDATA;

    if (version >= 2) {
        avio_skip(pb, 4);
        codec = avio_rl16(pb);
        start_offset = 0x20;
    } else {
        if (version == 1)
            start_offset = 0x1c;
        codec = 1;
    }

    if (codec == 1) {
        switch (bps) {
            case 8:
                codec = AV_CODEC_ID_PCM_U8;
                break;
            case 16:
                codec = AV_CODEC_ID_PCM_S16LE;
                break;
            default:
                avpriv_request_sample(s, "bps %d", bps);
                return AVERROR_PATCHWELCOME;
        }
        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = channels * (bps+7) / 8;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    } else if (codec == 2) {
        FFStream *sti;

        codec = AV_CODEC_ID_VORBIS;
        if (!(his->ogg_ctx = avformat_alloc_context()))
            return AVERROR(ENOMEM);

        if ((ret = ff_copy_whiteblacklists(his->ogg_ctx, s)) < 0) {
            avformat_free_context(his->ogg_ctx);
            his->ogg_ctx = NULL;

            return ret;
        }

        his->ogg_ctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
        his->ogg_ctx->ctx_flags |= AVFMTCTX_UNSEEKABLE;
        his->ogg_ctx->probesize = 0;
        his->ogg_ctx->max_analyze_duration = 0;
        his->ogg_ctx->interrupt_callback = s->interrupt_callback;
        his->ogg_ctx->pb = pb;
        his->ogg_ctx->io_open = NULL;

        ret = avformat_open_input(&his->ogg_ctx, "", &ff_ogg_demuxer.p, NULL);
        if (ret < 0)
            return ret;

        ret = avformat_find_stream_info(his->ogg_ctx, NULL);
        if (ret < 0)
            return ret;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->id = his->ogg_ctx->streams[0]->id;
        st->duration = his->ogg_ctx->streams[0]->duration;
        st->time_base = his->ogg_ctx->streams[0]->time_base;
        st->start_time = his->ogg_ctx->streams[0]->start_time;
        st->pts_wrap_bits = his->ogg_ctx->streams[0]->pts_wrap_bits;

        ret = avcodec_parameters_copy(st->codecpar, his->ogg_ctx->streams[0]->codecpar);
        if (ret < 0)
            return ret;

        ret = av_dict_copy(&st->metadata, his->ogg_ctx->streams[0]->metadata, 0);
        if (ret < 0)
            return ret;

        sti = ffstream(st);
        sti->request_probe = 0;
        sti->need_parsing = ffstream(his->ogg_ctx->streams[0])->need_parsing;
    } else {
        avpriv_request_sample(s, "codec %d", codec);
        return AVERROR_PATCHWELCOME;
    }

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    HIS0DemuxContext *his = s->priv_data;
    int ret;

    if (his->ogg_ctx) {
        ret = av_read_frame(his->ogg_ctx, pkt);
        pkt->stream_index = 0;
    } else {
        ret = ff_pcm_read_packet(s, pkt);
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int index,
                     int64_t ts, int flags)
{
    HIS0DemuxContext *his = s->priv_data;

    if (his->ogg_ctx)
        return av_seek_frame(his->ogg_ctx, 0, ts, flags);
    else
        return -1;
}

static int read_close(AVFormatContext *s)
{
    HIS0DemuxContext *his = s->priv_data;

    avformat_close_input(&his->ogg_ctx);

    return 0;
}

const FFInputFormat ff_his0_demuxer = {
    .p.name         = "his0",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Her Interactive Sound 0"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "his",
    .priv_data_size = sizeof(HIS0DemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
