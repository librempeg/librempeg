/*
 * IKM demuxer
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

typedef struct IKMDemuxContext {
    AVFormatContext *ctx;
    AVFormatContext *parent;
    FFIOContext pb;
    int64_t offset;
} IKMDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('I','K','M','\0'))
        return 0;

    if (p->buf_size < 36)
        return 0;

    if (AV_RL32(p->buf + 32) != 0 &&
        AV_RL32(p->buf + 32) != 1 &&
        AV_RL32(p->buf + 32) != 3)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    IKMDemuxContext *n = opaque;
    AVFormatContext *s = n->parent;
    AVIOContext *pb = s->pb;

    return avio_read(pb, buf, buf_size);
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    IKMDemuxContext *n = opaque;
    AVFormatContext *s = n->parent;
    AVIOContext *pb = s->pb;

    return avio_seek(pb, offset + n->offset, whence);
}

static int read_header(AVFormatContext *s)
{
    IKMDemuxContext *n = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret, type;
    FFStream *sti;
    AVStream *st;

    avio_skip(pb, 32);
    type = avio_rl32(pb);
    switch (type) {
    case 0:
        n->offset = 0x800;
        break;
    case 1:
        n->offset = 0x30;
        break;
    case 3:
        n->offset = 0x40;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    if (type != 3) {
        if (!(n->ctx = avformat_alloc_context()))
            return AVERROR(ENOMEM);

        if ((ret = ff_copy_whiteblacklists(n->ctx, s)) < 0) {
            avformat_free_context(n->ctx);
            n->ctx = NULL;

            return ret;
        }

        ffio_init_context(&n->pb, NULL, 0, 0, n,
                          read_data, NULL, seek_data);

        n->ctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
        n->ctx->probesize = 0;
        n->ctx->max_analyze_duration = 0;
        n->ctx->interrupt_callback = s->interrupt_callback;
        n->ctx->pb = &n->pb.pub;
        n->ctx->io_open = NULL;
        n->ctx->skip_initial_bytes = 0;
        n->parent = s;

        avio_seek(pb, n->offset, SEEK_SET);
        ret = avformat_open_input(&n->ctx, "", NULL, NULL);
        if (ret < 0)
            return ret;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->id = n->ctx->streams[0]->id;
        st->duration = n->ctx->streams[0]->duration;
        st->time_base = n->ctx->streams[0]->time_base;
        st->start_time = n->ctx->streams[0]->start_time;
        st->pts_wrap_bits = n->ctx->streams[0]->pts_wrap_bits;

        ret = avcodec_parameters_copy(st->codecpar, n->ctx->streams[0]->codecpar);
        if (ret < 0)
            return ret;

        ret = av_dict_copy(&st->metadata, n->ctx->streams[0]->metadata, 0);
        if (ret < 0)
            return ret;

        sti = ffstream(st);
        sti->request_probe = 0;
        sti->need_parsing = ffstream(n->ctx->streams[0])->need_parsing;
    } else {
        int rate, channels;

        avio_seek(pb, n->offset, SEEK_SET);

        if (avio_rb32(pb) != MKBETAG('A','S','T','\0'))
            return AVERROR_INVALIDDATA;

        rate = avio_rl32(pb);
        avio_skip(pb, 8);
        channels = avio_rl32(pb);
        if (rate <= 0 || channels <= 0 || channels >= INT_MAX/16)
            return AVERROR_INVALIDDATA;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = 16 * channels;
        st->codecpar->bit_rate = 16LL * channels * 8 * rate / 28;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        avio_seek(pb, 0x800, SEEK_SET);
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    IKMDemuxContext *n = s->priv_data;
    int ret;

    if (n->ctx)
        ret = av_read_frame(n->ctx, pkt);
    else
        ret = ff_pcm_read_packet(s, pkt);
    pkt->stream_index = 0;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    IKMDemuxContext *n = s->priv_data;

    if (n->ctx)
        return av_seek_frame(n->ctx, 0, ts, flags);
    return -1;
}

static int read_close(AVFormatContext *s)
{
    IKMDemuxContext *n = s->priv_data;

    avformat_close_input(&n->ctx);

    return 0;
}

const FFInputFormat ff_ikm_demuxer = {
    .p.name         = "ikm",
    .p.long_name    = NULL_IF_CONFIG_SMALL("MiCROViSiON IKM"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "ikm",
    .priv_data_size = sizeof(IKMDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
