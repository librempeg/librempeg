/*
 * SDT demuxer
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
#include "avformat_internal.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"

typedef struct SDTDemuxContext {
    int current_stream;
} SDTDemuxContext;

typedef struct SDTStream {
    int64_t start_offset;
    int64_t stop_offset;
    int64_t data_offset;
    int type;

    AVFormatContext *xctx;
    AVFormatContext *parent;
    FFIOContext apb;
} SDTStream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB16(p->buf+2) == 0)
        return 0;

    if (av_match_ext(p->filename, "sdt") == 0)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const SDTStream *as1 = s1->priv_data;
    const SDTStream *as2 = s2->priv_data;

    return FFDIFFSIGN(as1->start_offset, as2->start_offset);
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    SDTStream *ast = opaque;
    AVFormatContext *s = ast->parent;
    AVIOContext *pb = s->pb;

    return avio_read(pb, buf, buf_size);
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    SDTStream *ast = opaque;
    AVFormatContext *s = ast->parent;
    AVIOContext *pb = s->pb;

    return avio_seek(pb, offset + ast->start_offset, whence);
}

static int read_header(AVFormatContext *s)
{
    int64_t next_start_offset;
    AVIOContext *pb = s->pb;
    int ret, nb_streams;

    avio_skip(pb, 2);
    nb_streams = avio_rb16(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 4 * nb_streams);
    next_start_offset = avio_tell(pb) + 0x28 * nb_streams;

    for (int i = 0; i < nb_streams; i++) {
        char title[16] = { 0 };
        SDTStream *ast;
        AVStream *st;

        if (avio_rb32(pb) != 0x28)
            return AVERROR_INVALIDDATA;

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        ast = av_mallocz(sizeof(*ast));
        if (!ast)
            return AVERROR(ENOMEM);

        st->priv_data = ast;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;

        ast->start_offset = next_start_offset;
        ast->stop_offset = ast->start_offset;
        ast->stop_offset += avio_rb32(pb);
        next_start_offset = ast->stop_offset;

        if ((ret = avio_get_str(pb, 0x10, title, sizeof(title))) < 0)
            return ret;

        st->codecpar->sample_rate = avio_rb16(pb);
        ast->type = avio_rb16(pb);

        if (title[0] != '\0')
            av_dict_set(&st->metadata, "title", title, 0);

        avio_skip(pb, 12);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        SDTStream *ast = st->priv_data;
        int rate;

        if (!(ast->xctx = avformat_alloc_context()))
            return AVERROR(ENOMEM);

        if ((ret = ff_copy_whiteblacklists(ast->xctx, s)) < 0) {
            avformat_free_context(ast->xctx);
            ast->xctx = NULL;

            return ret;
        }

        ffio_init_context(&ast->apb, NULL, 0, 0, ast,
                          read_data, NULL, seek_data);

        ast->xctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
        ast->xctx->probesize = 0;
        ast->xctx->max_analyze_duration = 0;
        ast->xctx->interrupt_callback = s->interrupt_callback;
        ast->xctx->pb = &ast->apb.pub;
        ast->xctx->io_open = NULL;
        ast->xctx->skip_initial_bytes = 0;
        ast->parent = s;

        avio_seek(pb, ast->start_offset, SEEK_SET);
        const AVInputFormat *in_fmt = av_find_input_format("s16be");
        if ((ret = avformat_open_input(&ast->xctx, "", (ast->type == 0x1003) ? in_fmt : NULL, NULL)) < 0)
            return ret;

        st->id = ast->xctx->streams[0]->id;
        st->duration = ast->xctx->streams[0]->duration;
        st->time_base = ast->xctx->streams[0]->time_base;
        st->start_time = ast->xctx->streams[0]->start_time;
        st->pts_wrap_bits = ast->xctx->streams[0]->pts_wrap_bits;

        rate = st->codecpar->sample_rate;
        ret = avcodec_parameters_copy(st->codecpar, ast->xctx->streams[0]->codecpar);
        if (ret < 0)
            return ret;
        st->codecpar->sample_rate = rate;
        st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;

        ret = av_dict_copy(&st->metadata, ast->xctx->streams[0]->metadata, 0);
        if (ret < 0)
            return ret;

        ffstream(st)->request_probe = 0;
        ffstream(st)->need_parsing = ffstream(ast->xctx->streams[0])->need_parsing;

        ast->data_offset = avio_tell(pb);
    }

    {
        AVStream *st = s->streams[0];
        SDTStream *ast = st->priv_data;

        avio_seek(pb, ast->data_offset, SEEK_SET);
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SDTDemuxContext *a = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    SDTStream *ast;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (a->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[a->current_stream];
    ast = st->priv_data;
    if (do_seek)
        avio_seek(pb, ast->data_offset, SEEK_SET);

    if (avio_tell(pb) >= ast->stop_offset) {
        do_seek = 1;
        a->current_stream++;
        goto redo;
    }

    ret = av_read_frame(ast->xctx, pkt);
    pkt->stream_index = st->index;
    if (ret == AVERROR_EOF) {
        a->current_stream++;
        goto redo;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    SDTDemuxContext *sdt = s->priv_data;
    SDTStream *ast;
    AVStream *st;

    sdt->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[sdt->current_stream];
    ast = st->priv_data;

    return av_seek_frame(ast->xctx, 0, ts, flags);
}

static int read_close(AVFormatContext *s)
{
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        SDTStream *ast = st->priv_data;

        avformat_close_input(&ast->xctx);
    }

    return 0;
}

const FFInputFormat ff_sdt_demuxer = {
    .p.name         = "sdt",
    .p.long_name    = NULL_IF_CONFIG_SMALL("SDT"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "sdt",
    .priv_data_size = sizeof(SDTDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
