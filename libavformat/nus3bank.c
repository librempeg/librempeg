/*
 * NUS3BANK demuxer
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

typedef struct NUS3BankDemuxContext {
    int current_stream;
} NUS3BankDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('N','U','S','3'))
        return 0;
    if (p->buf_size < 32)
        return 0;
    if (AV_RB32(p->buf+8) != MKBETAG('B','A','N','K'))
        return 0;
    if (AV_RB32(p->buf+12) != MKBETAG('T','O','C',' '))
        return 0;
    if (AV_RL32(p->buf+20) <= 1)
        return 0;

    return AVPROBE_SCORE_MAX;
}

typedef struct TNUS3Stream {
    int64_t tone_header_offset;
} TNUS3Stream;

typedef struct NUS3Stream {
    int64_t name_offset;
    int64_t start_offset;
    int64_t stop_offset;
    int64_t data_offset;

    AVFormatContext *xctx;
    AVFormatContext *parent;
    FFIOContext apb;
} NUS3Stream;

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const NUS3Stream *ns1 = s1->priv_data;
    const NUS3Stream *ns2 = s2->priv_data;

    return FFDIFFSIGN(ns1->start_offset, ns2->start_offset);
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    NUS3Stream *nst = opaque;
    AVFormatContext *s = nst->parent;
    AVIOContext *pb = s->pb;
    int ret;

    ret = avio_read(pb, buf, buf_size);
    return ret;
}

static int read_header(AVFormatContext *s)
{
    int64_t first_start_offset, offset, tone_offset = 0, pack_offset = 0;
    int ret, chunk_count, entries;
    AVIOContext *pb = s->pb;
    TNUS3Stream *tmp;

    avio_skip(pb, 16);
    offset = avio_rl32(pb);
    offset += 0x14;
    chunk_count = avio_rl32(pb);
    if (chunk_count <= 1)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < chunk_count; n++) {
        int64_t chunk_size;
        uint32_t chunk_id;

        chunk_id = avio_rb32(pb);
        chunk_size = avio_rl32(pb);

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        switch (chunk_id) {
        case MKBETAG('T','O','N','E'):
            tone_offset = 0x08 + offset;
            break;
        case MKBETAG('P','A','C','K'):
            pack_offset = 0x08 + offset;
            break;
        default:
            break;
        }

        offset += 0x08 + chunk_size;
    }

    if (tone_offset == 0 || pack_offset == 0)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, tone_offset, SEEK_SET);
    entries = avio_rl32(pb);
    if (entries <= 0)
        return AVERROR_INVALIDDATA;
    tmp = av_calloc(entries, sizeof(*tmp));
    if (!tmp)
        return AVERROR(ENOMEM);

    for (int n = 0; n < entries; n++) {
        TNUS3Stream *tnst = &tmp[n];
        int64_t tone_header_offset;
        int tone_header_size;

        tone_header_offset = avio_rl32(pb);
        tone_header_size = avio_rl32(pb);
        if (tone_header_size <= 12)
            continue;

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        tnst->tone_header_offset = tone_header_offset;
    }

    for (int n = 0; n < entries; n++) {
        TNUS3Stream *tnst = &tmp[n];
        int64_t stream_name_offset;
        int stream_name_size;
        NUS3Stream *nst;
        char title[512];
        int64_t offset;
        int flags, ret;
        AVStream *st;

        if (tnst->tone_header_offset <= 0)
            continue;

        offset = tone_offset + tnst->tone_header_offset;
        avio_seek(pb, offset + 7, SEEK_SET);
        flags = avio_r8(pb);
        offset += 8;

        if (flags & 0x80) {
            offset += 4;
            avio_skip(pb, 4);
        }
        stream_name_size = avio_r8(pb) + 1;
        stream_name_offset = offset + 1;
        stream_name_size = FFALIGN(stream_name_size, 4);
        offset += stream_name_size;

        avio_seek(pb, offset + 4, SEEK_SET);
        if (avio_rl32(pb) != 8)
            continue;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        nst = av_mallocz(sizeof(*nst));
        if (!nst)
            return AVERROR(ENOMEM);

        st->priv_data = nst;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;

        nst->name_offset = stream_name_offset;
        nst->start_offset = avio_rl32(pb) + pack_offset;
        nst->stop_offset = nst->start_offset;
        nst->stop_offset += avio_rl32(pb);

        avio_seek(pb, stream_name_offset, SEEK_SET);
        if ((ret = avio_get_str(pb, stream_name_size-1, title, sizeof(title))) < 0)
            return ret;
        if (title[0] != '\0')
            av_dict_set(&st->metadata, "title", title, 0);
    }

    av_freep(&tmp);
    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        NUS3Stream *nst = st->priv_data;

        if (!(nst->xctx = avformat_alloc_context()))
            return AVERROR(ENOMEM);

        if ((ret = ff_copy_whiteblacklists(nst->xctx, s)) < 0) {
            avformat_free_context(nst->xctx);
            nst->xctx = NULL;

            return ret;
        }

        ffio_init_context(&nst->apb, NULL, 0, 0, nst,
                          read_data, NULL, NULL);

        nst->xctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
        nst->xctx->ctx_flags |= AVFMTCTX_UNSEEKABLE;
        nst->xctx->probesize = 0;
        nst->xctx->max_analyze_duration = 0;
        nst->xctx->interrupt_callback = s->interrupt_callback;
        nst->xctx->pb = &nst->apb.pub;
        nst->xctx->io_open = NULL;
        nst->xctx->skip_initial_bytes = 0;
        nst->parent = s;

        avio_seek(pb, nst->start_offset, SEEK_SET);
        ret = avformat_open_input(&nst->xctx, "", NULL, NULL);
        if (ret < 0)
            return ret;

        st->id = nst->xctx->streams[0]->id;
        st->duration = nst->xctx->streams[0]->duration;
        st->start_time = nst->xctx->streams[0]->start_time;
        st->codecpar->codec_id = nst->xctx->streams[0]->codecpar->codec_id;
        st->codecpar->sample_rate = nst->xctx->streams[0]->codecpar->sample_rate;
        st->codecpar->block_align = nst->xctx->streams[0]->codecpar->block_align;
        ret = av_channel_layout_copy(&st->codecpar->ch_layout, &nst->xctx->streams[0]->codecpar->ch_layout);
        if (ret < 0)
            return ret;

        ret = ff_alloc_extradata(st->codecpar, nst->xctx->streams[0]->codecpar->extradata_size);
        if (ret < 0)
            return ret;
        memcpy(st->codecpar->extradata, nst->xctx->streams[0]->codecpar->extradata,
               nst->xctx->streams[0]->codecpar->extradata_size);

        ret = av_dict_copy(&st->metadata, nst->xctx->streams[0]->metadata, 0);
        if (ret < 0)
            return ret;

        ffstream(st)->request_probe = 0;
        ffstream(st)->need_parsing = AVSTREAM_PARSE_HEADERS;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        nst->data_offset = avio_tell(pb);
    }

    {
        AVStream *st = s->streams[0];
        NUS3Stream *nst = st->priv_data;

        first_start_offset = nst->data_offset;
    }

    avio_seek(pb, first_start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    NUS3BankDemuxContext *g = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    NUS3Stream *nst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (g->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[g->current_stream];
    nst = st->priv_data;
    if (do_seek)
        avio_seek(pb, nst->data_offset, SEEK_SET);

    if (avio_tell(pb) >= nst->stop_offset) {
        do_seek = 1;
        g->current_stream++;
        goto redo;
    }

    ret = av_read_frame(nst->xctx, pkt);
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
    NUS3BankDemuxContext *n = s->priv_data;
    NUS3Stream *nst;
    AVStream *st;

    n->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[n->current_stream];
    nst = st->priv_data;

    return av_seek_frame(nst->xctx, 0, ts, flags);
}

static int read_close(AVFormatContext *s)
{
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        NUS3Stream *nst = st->priv_data;

        avformat_close_input(&nst->xctx);
    }

    return 0;
}

const FFInputFormat ff_nus3bank_demuxer = {
    .p.name         = "nus3bank",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Namco NUS3 Bank"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "nus3bank,nub2",
    .priv_data_size = sizeof(NUS3BankDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
