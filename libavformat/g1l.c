/*
 * G1L demuxer
 * Copyright (c) 2025 smiRaphi
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

typedef struct G1LDemuxContext {
    int little_endian;
    int current_stream;
} G1LDemuxContext;

typedef struct G1LStream {
    int64_t start_offset;
    int64_t data_offset;
    int64_t stop_offset;

    AVFormatContext *xctx;
    AVFormatContext *parent;
    FFIOContext apb;
} G1LStream;

static int g1l_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "G1L_", 4) && memcmp(p->buf, "_L1G", 4))
        return 0;
    if (memcmp(p->buf + 4, "0000", 4))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    G1LStream *gst = opaque;
    AVFormatContext *s = gst->parent;
    AVIOContext *pb = s->pb;

    return avio_read(pb, buf, buf_size);
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    G1LStream *gst = opaque;
    AVFormatContext *s = gst->parent;
    AVIOContext *pb = s->pb;

    return avio_seek(pb, offset + gst->start_offset, whence);
}

static av_always_inline unsigned int read32(AVFormatContext *s)
{
    G1LDemuxContext *g = s->priv_data;
    if (g->little_endian)
        return avio_rl32(s->pb);
    else
        return avio_rb32(s->pb);
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const G1LStream *gs1 = s1->priv_data;
    const G1LStream *gs2 = s2->priv_data;

    return FFDIFFSIGN(gs1->start_offset, gs2->start_offset);
}

static int g1l_read_header(AVFormatContext *s)
{
    G1LDemuxContext *g = s->priv_data;
    AVIOContext *pb = s->pb;
    uint32_t bom, format;
    int ret, nb_streams;

    bom = avio_rb32(pb);
    if (bom == MKTAG('G','1','L','_'))
        g->little_endian = 1;
    else if (bom == MKTAG('_','L','1','G'))
        g->little_endian = 0;
    else
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 12);

    format = read32(s);
    nb_streams = read32(s);
    if (nb_streams < 1)
        return AVERROR_INVALIDDATA;

    for (int i = 0; i < nb_streams; i++) {
        G1LStream *gst;
        AVStream *st;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        gst = av_mallocz(sizeof(*gst));
        if (!gst)
            return AVERROR(ENOMEM);

        st->start_time = 0;
        st->priv_data = gst;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        gst->start_offset = read32(s);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        G1LStream *gst = st->priv_data;

        st->index = n;

        if (n == (s->nb_streams - 1))
            gst->stop_offset = avio_size(pb);

        if (n > 0) {
            AVStream *prev_st = s->streams[n-1];
            G1LStream *prev_gst = prev_st->priv_data;

            prev_gst->stop_offset = gst->start_offset;
        }
    }

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        G1LStream *gst = st->priv_data;
        uint32_t chunk;

        if (!(gst->xctx = avformat_alloc_context()))
            return AVERROR(ENOMEM);

        if ((ret = ff_copy_whiteblacklists(gst->xctx, s)) < 0) {
            avformat_free_context(gst->xctx);
            gst->xctx = NULL;

            return ret;
        }

        ffio_init_context(&gst->apb, NULL, 0, 0, gst,
                          read_data, NULL, seek_data);

        gst->xctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
        gst->xctx->probesize = 0;
        gst->xctx->max_analyze_duration = 0;
        gst->xctx->interrupt_callback = s->interrupt_callback;
        gst->xctx->pb = &gst->apb.pub;
        gst->xctx->io_open = NULL;
        gst->xctx->skip_initial_bytes = 0;
        gst->parent = s;

        avio_seek(pb, gst->start_offset, SEEK_SET);

        chunk = avio_rb32(pb);
        if (chunk == MKBETAG('A','T','S','L')) {
            int64_t size;

            avio_skip(pb, 40);
            size = avio_rl32(pb);
            gst->start_offset += size;
        }
        avio_seek(pb, gst->start_offset, SEEK_SET);

        switch (format) {
        case 0x00: /* KOVS (OGG) Romance of the Three Kingdoms XIII (PC) */
        case 0x0A: /* KOVS (OGG) Dragon Quest Heroes (PC), Bladestorm (PC) */
        case 0x01: /* RIFF (ATRAC3P) One Piece Pirate Warriors 2 (PS3) */
        case 0x06: /* RIFF (ATRAC9) One Piece Pirate Warriors 3 (Vita) */
        case 0x05: /* KTSS (DSP ADPCM) Shingeki no Kyojin - Shichi-Kara no Dasshutsu (3DS) */
        case 0x10: /* KTSS (OPUS) Dead Or Alive Xtreme 3 Scarlet (Switch) */
        case 0x09: /* WiiBGM (DSP ADPCM) Hyrule Warriors (Wii U) */
            ret = avformat_open_input(&gst->xctx, "", NULL, NULL);
            break;
        default:
            avpriv_request_sample(st, "format 0x%X", format);
            return AVERROR_PATCHWELCOME;
        }
        if (ret < 0)
            return ret;

        ret = avformat_find_stream_info(gst->xctx, NULL);
        if (ret < 0)
            return ret;

        st->id = gst->xctx->streams[0]->id;
        st->duration = gst->xctx->streams[0]->duration;
        st->time_base = gst->xctx->streams[0]->time_base;
        st->start_time = gst->xctx->streams[0]->start_time;
        st->pts_wrap_bits = gst->xctx->streams[0]->pts_wrap_bits;

        ret = avcodec_parameters_copy(st->codecpar, gst->xctx->streams[0]->codecpar);
        if (ret < 0)
            return ret;

        ret = av_dict_copy(&st->metadata, gst->xctx->streams[0]->metadata, 0);
        if (ret < 0)
            return ret;

        ffstream(st)->request_probe = 0;
        ffstream(st)->need_parsing = ffstream(gst->xctx->streams[0])->need_parsing;

        gst->data_offset = avio_tell(pb);
        if (gst->data_offset > gst->stop_offset)
            gst->data_offset = gst->stop_offset;
    }

    {
        AVStream *st = s->streams[0];
        G1LStream *gst = st->priv_data;

        avio_seek(pb, gst->data_offset, SEEK_SET);
    }

    return 0;
}

static int g1l_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    G1LDemuxContext *g = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    G1LStream *gst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (g->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[g->current_stream];
    gst = st->priv_data;

    if (avio_tell(pb) >= gst->stop_offset) {
        if (g->current_stream+1 < s->nb_streams) {
            g->current_stream++;
            st = s->streams[g->current_stream];
            gst = st->priv_data;

            avio_seek(pb, gst->data_offset, SEEK_SET);
            goto redo;
        }
    }

    ret = av_read_frame(gst->xctx, pkt);
    pkt->stream_index = st->index;
    if (ret == AVERROR_EOF) {
        if (g->current_stream+1 < s->nb_streams) {
            g->current_stream++;
            st = s->streams[g->current_stream];
            gst = st->priv_data;

            avio_seek(pb, gst->data_offset, SEEK_SET);
            goto redo;
        }
    }

    return ret;
}

static int g1l_read_seek(AVFormatContext *s, int stream_index,
                         int64_t ts, int flags)
{
    G1LDemuxContext *g = s->priv_data;
    G1LStream *gst;
    AVStream *st;

    g->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[g->current_stream];
    gst = st->priv_data;

    return av_seek_frame(gst->xctx, 0, ts, flags);
}

static int g1l_read_close(AVFormatContext *s)
{
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        G1LStream *gst = st->priv_data;

        avformat_close_input(&gst->xctx);
    }

    return 0;
}

const FFInputFormat ff_g1l_demuxer = {
    .p.name         = "g1l",
    .p.long_name    = NULL_IF_CONFIG_SMALL("G1L (Koei Tecmo)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "g1l,xhs",
    .priv_data_size = sizeof(G1LDemuxContext),
    .read_probe     = g1l_probe,
    .read_header    = g1l_read_header,
    .read_packet    = g1l_read_packet,
    .read_seek      = g1l_read_seek,
    .read_close     = g1l_read_close,
};
