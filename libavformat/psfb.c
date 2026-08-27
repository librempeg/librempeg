/*
 * PSF Bank demuxer
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

typedef struct PSFDemuxContext {
    int current_stream;
} PSFDemuxContext;

typedef struct PSFStream {
    int64_t start_offset;
    int64_t data_offset;
    int64_t stop_offset;

    AVFormatContext *xctx;
    AVFormatContext *parent;
    FFIOContext apb;
} PSFStream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('P','S','F','\x60') &&
        AV_RB32(p->buf) != MKBETAG('P','S','F','\x31'))
        return 0;

    if (((int)AV_RL32(p->buf + 4) - 1) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int sort_offsets(const void *a, const void *b)
{
    const uint32_t *s1p = a;
    const uint32_t *s2p = b;
    const int64_t s1 = *s1p;
    const int64_t s2 = *s2p;

    return FFDIFFSIGN(s1, s2);
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    PSFStream *pst = opaque;
    AVFormatContext *s = pst->parent;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int size;

    if (pos + buf_size <= pst->start_offset)
        return AVERROR(EIO);

    if (pos >= pst->stop_offset)
        return AVERROR_EOF;

    size = FFMIN3(pos + buf_size - pst->start_offset, pst->stop_offset - pos, buf_size);

    return avio_read(pb, buf, size);
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    PSFStream *pst = opaque;
    AVFormatContext *s = pst->parent;
    AVIOContext *pb = s->pb;
    int64_t new_offset;
    int64_t ret;

    if (whence == SEEK_CUR) {
        int64_t pos = avio_tell(pb);

        new_offset = av_clip64(pos + offset, pst->start_offset, pst->stop_offset-1);
        ret = avio_seek(pb, new_offset, whence);
        if (ret < 0)
            return ret;
        return ret - pst->start_offset;
    }

    if (whence & AVSEEK_SIZE)
        return pst->stop_offset - pst->start_offset;

    new_offset = av_clip64(offset + pst->start_offset, pst->start_offset, pst->stop_offset-1);

    ret = avio_seek(pb, new_offset, whence);
    if (ret < 0)
        return ret;

    return ret - pst->start_offset;
}

static int read_header(AVFormatContext *s)
{
    uint32_t *offsets, prev_offset = 0;
    int64_t first_start_offset;
    AVIOContext *pb = s->pb;
    int ret, nb_offsets;

    avio_skip(pb, 4);
    nb_offsets = avio_rl32(pb);
    nb_offsets--;
    if (nb_offsets <= 0)
        return AVERROR_INVALIDDATA;

    offsets = av_calloc(nb_offsets, sizeof(*offsets));
    if (!offsets)
        return AVERROR(ENOMEM);

    avio_skip(pb, 12);
    for (int n = 0; n < nb_offsets; n++) {
        if (avio_feof(pb)) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }

        offsets[n] = avio_rl32(pb);
        avio_skip(pb, 8);
    }

    qsort(offsets, nb_offsets, sizeof(*offsets), sort_offsets);
    for (int n = 0; n < nb_offsets; n++) {
        PSFStream *pst;
        AVStream *st;

        if (prev_offset == offsets[n])
            continue;
        prev_offset = offsets[n];

        if (avio_feof(pb)) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }

        st = avformat_new_stream(s, NULL);
        if (!st) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        pst = av_mallocz(sizeof(*pst));
        if (!pst) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        st->priv_data = pst;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;

        pst->start_offset = offsets[n];
        if (!(pst->xctx = avformat_alloc_context())) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        if ((ret = ff_copy_whiteblacklists(pst->xctx, s)) < 0) {
            avformat_free_context(pst->xctx);
            pst->xctx = NULL;
            goto fail;
        }

        ffio_init_context(&pst->apb, NULL, 0, 0, pst,
                          read_data, NULL, seek_data);

        pst->xctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
        pst->xctx->probesize = 0;
        pst->xctx->max_analyze_duration = 0;
        pst->xctx->interrupt_callback = s->interrupt_callback;
        pst->xctx->pb = &pst->apb.pub;
        pst->xctx->io_open = NULL;
        pst->xctx->skip_initial_bytes = 0;
        pst->parent = s;
    }

    av_freep(&offsets);

    for (int n = 0; n < s->nb_streams; n++) {
        PSFStream *pst;
        FFStream *sti;
        AVStream *st;

        st = s->streams[n];
        pst = st->priv_data;
        if (n < s->nb_streams - 1) {
            AVStream *next_st = s->streams[n+1];
            PSFStream *next_pst = next_st->priv_data;

            pst->stop_offset = next_pst->start_offset;
        } else {
            pst->stop_offset = avio_size(pb);
        }

        avio_seek(pb, pst->start_offset, SEEK_SET);
        ret = avformat_open_input(&pst->xctx, "", NULL, NULL);
        if (ret < 0)
            return ret;

        ret = avformat_find_stream_info(pst->xctx, NULL);
        if (ret < 0)
            return ret;

        st->id = pst->xctx->streams[0]->id;
        st->duration = pst->xctx->streams[0]->duration;
        st->time_base = pst->xctx->streams[0]->time_base;
        st->start_time = pst->xctx->streams[0]->start_time;
        st->pts_wrap_bits = pst->xctx->streams[0]->pts_wrap_bits;

        ret = avcodec_parameters_copy(st->codecpar, pst->xctx->streams[0]->codecpar);
        if (ret < 0)
            return ret;

        ret = av_dict_copy(&st->metadata, pst->xctx->streams[0]->metadata, 0);
        if (ret < 0)
            return ret;

        sti = ffstream(st);
        sti->request_probe = 0;
        sti->need_parsing = ffstream(pst->xctx->streams[0])->need_parsing;

        pst->data_offset = avio_tell(pb);
    }

    {
        AVStream *st = s->streams[0];
        PSFStream *pst = st->priv_data;

        first_start_offset = pst->data_offset;
    }

    avio_seek(pb, first_start_offset, SEEK_SET);

    return 0;
fail:
    av_freep(&offsets);
    return ret;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    PSFDemuxContext *psf = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    PSFStream *pst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (psf->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[psf->current_stream];
    pst = st->priv_data;
    if (do_seek)
        avio_seek(pb, pst->data_offset, SEEK_SET);

    if (avio_tell(pb) >= pst->stop_offset) {
        do_seek = 1;
        psf->current_stream++;
        goto redo;
    }

    ret = av_read_frame(pst->xctx, pkt);
    pkt->stream_index = st->index;
    if (ret == AVERROR_EOF) {
        psf->current_stream++;
        goto redo;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    PSFDemuxContext *psf = s->priv_data;
    PSFStream *pst;
    AVStream *st;

    psf->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[psf->current_stream];
    pst = st->priv_data;

    return av_seek_frame(pst->xctx, 0, ts, flags);
}

static int read_close(AVFormatContext *s)
{
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        PSFStream *pst = st->priv_data;

        avformat_close_input(&pst->xctx);
    }

    return 0;
}

const FFInputFormat ff_psfb_demuxer = {
    .p.name         = "psfb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Pivotal PSF Bank"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "psf",
    .priv_data_size = sizeof(PSFDemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
