/*
 * AKB2 demuxer
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

typedef struct AKB2DemuxContext {
    int current_stream;
} AKB2DemuxContext;

typedef struct AKB2Stream {
    int64_t start_offset;
    int64_t data_offset;
    int64_t stop_offset;

    AVFormatContext *xctx;
    AVFormatContext *parent;
    FFIOContext apb;
} AKB2Stream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('A','K','B','2'))
        return 0;

    if (p->buf_size < 13)
        return 0;
    if (AV_RL16(p->buf+6) == 0)
        return 0;
    if (p->buf[12] <= 0 ||
        p->buf[12] > 2)
        return 0;

    return AVPROBE_SCORE_MAX/2;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const AKB2Stream *as1 = s1->priv_data;
    const AKB2Stream *as2 = s2->priv_data;

    return FFDIFFSIGN(as1->start_offset, as2->start_offset);
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    AKB2Stream *ast = opaque;
    AVFormatContext *s = ast->parent;
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int size;

    if (pos + buf_size <= ast->start_offset)
        return AVERROR(EIO);

    if (pos >= ast->stop_offset)
        return AVERROR_EOF;

    size = FFMIN3(pos + buf_size - ast->start_offset, ast->stop_offset - pos, buf_size);

    return avio_read(pb, buf, size);
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    AKB2Stream *ast = opaque;
    AVFormatContext *s = ast->parent;
    AVIOContext *pb = s->pb;
    int64_t new_offset;
    int64_t ret;

    if (whence == SEEK_CUR) {
        int64_t pos = avio_tell(pb);

        new_offset = av_clip64(pos + offset, ast->start_offset, ast->stop_offset-1);
        ret = avio_seek(pb, new_offset, whence);
        if (ret < 0)
            return ret;
        return ret - ast->start_offset;
    }

    if (whence & AVSEEK_SIZE)
        return ast->stop_offset - ast->start_offset;

    new_offset = av_clip64(offset + ast->start_offset, ast->start_offset, ast->stop_offset-1);

    ret = avio_seek(pb, new_offset, whence);
    if (ret < 0)
        return ret;

    return ret - ast->start_offset;
}

static int read_header(AVFormatContext *s)
{
    int header_size, ret, tab_count, nb_streams, tab_size;
    int64_t first_start_offset, tab_offset;
    AVIOContext *pb = s->pb;

    avio_skip(pb, 6);
    header_size = avio_rl16(pb);
    avio_skip(pb, 4);
    tab_count = avio_r8(pb);
    if (tab_count == 0 || tab_count > 2)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, header_size + (tab_count-1)*16 + 4, SEEK_SET);
    tab_offset = avio_rl32(pb);
    avio_seek(pb, tab_offset + 2, SEEK_SET);
    tab_size = avio_rl16(pb);
    avio_seek(pb, tab_offset + 15, SEEK_SET);
    nb_streams = avio_r8(pb);
    if (nb_streams < 1)
        return AVERROR_INVALIDDATA;

    for (int n = 0; n < nb_streams; n++) {
        int codec, flags, channels, rate, stream_header_size, extradata_size, align = 0;
        int64_t stream_size, duration, start_offset, extradata_offset;
        AKB2Stream *ast;
        AVStream *st;

        avio_seek(pb, tab_offset + tab_size + n*16 + 4, SEEK_SET);
        int64_t offset = tab_offset + avio_rl32(pb);
        avio_seek(pb, offset, SEEK_SET);

        avio_skip(pb, 1);
        codec = avio_r8(pb);
        channels = avio_r8(pb);
        flags = avio_r8(pb);
        stream_header_size = avio_rl16(pb);
        rate = avio_rl16(pb);
        stream_size = avio_rl32(pb);
        duration = avio_rl32(pb);
        avio_skip(pb, 8);
        extradata_size = avio_rl32(pb);
        extradata_offset = offset + stream_header_size;
        start_offset = offset + stream_header_size + extradata_size;

        if (flags & 0x08)
            continue;

        if (rate <= 0 || channels <= 0)
            return AVERROR_INVALIDDATA;

        switch (codec) {
        case 1:
            codec = AV_CODEC_ID_PCM_S16LE;
            align = 2;
            break;
        case 2:
            codec = AV_CODEC_ID_ADPCM_MS;
            avio_seek(pb, extradata_offset+2, SEEK_SET);
            align = avio_rl16(pb) / channels;
            break;
        case 5:
            codec = AV_CODEC_ID_VORBIS;
            align = 1024;
            break;
        default:
            avpriv_request_sample(s, "codec %X\n", codec);
            return AVERROR_PATCHWELCOME;
        }

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        ast = av_mallocz(sizeof(*ast));
        if (!ast)
            return AVERROR(ENOMEM);

        st->priv_data = ast;
        st->start_time = 0;
        st->duration = duration;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->codec_id = codec;
        st->codecpar->block_align = align * channels;

        ast->start_offset = ast->data_offset = start_offset;
        ast->stop_offset = start_offset + stream_size;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    if (s->nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);

    for (int n = 0; n < nb_streams; n++) {
        AKB2Stream *ast;
        FFStream *sti;
        AVStream *st;

        st = s->streams[n];
        ast = st->priv_data;

        if (st->codecpar->codec_id != AV_CODEC_ID_VORBIS)
            continue;

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
        ret = avformat_open_input(&ast->xctx, "", NULL, NULL);
        if (ret < 0)
            return ret;

        ret = avformat_find_stream_info(ast->xctx, NULL);
        if (ret < 0)
            return ret;

        st->id = ast->xctx->streams[0]->id;
        st->duration = ast->xctx->streams[0]->duration;
        st->time_base = ast->xctx->streams[0]->time_base;
        st->start_time = ast->xctx->streams[0]->start_time;
        st->pts_wrap_bits = ast->xctx->streams[0]->pts_wrap_bits;

        ret = avcodec_parameters_copy(st->codecpar, ast->xctx->streams[0]->codecpar);
        if (ret < 0)
            return ret;

        ret = av_dict_copy(&st->metadata, ast->xctx->streams[0]->metadata, 0);
        if (ret < 0)
            return ret;

        sti = ffstream(st);
        sti->request_probe = 0;
        sti->need_parsing = ffstream(ast->xctx->streams[0])->need_parsing;

        ast->data_offset = avio_tell(pb);
    }

    {
        AVStream *st = s->streams[0];
        AKB2Stream *ast = st->priv_data;

        first_start_offset = ast->data_offset;
    }

    avio_seek(pb, first_start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AKB2DemuxContext *akb2 = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    AKB2Stream *ast;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (akb2->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[akb2->current_stream];
    ast = st->priv_data;
    if (do_seek)
        avio_seek(pb, ast->data_offset, SEEK_SET);

    if (avio_tell(pb) >= ast->stop_offset) {
        do_seek = 1;
        akb2->current_stream++;
        goto redo;
    }

    if (ast->xctx) {
        ret = av_read_frame(ast->xctx, pkt);
    } else {
        const int64_t pos = avio_tell(pb);
        int size = ff_pcm_default_packet_size(st->codecpar);
        size = FFMIN(ast->stop_offset - pos, size);

        ret = av_get_packet(pb, pkt, size);
        pkt->pos = pos;
    }

    pkt->stream_index = st->index;
    if (ret == AVERROR_EOF) {
        akb2->current_stream++;
        goto redo;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    AKB2DemuxContext *akb2 = s->priv_data;
    AKB2Stream *ast;
    AVStream *st;

    akb2->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[akb2->current_stream];
    ast = st->priv_data;

    if (ast->xctx) {
        return av_seek_frame(ast->xctx, 0, ts, flags);
    } else {
        AVIOContext *pb = s->pb;
        int64_t pos = avio_tell(pb);

        if (pos < ast->data_offset) {
            avio_seek(pb, ast->data_offset, SEEK_SET);
            return 0;
        }

        return -1;
    }
}

static int read_close(AVFormatContext *s)
{
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        AKB2Stream *ast = st->priv_data;

        avformat_close_input(&ast->xctx);
    }

    return 0;
}

const FFInputFormat ff_akb2_demuxer = {
    .p.name         = "akb2",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Square Enix AKB2"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "akb",
    .priv_data_size = sizeof(AKB2DemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
