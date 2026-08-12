/*
 * Square Enix AKB demuxer
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
#include "pcm.h"

typedef struct AKBStream {
    int64_t start_offset;

    AVFormatContext *xctx;
    AVFormatContext *parent;
    FFIOContext apb;
} AKBStream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('A','K','B',' '))
        return 0;

    if (p->buf_size < 16)
        return 0;
    if (p->buf[0x0d] == 0)
        return 0;
    if ((int)AV_RL16(p->buf + 0x0e) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    AKBStream *ast = opaque;
    AVFormatContext *s = ast->parent;
    AVIOContext *pb = s->pb;

    return avio_read(pb, buf, buf_size);
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    AKBStream *ast = opaque;
    AVFormatContext *s = ast->parent;
    AVIOContext *pb = s->pb;

    return avio_seek(pb, offset + ast->start_offset, whence);
}

static int read_header(AVFormatContext *s)
{
    int flags = 0, align = 0, codec, rate, channels, version, header_size, extradata_size, subheader_size;
    int64_t start, duration, extradata_offset;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 4);
    version = avio_r8(pb);
    avio_skip(pb, 1);
    header_size = avio_rl16(pb);
    avio_skip(pb, 4);
    codec = avio_r8(pb);
    channels = avio_r8(pb);
    rate = avio_rl16(pb);
    duration = avio_rl32(pb);
    avio_skip(pb, 8);
    if (header_size >= 0x44) {
        extradata_size = avio_rl16(pb);
        avio_skip(pb, 10);
        subheader_size = avio_rl16(pb);
        flags = avio_r8(pb);
        extradata_offset = header_size + subheader_size;
        start = extradata_offset + extradata_size;
    } else {
        start = header_size;
    }

    switch (codec) {
    case 1:
        codec = AV_CODEC_ID_PCM_S16LE;
        break;
    case 2:
        codec = AV_CODEC_ID_ADPCM_MS;
        avio_seek(pb, extradata_offset + 2, SEEK_SET);
        align = avio_rl16(pb);

        if ((version >= 3) && (flags & 8))
            return AVERROR_INVALIDDATA;
        break;
    case 5:
        codec = AV_CODEC_ID_VORBIS;
        break;
    default:
        avpriv_request_sample(s, "codec %02x", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (rate <= 0 || channels <= 0 || (align > 0 && channels >= INT_MAX/align))
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;
    if (align > 0)
        st->codecpar->block_align = align * channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if (codec == AV_CODEC_ID_VORBIS) {
        AKBStream *ast = st->priv_data;
        int ret;

        ast = av_mallocz(sizeof(*ast));
        if (!ast)
            return AVERROR(ENOMEM);

        st->priv_data = ast;
        ast->start_offset = start;

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
        if ((ret = avformat_open_input(&ast->xctx, "", NULL, NULL)) < 0)
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

        ffstream(st)->request_probe = 0;
        ffstream(st)->need_parsing = ffstream(ast->xctx->streams[0])->need_parsing;
    }

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    AKBStream *ast;
    AVStream *st;

    if (avio_feof(pb))
        return AVERROR_EOF;

    st = s->streams[0];
    ast = st->priv_data;

    if (ast) {
        ret = av_read_frame(ast->xctx, pkt);
    } else {
        ret = ff_pcm_read_packet(s, pkt);
    }

    pkt->stream_index = st->index;

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    AKBStream *ast;
    AVStream *st;

    st = s->streams[0];
    ast = st->priv_data;
    if (ast)
        return av_seek_frame(ast->xctx, 0, ts, flags);
    return -1;
}

static int read_close(AVFormatContext *s)
{
    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        AKBStream *ast = st->priv_data;

        avformat_close_input(&ast->xctx);
    }

    return 0;
}

const FFInputFormat ff_akb_demuxer = {
    .p.name         = "akb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Square Enix AKB"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "akb",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
