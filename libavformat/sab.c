/*
 * Square Enix SAB demuxer
 * Copyright (C) 2026 Paul B Mahol
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

#include <stddef.h>

#include "libavutil/intreadwrite.h"
#include "libavutil/internal.h"
#include "libavutil/macros.h"
#include "libavutil/mem.h"
#include "internal.h"
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"

#define SABF 1
#define MABF 2

typedef struct SABStream {
    int64_t start_offset;
    int64_t stop_offset;

    AVFormatContext *xctx;
    AVFormatContext *parent;
    FFIOContext apb;
} SABStream;

typedef struct SABDemuxContext {
    int big_endian;
    int type;
    int filename_offset;
    int filename_size;
    int sections_offset;
    int current_stream;

    int64_t snd_section_offset;
    int64_t seq_section_offset;
    int64_t trk_section_offset;
    int64_t mtrl_section_offset;
    int64_t musc_section_offset;
    int64_t inst_section_offset;
} SABDemuxContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('s','a','b','f') &&
        AV_RB32(p->buf) != MKBETAG('m','a','b','f'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    SABStream *sst = opaque;
    AVFormatContext *s = sst->parent;
    AVIOContext *pb = s->pb;

    return avio_read(pb, buf, buf_size);
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    SABStream *sst = opaque;
    AVFormatContext *s = sst->parent;
    AVIOContext *pb = s->pb;

    return avio_seek(pb, offset + sst->start_offset, whence);
}

static int guess_endian16(AVIOContext *pb)
{
    uint8_t buf[2];

    if (avio_read(pb, buf, sizeof(buf)) != sizeof(buf))
        return AVERROR(EIO);

    return AV_RL16(buf) > AV_RB16(buf);
}

typedef unsigned int (*avio_r32)(AVIOContext *s);
typedef unsigned int (*avio_r16)(AVIOContext *s);

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const SABStream *xs1 = s1->priv_data;
    const SABStream *xs2 = s2->priv_data;

    return FFDIFFSIGN(xs1->start_offset, xs2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    SABDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    avio_r32 avio_r32;
    avio_r16 avio_r16;
    uint32_t tag;
    int entries;

    tag = avio_rb32(pb);
    switch (tag) {
    case MKBETAG('s','a','b','f'):
        ctx->type = SABF;
        break;
    case MKBETAG('m','a','b','f'):
        ctx->type = MABF;
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    avio_skip(pb, 2);
    ctx->big_endian = guess_endian16(pb);
    if (ctx->big_endian) {
        avio_r32 = avio_rb32;
        avio_r16 = avio_rb16;
    } else {
        avio_r32 = avio_rl32;
        avio_r16 = avio_rl16;
    }
    avio_skip(pb, 1);
    ctx->filename_size = avio_r8(pb);
    ctx->filename_offset = 0x10;
    ctx->sections_offset = ctx->filename_offset + (ctx->filename_size + 1);
    ctx->sections_offset = FFALIGN(ctx->sections_offset, 16);

    avio_seek(pb, ctx->sections_offset, SEEK_SET);
    if (ctx->type == SABF) {
        tag = avio_rb32(pb);
        if (tag != MKBETAG('s','n','d',' '))
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 4);
        ctx->snd_section_offset = avio_r32(pb);
        avio_skip(pb, 4);
        tag = avio_rb32(pb);
        if (tag != MKBETAG('s','e','q',' '))
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 4);
        ctx->seq_section_offset = avio_r32(pb);
        avio_skip(pb, 4);
        tag = avio_rb32(pb);
        if (tag != MKBETAG('t','r','k',' '))
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 4);
        ctx->trk_section_offset = avio_r32(pb);
        avio_skip(pb, 4);
        tag = avio_rb32(pb);
        if (tag != MKBETAG('m','t','r','l'))
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 4);
        ctx->mtrl_section_offset = avio_r32(pb);
        avio_skip(pb, 4);
    } else {
        tag = avio_rb32(pb);
        if (tag != MKBETAG('m','u','s','c'))
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 4);
        ctx->musc_section_offset = avio_r32(pb);
        avio_skip(pb, 4);
        tag = avio_rb32(pb);
        if (tag != MKBETAG('i','n','s','t'))
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 4);
        ctx->inst_section_offset = avio_r32(pb);
        avio_skip(pb, 4);
        tag = avio_rb32(pb);
        if (tag != MKBETAG('m','t','r','l'))
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 4);
        ctx->mtrl_section_offset = avio_r32(pb);
        avio_skip(pb, 4);
    }

    avio_seek(pb, ctx->mtrl_section_offset + 4, SEEK_SET);
    entries = avio_r16(pb);
    for (int n = 0; n < entries; n++) {
        int nb_channels, codec, rate, extradata_size, block_align = 0, need_parsing = 0, ret, get_extradata = 0, need_xctx = 0;
        int64_t entry_offset, stream_size, extradata_offset;
        SABStream *sst;
        AVStream *st;

        avio_seek(pb, ctx->mtrl_section_offset + 0x10 + 4 * n, SEEK_SET);
        entry_offset = ctx->mtrl_section_offset;
        entry_offset += avio_r32(pb);
        extradata_offset = entry_offset + 0x20;

        avio_seek(pb, entry_offset, SEEK_SET);
        avio_skip(pb, 4);
        nb_channels = avio_r8(pb);
        codec = avio_r8(pb);
        avio_skip(pb, 2);
        rate = avio_r32(pb);
        avio_skip(pb, 8);
        extradata_size = avio_r32(pb);
        stream_size = avio_r32(pb);

        switch (codec) {
        case 0x01:
            codec = AV_CODEC_ID_PCM_S16LE;
            block_align = 512 * nb_channels;
            break;
        case 0x02:
            codec = AV_CODEC_ID_ADPCM_MS;
            avio_seek(pb, extradata_offset + 4, SEEK_SET);
            block_align = avio_r16(pb);
            break;
        case 0x04:
            codec = AV_CODEC_ID_ATRAC9;
            avio_seek(pb, extradata_offset + 4, SEEK_SET);
            block_align = avio_r16(pb);
            avio_skip(pb, 2);
            need_parsing = 1;
            get_extradata = 12;
            break;
        case 0x03:
            need_xctx = 1;
            need_parsing = 1;
            break;
        case 0x07:
            need_xctx = 1;
            avio_seek(pb, extradata_offset, SEEK_SET);
            get_extradata = extradata_size;
            break;
        case 0:
            continue;
        default:
            av_log(s, AV_LOG_WARNING, "Unsupported codec(%02X)\n", codec);
            break;
        }

        if ((codec == 0 || nb_channels == 0 || rate <= 0 || block_align <= 0 ||
            extradata_size < 0 || stream_size <= 0) && !need_xctx)
            continue;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        sst = av_mallocz(sizeof(*sst));
        if (!sst)
            return AVERROR(ENOMEM);
        st->priv_data = sst;
        sst->start_offset = extradata_offset + extradata_size;
        sst->stop_offset = sst->start_offset + stream_size;

        st->start_time = 0;
        if (need_xctx) {
            if (!(sst->xctx = avformat_alloc_context()))
                return AVERROR(ENOMEM);

            if ((ret = ff_copy_whiteblacklists(sst->xctx, s)) < 0) {
                avformat_free_context(sst->xctx);
                sst->xctx = NULL;

                return ret;
            }

            ffio_init_context(&sst->apb, NULL, 0, 0, sst,
                              read_data, NULL, seek_data);

            sst->xctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
            sst->xctx->probesize = 0;
            sst->xctx->max_analyze_duration = 0;
            sst->xctx->interrupt_callback = s->interrupt_callback;
            sst->xctx->pb = &sst->apb.pub;
            sst->xctx->io_open = NULL;
            sst->xctx->skip_initial_bytes = 0;
            sst->parent = s;

            avio_seek(pb, sst->start_offset, SEEK_SET);
            if ((ret = avformat_open_input(&sst->xctx, "", NULL, NULL)) < 0)
                return ret;

            st->id = sst->xctx->streams[0]->id;
            st->duration = sst->xctx->streams[0]->duration;
            st->time_base = sst->xctx->streams[0]->time_base;
            st->start_time = sst->xctx->streams[0]->start_time;
            st->pts_wrap_bits = sst->xctx->streams[0]->pts_wrap_bits;

            ret = avcodec_parameters_copy(st->codecpar, sst->xctx->streams[0]->codecpar);
            if (ret < 0)
                return ret;

            ret = av_dict_copy(&st->metadata, sst->xctx->streams[0]->metadata, 0);
            if (ret < 0)
                return ret;

            sst->start_offset = avio_tell(pb);
            ffstream(st)->request_probe = 0;
            ffstream(st)->need_parsing = ffstream(sst->xctx->streams[0])->need_parsing;
        } else {
            st->codecpar->codec_id = codec;
            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->ch_layout.nb_channels = nb_channels;
            st->codecpar->sample_rate = rate;
            st->codecpar->block_align = block_align;

            if (get_extradata > 0) {
                ret = ff_alloc_extradata(st->codecpar, get_extradata);
                if (ret < 0)
                    return ret;
                avio_read(pb, st->codecpar->extradata, get_extradata);
                if (codec == AV_CODEC_ID_ATRAC9 && get_extradata >= 12) {
                    AV_WL32(st->codecpar->extradata, 3);
                    if (!ctx->big_endian) {
                        AV_WL32(st->codecpar->extradata+4, AV_RB32(st->codecpar->extradata+4));
                        AV_WL32(st->codecpar->extradata+8, AV_RB32(st->codecpar->extradata+8));
                    }
                }
            }
        }

        if (need_parsing)
            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    if (s->nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;

        if (n == 0) {
            SABStream *sst = st->priv_data;
            avio_seek(pb, sst->start_offset, SEEK_SET);
        }
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SABDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    SABStream *sst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (ctx->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[ctx->current_stream];
    sst = st->priv_data;
    if (do_seek)
        avio_seek(pb, sst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= sst->stop_offset) {
        do_seek = 1;
        ctx->current_stream++;
        goto redo;
    }

    if (sst->xctx) {
        ret = av_read_frame(sst->xctx, pkt);
        if (ret == AVERROR_EOF) {
            ctx->current_stream++;
            goto redo;
        }
    } else {
        const int64_t pos = avio_tell(pb);
        const int size = FFMIN(st->codecpar->block_align, sst->stop_offset - pos);

        ret = av_get_packet(pb, pkt, size);
        pkt->pos = pos;
    }
    pkt->stream_index = st->index;
    if (ret == AVERROR_EOF) {
        ctx->current_stream++;
        goto redo;
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    SABDemuxContext *ctx = s->priv_data;
    SABStream *sst;
    AVStream *st;

    ctx->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[ctx->current_stream];
    sst = st->priv_data;
    if (sst->xctx) {
        return av_seek_frame(sst->xctx, 0, ts, flags);
    } else {
        AVIOContext *pb = s->pb;
        int64_t pos = avio_tell(pb);

        if (pos < sst->start_offset) {
            avio_seek(pb, sst->start_offset, SEEK_SET);
            return 0;
        }

        return -1;
    }
}

const FFInputFormat ff_sab_demuxer = {
    .p.name         = "sab",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Square Enix SAB"),
    .p.extensions   = "sab,mab,sbin",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(SABDemuxContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
