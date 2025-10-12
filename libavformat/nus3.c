/*
 * NUS3 demuxer
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
#include "avformat_internal.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"

typedef struct NUS3Stream {
    int64_t start, stop, name;

    AVFormatContext *xctx;
    AVFormatContext *parent;
    FFIOContext apb;
} NUS3Stream;

typedef struct NUS3DemuxContext {
    int current_stream;
} NUS3DemuxContext;

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    NUS3Stream *nst = opaque;
    AVFormatContext *s = nst->parent;
    AVIOContext *pb = s->pb;

    return avio_read(pb, buf, buf_size);
}

static int64_t seek_data(void *opaque, int64_t offset, int whence)
{
    NUS3Stream *nst = opaque;
    AVFormatContext *s = nst->parent;
    AVIOContext *pb = s->pb;

    return avio_seek(pb, offset + nst->start, whence);
}

static int read_probe(const AVProbeData *p)
{
    int score = 50;

    if (AV_RL32(p->buf) != MKTAG('N','U','S','3'))
        return 0;

    if (p->buf_size < 16)
        return 0;

    if (AV_RL32(p->buf+8) != MKTAG('A','U','D','I'))
        return 0;

    if (AV_RL32(p->buf+12) == MKTAG('I','N','D','X'))
        score += 50;

    return score;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const NUS3Stream *as1 = s1->priv_data;
    const NUS3Stream *as2 = s2->priv_data;

    return FFDIFFSIGN(as1->start, as2->start);
}

static int read_header(AVFormatContext *s)
{
    int32_t *name_offsets = NULL;
    AVIOContext *pb = s->pb;
    int nb_streams = 0;
    int found = 0;

    avio_skip(pb, 12);

    while (!avio_feof(pb)) {
        uint32_t chunk, size;

        chunk = avio_rb32(pb);
        size = avio_rl32(pb);

        switch (chunk) {
        case MKBETAG('I','N','D','X'):
            if (!nb_streams) {
                nb_streams = avio_rl32(pb);
                if (nb_streams < 1)
                    return AVERROR_INVALIDDATA;

                size -= 4;
            } else {
                return AVERROR_INVALIDDATA;
            }
            break;
        case MKBETAG('A','D','O','F'):
            if (size < 8LL * nb_streams)
                return AVERROR_INVALIDDATA;

            for (int n = 0; n < nb_streams; n++) {
                int64_t start, stop;
                NUS3Stream *nst;
                AVStream *st;

                if (avio_feof(pb))
                    return AVERROR_INVALIDDATA;

                start = avio_rl32(pb);
                stop = start;
                stop += avio_rl32(pb);
                size -= 8;
                if (start <= 0)
                    continue;

                st = avformat_new_stream(s, NULL);
                if (!st)
                    return AVERROR(ENOMEM);

                nst = av_mallocz(sizeof(*nst));
                if (!nst)
                    return AVERROR(ENOMEM);

                st->priv_data = nst;

                nst->start = start;
                nst->stop = stop;
                if (name_offsets)
                    nst->name = name_offsets[n];
            }

            found = 1;
            break;
        case MKBETAG('N','M','O','F'):
            if (size < 4LL * nb_streams)
                return AVERROR_INVALIDDATA;

            if (name_offsets) {
                return AVERROR_INVALIDDATA;
            } else {
                name_offsets = av_calloc(nb_streams, sizeof(*name_offsets));
                if (!name_offsets)
                    return AVERROR(ENOMEM);
            }

            for (int n = 0; n < nb_streams; n++) {
                if (avio_feof(pb))
                    return AVERROR_INVALIDDATA;

                name_offsets[n] = avio_rl32(pb);

                size -= 4;
            }
            break;
        case MKBETAG('P','A','C','K'):
        case MKBETAG('J','U','N','K'):
        default:
            break;
        }

        if (found)
            break;

        avio_skip(pb, size);
    }

    av_freep(&name_offsets);
    if (!found)
        return AVERROR_INVALIDDATA;

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        NUS3Stream *nst = st->priv_data;
        char title[0x31] = { 0 };
        int ret;

        if (!nst->name)
            continue;

        avio_seek(pb, nst->name, SEEK_SET);
        if ((ret = avio_get_str(pb, 0x30, title, sizeof(title))) < 0)
            return ret;
        if (title[0] != '\0')
            av_dict_set(&st->metadata, "title", title, 0);
    }

    for (int n = 0; n < s->nb_streams; n++) {
        int ret, nb_channels = 0, rate = 0, codec_id = AV_CODEC_ID_NONE;
        uint32_t codec, duration = 0, skip = 0, new_size = 0;
        AVStream *st = s->streams[n];
        NUS3Stream *nst = st->priv_data;
        int64_t pos, offset;

        avio_seek(pb, nst->start, SEEK_SET);

        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;

        codec = avio_rb32(pb);
        switch (codec) {
        case MKBETAG('O','P','U','S'):
            codec_id = AV_CODEC_ID_OPUS;
            avio_skip(pb, 4);
            duration = avio_rb32(pb);
            avio_skip(pb, 52);
            pos = avio_tell(pb);
            if (avio_rl32(pb) != 0x80000001)
                return AVERROR_INVALIDDATA;
            avio_skip(pb, 5);
            nb_channels = avio_r8(pb);
            avio_skip(pb, 2);
            rate = avio_rl32(pb);
            offset = avio_rl32(pb) + 8LL;
            avio_skip(pb, 8);
            skip = avio_rl16(pb);
            avio_skip(pb, 2);
            if (avio_rl32(pb) != 0x80000004)
                return AVERROR_INVALIDDATA;

            new_size = avio_rl32(pb);
            nst->stop = nst->start + new_size;

            if (rate <= 0 || nb_channels <= 0)
                return AVERROR_INVALIDDATA;

            st->start_time = 0;
            st->duration = duration;
            st->codecpar->codec_id = codec_id;
            st->codecpar->ch_layout.nb_channels = nb_channels;
            st->codecpar->sample_rate = rate;

            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

            ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;

            nst->start = pos + offset;
            break;
        case MKBETAG('R','I','F','F'):
            if (!(nst->xctx = avformat_alloc_context()))
                return AVERROR(ENOMEM);

            if ((ret = ff_copy_whiteblacklists(nst->xctx, s)) < 0) {
                avformat_free_context(nst->xctx);
                nst->xctx = NULL;

                return ret;
            }

            ffio_init_context(&nst->apb, NULL, 0, 0, nst,
                              read_data, NULL, seek_data);

            nst->xctx->flags = AVFMT_FLAG_CUSTOM_IO | AVFMT_FLAG_GENPTS;
            nst->xctx->probesize = 0;
            nst->xctx->max_analyze_duration = 0;
            nst->xctx->interrupt_callback = s->interrupt_callback;
            nst->xctx->pb = &nst->apb.pub;
            nst->xctx->io_open = NULL;
            nst->xctx->skip_initial_bytes = 0;
            nst->parent = s;

            avio_seek(pb, nst->start, SEEK_SET);
            if ((ret = avformat_open_input(&nst->xctx, "", NULL, NULL)) < 0)
                return ret;

            st->id = nst->xctx->streams[0]->id;
            st->duration = nst->xctx->streams[0]->duration;
            st->time_base = nst->xctx->streams[0]->time_base;
            st->start_time = nst->xctx->streams[0]->start_time;
            st->pts_wrap_bits = nst->xctx->streams[0]->pts_wrap_bits;
            st->codecpar->codec_id = nst->xctx->streams[0]->codecpar->codec_id;
            st->codecpar->bit_rate = nst->xctx->streams[0]->codecpar->bit_rate;
            st->codecpar->sample_rate = nst->xctx->streams[0]->codecpar->sample_rate;
            st->codecpar->block_align = nst->xctx->streams[0]->codecpar->block_align;
            if ((ret = av_channel_layout_copy(&st->codecpar->ch_layout, &nst->xctx->streams[0]->codecpar->ch_layout)) < 0)
                return ret;

            if ((ret = ff_alloc_extradata(st->codecpar, nst->xctx->streams[0]->codecpar->extradata_size)))
                return ret;
            memcpy(st->codecpar->extradata, nst->xctx->streams[0]->codecpar->extradata, nst->xctx->streams[0]->codecpar->extradata_size);

            ret = av_dict_copy(&st->metadata, nst->xctx->streams[0]->metadata, 0);
            if (ret < 0)
                return ret;

            ffstream(st)->request_probe = 0;
            ffstream(st)->need_parsing = ffstream(nst->xctx->streams[0])->need_parsing;

            nst->start = avio_tell(pb);
            break;
        default:
            avpriv_request_sample(s, "codec 0x%04X", codec);
            return AVERROR_PATCHWELCOME;
        }

        switch (codec) {
        case MKBETAG('O','P','U','S'):
            ret = ff_alloc_extradata(st->codecpar, 19);
            if (ret < 0)
                return ret;
            memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

            memcpy(st->codecpar->extradata, "OpusHead", 8);
            st->codecpar->extradata[8] = 1;
            st->codecpar->extradata[9] = nb_channels;
            AV_WL16(st->codecpar->extradata + 10, skip);
            AV_WL32(st->codecpar->extradata + 12, 48000);
            break;
        }
    }

    {
        AVStream *st = s->streams[0];
        NUS3Stream *nst = st->priv_data;

        avio_seek(pb, nst->start, SEEK_SET);
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int64_t pos;

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        NUS3Stream *nst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= nst->start && pos < nst->stop) {
            if (nst->xctx) {
                ret = av_read_frame(nst->xctx, pkt);
                pkt->stream_index = st->index;
            } else {
                int size = avio_rb32(pb);

                if (size < 2 ||
                    size > nst->stop - pos - 4) {
                    size = nst->stop - pos - 4;
                }

                avio_skip(pb, 4);
                ret = av_get_packet(pb, pkt, size);
                pkt->pos = pos;
                pkt->stream_index = st->index;
            }
            break;
        } else if (pos >= nst->stop && i+1 < s->nb_streams) {
            AVStream *st_next = s->streams[i+1];
            NUS3Stream *nst_next = st_next->priv_data;
            if (nst_next->start > pos)
                avio_seek(pb, nst_next->start, SEEK_SET);
        }
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t timestamp, int flags)
{
    NUS3DemuxContext *nus3 = s->priv_data;
    NUS3Stream *nst;
    AVStream *st;

    nus3->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[nus3->current_stream];
    nst = st->priv_data;
    if (nst->xctx)
        return av_seek_frame(nst->xctx, 0, timestamp, flags);

    return -1;
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

const FFInputFormat ff_nus3_demuxer = {
    .p.name         = "nus3",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Namco NUS3"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "nus3audio",
    .priv_data_size = sizeof(NUS3DemuxContext),
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
