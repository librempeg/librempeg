/*
 * Sensaura GameCODA SAB demuxer
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

typedef struct SABStream {
    int64_t start_offset;
    int64_t stop_offset;
} SABStream;

typedef struct SABContext {
    int target_stream;
    int block_size;
    int interleaved;
} SABContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('C','S','W','2') &&
        AV_RB32(p->buf) != MKBETAG('C','S','P','2') &&
        AV_RB32(p->buf) != MKBETAG('C','S','X','2'))
        return 0;

    if (p->buf_size < 16)
        return 0;
    if ((int)AV_RL32(p->buf+8) <= 0)
        return 0;
    if ((int)AV_RL32(p->buf+12) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const SABStream *as1 = s1->priv_data;
    const SABStream *as2 = s2->priv_data;

    return FFDIFFSIGN(as1->start_offset, as2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    int nb_streams, block_size, interleaved, is_extra;
    SABContext *a = s->priv_data;
    AVIOContext *pb = s->pb;
    int64_t stream_offset;
    uint32_t flags;

    avio_skip(pb, 4);
    flags = avio_rl32(pb);
    nb_streams = avio_rl32(pb);
    a->block_size = block_size = avio_rl32(pb);
    a->interleaved = interleaved = !!(flags & 0x04);
    is_extra = !!(flags & 0x10);
    if (nb_streams <= 0 || block_size <= 0)
        return AVERROR_INVALIDDATA;

    avio_seek(pb, 0x18, SEEK_SET);
    for (int n = 0; n < nb_streams; n++) {
        int codec, channels, rate, align;
        SABStream *ast;
        AVStream *st;

        if (interleaved)
            avio_seek(pb, 0x18, SEEK_SET);

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        ast = av_mallocz(sizeof(*ast));
        if (!ast)
            return AVERROR(ENOMEM);
        st->priv_data = ast;

        codec = avio_rl32(pb);
        channels = avio_rl32(pb);
        rate = avio_rl32(pb);
        ast->stop_offset = avio_rl32(pb);
        avio_skip(pb, 8);
        ast->start_offset = avio_rl32(pb);
        ast->stop_offset += ast->start_offset;

        switch (codec) {
        case 1:
            codec = AV_CODEC_ID_PCM_S16LE;
            align = 2;
            break;
        case 4:
            codec = AV_CODEC_ID_ADPCM_PSX;
            align = 16;
            break;
        case 8:
            codec = AV_CODEC_ID_ADPCM_IMA_XBOX;
            align = 36;
            break;
        default:
            avpriv_request_sample(s, "codec %X", codec);
            return AVERROR_PATCHWELCOME;
        }

        if (channels <= 0 || rate <= 0 || channels >= INT_MAX/align)
            return AVERROR_INVALIDDATA;

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = align * channels;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    if (interleaved) {
        stream_offset = block_size;
    } else {
        stream_offset = 0x18 + 0x1c * nb_streams;
        if (stream_offset % block_size)
            stream_offset += block_size - (stream_offset % block_size);
    }

    if (is_extra)
        stream_offset += block_size;

    if (s->nb_streams > 1)
        qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        SABStream *ast = st->priv_data;

        st->index = n;
        ast->start_offset += stream_offset;
        ast->stop_offset += stream_offset;
        if (n == 0 && !interleaved)
            avio_seek(pb, ast->start_offset, SEEK_SET);
    }

    if (interleaved)
        avio_seek(pb, stream_offset, SEEK_SET);

    a->target_stream = 0;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    SABContext *a = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (a->interleaved) {
        pos = avio_tell(pb);
        ret = av_get_packet(pb, pkt, a->block_size);
        pkt->stream_index = ((pos - ffformatcontext(s)->data_offset) / a->block_size) % s->nb_streams;
    } else {
        if (a->target_stream >= s->nb_streams)
            return AVERROR_EOF;

        for (int n = 0; n < s->nb_streams; n++) {
            AVStream *st = s->streams[n];
            AVCodecParameters *par = st->codecpar;
            SABStream *ast = st->priv_data;

            if (avio_feof(pb))
                return AVERROR_EOF;

            pos = avio_tell(pb);
            if (pos >= ast->start_offset && pos < ast->stop_offset) {
                const int block_size = ff_pcm_default_packet_size(par);
                const int size = FFMIN(block_size, ast->stop_offset - pos);

                ret = av_get_packet(pb, pkt, size);

                pkt->pos = pos;
                pkt->stream_index = st->index;

                break;
            } else if (pos >= ast->stop_offset && n+1 < s->nb_streams) {
                AVStream *st_next = s->streams[n+1];
                SABStream *ast_next = st_next->priv_data;

                if (ast_next->start_offset > pos)
                    avio_skip(pb, ast_next->start_offset - pos);
            }
        }
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t timestamp, int flags)
{
    SABContext *a = s->priv_data;

    a->target_stream = FFMAX(0, stream_index);

    return -1;
}

const FFInputFormat ff_csx2_demuxer = {
    .p.name         = "csx2",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sensaura GameCODA SAB"),
    .priv_data_size = sizeof(SABContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "sab",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
