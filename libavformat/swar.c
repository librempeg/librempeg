/*
 * SWAR demuxer
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
#include "demux.h"
#include "internal.h"

typedef struct SWARStream {
    int64_t start_offset;
    int64_t stop_offset;
} SWARStream;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('S','W','A','R'))
        return 0;

    if (p->buf_size < 60)
        return 0;

    if (AV_RB16(p->buf+4) != 0xFFFE)
        return 0;

    if (AV_RB16(p->buf+6) != 0x1)
        return 0;

    if ((int32_t)AV_RL32(p->buf+8) <= 0)
        return 0;

    if (AV_RB32(p->buf+16) != MKBETAG('D','A','T','A'))
        return 0;

    if ((int32_t)AV_RL32(p->buf+56) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const SWARStream *ps1 = s1->priv_data;
    const SWARStream *ps2 = s2->priv_data;

    return FFDIFFSIGN(ps1->start_offset, ps2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    int ret, nb_streams;
    int64_t start, size;

    avio_skip(pb, 0x8);
    size = avio_rl32(pb);
    avio_skip(pb, 0x2c);
    nb_streams = avio_rl32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    for (int si = 0; si < nb_streams; si++) {
        int64_t head_offset, loop_start, loop_stop;
        int codec, rate, loop;
        SWARStream *sst;
        AVStream *st;

        avio_seek(pb, 0x3c + 4LL * si, SEEK_SET);
        head_offset = avio_rl32(pb);
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        avio_seek(pb, head_offset, SEEK_SET);

        codec = avio_r8(pb);
        loop = avio_r8(pb) != 0;
        rate = avio_rl16(pb);
        if (rate <= 0)
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 2);
        loop_start = avio_rl16(pb);
        loop_stop = avio_rl32(pb);

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        sst = av_mallocz(sizeof(*sst));
        if (!sst)
            return AVERROR(ENOMEM);
        st->priv_data = sst;

        sst->start_offset = head_offset + 0x10;
        if (si > 0) {
            SWARStream *prev_sst = s->streams[si-1]->priv_data;

            prev_sst->stop_offset = sst->start_offset;
        }

        if (si == nb_streams-1)
            sst->stop_offset = size;

        if (si == 0)
            start = sst->start_offset;

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->ch_layout.nb_channels = 1;
        st->codecpar->sample_rate = rate;

        switch (codec) {
        case 0:
            st->codecpar->codec_id = AV_CODEC_ID_PCM_S8;
            st->codecpar->bits_per_coded_sample = 8;
            st->codecpar->block_align = 1024 * st->codecpar->ch_layout.nb_channels;
            break;
        case 1:
            st->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
            st->codecpar->bits_per_coded_sample = 16;
            st->codecpar->block_align = 512 * st->codecpar->ch_layout.nb_channels;
            break;
        case 2:
            st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_WS;
            st->codecpar->bits_per_coded_sample = 4;
            st->codecpar->block_align = 0x40 * st->codecpar->ch_layout.nb_channels;
            ret = ff_alloc_extradata(st->codecpar, 2);
            if (ret < 0)
                return ret;
            AV_WL16(st->codecpar->extradata, 3);
            break;
        default:
            avpriv_request_sample(s, "codec 0x%X", codec);
            return AVERROR_PATCHWELCOME;
        }

        if (loop) {
            loop_start = loop_start * 32 / st->codecpar->bits_per_coded_sample;
            loop_stop = loop_stop * 32 / st->codecpar->bits_per_coded_sample + loop_start;
            if (codec == 2) {
                loop_start -= 32 / st->codecpar->bits_per_coded_sample;
                loop_stop -= 32 / st->codecpar->bits_per_coded_sample;
            }

            av_dict_set_int(&st->metadata, "loop_start", loop_start, 0);
            av_dict_set_int(&st->metadata, "loop_stop", loop_stop, 0);
        }

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        SWARStream *pst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= pst->start_offset && pos < pst->stop_offset) {
            const int size = FFMIN(par->block_align, pst->stop_offset - pos);

            ret = av_get_packet(pb, pkt, size);

            pkt->pos = pos;
            pkt->stream_index = st->index;

            break;
        } else if (pos >= pst->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            SWARStream *pst_next = st_next->priv_data;

            if (pst_next->start_offset > pos)
                avio_skip(pb, pst_next->start_offset - pos);
        }
    }

    return ret;
}

const FFInputFormat ff_swar_demuxer = {
    .p.name         = "swar",
    .p.long_name    = NULL_IF_CONFIG_SMALL("SWAR (Nintendo DS SWAR)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "swar",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
