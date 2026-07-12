/*
 * PPHD demuxer
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

typedef struct PPHDContext {
    AVClass     *class;
    AVIOContext *pb;
} PPHDContext;

typedef struct PPHDStream {
    int64_t start_offset;
    int64_t stop_offset;
} PPHDStream;

static int read_probe(const AVProbeData *p)
{
    int64_t offset;

    if (AV_RB32(p->buf) != MKBETAG('P','P','H','D'))
        return 0;

    if (p->buf_size < 28)
        return 0;
    offset = AV_RL32(p->buf+24);
    if (offset + 4 > p->buf_size)
        return 0;
    if (AV_RB32(p->buf+offset) != MKBETAG('P','P','V','A'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const PPHDStream *ps1 = s1->priv_data;
    const PPHDStream *ps2 = s2->priv_data;

    return FFDIFFSIGN(ps1->start_offset, ps2->start_offset);
}

static int read_header(AVFormatContext *s)
{
    int64_t offset, max_offset;
    AVIOContext *pb = s->pb;
    int ret, nb_streams;

    avio_skip(pb, 24);
    offset = avio_rl32(pb);
    avio_seek(pb, offset + 20, SEEK_SET);
    nb_streams = avio_rl32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;
    max_offset = offset + 32 + 16 * nb_streams;
    if (avio_size(pb) > max_offset)
        nb_streams++;

    avio_seek(pb, offset + 32, SEEK_SET);
    for (int n = 0; n < nb_streams; n++) {
        int64_t stream_offset, stream_size;
        PPHDStream *pst;
        uint32_t flags;
        AVStream *st;
        int rate;

        if (avio_feof(pb))
            break;

        stream_offset = avio_rl32(pb);
        rate = avio_rl32(pb);
        stream_size = avio_rl32(pb);
        flags = avio_rl32(pb);
        if (rate <= 0 || stream_size <= 0 || flags != 0xFFFFFFFFU)
            continue;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        pst = av_mallocz(sizeof(*pst));
        if (!pst)
            return AVERROR(ENOMEM);
        st->priv_data = pst;
        pst->start_offset = stream_offset;
        pst->stop_offset = pst->start_offset;
        pst->stop_offset += stream_size;

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
        st->codecpar->ch_layout.nb_channels = 1;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = 2048;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    if (s->nb_streams <= 0)
        return AVERROR_INVALIDDATA;

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        PPHDStream *pst = st->priv_data;

        st->duration = ((pst->stop_offset - pst->start_offset) / 16) * 28LL;
    }

    {
        PPHDContext *pphd = s->priv_data;
        char *pbd_file_name = av_strdup(s->url);
        AVDictionary *tmp = NULL;
        int len;

        if (!pbd_file_name)
            return AVERROR(ENOMEM);

        len = strlen(pbd_file_name);
        if (len > 3) {
            if (pbd_file_name[len-2] == 'h')
                pbd_file_name[len-2] = 'b';
            else
                pbd_file_name[len-2] = 'B';
        } else {
            return AVERROR_INVALIDDATA;
        }

        ret = s->io_open(s, &pphd->pb, pbd_file_name, AVIO_FLAG_READ, &tmp);
        av_freep(&pbd_file_name);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    PPHDContext *pphd = s->priv_data;
    AVIOContext *pb = pphd->pb;
    int ret = AVERROR_EOF;
    int64_t pos;

    if (avio_feof(pb))
        return AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        PPHDStream *pst = st->priv_data;

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
            PPHDStream *pst_next = st_next->priv_data;

            if (pst_next->start_offset > pos)
                avio_skip(pb, pst_next->start_offset - pos);
        }
    }

    return ret;
}

static int read_seek(AVFormatContext *s, int stream_index,
                     int64_t ts, int flags)
{
    PPHDContext *pphd = s->priv_data;
    AVIOContext *pb = pphd->pb;
    const int sti = av_clip(stream_index, 0, s->nb_streams-1);
    AVStream *st = s->streams[sti];
    PPHDStream *pst = st->priv_data;
    int block_align, byte_rate;
    int64_t pos;

    if (ts < 0)
        ts = 0;

    block_align = st->codecpar->block_align;
    byte_rate = 16LL * st->codecpar->ch_layout.nb_channels *
                       st->codecpar->sample_rate / 28;
    pos = av_rescale_rnd(ts * byte_rate,
                         st->time_base.num,
                         st->time_base.den * (int64_t)block_align,
                         (flags & AVSEEK_FLAG_BACKWARD) ? AV_ROUND_DOWN : AV_ROUND_UP);
    pos *= block_align;
    ffstream(st)->cur_dts = av_rescale(pos, st->time_base.den, byte_rate * (int64_t)st->time_base.num);
    avio_seek(pb, pst->start_offset + pos, SEEK_SET);

    return 0;
}

static int read_close(AVFormatContext *s)
{
    PPHDContext *pphd = s->priv_data;

    s->io_close2(s, pphd->pb);

    return 0;
}

const FFInputFormat ff_pphd_demuxer = {
    .p.name         = "pphd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony PSP PHD+PBD"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .priv_data_size = sizeof(PPHDContext),
    .p.extensions   = "phd",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
    .read_close     = read_close,
};
