/*
 * HDBD demuxer
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

typedef struct HDBDStream {
    int64_t info_offset;
    int64_t start_offset;
    int64_t stop_offset;
} HDBDStream;

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 24)
        return 0;
    if (memcmp(p->buf, "IECSsreV", 8))
        return 0;
    if (memcmp(p->buf+16, "IECSdaeH", 8))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const HDBDStream *hs1 = s1->priv_data;
    const HDBDStream *hs2 = s2->priv_data;

    return FFDIFFSIGN(hs1->start_offset, hs2->start_offset);
}

static int sorti_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const HDBDStream *hs1 = s1->priv_data;
    const HDBDStream *hs2 = s2->priv_data;

    return FFDIFFSIGN(hs1->info_offset, hs2->info_offset);
}

static int read_header(AVFormatContext *s)
{
    int64_t hd_size, bd_size, vagi_offset;
    AVIOContext *pb = s->pb;
    int ret, nb_streams;

    avio_skip(pb, 28);
    hd_size = avio_rl32(pb);
    bd_size = avio_rl32(pb);
    avio_skip(pb, 12);
    vagi_offset = avio_rl32(pb);
    avio_seek(pb, vagi_offset, SEEK_SET);
    if (avio_rb64(pb) != AV_RB64("IECSigaV"))
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4);
    nb_streams = avio_rl32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;
    nb_streams++;

    for (int n = 0; n < nb_streams; n++) {
        HDBDStream *hst;
        AVStream *st;
        int64_t off;

        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        off = avio_rl32(pb);
        if (off == 0 || off >= bd_size)
            continue;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        hst = av_mallocz(sizeof(*hst));
        if (!hst)
            return AVERROR(ENOMEM);
        hst->info_offset = off + vagi_offset;
        st->priv_data = hst;
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sorti_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        HDBDStream *hst = st->priv_data;
        uint8_t flags0, flags1;
        int rate;

        avio_seek(pb, hst->info_offset, SEEK_SET);
        if (avio_feof(pb))
            return AVERROR_INVALIDDATA;

        hst->start_offset = avio_rl32(pb);
        rate = avio_rl16(pb);
        flags0 = avio_r8(pb);
        flags1 = avio_r8(pb);
        if (rate == 0 || flags0 > 0x1 || (flags1 != 0 && flags1 != 0xff))
            return AVERROR_INVALIDDATA;

        st->start_time = 0;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;
        st->codecpar->ch_layout.nb_channels = 1;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = 2048;

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        HDBDStream *hst = st->priv_data;

        st->index = n;
        if (n + 1 < s->nb_streams) {
            AVStream *next_st = s->streams[n+1];
            HDBDStream *next_hst = next_st->priv_data;

            hst->stop_offset = next_hst->start_offset;
        } else {
            hst->stop_offset = bd_size;
        }
    }

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        HDBDStream *hst = st->priv_data;

        st->duration = ((hst->stop_offset - hst->start_offset) / 16) * 28LL;
    }

    if (avio_size(pb) >= hd_size+bd_size) {
        for (int n = 0; n < s->nb_streams; n++) {
            AVStream *st = s->streams[n];
            HDBDStream *hst = st->priv_data;

            hst->start_offset += hd_size;
            hst->stop_offset += hd_size;
        }

        avio_seek(pb, hd_size, SEEK_SET);
    } else {
        extern const FFInputFormat ff_hdbd_demuxer;
        char *bd_file_name = av_strdup(s->url);
        AVDictionary *tmp = NULL;
        int len;

        if (!bd_file_name)
            return AVERROR(ENOMEM);

        len = strlen(bd_file_name);
        if (len > 3) {
            bd_file_name[len-2] = 'b';
        } else {
            return AVERROR_INVALIDDATA;
        }

        ret = s->io_close2(s, s->pb);
        if (ret < 0)
            return ret;
        ret = s->io_open(s, &s->pb, bd_file_name, AVIO_FLAG_READ, &tmp);
        av_freep(&bd_file_name);
        if (ret < 0)
            return ret;
    }

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
        HDBDStream *hst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= hst->start_offset && pos < hst->stop_offset) {
            const int size = FFMIN(par->block_align, hst->stop_offset - pos);

            ret = av_get_packet(pb, pkt, size);

            pkt->pos = pos;
            pkt->stream_index = st->index;

            break;
        } else if (pos >= hst->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            HDBDStream *pst_next = st_next->priv_data;

            if (pst_next->start_offset > pos)
                avio_skip(pb, pst_next->start_offset - pos);
        }
    }

    return ret;
}

const FFInputFormat ff_hdbd_demuxer = {
    .p.name         = "hdbd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Sony HD+BD"),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "hd,hbd",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
