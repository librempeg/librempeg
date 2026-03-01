/*
 * 2dx demuxer
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
#include "demux.h"
#include "riff.h"

typedef struct TwoDXStream {
    int64_t start_offset;
    int64_t stop_offset;
} TwoDXStream;

static int twodx9_probe_sub(const AVProbeData *p, uint32_t off)
{
    if (AV_RL32(p->buf + off + 0x00) != MKTAG('2','D','X','9') || AV_RL32(p->buf + off + 0x04) != 0x18 ||
        AV_RL32(p->buf + off + 0x18) != MKTAG('R','I','F','F') || AV_RL32(p->buf + off + 0x20) != MKTAG('W','A','V','E') ||
        AV_RL32(p->buf + off + 0x24) != MKTAG('f','m','t',' ') || AV_RL32(p->buf + off + 0x5E) != MKTAG('f','a','c','t') ||
        AV_RL32(p->buf + off + 0x62) != 4                      || AV_RL32(p->buf + off + 0x6A) != MKTAG('d','a','t','a'))
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int twodx9_probe(const AVProbeData *p)
{
    if (p->buf_size < 0x6E)
        return 0;

    return twodx9_probe_sub(p, 0);
}

static int twodx_probe(const AVProbeData *p)
{
    if (p->buf_size < 0x4C)
        return 0;

    if (AV_RL32(p->buf + 0x10) != AV_RL32(p->buf + 0x48) || AV_RL32(p->buf + 0x10) != (0x48 + AV_RL32(p->buf + 0x14) * 4))
        return 0;

    if (AV_RL32(p->buf + 0x10) > p->buf_size - 0x6E)
        return 0;

    return twodx9_probe_sub(p, AV_RL32(p->buf + 0x10));
}

static int twodx9_read_stream(AVFormatContext *s, AVStream **stp, int64_t start_offset)
{
    int64_t tst_start_offset, tst_stop_offset, wav_offset;
    AVCodecParameters par = {0};
    AVIOContext *pb = s->pb;
    uint32_t loop_start;
    TwoDXStream *tst;
    AVStream *st;
    int ret;

    avio_seek(pb, start_offset, SEEK_SET);
    if (avio_rl32(pb) != MKTAG('2','D','X','9'))
        return AVERROR_INVALIDDATA;

    wav_offset = avio_rl32(pb);
    tst_start_offset = start_offset + 0x5A + wav_offset;

    avio_skip(pb, 12);
    loop_start = avio_rl32(pb);

    avio_skip(pb, wav_offset - 0x18);
    if (avio_rl32(pb) != MKTAG('R','I','F','F'))
        return AVERROR_INVALIDDATA;
    avio_skip(pb, 4);
    if (avio_rl32(pb) != MKTAG('W','A','V','E') || avio_rl32(pb) != MKTAG('f','m','t',' '))
        return AVERROR_INVALIDDATA;

    par.format = -1;
    av_channel_layout_uninit(&par.ch_layout);
    if ((ret = ff_get_wav_header(s, pb, &par, avio_rl32(pb), 0)) < 0)
        return ret;

    if (par.codec_id != AV_CODEC_ID_ADPCM_MS) {
        avpriv_request_sample(s, "codec id isn't MSADPCM");
        ret = AVERROR_PATCHWELCOME;
        goto fail;
    }

    if (par.codec_id == AV_CODEC_ID_ADPCM_MS &&
        par.ch_layout.nb_channels > 2 &&
        par.block_align < INT_MAX / par.ch_layout.nb_channels)
        par.block_align *= par.ch_layout.nb_channels;

    if (avio_rl32(pb) != MKTAG('f','a','c','t') || avio_rl32(pb) != 4) {
        ret = AVERROR_INVALIDDATA;
        goto fail;
    }
    avio_skip(pb, 4);

    if (avio_rl32(pb) != MKTAG('d','a','t','a'))
        return AVERROR_INVALIDDATA;
    tst_stop_offset = avio_rl32(pb);

    stp[0] = avformat_new_stream(s, NULL);
    if (!stp[0]) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    st = stp[0];
    if (loop_start > 0)
        av_dict_set_int(&st->metadata, "loop_start", loop_start / 2 / par.ch_layout.nb_channels, 0);

    tst = av_mallocz(sizeof(TwoDXStream));
    if (!tst) {
        ret = AVERROR(ENOMEM);
        goto fail;;
    }
    tst->start_offset = tst_start_offset;
    tst->stop_offset = tst->start_offset + tst_stop_offset;

    st->start_time = 0;
    st->priv_data = tst;
    ret = avcodec_parameters_copy(st->codecpar, &par);
    av_freep(&par.extradata);
    if (ret < 0)
        return ret;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
fail:
    av_freep(&par.extradata);
    return ret;
}

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const TwoDXStream *ts1 = s1->priv_data;
    const TwoDXStream *ts2 = s2->priv_data;

    return FFDIFFSIGN(ts1->start_offset, ts2->start_offset);
}

static int twodx_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    char title[0x10];
    int nb_streams;
    int64_t pos;
    int ret;

    if ((ret = avio_get_str(pb, 0x10, title, sizeof(title))) < 0)
        return ret;
    av_dict_set(&s->metadata, "title", title, 0);

    avio_seek(pb, 0x14, SEEK_SET);
    nb_streams = avio_rl32(pb);
    if (nb_streams < 1)
        return AVERROR_INVALIDDATA;

    avio_skip(pb, 0x30);
    for (int i = 0; i < nb_streams; i++) {
        int64_t offset = avio_rl32(pb);
        AVStream *st;

        pos = avio_tell(pb);
        if ((ret = twodx9_read_stream(s, &st, offset)) < 0)
            return ret;
        avio_seek(pb, pos, SEEK_SET);
    }

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        st->index = n;
    }

    {
        AVStream *st = s->streams[0];
        TwoDXStream *tst = st->priv_data;

        avio_seek(pb, tst->start_offset, SEEK_SET);
    }

    return 0;
}

static int twodx9_read_header(AVFormatContext *s)
{
    AVStream *st;
    int ret;

    if ((ret = twodx9_read_stream(s, &st, 0)) < 0)
        return ret;

    {
        AVStream *st = s->streams[0];
        TwoDXStream *tst = st->priv_data;

        avio_seek(s->pb, tst->start_offset, SEEK_SET);
    }
    return 0;
}

static int twodx_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int64_t pos, block_size;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;

    for (int i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        TwoDXStream *tst = st->priv_data;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= tst->start_offset && pos < tst->stop_offset) {
            block_size = FFMIN(tst->stop_offset - pos, st->codecpar->block_align);

            ret = av_get_packet(pb, pkt, block_size);
            pkt->pos = pos;
            pkt->stream_index = st->index;
            break;
        } else if (pos >= tst->stop_offset && i+1 < s->nb_streams) {
            AVStream *st_next = s->streams[i+1];
            TwoDXStream *tst_next = st_next->priv_data;
            if (tst_next->start_offset > pos)
                avio_seek(pb, tst_next->start_offset, SEEK_SET);
        }
    }

    return ret;
}

const FFInputFormat ff_twodx_demuxer = {
    .p.name         = "2dx",
    .p.long_name    = NULL_IF_CONFIG_SMALL("2DX (Konami)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "2dx",
    .read_probe     = twodx_probe,
    .read_header    = twodx_read_header,
    .read_packet    = twodx_read_packet,
};

const FFInputFormat ff_twodx9_demuxer = {
    .p.name         = "2dx9",
    .p.long_name    = NULL_IF_CONFIG_SMALL("2DX9 (Konami)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "2dx9",
    .read_probe     = twodx9_probe,
    .read_header    = twodx9_read_header,
    .read_packet    = twodx_read_packet,
};
