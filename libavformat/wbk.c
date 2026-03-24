/*
 * Treyarch WBK demuxer
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

#include "libavutil/bswap.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavcodec/bytestream.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "WAVEBK11", 8))
        return 0;

    if (p->buf_size < 0x44)
        return 0;
    if ((int)AV_RL32(p->buf+0x40) <= 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

typedef struct WBKStream {
    int64_t start_offset;
    int64_t stop_offset;
} WBKStream;

typedef struct WBKDemuxContext {
    int current_stream;
} WBKDemuxContext;

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const WBKStream *ws1 = s1->priv_data;
    const WBKStream *ws2 = s2->priv_data;

    return FFDIFFSIGN(ws1->start_offset, ws2->start_offset);
}

static const int16_t coef_table[16] =
{
    0x0216, 0xfc9f, 0x026c, 0x04b4, 0x065e, 0xfdec, 0x0a11, 0xfd1e,
    0x0588, 0xfc38, 0x05ad, 0x01da, 0x083b, 0xfdbc, 0x08c3, 0xff18,
};

static int read_header(AVFormatContext *s)
{
    int64_t start_offset, data_offset, table_offset, coefsec_offset, strings_offset;
    AVIOContext *pb = s->pb;
    int nb_streams;

    avio_skip(pb, 16);
    data_offset = avio_rl32(pb);
    avio_skip(pb, 44);
    nb_streams = avio_rl32(pb);
    if (nb_streams <= 0)
        return AVERROR_INVALIDDATA;
    table_offset = avio_rl32(pb);
    avio_skip(pb, 20);
    coefsec_offset = avio_rl32(pb);
    avio_skip(pb, 4);
    strings_offset = avio_rl32(pb);

    for (int n = 0; n < nb_streams; n++) {
        int64_t name_offset, sound_size, duration, sound_offset, coef_offset;
        int codec, flags, rate, channels, align, bps = -1, profile = -1;
        WBKStream *wst;
        AVStream *st;

        avio_seek(pb, table_offset + n * 0x28LL, SEEK_SET);
        if (avio_feof(pb))
            break;

        name_offset = avio_rl32(pb);
        codec = avio_r8(pb);
        flags = avio_r8(pb);
        channels = (avio_r8(pb) == 3) ? 2 : 1;
        avio_skip(pb, 1);
        sound_size = avio_rl32(pb);
        duration = avio_rl32(pb);
        avio_skip(pb, 8);
        coef_offset = avio_rl32(pb);
        sound_offset = avio_rl32(pb);
        rate = avio_rl32(pb);
        if (!(flags & 0x02))
            sound_offset += data_offset;

        switch (codec) {
        case 0x03:
            codec = AV_CODEC_ID_ADPCM_NDSP;
            align = 0x8000;
            break;
        case 0x04:
            codec = AV_CODEC_ID_ADPCM_PSX;
            align = 0x800;
            break;
        case 0x05:
            codec = AV_CODEC_ID_ADPCM_IMA_XBOX;
            align = 0x24;
            break;
        case 0x07:
            codec = AV_CODEC_ID_ADPCM_IMA_WS;
            bps = 4;
            align = 0x40;
            profile = 3;
            break;
        default:
            avpriv_request_sample(s, "codec %02X", codec);
            codec = AV_CODEC_ID_NONE;
            align = 1024;
            break;
        }

        if (channels <= 0 || rate <= 0 || align <= 0 || align > INT_MAX/channels)
            return AVERROR_INVALIDDATA;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        wst = av_mallocz(sizeof(*wst));
        if (!wst)
            return AVERROR(ENOMEM);
        st->priv_data = wst;

        wst->start_offset = sound_offset;
        wst->stop_offset = wst->start_offset + sound_size;

        st->start_time = 0;
        st->duration = duration;
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = codec;
        st->codecpar->ch_layout.nb_channels = channels;
        st->codecpar->sample_rate = rate;
        st->codecpar->block_align = align * channels;
        if (bps > 0)
            st->codecpar->bits_per_coded_sample = bps;
        if (profile >= 0)
            st->codecpar->profile = profile;

        if (codec == AV_CODEC_ID_ADPCM_NDSP) {
            if (coef_offset == UINT32_MAX || coefsec_offset == 0) {
                int ret = ff_alloc_extradata(st->codecpar, 32 * channels);
                if (ret < 0)
                    return ret;

                for (int ch = 0; ch < channels; ch++) {
                    uint8_t *ptr = st->codecpar->extradata + ch * 32;

                    for (int i = 0; i < 32; i++)
                        bytestream_put_be16(&ptr, coef_table[i]);
                }
            } else {
                if (coefsec_offset == UINT32_MAX)
                    return AVERROR_INVALIDDATA;

                int ret = ff_alloc_extradata(st->codecpar, 32 * channels);
                if (ret < 0)
                    return ret;

                avio_seek(pb, coefsec_offset + coef_offset, SEEK_SET);
                for (int ch = 0; ch < channels; ch++) {
                    avio_read(pb, st->codecpar->extradata + ch * 32, 32);
                    avio_skip(pb, 8);
                }
            }
        }

        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

        {
            char title[0x81] = { 0 };
            int ret;

            avio_seek(pb, strings_offset + name_offset, SEEK_SET);
            if ((ret = avio_get_str(pb, 0x81, title, sizeof(title))) < 0)
                return ret;
            if (title[0] != '\0')
                av_dict_set(&st->metadata, "title", title, 0);
        }
    }

    if (s->nb_streams == 0)
        return AVERROR_INVALIDDATA;

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        if (n == 0) {
            WBKStream *wst = st->priv_data;

            start_offset = wst->start_offset;
        }

        st->index = n;
    }

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    WBKDemuxContext *ctx = s->priv_data;
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;
    int do_seek = 0;
    WBKStream *wst;
    AVStream *st;

redo:
    if (avio_feof(pb))
        return AVERROR_EOF;

    if (ctx->current_stream >= s->nb_streams)
        return AVERROR_EOF;

    st = s->streams[ctx->current_stream];
    wst = st->priv_data;
    if (do_seek)
        avio_seek(pb, wst->start_offset, SEEK_SET);

    if (avio_tell(pb) >= wst->stop_offset) {
        do_seek = 1;
        ctx->current_stream++;
        goto redo;
    }

    {
        const int64_t pos = avio_tell(pb);
        const int block_size = ff_pcm_default_packet_size(st->codecpar);
        const int size = FFMIN(block_size, wst->stop_offset - pos);

        ret = av_get_packet(pb, pkt, size);
        pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
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
    WBKDemuxContext *ctx = s->priv_data;
    WBKStream *wst;
    AVStream *st;

    ctx->current_stream = av_clip(stream_index, 0, s->nb_streams-1);
    st = s->streams[ctx->current_stream];
    wst = st->priv_data;
    {
        AVIOContext *pb = s->pb;
        int64_t pos = avio_tell(pb);

        if (pos < wst->start_offset) {
            avio_seek(pb, wst->start_offset, SEEK_SET);
            return 0;
        }

        return -1;
    }
}

const FFInputFormat ff_wbk_demuxer = {
    .p.name         = "wbk",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Treyarch WBK"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(WBKDemuxContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.extensions   = "wbk",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_seek      = read_seek,
};
