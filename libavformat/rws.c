/*
 * RWS demuxer
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

#include "libavutil/bswap.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef unsigned int (*avio_r32)(AVIOContext *s);
typedef unsigned int (*avio_r16)(AVIOContext *s);

static int read_probe(const AVProbeData *p)
{
    uint32_t header_size;

    if (p->buf_size < 20)
        return 0;

    if (AV_RL32(p->buf) != 0x0000080d)
        return 0;
    if (AV_RL32(p->buf+12) != 0x0000080e)
        return 0;

    header_size = AV_RL32(p->buf + 16);
    if (p->buf_size < header_size + 0x0cLL + 0x0cLL + 4LL)
        return 0;
    if (AV_RL32(p->buf+0x0cLL + 0x0cLL + header_size) != 0x0000080f)
        return 0;

    return AVPROBE_SCORE_MAX;
}

typedef struct RWSStream {
    int64_t start_offset;
    int64_t stop_offset;
    uint32_t segment_layer_size;
    uint32_t usable_size;
    int64_t segment_offset;
    uint16_t interleave;
    uint16_t frame_size;
    uint32_t block_size;
    uint32_t layer_start;
} RWSStream;

static int sort_streams(const void *a, const void *b)
{
    const AVStream *const *s1p = a;
    const AVStream *const *s2p = b;
    const AVStream *s1 = *s1p;
    const AVStream *s2 = *s2p;
    const RWSStream *rs1 = s1->priv_data;
    const RWSStream *rs2 = s2->priv_data;

    return FFDIFFSIGN(rs1->start_offset, rs2->start_offset);
}

static int guess_endian32(AVIOContext *pb)
{
    uint8_t buf[4];

    if (avio_read(pb, buf, sizeof(buf)) != sizeof(buf))
        return AVERROR(EIO);

    return AV_RL32(buf) > AV_RB32(buf);
}

static int read_header(AVFormatContext *s)
{
    int big_endian, total_segments, total_layers, nb_streams;
    av_unused int64_t block_layers_size = 0;
    int64_t data_offset, offset;
    av_unused uint32_t data_size;
    uint32_t first_start_offset;
    char title[65] = { 0 };
    AVIOContext *pb = s->pb;
    uint32_t header_size;
    avio_r32 avio_r32;
    avio_r16 avio_r16;
    int ret;

    avio_skip(pb, 16);
    header_size = avio_rl32(pb);
    offset = 0x0c + 0x0c;
    avio_seek(pb, offset, SEEK_SET);
    ret = guess_endian32(pb);
    if (ret < 0)
        return ret;
    big_endian = ret;

    avio_r32 = big_endian ? avio_rb32 : avio_rl32;
    avio_r16 = big_endian ? avio_rb16 : avio_rl16;

    avio_seek(pb, offset + 0x20, SEEK_SET);
    total_segments = avio_r32(pb);
    avio_skip(pb, 4);
    total_layers = avio_r32(pb);

    if (total_segments <= 0 || total_layers <= 0 ||
        total_layers > INT_MAX/total_segments ||
        total_segments > INT_MAX/total_layers)
        return AVERROR_INVALIDDATA;
    data_offset = offset + header_size;
    nb_streams = total_layers * total_segments;

    offset += 0x50;
    avio_seek(pb, offset, SEEK_SET);
    if ((ret = avio_get_str(pb, 1024, title, sizeof(title))) < 0)
        return ret;
    if (title[0] != '\0')
        av_dict_set(&s->metadata, "title", title, 0);
    avio_skip(pb, FFALIGN(ret, 16) - ret);
    offset += FFALIGN(ret, 16);

    for (int n = 0; n < nb_streams; n++) {
        RWSStream *rst;
        AVStream *st;

        st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        rst = av_mallocz(sizeof(*rst));
        if (!rst)
            return AVERROR(ENOMEM);

        st->priv_data = rst;
    }

    for (int i = 0; i < total_segments; i++) {
        uint32_t segment_layer_size;
        int64_t segment_offset;

        avio_skip(pb, 0x18);
        segment_layer_size = avio_r32(pb);
        segment_offset = avio_r32(pb);

        for (int l = 0; l < total_layers; l++) {
            RWSStream *rst;
            AVStream *st;
            int sti = i * total_layers + l;

            st = s->streams[sti];
            rst = st->priv_data;
            rst->segment_layer_size = segment_layer_size;
            rst->segment_offset = segment_offset;
        }

        offset += 0x20;
    }

    for (int i = 0; i < nb_streams; i++) {
        AVStream *st = s->streams[i];
        RWSStream *rst = st->priv_data;

        rst->usable_size = avio_r32(pb);

        offset += 0x04;
    }

    offset += 0x10 * total_segments;
    avio_skip(pb, 16 * total_segments);

    for (int i = 0; i < total_segments; i++) {
        if ((ret = avio_get_str(pb, 1024, title, sizeof(title))) < 0)
            return ret;
        offset += FFALIGN(ret, 16);
        avio_skip(pb, FFALIGN(ret, 16) - ret);
    }

    for (int i = 0; i < total_layers; i++) {
        uint32_t block_size, layer_start;
        uint16_t interleave, frame_size;

        avio_seek(pb, offset, SEEK_SET);

        avio_skip(pb, 16);
        block_layers_size += avio_r32(pb);
        avio_skip(pb, 4);
        interleave = avio_r16(pb);
        frame_size = avio_r16(pb);
        avio_skip(pb, 4);
        block_size = avio_r32(pb);
        layer_start = avio_r32(pb);

        for (int se = 0; se < total_segments; se++) {
            RWSStream *rst;
            AVStream *st;
            int sti = se * total_layers + i;

            st = s->streams[sti];
            rst = st->priv_data;
            rst->interleave = interleave;
            rst->frame_size = frame_size;
            rst->block_size = block_size;
            rst->layer_start = layer_start;
        }

        offset += 0x28;
    }

    for (int i = 0; i < total_layers; i++) {
        int codec, sample_rate, channels;
        int64_t coefs_offset;

        avio_seek(pb, offset, SEEK_SET);

        sample_rate = avio_r32(pb);
        avio_skip(pb, 9);
        channels =  avio_r8(pb);
        if (channels == 0 || sample_rate <= 0)
            return AVERROR_INVALIDDATA;
        avio_skip(pb, 14);
        codec = avio_r32(pb);
        avio_skip(pb, 12);
        offset += 0x2c;

        if (codec == 0xF86215B0) {
            coefs_offset = offset + 0x1c;
            offset += 0x60;
        }

        for (int se = 0; se < total_segments; se++) {
            RWSStream *rst;
            AVStream *st;
            int sti = se * total_layers + i;

            st = s->streams[sti];
            rst = st->priv_data;
            st->start_time = 0;
            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->sample_rate = sample_rate;
            st->codecpar->ch_layout.nb_channels = channels;
            st->codecpar->block_align = rst->block_size;
            rst->start_offset = data_offset + 0x0c + rst->segment_offset + rst->layer_start;
            rst->stop_offset = rst->start_offset;
            rst->stop_offset += rst->usable_size;

            switch (codec) {
            case 0xF86215B0:
                st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;
                avio_seek(pb, coefs_offset, SEEK_SET);

                ret = ff_alloc_extradata(st->codecpar, 32 * channels);
                if (ret < 0)
                    return ret;
                avio_read(pb, st->codecpar->extradata, 32);
                for (int ch = 1; ch < channels; ch++)
                    memcpy(st->codecpar->extradata + ch*32, st->codecpar->extradata, 32);
                break;
            }

            avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);
        }

        offset += 0x04;
    }

    offset += 0x10 * total_layers;

    avio_seek(pb, data_offset, SEEK_SET);
    if (avio_rl32(pb) != 0x0000080f)
        return AVERROR_INVALIDDATA;
    data_size = avio_rl32(pb);

    qsort(s->streams, s->nb_streams, sizeof(AVStream *), sort_streams);
    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];

        if (n == 0) {
            RWSStream *rst = st->priv_data;

            first_start_offset = rst->start_offset;
        }

        st->index = n;
    }

    avio_seek(pb, first_start_offset, SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = AVERROR_EOF;

    for (int n = 0; n < s->nb_streams; n++) {
        AVStream *st = s->streams[n];
        AVCodecParameters *par = st->codecpar;
        RWSStream *rst = st->priv_data;
        int64_t pos;

        if (avio_feof(pb))
            return AVERROR_EOF;

        pos = avio_tell(pb);
        if (pos >= rst->start_offset && pos < rst->stop_offset) {
            const int size = FFMIN(par->block_align, rst->stop_offset - pos);

            ret = av_get_packet(pb, pkt, size);
            pkt->stream_index = st->index;
            break;
        } else if (pos >= rst->stop_offset && n+1 < s->nb_streams) {
            AVStream *st_next = s->streams[n+1];
            RWSStream *rst_next = st_next->priv_data;

            if (rst_next->start_offset > pos)
                avio_seek(pb, rst_next->start_offset, SEEK_SET);
        }
    }
    return ret;
}

const FFInputFormat ff_rws_demuxer = {
    .p.name         = "rws",
    .p.long_name    = NULL_IF_CONFIG_SMALL("RWS (RenderWare Stream)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "rws",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
