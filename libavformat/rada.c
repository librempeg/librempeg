/*
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
#include "libavcodec/mathops.h"

#include "avio_internal.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct RADAContext {
    int bias;
} RADAContext;

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKTAG('R','A','D','A'))
        return 0;
    if (p->buf[9] == 0)
        return 0;
    if (p->buf[15] > 3)
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int ret, channels, rate, seek_count, bits_seek_bytes, bits_seek_samples, seek_tab_size;
    RADAContext *rada = s->priv_data;
    AVIOContext *pb = s->pb;
    uint64_t header = 0;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 9);
    channels = avio_r8(pb);
    avio_skip(pb, 3);
    bits_seek_bytes = avio_r8(pb);
    bits_seek_samples = avio_r8(pb);
    rate = avio_r8(pb);
    duration = avio_rl64(pb);
    avio_skip(pb, 8);
    seek_count = avio_rl16(pb);
    seek_tab_size = 8 * (((bits_seek_bytes + bits_seek_samples) * seek_count + 63) / 64);
    switch (rate) {
    case 0:
        rate = 24000;
        break;
    case 1:
        rate = 32000;
        break;
    case 2:
        rate = 44100;
        break;
    case 3:
        rate = 48000;
        break;
    default:
        rate = 0;
        break;
    }

    if (channels <= 0 || rate <= 0)
        return AVERROR_INVALIDDATA;

    while (!avio_feof(pb)) {
        header <<= 8;
        header |= avio_r8(pb);

        if (!memcmp(&header, "\032duAdaR`", 8))
            break;
    }

    if (avio_feof(pb))
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_RADA;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->sample_rate = rate;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if ((ret = ff_get_extradata(s, st->codecpar, pb, 120)) < 0)
        return ret;

    rada->bias = AV_RL16(st->codecpar->extradata + 6);

    avio_skip(pb, seek_tab_size);

    return 0;
}

static int parity(uint32_t f)
{
    f = ((f>>1) & 0x5555) + (f & 0x5555);
    f = ((f>>2) & 0x3333) + (f & 0x3333);
    return ((f>>4) & 0x0f0f) + (f & 0x0f0f);
}

static int get_block_size(AVFormatContext *s, AVIOContext *pb, int *block_size)
{
    RADAContext *rada = s->priv_data;
    int block_bytes_size, stream_block_size, runlen_block_size;
    uint8_t first = avio_r8(pb);

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (first == 0x60)
        return AVERROR_INVALIDDATA;

    if (first != 0x55)
        return AVERROR_INVALIDDATA;

    first = avio_r8(pb);

    if (!(parity(first) & 1))
        return AVERROR_INVALIDDATA;

    uint32_t flags = first;
    if (flags & 0x80)
        flags |= (avio_r8(pb) << 8);

    int short_block  = !!(flags & (1U <<  1));
    int stereo_block = !!(flags & (1U <<  3));
    int final_block  = !!(flags & (1U << 15));
    int block_bytes  = !!(flags & (1U << 14));
    int stream_block = !!(flags & (1U << 13));
    int runlen_block = !!(flags & (1U << 12));

    if (block_bytes) {
        block_bytes_size = avio_rl16(pb);
        if (block_bytes_size < 256)
            return AVERROR_INVALIDDATA;
    } else {
        block_bytes_size = avio_r8(pb);
    }

    if (runlen_block) {
        runlen_block_size = avio_rl16(pb);
        if (runlen_block_size < 256)
            return AVERROR_INVALIDDATA;
    } else {
        runlen_block_size = avio_r8(pb);
    }

    if (stream_block) {
        stream_block_size = avio_rl16(pb);
        if (stream_block_size < 256)
            return AVERROR_INVALIDDATA;
    } else {
        stream_block_size = avio_r8(pb);
    }

    if (final_block)
        avio_rl16(pb);

    block_size[0] = sign_extend(block_bytes_size, 16);
    if (!short_block)
        block_size[0] += rada->bias >> (!stereo_block);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int block_size = 0;
    int header_size;
    int64_t pos;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    ret = ffio_ensure_seekback(pb, 11);
    if (ret < 0)
        return ret;

    ret = get_block_size(s, pb, &block_size);
    if (ret < 0)
        return ret;
    if (block_size <= 0)
        return AVERROR_INVALIDDATA;
    header_size = avio_tell(pb) - pos;
    if (header_size <= 0)
        return AVERROR_INVALIDDATA;
    avio_seek(pb, pos, SEEK_SET);

    ret = av_get_packet(pb, pkt, header_size + block_size);
    pkt->stream_index = 0;
    return ret;
}

const FFInputFormat ff_rada_demuxer = {
    .p.name         = "rada",
    .p.long_name    = NULL_IF_CONFIG_SMALL("RAD Audio"),
    .p.extensions   = "rada",
    .priv_data_size = sizeof(RADAContext),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
