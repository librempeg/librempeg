/*
 * Beyond Reality SDF demuxer
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

static int read_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "SDF\0\3\0\0\0", 8))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int rate, channels = 0, codec = AV_CODEC_ID_NONE, value = 0, align = 0, profile = -1;
    int64_t start, data_size, offset = 0, bit_rate = 0, full_size;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 8);
    data_size = avio_rl32(pb);
    full_size = avio_size(pb);
    start = full_size - data_size;
    value = avio_rl32(pb);
    switch (start) {
    case 0x18:
        rate = avio_rl32(pb);
        if (rate > 6) {
            codec = avio_r8(pb);
            channels = avio_r8(pb);
            align = avio_rl16(pb);

            switch (codec) {
            case 0:
                codec = AV_CODEC_ID_PCM_S8;
                break;
            case 1:
                codec = AV_CODEC_ID_PCM_S16LE;
                break;
            case 2:
                codec = AV_CODEC_ID_ADPCM_IMA_WS;
                profile = 3;
                break;
            default:
                codec = AV_CODEC_ID_NONE;
                break;
            }
        } else {
            rate = value;
            channels = rate;
            align = avio_rl32(pb);
            codec = AV_CODEC_ID_ADPCM_PSX;
        }
        break;
    case 0x78:
        rate = avio_rl32(pb);
        channels = avio_rl32(pb);
        align = avio_rl32(pb);
        codec = AV_CODEC_ID_ADPCM_NDSP_LE;
        offset = 0x1c;
        if (channels > 0 && rate > 0) {
            if (align == 0)
                align = data_size / channels;
            bit_rate = 8LL * channels * 8 * rate / 14;
        }
        break;
    case 0x84:
        rate = avio_rl32(pb);
        channels = avio_rl32(pb);
        align = avio_rl32(pb);
        data_size = avio_rl32(pb);
        codec = AV_CODEC_ID_ADPCM_NDSP_LE;
        offset = 0x28;
        if (channels > 0 && rate > 0)
            bit_rate = 8LL * channels * 8 * rate / 14;
        break;
    }

    if (codec == AV_CODEC_ID_NONE || rate <= 0 || channels <= 0 || align <= 0 || channels >= INT_MAX/align)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->ch_layout.nb_channels = channels;
    if (profile >= 0)
        st->codecpar->profile = profile;
    if (bit_rate > 0)
        st->codecpar->bit_rate = bit_rate;
    st->codecpar->sample_rate = rate;
    st->codecpar->block_align = align * channels;

    if (offset > 0) {
        int ret = ff_alloc_extradata(st->codecpar, 32 * channels);
        if (ret < 0)
            return ret;

        avio_seek(pb, offset, SEEK_SET);
        for (int ch = 0; ch < channels; ch++) {
            avio_read(pb, st->codecpar->extradata + 32 * ch, 32);
            avio_skip(pb, 0xe);
        }
    }

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_sdf_demuxer = {
    .p.name         = "sdf",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Beyond Reality SDF"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "sdf",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};

