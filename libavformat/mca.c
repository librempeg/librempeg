/*
 * MCA demuxer
 * Copyright (c) 2020 Zixing Liu
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
#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) != MKTAG('M', 'A', 'D', 'P'))
        return 0;

    if (p->buf_size < 36)
        return 0;
    if (AV_RL16(p->buf + 4) > 5)
        return 0;
    if (AV_RL16(p->buf + 8) <= 0)
        return 0;
    if (AV_RL16(p->buf + 10) == 0)
        return 0;
    if ((int)AV_RL32(p->buf + 16) <= 0)
        return 0;
    if (AV_RL32(p->buf + 32) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int64_t header_size, data_size, duration, data_start, coef_offset, coef_start;
    int nb_metadata, ret, block_size, nb_channels, rate;
    uint32_t loop_start, loop_end;
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    uint16_t version;
    AVStream *st;

    avio_skip(pb, 4);
    version = avio_rl16(pb);
    avio_skip(pb, 2);
    nb_channels = avio_rl16(pb);
    block_size = avio_rl16(pb);
    duration = avio_rl32(pb);
    rate = avio_rl32(pb);
    loop_start = avio_rl32(pb);
    loop_end = avio_rl32(pb);
    header_size = avio_rl32(pb);
    data_size = avio_rl32(pb);
    avio_skip(pb, 4);
    nb_metadata = avio_rl16(pb);
    avio_skip(pb, 2);

    if (nb_channels <= 0 || rate <= 0 || block_size <= 0 || block_size > INT_MAX/nb_channels)
        return AVERROR_INVALIDDATA;

    coef_start = header_size - 0x30LL * nb_channels;
    coef_offset = coef_start + nb_metadata * 0x14LL;

    switch (version) {
    case 3:
        data_start = header_size;
        break;
    case 4:
        data_start = avio_size(pb) - data_size;
        break;
    case 5:
        avio_seek(pb, coef_start - 4, SEEK_SET);
        data_start = avio_rl32(pb);
        break;
    default:
        avpriv_request_sample(s, "version %d", version);
        return AVERROR_PATCHWELCOME;
    }

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    par = st->codecpar;
    par->codec_type = AVMEDIA_TYPE_AUDIO;
    par->codec_id = AV_CODEC_ID_ADPCM_NDSP_LE;
    par->sample_rate = rate;
    par->ch_layout.nb_channels = nb_channels;
    par->block_align = block_size * nb_channels;

    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    if (loop_end > loop_start) {
        if ((ret = av_dict_set_int(&s->metadata, "loop_start", loop_start, 0)) < 0)
            return ret;
        if ((ret = av_dict_set_int(&s->metadata, "loop_end", loop_end, 0)) < 0)
            return ret;
    }

    ret = ff_alloc_extradata(st->codecpar, 32 * par->ch_layout.nb_channels);
    if (ret < 0)
        return ret;

    avio_seek(pb, coef_offset, SEEK_SET);
    for (int ch = 0; ch < par->ch_layout.nb_channels; ch++) {
        avio_read(pb, par->extradata + ch * 32, 32);
        avio_skip(pb, 0x10);
    }

    avio_seek(pb, data_start, SEEK_SET);

    return 0;
}

const FFInputFormat ff_mca_demuxer = {
    .p.name         = "mca",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Capcom 3DS MCA"),
    .p.extensions   = "mca",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
