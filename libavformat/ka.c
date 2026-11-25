/*
 * KA Knowledge Adventure demuxer
 *
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

#include "libavutil/channel_layout.h"
#include "libavutil/internal.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 15)
        return 0;

    if (memcmp(p->buf, "KA Sound File", 13))
        return 0;
    if (p->buf[14] != 1 && p->buf[14] != 2)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    int align, codec, channels, rate, bps, version, extra, ret;
    AVIOContext *pb = s->pb;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 14);
    version = avio_r8(pb);
    if (version != 1 && version != 2)
        return AVERROR_INVALIDDATA;
    codec = avio_r8(pb);
    channels = avio_r8(pb);
    avio_skip(pb, 2);
    bps = avio_r8(pb);
    rate = avio_r8(pb);
    avio_skip(pb, 11);
    duration = avio_rl32(pb);
    avio_skip(pb, 16);
    extra = avio_rl16(pb);

    switch (rate) {
    case 1:
        rate = 11025;
        break;
    case 2:
        rate = 22050;
        break;
    case 3:
        rate = 44100;
        break;
    default:
        rate = 0;
    }

    switch (codec) {
    case 1:
        codec = (bps == 8) ? AV_CODEC_ID_PCM_U8 : AV_CODEC_ID_PCM_S16LE;
        align = (bps == 8) ? 1 : 2;
        break;
    default:
        avpriv_request_sample(s, "codec %d", codec);
        return AVERROR_PATCHWELCOME;
    }

    if (channels <= 0 || rate <= 0 || bps <= 0 || align <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->duration = duration;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = codec;
    st->codecpar->bits_per_coded_sample = bps;
    st->codecpar->ch_layout.nb_channels = channels;
    st->codecpar->block_align = align * channels;
    st->codecpar->sample_rate = rate;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if (extra > 0) {
        avio_seek(pb, 0xe, SEEK_SET);
        ret = ff_get_extradata(s, st->codecpar, pb, extra + 0x36 - 0xe);
        if (ret < 0)
            return ret;
    }

    return 0;
}

const FFInputFormat ff_ka_demuxer = {
    .p.name         = "ka",
    .p.long_name    = NULL_IF_CONFIG_SMALL("KA (Knowledge Adventure)"),
    .p.extensions   = "snd",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
};
