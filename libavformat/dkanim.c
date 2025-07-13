/*
 * DK Animation demuxer
 *
 * Copyright (C) 2025 Paul B Mahol
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
#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/internal.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"

typedef struct DKAnimDemuxContext {
    int image_factor;
    int audio_size;
    int got_audio;
} DKAnimDemuxContext;

static int dkanim_probe(const AVProbeData *p)
{
    int score = 0;

    if (AV_RL16(p->buf) != 1)
        return 0;

    if (AV_RL16(p->buf+2) == 0)
        return 0;

    for (int i = 4; i+6 < p->buf_size; i++) {
        if (!memcmp(p->buf+i, "funky!", 6)) {
            score += 2;
            if (score >= AVPROBE_SCORE_MAX)
                break;
        }
    }

    return score;
}

static int dkanim_read_header(AVFormatContext *s)
{
    int audio_size, sample_rate, codec, nb_channels, bps, ba;
    DKAnimDemuxContext *d = s->priv_data;
    AVIOContext *pb = s->pb;
    AVStream *st;

    avio_skip(pb, 8);

    d->image_factor = avio_rl16(pb);
    audio_size = avio_rl16(pb);
    avio_skip(pb, 2);
    codec = avio_rl16(pb);
    nb_channels = avio_rl16(pb);
    sample_rate = avio_rl32(pb);
    avio_skip(pb, 4);
    ba = avio_rl16(pb);
    bps = avio_rl16(pb);

    if (nb_channels <= 0 || sample_rate <= 0 || bps <= 0 || ba <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->block_align = ba;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->bits_per_coded_sample = bps;
    switch (codec) {
    case 1:
        st->codecpar->codec_id = ff_get_pcm_codec_id(st->codecpar->bits_per_coded_sample,
                                                     0, 0, 0);
        break;
    case 2:
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_MS;
        break;
    default:
        avpriv_request_sample(s, "codec %d", codec);
        return AVERROR_PATCHWELCOME;
    }
    st->codecpar->ch_layout.nb_channels = nb_channels;
    st->codecpar->sample_rate = sample_rate;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, 0xE + audio_size, SEEK_SET);

    return 0;
}

static int dkanim_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret = 0, audio_size, video_size;
    int64_t pos = avio_tell(pb);

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (avio_rl16(pb) != 1)
        return AVERROR_INVALIDDATA;
    audio_size = avio_rl32(pb);
    avio_skip(pb, 8);
    avio_skip(pb, 4);
    video_size = avio_rl32(pb);
    avio_skip(pb, 6);

    if (audio_size > 0)
        ret = av_get_packet(pb, pkt, audio_size);
    avio_skip(pb, video_size);

    pkt->stream_index = 0;
    pkt->pos = pos;

    return ret;
}

const FFInputFormat ff_dkanim_demuxer = {
    .p.name         = "dkanim",
    .p.long_name    = NULL_IF_CONFIG_SMALL("DK Animation"),
    .p.extensions   = "ani",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(DKAnimDemuxContext),
    .read_probe     = dkanim_probe,
    .read_header    = dkanim_read_header,
    .read_packet    = dkanim_read_packet,
};
