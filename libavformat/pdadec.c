/*
 * PDA demuxer
 * Copyright (c) 2024 Paul B Mahol
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
#include "demux.h"
#include "internal.h"
#include "pcm.h"

#define PDA_MAGIC "Playdate AUD"

static int pda_probe(const AVProbeData *pd)
{
    if (memcmp(pd->buf, PDA_MAGIC, sizeof(PDA_MAGIC)-1))
        return 0;
    if (AV_RL24(pd->buf + 12) == 0)
        return 0;
    if (pd->buf[15] > 5)
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int pda_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    AVStream *st;
    int type;

    avio_skip(pb, sizeof(PDA_MAGIC)-1);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    par              = st->codecpar;
    par->codec_type  = AVMEDIA_TYPE_AUDIO;
    par->codec_id    = AV_CODEC_ID_PCM_S16LE;
    st->start_time   = 0;
    par->sample_rate = avio_rl24(pb);
    type             = avio_r8(pb);
    par->ch_layout.nb_channels = 1 + (type & 1);

    switch (type >> 1) {
    case 0:
        par->codec_id = AV_CODEC_ID_PCM_S8;
        par->bits_per_coded_sample = 8;
        break;
    case 1:
        par->codec_id = AV_CODEC_ID_PCM_S16LE;
        par->bits_per_coded_sample = 16;
        break;
    case 2:
        par->codec_id = AV_CODEC_ID_ADPCM_IMA_PDA;
        par->bits_per_coded_sample = 4;
        break;
    }

    if (type > 3) {
        par->block_align = avio_rl16(pb);
        par->bit_rate = (8LL * par->block_align * par->sample_rate / av_get_audio_frame_duration2(par, par->block_align));
    } else {
        par->block_align = par->bits_per_coded_sample *
                           par->ch_layout.nb_channels / 8;
    }

    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    return 0;
}

static int pda_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;

    if (par->codec_id == AV_CODEC_ID_ADPCM_IMA_PDA) {
        int ret = av_get_packet(s->pb, pkt, par->block_align);

        pkt->stream_index = 0;
        pkt->duration = av_get_audio_frame_duration2(par, pkt->size);
        return ret;
    } else {
        return ff_pcm_read_packet(s, pkt);
    }
}

const FFInputFormat ff_pda_demuxer = {
    .p.name         = "pda",
    .p.long_name    = NULL_IF_CONFIG_SMALL("PlayDate Audio"),
    .p.extensions   = "pda",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = pda_probe,
    .read_header    = pda_read_header,
    .read_packet    = pda_read_packet,
};
