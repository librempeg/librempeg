/*
 * KTSS demuxer
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
#include "internal.h"

typedef struct KTSSDemuxContext {
    int64_t data_end;
} KTSSDemuxContext;

static int ktss_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "KTSS", 4) || AV_RL32(p->buf + 4) <= 0x40 || p->buf_size <= 0x40 ||
        AV_RL64(p->buf + 8) != 0 || AV_RL64(p->buf + 16) != 0 || AV_RL64(p->buf + 24) != 0)
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int ktss_read_header(AVFormatContext *s)
{
    uint32_t start_offset, loop_start, loop_length, coeff_skip, coeff_gap, frames;
    KTSSDemuxContext *kc = s->priv_data;
    uint16_t format, channels;
    AVIOContext *pb = s->pb;
    uint8_t version;
    int ret, skip;
    AVStream *st;

    avio_skip(pb, 0x20);
    format = avio_rl16(pb);
    version = avio_r8(pb);
    /* version2 = */avio_r8(pb);
    start_offset = avio_rl32(pb) + 0x20;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    channels = avio_r8(pb) * avio_r8(pb);
    if (channels <= 0)
        return AVERROR_INVALIDDATA;
    st->codecpar->ch_layout.nb_channels = channels;
    avio_skip(pb, 2);
    st->codecpar->sample_rate = avio_rl32(pb);
    if (st->codecpar->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    st->duration = avio_rl32(pb);

    loop_start = avio_rl32(pb);
    loop_length = avio_rl32(pb);
    if (loop_length > 0) {
        av_dict_set_int(&s->metadata, "loop_start", loop_start, 0);
        av_dict_set_int(&s->metadata, "loop_end", loop_start + loop_length, 0);
    }

    switch (format) {
    case 0x02: /* DSP ADPCM - Shingeki no Kyojin - Shichi-Kara no Dasshutsu (3DS) */
        switch (version) {
        case 0x1:
            coeff_skip = 4;
            coeff_gap = 14;
            break;
        case 0x3:
            coeff_skip = 32;
            coeff_gap = 0x40;
            break;
        default:
            avpriv_request_sample(st, "format 0x%X, version 0x%X", format, version);
            return AVERROR_PATCHWELCOME;
        }

        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP_LE;
        st->codecpar->block_align = 8 * st->codecpar->ch_layout.nb_channels;

        ret = ff_alloc_extradata(st->codecpar, 0x20 * st->codecpar->ch_layout.nb_channels);
        if (ret < 0)
            return ret;
        avio_skip(pb, coeff_skip);
        for (int c = 0; c < st->codecpar->ch_layout.nb_channels; c++) {
            avio_read(pb, st->codecpar->extradata + 0x20 * c, 32);
            avio_skip(pb, coeff_gap);
        }

        break;
    case 0x09: /* OPUS - Dead or Alive Xtreme 3: Scarlet (Switch), Fire Emblem: Three Houses (Switch) */
        if (channels > 8)
            return AVERROR_INVALIDDATA;

        avio_skip(pb, 4);
        st->codecpar->codec_id = AV_CODEC_ID_OPUS;

        start_offset = avio_rl32(pb);
        kc->data_end = start_offset + avio_rl32(pb);
        avio_skip(pb, 4);

        frames = 0;
        if (st->duration == 0) {
            frames = avio_rl32(pb);
            avio_skip(pb, 2);
            st->duration = frames * avio_rl16(pb);
        } else {
            avio_skip(pb, 8);
        }
        if (st->codecpar->sample_rate == 0)
            st->codecpar->sample_rate = avio_rl32(pb);
        else
            avio_skip(pb, 4);
        skip = avio_rl16(pb);
        if (frames != 0)
            st->duration -= skip;

        /* channel mapping is always specified */
        ret = ff_alloc_extradata(st->codecpar, 21 + channels);
        if (ret < 0)
            return ret;
        memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);

        memcpy(st->codecpar->extradata, "OpusHead", 8);
        st->codecpar->extradata[8] = 1;
        st->codecpar->extradata[9] = channels;
        AV_WL16(st->codecpar->extradata + 10, skip);
        AV_WL32(st->codecpar->extradata + 12, st->codecpar->sample_rate);

        st->codecpar->extradata[18] = 1;
        st->codecpar->extradata[19] = avio_r8(pb);
        st->codecpar->extradata[20] = avio_r8(pb);

        for (int c = 0; c < channels; c++)
            st->codecpar->extradata[21 + c] = avio_r8(pb);

        if (channels == 6) {
            /* undo KTSS opus reordering: FL FC FR BL LFE BR > FL FR FC LFE BL BR */
            st->codecpar->ch_layout.u.map[0].id = AV_CHAN_FRONT_LEFT;
            st->codecpar->ch_layout.u.map[1].id = AV_CHAN_FRONT_CENTER;
            st->codecpar->ch_layout.u.map[2].id = AV_CHAN_FRONT_RIGHT;
            st->codecpar->ch_layout.u.map[3].id = AV_CHAN_BACK_LEFT;
            st->codecpar->ch_layout.u.map[4].id = AV_CHAN_LOW_FREQUENCY;
            st->codecpar->ch_layout.u.map[5].id = AV_CHAN_BACK_RIGHT;
        }

        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        break;
    default:
        avpriv_request_sample(st, "format 0x%X", format);
        return AVERROR_PATCHWELCOME;
    }
    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    avio_seek(pb, start_offset, SEEK_SET);

    return 0;
}

static int ktss_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVIOContext *pb = s->pb;
    int ret;

    if (s->streams[0]->codecpar->codec_id == AV_CODEC_ID_OPUS) {
        KTSSDemuxContext *kc = s->priv_data;
        int64_t pos = avio_tell(pb);
        uint32_t size;

        if (avio_feof(pb) || pos >= kc->data_end)
            return AVERROR_EOF;

        size = avio_rb32(pb);
        if (size < 2)
            return AVERROR_INVALIDDATA;

        avio_skip(pb, 4);
        ret = av_get_packet(pb, pkt, size);
        pkt->pos = pos;
    } else {
        ret = av_get_packet(pb, pkt, s->streams[0]->codecpar->block_align);
    }
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_ktss_demuxer = {
    .p.name         = "ktss",
    .p.long_name    = NULL_IF_CONFIG_SMALL("KTSS (Koei Tecmo)"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "ktss,kns,kno",
    .priv_data_size = sizeof(KTSSDemuxContext),
    .read_probe     = ktss_probe,
    .read_header    = ktss_read_header,
    .read_packet    = ktss_read_packet,
};
