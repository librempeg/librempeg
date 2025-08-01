/*
 * GENH demuxer
 * Copyright (c) 2015 Paul B Mahol
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
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

typedef struct GENHDemuxContext {
    unsigned dsp_int_type;
    unsigned interleave_size;
} GENHDemuxContext;

static int genh_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) != MKTAG('G','E','N','H'))
        return 0;
    if (AV_RL32(p->buf+4) <= 0 || AV_RL32(p->buf+4) > 0xFFFF) // channels
        return 0;

    return AVPROBE_SCORE_MAX / 3 * 2;
}

static int genh_read_header(AVFormatContext *s)
{
    unsigned start_offset, header_size, codec, coef_type, coef[2];
    GENHDemuxContext *c = s->priv_data;
    av_unused unsigned coef_splitted[2];
    int align, ch, ret;
    AVStream *st;

    avio_skip(s->pb, 4);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->ch_layout.nb_channels    = avio_rl32(s->pb);
    if (st->codecpar->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;
    if (st->codecpar->ch_layout.nb_channels == 1)
        st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO;
    else if (st->codecpar->ch_layout.nb_channels == 2)
        st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
    align                  =
    c->interleave_size     = avio_rl32(s->pb);
    if (align < 0 || align > INT_MAX / st->codecpar->ch_layout.nb_channels)
        return AVERROR_INVALIDDATA;
    st->codecpar->block_align = align * st->codecpar->ch_layout.nb_channels;
    st->codecpar->sample_rate = avio_rl32(s->pb);
    if (st->codecpar->sample_rate < 0)
        return AVERROR_INVALIDDATA;

    avio_skip(s->pb, 4);
    st->duration = avio_rl32(s->pb);

    codec = avio_rl32(s->pb);
    switch (codec) {
    case  0: st->codecpar->codec_id = AV_CODEC_ID_ADPCM_PSX;        break;
    case  1:
    case 11: st->codecpar->bits_per_coded_sample = 4;
             if (st->codecpar->ch_layout.nb_channels > INT_MAX / 36)
                return AVERROR_INVALIDDATA;
             st->codecpar->block_align = 36 * st->codecpar->ch_layout.nb_channels;
             st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_WAV;    break;
    case  2: st->codecpar->codec_id = AV_CODEC_ID_ADPCM_DTK;        break;
    case  3: st->codecpar->codec_id = st->codecpar->block_align > 0 ?
                                   AV_CODEC_ID_PCM_S16BE_PLANAR :
                                   AV_CODEC_ID_PCM_S16BE;        break;
    case  4: st->codecpar->codec_id = st->codecpar->block_align > 0 ?
                                   AV_CODEC_ID_PCM_S16LE_PLANAR :
                                   AV_CODEC_ID_PCM_S16LE;        break;
    case  5: st->codecpar->codec_id = st->codecpar->block_align > 0 ?
                                   AV_CODEC_ID_PCM_S8_PLANAR :
                                   AV_CODEC_ID_PCM_S8;           break;
    case  6: if (st->codecpar->block_align > INT_MAX/1024)
                 return AVERROR_INVALIDDATA;
             st->codecpar->codec_id = AV_CODEC_ID_SDX2_DPCM;        break;
    case  7: ret = ff_alloc_extradata(st->codecpar, 2);
             if (ret < 0)
                 return ret;
             AV_WL16(st->codecpar->extradata, 3);
             st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_WS;     break;
    case 10: st->codecpar->codec_id = AV_CODEC_ID_ADPCM_AICA;       break;
    case 12: st->codecpar->codec_id = AV_CODEC_ID_ADPCM_NDSP;       break;
    case 13: st->codecpar->codec_id = AV_CODEC_ID_PCM_U8;           break;
    case 17: st->codecpar->codec_id = AV_CODEC_ID_ADPCM_IMA_QT;     break;
    default:
             avpriv_request_sample(s, "codec %d", codec);
             return AVERROR_PATCHWELCOME;
    }

    start_offset = avio_rl32(s->pb);
    header_size  = avio_rl32(s->pb);

    if (header_size > start_offset)
        return AVERROR_INVALIDDATA;

    if (header_size == 0)
        start_offset = 0x800;

    coef[0]          = avio_rl32(s->pb);
    coef[1]          = avio_rl32(s->pb);
    c->dsp_int_type  = avio_rl32(s->pb);
    coef_type        = avio_rl32(s->pb);
    coef_splitted[0] = avio_rl32(s->pb);
    coef_splitted[1] = avio_rl32(s->pb);

    if (st->codecpar->codec_id == AV_CODEC_ID_ADPCM_NDSP) {
        if (st->codecpar->ch_layout.nb_channels > 2) {
            avpriv_request_sample(s, "channels %d>2", st->codecpar->ch_layout.nb_channels);
            return AVERROR_PATCHWELCOME;
        }

        ff_alloc_extradata(st->codecpar, 32 * st->codecpar->ch_layout.nb_channels);
        for (ch = 0; ch < st->codecpar->ch_layout.nb_channels; ch++) {
            if (coef_type & 1) {
                avpriv_request_sample(s, "coef_type & 1");
                return AVERROR_PATCHWELCOME;
            } else {
                avio_seek(s->pb, coef[ch], SEEK_SET);
                avio_read(s->pb, st->codecpar->extradata + 32 * ch, 32);
            }
        }

        if (c->dsp_int_type == 1) {
            st->codecpar->block_align = 8 * st->codecpar->ch_layout.nb_channels;
            if (c->interleave_size != 1 &&
                c->interleave_size != 2 &&
                c->interleave_size != 4)
                return AVERROR_INVALIDDATA;
        }
    }

    if (st->codecpar->block_align <= 0)
        return AVERROR_INVALIDDATA;

    avio_skip(s->pb, start_offset - avio_tell(s->pb));

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int genh_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    GENHDemuxContext *c = s->priv_data;
    int ret;

    if (c->dsp_int_type == 1 && par->codec_id == AV_CODEC_ID_ADPCM_NDSP &&
        par->ch_layout.nb_channels > 1) {
        int i, ch;

        if (avio_feof(s->pb))
            return AVERROR_EOF;
        ret = av_new_packet(pkt, 8 * par->ch_layout.nb_channels);
        if (ret < 0)
            return ret;
        for (i = 0; i < 8 / c->interleave_size; i++) {
            for (ch = 0; ch < par->ch_layout.nb_channels; ch++) {
                pkt->data[ch * 8 + i*c->interleave_size+0] = avio_r8(s->pb);
                pkt->data[ch * 8 + i*c->interleave_size+1] = avio_r8(s->pb);
            }
        }
        ret = 0;
    } else if (par->codec_id == AV_CODEC_ID_SDX2_DPCM) {
        ret = av_get_packet(s->pb, pkt, par->block_align * 1024);

    } else {
        ret = av_get_packet(s->pb, pkt, par->block_align ? par->block_align :
                                                           1024 * par->ch_layout.nb_channels);
    }

    pkt->stream_index = 0;
    return ret;
}

const FFInputFormat ff_genh_demuxer = {
    .p.name         = "genh",
    .p.long_name    = NULL_IF_CONFIG_SMALL("GENeric Header"),
    .p.extensions   = "genh",
    .priv_data_size = sizeof(GENHDemuxContext),
    .read_probe     = genh_probe,
    .read_header    = genh_read_header,
    .read_packet    = genh_read_packet,
};
