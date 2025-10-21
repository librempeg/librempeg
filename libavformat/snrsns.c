/*
 * EA SNR/SNS demuxer
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

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavcodec/mathops.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

typedef struct EAACHeader {
    int version;
    int codec;
    int sample_rate;
    int type;

    int streamed;
    int channels;

    int nb_samples;
    int32_t loop_start;
    int loop_end;
    int loop_flag;

    uint32_t header_size;
} EAACHeader;

#define EAAC_CODEC_NONE               0x00
#define EAAC_CODEC_RESERVED           0x01
#define EAAC_CODEC_PCM16BE            0x02
#define EAAC_CODEC_EAXMA              0x03
#define EAAC_CODEC_XAS1               0x04
#define EAAC_CODEC_EALAYER3_V1        0x05
#define EAAC_CODEC_EALAYER3_V2_PCM    0x06
#define EAAC_CODEC_EALAYER3_V2_SPIKE  0x07
#define EAAC_CODEC_GCADPCM            0x08
#define EAAC_CODEC_EASPEEX            0x09
#define EAAC_CODEC_EATRAX             0x0a
#define EAAC_CODEC_EAMP3              0x0b
#define EAAC_CODEC_EAOPUS             0x0c
#define EAAC_CODEC_EAATRAC9           0x0d
#define EAAC_CODEC_EAOPUSM            0x0e
#define EAAC_CODEC_EAOPUSMU           0x0f

#define EAAC_TYPE_RAM        0x00
#define EAAC_TYPE_STREAM     0x01
#define EAAC_TYPE_GIGASAMPLE 0x02

static int eaac_parse_header(EAACHeader *h,
                             const uint32_t header1, const uint32_t header2)
{
    h->version     =  (header1 >> 28) & 0x0F;
    h->codec       =  (header1 >> 24) & 0x0F;
    h->channels    = ((header1 >> 18) & 0x3F) + 1;
    h->sample_rate =  (header1 >>  0) & 0x03FFFF;
    h->type        =  (header2 >> 30) & 0x03;
    h->loop_flag   =  (header2 >> 29) & 0x01;
    h->nb_samples  =  (header2 >>  0) & 0x1FFFFFFF;

    if (h->version != 0 && h->version != 1)
        return -1;

    if (h->sample_rate > 200000 ||
        h->sample_rate == 0)
        return -1;

    h->streamed = h->type != EAAC_TYPE_RAM;
    if (h->type != EAAC_TYPE_RAM && h->type != EAAC_TYPE_STREAM && h->type != EAAC_TYPE_GIGASAMPLE)
        return -1;

    if (h->version == 1 && h->type == EAAC_TYPE_GIGASAMPLE)
        return -1;

    return 0;
}

static int snrsns_read_probe(const AVProbeData *p)
{
    uint32_t header[2];
    EAACHeader h;
    int ret;

    if (av_match_ext(p->filename, "snr") == 0)
        return 0;

    header[0] = AV_RB32(p->buf+0);
    header[1] = AV_RB32(p->buf+4);

    ret = eaac_parse_header(&h, header[0], header[1]);
    if (ret < 0)
        return 0;

    return AVPROBE_SCORE_MAX/3;
}

static int snrsns_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    uint32_t header[2];
    EAACHeader h;
    AVStream *st;
    int ret;

    header[0] = avio_rb32(pb);
    header[1] = avio_rb32(pb);

    ret = eaac_parse_header(&h, header[0], header[1]);
    if (ret < 0)
        return AVERROR_INVALIDDATA;

    if (h.loop_flag)
        h.loop_start = sign_extend(avio_rb32(pb), 32);

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    switch (h.codec) {
    case EAAC_CODEC_PCM16BE:
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S16BE;
        st->codecpar->block_align = 2 * h.channels;
        break;
    case EAAC_CODEC_EALAYER3_V1:
        st->codecpar->codec_id = AV_CODEC_ID_EALAYER3;
        ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        break;
    case EAAC_CODEC_XAS1:
        st->codecpar->codec_id = AV_CODEC_ID_ADPCM_EA_XAS;
        break;
    default:
        avpriv_request_sample(s, "codec %d", h.codec);
        return AVERROR_PATCHWELCOME;
    }

    st->start_time = 0;
    st->duration = h.nb_samples;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->sample_rate = h.sample_rate;
    st->codecpar->ch_layout.nb_channels = h.channels;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if (avio_size(pb) <= 8) {
        extern const FFInputFormat ff_hdbd_demuxer;
        char *sns_file_name = av_strdup(s->url);
        AVDictionary *tmp = NULL;
        int len;

        if (!sns_file_name)
            return AVERROR(ENOMEM);

        len = strlen(sns_file_name);
        if (len > 3) {
            sns_file_name[len-1] = 's';
        } else {
            return AVERROR_INVALIDDATA;
        }

        ret = s->io_open(s, &s->pb, sns_file_name, AVIO_FLAG_READ, &tmp);
        av_freep(&sns_file_name);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int snrsns_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVStream *st = s->streams[0];
    AVCodecParameters *par = st->codecpar;
    AVIOContext *pb = s->pb;
    int64_t pos;
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    pos = avio_tell(pb);
    if (par->block_align > 0) {
        ret = ff_pcm_read_packet(s, pkt);
    } else {
        int size, duration;

        size = avio_rb32(pb);
        duration = avio_rb32(pb);
        if (size <= 8)
            return AVERROR_INVALIDDATA;
        size -= 8;

        ret = av_get_packet(pb, pkt, size);
        pkt->stream_index = 0;
        pkt->duration = duration;
    }
    pkt->pos = pos;

    return ret;
}

const FFInputFormat ff_snrsns_demuxer = {
    .p.name         = "snrsns",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Electronic Arts SNR/SNS"),
    .p.extensions   = "snrsns",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = snrsns_read_probe,
    .read_header    = snrsns_read_header,
    .read_packet    = snrsns_read_packet,
};
