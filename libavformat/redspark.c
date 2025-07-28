/*
 * RedSpark demuxer
 * Copyright (c) 2013 James Almer
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

#include "libavcodec/bytestream.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "avio.h"
#include "demux.h"
#include "internal.h"

#define HEADER_SIZE 0x3000
#define rol(value, bits) (((value) << (bits)) | ((value) >> (32 - (bits))))

typedef struct RedSparkContext {
    int64_t data_stop;
} RedSparkContext;

static int redspark_probe(const AVProbeData *p)
{
    uint32_t key, data;
    uint8_t header[8];

    /* Decrypt first 8 bytes of the header */
    data = AV_RB32(p->buf);
    key  = data ^ 0x52656453;
    data ^= key;
    AV_WB32(header, data);
    key = rol(key, 11);

    key += rol(key, 3);
    data = AV_RB32(p->buf + 4) ^ key;
    AV_WB32(header + 4, data);

    if (AV_RB64(header) == AV_RB64("RedSpark"))
        return AVPROBE_SCORE_MAX;

    return 0;
}

static int redspark_read_header(AVFormatContext *s)
{
    int coef_off, ret = 0, bank_flag, encrypted;
    RedSparkContext *redspark = s->priv_data;
    uint32_t key, data, data_stop;
    uint8_t header[HEADER_SIZE];
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    GetByteContext gbc;
    int64_t data_start;
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);
    par = st->codecpar;

    /* Decrypt header */
    data = avio_rb32(pb);
    if (data == MKBETAG('R','e','d','S')) {
        encrypted = 0;
        AV_WB32(header, data);
        avio_read(pb, header + 4, sizeof(header) - 4);
    } else {
        encrypted = 1;
        key  = data ^ 0x52656453;
        data ^= key;
        AV_WB32(header, data);
        key = rol(key, 11);

        for (int i = 4; i < HEADER_SIZE; i += 4) {
            key += rol(key, 3);
            data = avio_rb32(pb) ^ key;
            AV_WB32(header + i, data);
        }
    }

    par->codec_id   = encrypted ? AV_CODEC_ID_ADPCM_THP : AV_CODEC_ID_ADPCM_THP_LE;
    par->codec_type = AVMEDIA_TYPE_AUDIO;

    bytestream2_init(&gbc, header, HEADER_SIZE);
    bytestream2_seek(&gbc, 0x18, SEEK_SET);
    data_start = encrypted ? bytestream2_get_be32u(&gbc) : bytestream2_get_le32u(&gbc);
    bank_flag = bytestream2_get_be16u(&gbc);
    if (bank_flag)
        return AVERROR_PATCHWELCOME;
    bytestream2_skipu(&gbc, 2);
    data_stop = encrypted ? bytestream2_get_be32u(&gbc) : bytestream2_get_le32u(&gbc);
    redspark->data_stop = data_stop;

    bytestream2_seek(&gbc, 0x3c, SEEK_SET);
    par->sample_rate = encrypted ? bytestream2_get_be32u(&gbc) : bytestream2_get_le32u(&gbc);
    if (par->sample_rate <= 0 || par->sample_rate > 96000) {
        av_log(s, AV_LOG_ERROR, "Invalid sample rate: %d\n", par->sample_rate);
        return AVERROR_INVALIDDATA;
    }

    st->duration = encrypted ? bytestream2_get_be32u(&gbc) * 14LL : bytestream2_get_le32u(&gbc);

    bytestream2_skipu(&gbc, 10);
    par->ch_layout.nb_channels = bytestream2_get_byteu(&gbc);
    if (!par->ch_layout.nb_channels)
        return AVERROR_INVALIDDATA;

    coef_off = 0x54 + par->ch_layout.nb_channels * 8;
    if (bytestream2_get_byteu(&gbc)) // Loop flag
        coef_off += 16;

    if (coef_off + par->ch_layout.nb_channels * (32 + 14) > HEADER_SIZE)
        return AVERROR_INVALIDDATA;

    ret = ff_alloc_extradata(par, 32 * par->ch_layout.nb_channels);
    if (ret < 0)
        return ret;

    /* Get the ADPCM table */
    bytestream2_seek(&gbc, coef_off, SEEK_SET);
    for (int i = 0; i < par->ch_layout.nb_channels; i++) {
        if (bytestream2_get_bufferu(&gbc, par->extradata + i * 32, 32) != 32) {
            return AVERROR_INVALIDDATA;
        }

        bytestream2_skipu(&gbc, 14);
    }

    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    avio_seek(pb, data_start, SEEK_SET);

    return ret;
}

static int redspark_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    RedSparkContext *redspark = s->priv_data;
    const uint32_t size = 8 * par->ch_layout.nb_channels;
    int ret;

    if (avio_tell(s->pb) >= redspark->data_stop)
        return AVERROR_EOF;

    if (avio_feof(s->pb))
        return AVERROR_EOF;

    ret = av_get_packet(s->pb, pkt, size);
    if (ret != size)
        return AVERROR(EIO);
    pkt->stream_index = 0;

    return ret;
}

const FFInputFormat ff_redspark_demuxer = {
    .p.name         =   "redspark",
    .p.long_name    =   NULL_IF_CONFIG_SMALL("RedSpark"),
    .p.extensions   =   "rsd",
    .priv_data_size =   sizeof(RedSparkContext),
    .read_probe     =   redspark_probe,
    .read_header    =   redspark_read_header,
    .read_packet    =   redspark_read_packet,
};
