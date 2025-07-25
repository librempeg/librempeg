/*
 * G.726 raw demuxer
 * Copyright 2017 Carl Eugen Hoyos
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

#include "config_components.h"

#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "libavutil/opt.h"

typedef struct G726Context {
    AVClass *class;
    int code_size;
    int sample_rate;
} G726Context;

static int g726_read_header(AVFormatContext *s)
{
    G726Context *c = s->priv_data;
    AVStream *st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id   = ffifmt(s->iformat)->raw_codec_id;

    st->codecpar->sample_rate           = c->sample_rate;
    st->codecpar->bits_per_coded_sample = c->code_size;
    st->codecpar->bit_rate              = ((int[]){ 16000, 24000, 32000, 40000 })[c->code_size - 2];
    st->codecpar->ch_layout.nb_channels = 1;

    return 0;
}

static int g726_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int res;
    res = av_get_packet(s->pb, pkt, 1020); // a size similar to RAW_PACKET_SIZE divisible by all code_size values
    if (res < 0)
        return res;
    return 0;
}

#define OFFSET(x) offsetof(G726Context, x)
static const AVOption options[] = {
    { "code_size", "Bits per G.726 code",
        OFFSET(code_size),   AV_OPT_TYPE_INT, {.i64 =    4}, 2,       5, AV_OPT_FLAG_DECODING_PARAM },
    { "sample_rate", "",
        OFFSET(sample_rate), AV_OPT_TYPE_INT, {.i64 = 8000}, 1, INT_MAX, AV_OPT_FLAG_DECODING_PARAM },
    { NULL },
};

static const AVClass g726_demuxer_class = {
    .class_name     = "G.726 demuxer",
    .option         = options,
    .version        = LIBAVUTIL_VERSION_INT,
};

#if CONFIG_G726_DEMUXER
const FFInputFormat ff_g726_demuxer = {
    .p.name         = "g726",
    .p.long_name    = NULL_IF_CONFIG_SMALL("raw big-endian G.726 (\"left aligned\")"),
    .p.priv_class   = &g726_demuxer_class,
    .read_header    = g726_read_header,
    .read_packet    = g726_read_packet,
    .priv_data_size = sizeof(G726Context),
    .raw_codec_id   = AV_CODEC_ID_ADPCM_G726,
};
#endif

#if CONFIG_G726LE_DEMUXER
const FFInputFormat ff_g726le_demuxer = {
    .p.name         = "g726le",
    .p.long_name    = NULL_IF_CONFIG_SMALL("raw little-endian G.726 (\"right aligned\")"),
    .p.priv_class   = &g726_demuxer_class,
    .read_header    = g726_read_header,
    .read_packet    = g726_read_packet,
    .priv_data_size = sizeof(G726Context),
    .raw_codec_id   = AV_CODEC_ID_ADPCM_G726LE,
};
#endif

