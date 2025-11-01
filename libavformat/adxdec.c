/*
 * Copyright (c) 2011 Justin Ruggles
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

/**
 * @file
 * CRI ADX demuxer
 */

#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "adx_keys.h"
#include "demux.h"
#include "internal.h"
#include "rawdec.h"
#include "pcm.h"

#define BLOCK_SIZE    18
#define BLOCK_SAMPLES 32

typedef struct ADXDemuxerContext {
    FFRawDemuxerContext rawctx;
    int header_size;
} ADXDemuxerContext;

static int adx_probe(const AVProbeData *p)
{
    int offset;
    if (AV_RB16(p->buf) != 0x8000)
        return 0;

    if (p->buf_size < 16)
        return 0;

    if (p->buf[4] != 3)
        return 0;

    if (p->buf[5] != 0x12)
        return 0;

    if (p->buf[6] != 4)
        return 0;

    if (p->buf[7] == 0 || p->buf[7] > 8)
        return 0;

    if ((int)AV_RB32(p->buf + 8) <= 0)
        return 0;

    if (AV_RB32(p->buf + 12) <= 0)
        return 0;

    offset = AV_RB16(&p->buf[2]);
    if (   offset < 8
        || offset > p->buf_size - 4
        || memcmp(p->buf + offset - 2, "(c)CRI", 6))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int ahx_probe(const AVProbeData *p)
{
    int offset;
    if (AV_RB16(p->buf) != 0x8000)
        return 0;

    if (p->buf_size < 16)
        return 0;

    if (p->buf[4] != 16 && p->buf[4] != 17)
        return 0;

    if (p->buf[5] != 0)
        return 0;

    if (p->buf[6] != 0)
        return 0;

    if (p->buf[7] == 0 || p->buf[7] > 1)
        return 0;

    if ((int)AV_RB32(p->buf + 8) <= 0)
        return 0;

    if (AV_RB32(p->buf + 12) <= 0)
        return 0;

    offset = AV_RB16(&p->buf[2]);
    if (   offset < 8
        || offset > p->buf_size - 4
        || memcmp(p->buf + offset - 2, "(c)CRI", 6))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int adx_read_header(AVFormatContext *s)
{
    ADXDemuxerContext *c = s->priv_data;
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    FFStream *sti;
    int ret, enc;
    int channels;

    AVStream *st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);
    par = s->streams[0]->codecpar;

    if (avio_rb16(pb) != 0x8000)
        return AVERROR_INVALIDDATA;
    c->header_size = avio_rb16(pb) + 4;
    avio_seek(pb, -4, SEEK_CUR);

    if ((ret = ff_alloc_extradata(par, c->header_size + 6)) < 0)
        return ret;

    if ((ret = avio_read(pb, par->extradata, c->header_size)) < 0)
        return ret;
    memset(par->extradata + c->header_size, 0, 6);

    if (par->extradata_size < 12) {
        av_log(s, AV_LOG_ERROR, "Invalid extradata size.\n");
        return AVERROR_INVALIDDATA;
    }
    channels = AV_RB8 (par->extradata + 7);
    par->sample_rate = AV_RB32(par->extradata + 8);

    if (channels <= 0) {
        av_log(s, AV_LOG_ERROR, "invalid number of channels %d\n", channels);
        return AVERROR_INVALIDDATA;
    }

    if (par->sample_rate <= 0) {
        av_log(s, AV_LOG_ERROR, "Invalid sample rate %d\n", par->sample_rate);
        return AVERROR_INVALIDDATA;
    }

    enc = par->extradata[19];
    if (enc == 8 || enc == 9) {
#define MAX_NB_FRAMES 512
        uint16_t xor_start, xor_mult, xor_add;
        int64_t pos = avio_tell(pb);
        uint16_t scales[MAX_NB_FRAMES] = { 0 };
        int nb_scales = 0;

        for (int i = 0; i < MAX_NB_FRAMES; i++) {
            if (avio_feof(pb))
                break;
            scales[i] = avio_rb16(pb);
            avio_skip(pb, BLOCK_SIZE-2);
            nb_scales++;
        }

        ret = ff_adx_find_key(enc, scales, nb_scales/2,
                              scales + nb_scales/2, nb_scales - nb_scales/2,
                              &xor_start, &xor_mult, &xor_add, 0);
        if (ret != 1) {
            av_log(s, AV_LOG_ERROR, "No valid keys found.\n");
            return AVERROR(EINVAL);
        }

        AV_WB16(par->extradata + par->extradata_size-6, xor_start);
        AV_WB16(par->extradata + par->extradata_size-4, xor_mult);
        AV_WB16(par->extradata + par->extradata_size-2, xor_add);

        avio_seek(pb, pos, SEEK_SET);
    }

    sti = ffstream(st);
    par->ch_layout.nb_channels = channels;
    par->codec_type  = AVMEDIA_TYPE_AUDIO;
    switch (par->extradata[4]) {
    case 3:
        par->codec_id = AV_CODEC_ID_ADPCM_ADX;
        par->bit_rate    = (int64_t)par->sample_rate * par->ch_layout.nb_channels * BLOCK_SIZE * 8LL / BLOCK_SAMPLES;
        par->block_align = BLOCK_SIZE * channels;
        avpriv_set_pts_info(st, 64, 1, par->sample_rate);
        break;
    case 16:
    case 17:
        sti->need_parsing = AVSTREAM_PARSE_FULL_RAW;
        par->codec_id = AV_CODEC_ID_AHX;
        avpriv_set_pts_info(st, 64, 1, par->sample_rate);
        break;
    default:
        av_log(s, AV_LOG_ERROR, "Unsupported format: %d\n", par->extradata[4]);
        return AVERROR_INVALIDDATA;
    }

    return 0;
}

const FFInputFormat ff_adx_demuxer = {
    .p.name         = "adx",
    .p.long_name    = NULL_IF_CONFIG_SMALL("CRI ADX"),
    .p.extensions   = "adx",
    .p.priv_class   = &ff_raw_demuxer_class,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = adx_probe,
    .priv_data_size = sizeof(ADXDemuxerContext),
    .read_header    = adx_read_header,
    .read_packet    = ff_pcm_read_packet,
};

const FFInputFormat ff_ahx_demuxer = {
    .p.name         = "ahx",
    .p.long_name    = NULL_IF_CONFIG_SMALL("CRI AHX"),
    .p.extensions   = "ahx",
    .p.priv_class   = &ff_raw_demuxer_class,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = ahx_probe,
    .priv_data_size = sizeof(ADXDemuxerContext),
    .read_header    = adx_read_header,
    .read_packet    = ff_raw_read_partial_packet,
};
