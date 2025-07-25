/*
 * SoX native format demuxer
 * Copyright (c) 2009 Daniel Verkamp <daniel@drv.nu>
 *
 * Based on libSoX sox-fmt.c
 * Copyright (c) 2008 robs@users.sourceforge.net
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
 * SoX native format demuxer
 * @author Daniel Verkamp
 * @see http://wiki.multimedia.cx/index.php?title=SoX_native_intermediate_format
 */

#include "libavutil/intreadwrite.h"
#include "libavutil/intfloat.h"
#include "libavutil/dict.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"
#include "sox.h"

static int sox_probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) != SOX_TAG && AV_RB32(p->buf) != SOX_TAG)
        return 0;
    if (AV_RN32(p->buf+4) == 0)
        return 0;
    if (p->buf_size < 28)
        return 0;

    if (AV_RN64(p->buf+16) == 0)
        return 0;
    if (AV_RN32(p->buf+24) == 0)
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int sox_read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    unsigned header_size, comment_size;
    double sample_rate, sample_rate_frac;
    int channels;
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;

    if (avio_rl32(pb) == SOX_TAG) {
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S32LE;
        header_size         = avio_rl32(pb);
        avio_skip(pb, 8); /* sample count */
        sample_rate         = av_int2double(avio_rl64(pb));
        channels            = avio_rl32(pb);
        comment_size        = avio_rl32(pb);
    } else {
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S32BE;
        header_size         = avio_rb32(pb);
        avio_skip(pb, 8); /* sample count */
        sample_rate         = av_int2double(avio_rb64(pb));
        channels            = avio_rb32(pb);
        comment_size        = avio_rb32(pb);
    }

    st->codecpar->ch_layout.nb_channels = channels;

    if (comment_size > 0xFFFFFFFFU - SOX_FIXED_HDR - 4U) {
        av_log(s, AV_LOG_ERROR, "invalid comment size (%u)\n", comment_size);
        return AVERROR_INVALIDDATA;
    }

    if (sample_rate <= 0 || sample_rate > INT_MAX) {
        av_log(s, AV_LOG_ERROR, "invalid sample rate (%f)\n", sample_rate);
        return AVERROR_INVALIDDATA;
    }

    sample_rate_frac = sample_rate - floor(sample_rate);
    if (sample_rate_frac)
        av_log(s, AV_LOG_WARNING,
               "truncating fractional part of sample rate (%f)\n",
               sample_rate_frac);

    if ((header_size + 4) & 7 || header_size < SOX_FIXED_HDR + comment_size
        || channels > 65535 || channels <= 0) /* Reserve top 16 bits */ {
        av_log(s, AV_LOG_ERROR, "invalid header\n");
        return AVERROR_INVALIDDATA;
    }

    if (comment_size && comment_size < UINT_MAX) {
        char *comment = av_malloc(comment_size+1);
        if(!comment)
            return AVERROR(ENOMEM);
        if (avio_read(pb, comment, comment_size) != comment_size) {
            av_freep(&comment);
            return AVERROR(EIO);
        }
        comment[comment_size] = 0;

        av_dict_set(&s->metadata, "comment", comment,
                               AV_DICT_DONT_STRDUP_VAL);
    }

    avio_skip(pb, header_size - SOX_FIXED_HDR - comment_size);

    st->codecpar->sample_rate           = sample_rate;
    st->codecpar->bits_per_coded_sample = 32;
    st->codecpar->bit_rate              = (int64_t)st->codecpar->sample_rate *
                                          st->codecpar->bits_per_coded_sample *
                                          channels;
    st->codecpar->block_align           = st->codecpar->bits_per_coded_sample *
                                          channels / 8;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_sox_demuxer = {
    .p.name         = "sox",
    .p.long_name    = NULL_IF_CONFIG_SMALL("SoX (Sound eXchange) native"),
    .read_probe     = sox_probe,
    .read_header    = sox_read_header,
    .read_packet    = ff_pcm_read_packet,
    .read_seek      = ff_pcm_read_seek,
};
