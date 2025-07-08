/*
 * NIST Sphere muxer
 * Copyright (c) 2023 Paul B Mahol
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
#include "avio_internal.h"
#include "internal.h"
#include "mux.h"
#include "pcm.h"
#include "rawenc.h"

typedef struct NISTContext {
    int64_t offset;
    int bps;
} NISTContext;

static int nist_write_header(AVFormatContext *s)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    NISTContext *n = s->priv_data;
    AVIOContext *pb = s->pb;
    const char *format[] = { "01", "10" };
    int index = 0, bps;

    switch (par->codec_id) {
    case AV_CODEC_ID_PCM_S16LE:
    case AV_CODEC_ID_PCM_S24LE:
    case AV_CODEC_ID_PCM_S32LE:
        index = 0;
        break;
    case AV_CODEC_ID_PCM_S16BE:
    case AV_CODEC_ID_PCM_S24BE:
    case AV_CODEC_ID_PCM_S32BE:
        index = 1;
        break;
    default:
        return AVERROR(EINVAL);
    }

    bps = (par->bits_per_raw_sample+7)/8;
    n->bps = bps * par->ch_layout.nb_channels;

    avio_write(pb, "NIST_1A\x0a", 8);
    avio_printf(pb, "   %d\x0a", 1024);
    avio_printf(pb, "channel_count -i %d\x0a", par->ch_layout.nb_channels);
    avio_printf(pb, "sample_rate -i %d\x0a", par->sample_rate);
    avio_printf(pb, "sample_n_bytes -i %d\x0a", bps);
    avio_printf(pb, "sample_sig_bits -i %d\x0a", par->bits_per_raw_sample);
    if (par->bits_per_raw_sample > 8)
        avio_printf(pb, "sample_byte_format -s2 %s\x0a", format[index]);
    avio_printf(pb, "sample_coding -s3 pcm\x0a");

    if (!(pb->seekable & AVIO_SEEKABLE_NORMAL))
        avio_write(pb, "end_head\x0a", 9);

    n->offset = avio_tell(pb);

    ffio_fill(pb, 0, 1024 - n->offset);

    return 0;
}

static int nist_write_trailer(AVFormatContext *s)
{
    NISTContext *n = s->priv_data;
    AVIOContext *pb = s->pb;

    if (pb->seekable & AVIO_SEEKABLE_NORMAL) {
        int64_t file_size = avio_tell(pb);
        int64_t sample_count = (file_size - 1024) / n->bps;

        avio_seek(pb, n->offset, SEEK_SET);
        avio_printf(pb, "sample_count -i %"PRId64"\x0a", sample_count);
        avio_write(pb, "end_head\x0a", 9);
        avio_seek(pb, file_size, SEEK_SET);
    }

    return 0;
}

const FFOutputFormat ff_nistsphere_muxer = {
    .p.name         = "nistsphere",
    .p.extensions   = "nist,sph",
    .p.long_name    = NULL_IF_CONFIG_SMALL("NIST SPeech HEader REsources"),
    .p.audio_codec  = AV_CODEC_ID_PCM_S16LE,
    .p.video_codec  = AV_CODEC_ID_NONE,
    .p.subtitle_codec = AV_CODEC_ID_NONE,
    .flags_internal = FF_OFMT_FLAG_MAX_ONE_OF_EACH,
    .priv_data_size = sizeof(NISTContext),
    .write_header   = nist_write_header,
    .write_packet   = ff_raw_write_packet,
    .write_trailer  = nist_write_trailer,
};
