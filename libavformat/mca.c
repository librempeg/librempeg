/*
 * MCA demuxer
 * Copyright (c) 2020 Zixing Liu
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
#include "demux.h"
#include "internal.h"

static int probe(const AVProbeData *p)
{
    if (AV_RL32(p->buf) == MKTAG('M', 'A', 'D', 'P') &&
        AV_RL16(p->buf + 4) <= 0x5)
        return AVPROBE_SCORE_MAX / 3 * 2;
    return 0;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVCodecParameters *par;
    int64_t file_size = avio_size(pb);
    uint16_t version;
    uint32_t header_size, data_size, data_offset, loop_start, loop_end,
        nb_samples, nb_metadata, coef_offset = 0;
    int ch, ret, block_size;
    int64_t ret_size, data_start;
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    par = st->codecpar;
    par->codec_type = AVMEDIA_TYPE_AUDIO;

    // parse file headers
    avio_skip(pb, 0x4);      // skip the file magic
    version = avio_rl16(pb);
    avio_skip(pb, 0x2);      // padding
    par->ch_layout.nb_channels = avio_r8(pb);
    avio_skip(pb, 0x1);      // padding
    block_size = avio_rl16(pb);
    nb_samples = avio_rl32(pb);
    par->sample_rate = avio_rl32(pb);
    loop_start = avio_rl32(pb);
    loop_end = avio_rl32(pb);
    header_size = avio_rl32(pb);
    data_size = avio_rl32(pb);
    avio_skip(pb, 0x4);
    nb_metadata = avio_rl16(pb);
    avio_skip(pb, 0x2);      // unknown u16 field

    st->duration = nb_samples;

    // sanity checks
    if (par->ch_layout.nb_channels <= 0 || par->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    if (block_size <= 0 || block_size > INT_MAX/par->ch_layout.nb_channels)
        return AVERROR_INVALIDDATA;

    if (loop_end > loop_start) {
        if ((ret = av_dict_set_int(&s->metadata, "loop_start", loop_start, 0)) < 0)
            return ret;
        if ((ret = av_dict_set_int(&s->metadata, "loop_end", loop_end, 0)) < 0)
            return ret;
    }
    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    if (version <= 4) {
        // version <= 4 needs to use the file size to calculate the offsets
        if (file_size < 0) {
            return AVERROR_INVALIDDATA;
        }
        if (file_size - data_size > UINT32_MAX)
            return AVERROR_INVALIDDATA;
        data_start = file_size - data_size;
        if (version <= 3) {
            nb_metadata = 0;
            // header_size is not available or incorrect in older versions
            header_size = data_start;
        }
    } else if (version == 5) {
        // read data_start location from the header
        if (0x30 * par->ch_layout.nb_channels + 0x4 > header_size)
            return AVERROR_INVALIDDATA;
        data_offset = header_size - 0x30 * par->ch_layout.nb_channels - 0x4;
        if ((ret_size = avio_seek(pb, data_offset, SEEK_SET)) < 0)
            return ret_size;
        data_start = avio_rl32(pb);
        // check if the metadata is reasonable
        if (file_size > 0 && (int64_t)data_start + data_size > file_size) {
            // the header is broken beyond repair
            if ((int64_t)header_size + data_size > file_size) {
                av_log(s, AV_LOG_ERROR,
                       "MCA metadata corrupted, unable to determine the data offset.\n");
                return AVERROR_INVALIDDATA;
            }
            // recover the data_start information from the data size
            av_log(s, AV_LOG_WARNING,
                   "Incorrect header size found in metadata, "
                   "header size approximated from the data size\n");
            if (file_size - data_offset > UINT32_MAX)
                return AVERROR_INVALIDDATA;
            data_start = file_size - data_size;
        }
    } else {
        avpriv_request_sample(s, "version %d", version);
        return AVERROR_PATCHWELCOME;
    }

    // coefficient alignment = 0x30; metadata size = 0x14
    if (0x30 * par->ch_layout.nb_channels + nb_metadata * 0x14 > header_size)
        return AVERROR_INVALIDDATA;
    coef_offset = header_size - 0x30 * par->ch_layout.nb_channels + nb_metadata * 0x14;

    st->start_time = 0;
    par->codec_id = AV_CODEC_ID_ADPCM_NDSP_LE;
    par->block_align = block_size * par->ch_layout.nb_channels;

    ret = ff_alloc_extradata(st->codecpar, 32 * par->ch_layout.nb_channels);
    if (ret < 0)
        return ret;

    if ((ret_size = avio_seek(pb, coef_offset, SEEK_SET)) < 0)
        return ret_size;
    for (ch = 0; ch < par->ch_layout.nb_channels; ch++) {
        if ((ret = ffio_read_size(pb, par->extradata + ch * 32, 32)) < 0)
            return ret;
        // 0x30 (alignment) - 0x20 (actual size, 32) = 0x10 (padding)
        avio_skip(pb, 0x10);
    }

    // seek to the beginning of the adpcm data
    // there are some files where the adpcm audio data is not immediately after the header
    if ((ret_size = avio_seek(pb, data_start, SEEK_SET)) < 0)
        return ret_size;

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AVCodecParameters *par = s->streams[0]->codecpar;
    int ret;

    if (avio_feof(s->pb))
        return AVERROR_EOF;

    if ((ret = av_get_packet(s->pb, pkt, par->block_align)) < 0)
        return ret;
    pkt->stream_index = 0;

    return 0;
}

const FFInputFormat ff_mca_demuxer = {
    .p.name         = "mca",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Capcom 3DS MCA"),
    .p.extensions   = "mca",
    .p.flags        = AVFMT_GENERIC_INDEX,
    .read_probe     = probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
};
