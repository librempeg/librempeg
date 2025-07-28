/*
 * DWD demuxer
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

#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int dwd_probe(const AVProbeData *p)
{
    if (memcmp(p->buf, "DiamondWare Digitized\n\0\x1a", 24))
        return 0;
    return AVPROBE_SCORE_MAX;
}

static int dwd_read_header(AVFormatContext *s)
{
    AVCodecParameters *par;
    AVIOContext *pb = s->pb;
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    avio_skip(pb, 31);
    if (avio_r8(pb))
        return AVERROR_PATCHWELCOME;

    par = st->codecpar;
    st->start_time = 0;
    par->sample_rate = avio_rl16(pb);
    if (par->sample_rate <= 0)
        return AVERROR_INVALIDDATA;
    par->codec_type = AVMEDIA_TYPE_AUDIO;
    par->ch_layout.nb_channels = avio_r8(pb);
    par->bits_per_coded_sample = avio_r8(pb);
    par->codec_id = (par->bits_per_coded_sample > 8) ? AV_CODEC_ID_PCM_S16LE : AV_CODEC_ID_PCM_S8;
    avio_skip(pb, 2);
    par->block_align = par->ch_layout.nb_channels * ((par->bits_per_coded_sample + 7) / 8);
    avio_skip(pb, 4);
    st->duration = avio_rl32(pb);

    avio_seek(pb, avio_rl32(pb), SEEK_SET);

    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    return 0;
}

const FFInputFormat ff_dwd_demuxer = {
    .p.name         = "dwd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("DWD (DiamondWare Digitized)"),
    .p.extensions   = "dwd",
    .read_probe     = dwd_probe,
    .read_header    = dwd_read_header,
    .read_packet    = ff_pcm_read_packet,
    .read_seek      = ff_pcm_read_seek,
};
