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

#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int read_probe(const AVProbeData *p)
{
    if (p->buf_size < 24)
        return 0;

    if (memcmp(p->buf, "DiamondWare Digitized\n\0\x1a", 24))
        return 0;

    if (p->buf_size < 37)
        return 0;

    if (AV_RL16(p->buf + 32) == 0)
        return 0;

    if (AV_RL8(p->buf + 34) == 0)
        return 0;

    if (AV_RL8(p->buf + 35) == 0)
        return 0;

    if (AV_RL8(p->buf + 36) == 0)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int read_header(AVFormatContext *s)
{
    AVCodecParameters *par;
    AVIOContext *pb = s->pb;
    int rate, bps, channels;
    int64_t duration;
    AVStream *st;

    avio_skip(pb, 31);
    if (avio_r8(pb))
        return AVERROR_PATCHWELCOME;

    rate = avio_rl16(pb);
    channels = avio_r8(pb);
    bps = avio_r8(pb);
    avio_skip(pb, 6);
    duration = avio_rl32(pb);
    if (rate <= 0 || channels == 0 || bps == 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    par = st->codecpar;
    st->start_time = 0;
    par->sample_rate = rate;
    par->codec_type = AVMEDIA_TYPE_AUDIO;
    par->ch_layout.nb_channels = channels;
    par->bits_per_coded_sample = bps;
    par->codec_id = (bps > 8) ? AV_CODEC_ID_PCM_S16LE : AV_CODEC_ID_PCM_S8;
    par->block_align = channels * ((bps + 7) / 8);
    st->duration = duration;

    avio_seek(pb, avio_rl32(pb), SEEK_SET);

    avpriv_set_pts_info(st, 64, 1, par->sample_rate);

    return 0;
}

const FFInputFormat ff_dwd_demuxer = {
    .p.name         = "dwd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("DWD (DiamondWare Digitized)"),
    .p.extensions   = "dwd",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = ff_pcm_read_packet,
    .read_seek      = ff_pcm_read_seek,
};
