/*
 * CDR format demuxer
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
#include "libavcodec/mathops.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "pcm.h"

static int cdr_probe(const AVProbeData *p)
{
    const uint8_t *buf = p->buf;
    int64_t score_l = 0;
    int64_t score_r = 0;
    int click_l = 0;
    int click_r = 0;
    int count_l = 0;
    int count_r = 0;

    if (p->buf_size < 44100 * 4)
        return 0;

    for (int n = 1; n < p->buf_size/4; n++) {
        const int l0 = sign_extend(AV_RB16(buf + (n-1)*4+0), 16);
        const int l1 = sign_extend(AV_RB16(buf + (n+0)*4+0), 16);
        const int r0 = sign_extend(AV_RB16(buf + (n-1)*4+2), 16);
        const int r1 = sign_extend(AV_RB16(buf + (n+0)*4+2), 16);
        const int al = FFABS(l1 - l0);
        const int ar = FFABS(r1 - r0);

        click_l += al >= 16384;
        if (al) {
            score_l += 256 - al;
            count_l++;
        }

        click_r += ar >= 16384;
        if (ar) {
            score_r += 256 - ar;
            count_r++;
        }
    }

    if (count_l == 0 || count_r == 0)
        return 0;

    if (score_l <= 0 || score_r <= 0)
        return 0;

    score_l /= count_l;
    score_r /= count_r;

    score_l *= AVPROBE_SCORE_MAX;
    score_r *= AVPROBE_SCORE_MAX;

    score_l /= 256;
    score_r /= 256;

    score_l -= click_l;
    score_r -= click_r;

    if (score_l <= 0 || score_r <= 0)
        return 0;

    return FFMIN3(score_l, score_r, AVPROBE_SCORE_MAX);
}

static int cdr_read_header(AVFormatContext *s)
{
    AVStream *st;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_PCM_S16BE;
    st->codecpar->ch_layout.nb_channels = 2;
    st->codecpar->bits_per_coded_sample = 16;
    st->codecpar->sample_rate = 44100;
    st->codecpar->block_align = 4;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

const FFInputFormat ff_cdr_demuxer = {
    .p.name         = "cdr",
    .p.long_name    = NULL_IF_CONFIG_SMALL("CDR (Compact Disc Audio)"),
    .read_probe     = cdr_probe,
    .read_header    = cdr_read_header,
    .read_packet    = ff_pcm_read_packet,
    .read_seek      = ff_pcm_read_seek,
};
