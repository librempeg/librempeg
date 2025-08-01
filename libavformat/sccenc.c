/*
 * SCC muxer
 * Copyright (c) 2017 Paul B Mahol
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
#include "internal.h"
#include "mux.h"
#include "libavutil/log.h"
#include "libavutil/intreadwrite.h"

typedef struct SCCContext {
    int prev_h, prev_m, prev_s, prev_f;
    int inside;
    int n;
} SCCContext;

static int scc_write_header(AVFormatContext *avf)
{
    SCCContext *scc = avf->priv_data;

    avpriv_set_pts_info(avf->streams[0], 64, 1, 1000);
    avio_printf(avf->pb, "Scenarist_SCC V1.0\n");

    scc->prev_h = scc->prev_m = scc->prev_s = scc->prev_f = -1;
    scc->inside = 0;

    return 0;
}

static int scc_write_packet(AVFormatContext *avf, AVPacket *pkt)
{
    SCCContext *scc = avf->priv_data;
    int64_t pts = pkt->pts;
    int i, h, m, s, f;

    if (pts == AV_NOPTS_VALUE) {
        av_log(avf, AV_LOG_WARNING,
               "Insufficient timestamps.\n");
        return 0;
    }

    h = (int)(pts / (3600000));
    m = (int)(pts / (60000)) % 60;
    s = (int)(pts /  1000) % 60;
    f = (int)(pts %  1000) / 33;

    for (i = 0; i < pkt->size - 2; i+=3) {
        if (pkt->data[i] == 0xfc && ((pkt->data[i + 1] != 0x80 || pkt->data[i + 2] != 0x80)))
            break;
    }
    if (i >= pkt->size - 2)
        return 0;

    if (!scc->inside && (scc->prev_h != h || scc->prev_m != m || scc->prev_s != s || scc->prev_f != f)) {
        avio_printf(avf->pb, "\n%02d:%02d:%02d:%02d\t", h, m, s, f);
        scc->inside = 1;
    }
    for (i = 0; i < pkt->size; i+=3) {
        if (i + 3 > pkt->size)
            break;

        if (pkt->data[i] != 0xfc || (pkt->data[i + 1] == 0x80 && pkt->data[i + 2] == 0x80))
            continue;
        if (!scc->inside) {
            avio_printf(avf->pb, "\n%02d:%02d:%02d:%02d\t", h, m, s, f);
            scc->inside = 1;
        }
        if (scc->n > 0)
            avio_w8(avf->pb, ' ');
        avio_printf(avf->pb, "%02x%02x", pkt->data[i + 1], pkt->data[i + 2]);
        scc->n++;
    }
    if (scc->inside && (scc->prev_h != h || scc->prev_m != m || scc->prev_s != s || scc->prev_f != f)) {
        avio_w8(avf->pb, '\n');
        scc->n = 0;
        scc->inside = 0;
    }

    scc->prev_h = h;
    scc->prev_m = m;
    scc->prev_s = s;
    scc->prev_f = f;
    return 0;
}

const FFOutputFormat ff_scc_muxer = {
    .p.name           = "scc",
    .p.long_name      = NULL_IF_CONFIG_SMALL("Scenarist Closed Captions"),
    .p.extensions     = "scc",
    .p.flags          = AVFMT_GLOBALHEADER | AVFMT_VARIABLE_FPS | AVFMT_TS_NONSTRICT,
    .p.video_codec    = AV_CODEC_ID_NONE,
    .p.audio_codec    = AV_CODEC_ID_NONE,
    .p.subtitle_codec = AV_CODEC_ID_EIA_608,
    .flags_internal   = FF_OFMT_FLAG_MAX_ONE_OF_EACH |
                        FF_OFMT_FLAG_ONLY_DEFAULT_CODECS,
    .priv_data_size = sizeof(SCCContext),
    .write_header   = scc_write_header,
    .write_packet   = scc_write_packet,
};
