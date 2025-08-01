/*
 * Copyright (c) 2012 Clément Bœsch
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
 * VPlayer subtitles format demuxer
 */

#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "subtitles.h"

typedef struct {
    FFDemuxSubtitlesQueue q;
} VPlayerContext;

static int vplayer_probe(const AVProbeData *p)
{
    char c;
    const unsigned char *ptr = p->buf;

    if ((sscanf(ptr, "%*3d:%*2d:%*2d.%*2d%c", &c) == 1 ||
         sscanf(ptr, "%*3d:%*2d:%*2d%c",      &c) == 1) && strchr(": =", c))
        return AVPROBE_SCORE_MAX;
    return 0;
}

static int64_t read_ts(char **line)
{
    char c;
    int hh, mm, ss, ms, n, len;

    if (((n = sscanf(*line, "%d:%d:%d.%d%c%n", &hh, &mm, &ss, &ms, &c, &len)) >= 5 ||
         (n = sscanf(*line, "%d:%d:%d%c%n",    &hh, &mm, &ss,      &c, &len)) >= 4) && strchr(": =", c)) {
        *line += len;
        return (hh*3600LL + mm*60LL + ss) * 100LL + (n < 5 ? 0 : ms);
    }
    return AV_NOPTS_VALUE;
}

static int vplayer_read_header(AVFormatContext *s)
{
    VPlayerContext *vplayer = s->priv_data;
    AVStream *st = avformat_new_stream(s, NULL);

    if (!st)
        return AVERROR(ENOMEM);
    avpriv_set_pts_info(st, 64, 1, 100);
    st->codecpar->codec_type = AVMEDIA_TYPE_SUBTITLE;
    st->codecpar->codec_id   = AV_CODEC_ID_VPLAYER;

    while (!avio_feof(s->pb)) {
        char line[4096];
        char *p = line;
        const int64_t pos = avio_tell(s->pb);
        int len = ff_get_line(s->pb, line, sizeof(line));
        int64_t pts_start;

        if (!len)
            break;

        line[strcspn(line, "\r\n")] = 0;

        pts_start = read_ts(&p);
        if (pts_start != AV_NOPTS_VALUE) {
            AVPacket *sub;

            sub = ff_subtitles_queue_insert(&vplayer->q, p, strlen(p), 0);
            if (!sub)
                return AVERROR(ENOMEM);
            sub->pos = pos;
            sub->pts = pts_start;
            sub->duration = -1;
        }
    }

    ff_subtitles_queue_finalize(s, &vplayer->q);
    return 0;
}

const FFInputFormat ff_vplayer_demuxer = {
    .p.name         = "vplayer",
    .p.long_name    = NULL_IF_CONFIG_SMALL("VPlayer subtitles"),
    .p.extensions   = "txt",
    .priv_data_size = sizeof(VPlayerContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = vplayer_probe,
    .read_header    = vplayer_read_header,
    .read_packet    = ff_subtitles_read_packet,
    .read_seek2     = ff_subtitles_read_seek,
    .read_close     = ff_subtitles_read_close,
};
