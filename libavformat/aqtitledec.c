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
 * AQTitle subtitles format demuxer
 *
 * @see http://web.archive.org/web/20070210095721/http://www.volny.cz/aberka/czech/aqt.html
 * @see https://trac.annodex.net/wiki/AQTitle
 */

#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "subtitles.h"
#include "libavutil/opt.h"

typedef struct {
    const AVClass *class;
    FFDemuxSubtitlesQueue q;
    AVRational frame_rate;
} AQTitleContext;

static int aqt_probe(const AVProbeData *p)
{
    int frame;
    const char *ptr = p->buf;

    if (sscanf(ptr, "-->> %d", &frame) == 1)
        return AVPROBE_SCORE_EXTENSION;
    return 0;
}

static int aqt_read_header(AVFormatContext *s)
{
    AQTitleContext *aqt = s->priv_data;
    AVStream *st = avformat_new_stream(s, NULL);
    int new_event = 1;
    int64_t pos = 0, frame = AV_NOPTS_VALUE;
    AVPacket *sub = NULL;

    if (!st)
        return AVERROR(ENOMEM);
    avpriv_set_pts_info(st, 64, aqt->frame_rate.den, aqt->frame_rate.num);
    st->codecpar->codec_type = AVMEDIA_TYPE_SUBTITLE;
    st->codecpar->codec_id   = AV_CODEC_ID_TEXT;

    while (!avio_feof(s->pb)) {
        char line[4096];
        int len = ff_get_line(s->pb, line, sizeof(line));

        if (!len)
            break;

        line[strcspn(line, "\r\n")] = 0;

        if (sscanf(line, "-->> %"SCNd64, &frame) == 1) {
            new_event = 1;
            pos = avio_tell(s->pb);
            if (sub) {
                if (frame >= sub->pts && (uint64_t)frame - sub->pts < INT64_MAX)
                    sub->duration = frame - sub->pts;
                sub = NULL;
            }
        } else if (*line) {
            if (!new_event) {
                sub = ff_subtitles_queue_insert(&aqt->q, "\n", 1, 1);
                if (!sub)
                    return AVERROR(ENOMEM);
            }
            sub = ff_subtitles_queue_insert(&aqt->q, line, strlen(line), !new_event);
            if (!sub)
                return AVERROR(ENOMEM);
            if (new_event) {
                sub->pts = frame;
                sub->duration = -1;
                sub->pos = pos;
            }
            new_event = 0;
        }
    }

    ff_subtitles_queue_finalize(s, &aqt->q);
    return 0;
}

static int aqt_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AQTitleContext *aqt = s->priv_data;
    return ff_subtitles_queue_read_packet(&aqt->q, pkt);
}

static int aqt_read_seek(AVFormatContext *s, int stream_index,
                         int64_t min_ts, int64_t ts, int64_t max_ts, int flags)
{
    AQTitleContext *aqt = s->priv_data;
    return ff_subtitles_queue_seek(&aqt->q, s, stream_index,
                                   min_ts, ts, max_ts, flags);
}

static int aqt_read_close(AVFormatContext *s)
{
    AQTitleContext *aqt = s->priv_data;
    ff_subtitles_queue_clean(&aqt->q);
    return 0;
}

#define OFFSET(x) offsetof(AQTitleContext, x)
#define SD AV_OPT_FLAG_SUBTITLE_PARAM|AV_OPT_FLAG_DECODING_PARAM
static const AVOption aqt_options[] = {
    { "subfps", "set the movie frame rate", OFFSET(frame_rate), AV_OPT_TYPE_RATIONAL, {.dbl=25}, 0, INT_MAX, SD },
    { NULL }
};

static const AVClass aqt_class = {
    .class_name = "aqtdec",
    .option     = aqt_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFInputFormat ff_aqtitle_demuxer = {
    .p.name         = "aqtitle",
    .p.long_name    = NULL_IF_CONFIG_SMALL("AQTitle subtitles"),
    .p.extensions   = "aqt",
    .p.priv_class   = &aqt_class,
    .priv_data_size = sizeof(AQTitleContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = aqt_probe,
    .read_header    = aqt_read_header,
    .read_packet    = aqt_read_packet,
    .read_seek2     = aqt_read_seek,
    .read_close     = aqt_read_close,
};
