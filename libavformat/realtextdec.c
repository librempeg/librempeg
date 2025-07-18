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
 * RealText subtitle demuxer
 * @see http://service.real.com/help/library/guides/ProductionGuide/prodguide/htmfiles/realtext.htm
 */

#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "subtitles.h"
#include "libavutil/avstring.h"
#include "libavutil/bprint.h"
#include "libavutil/mem.h"

typedef struct {
    FFDemuxSubtitlesQueue q;
} RealTextContext;

static int realtext_probe(const AVProbeData *p)
{
    char buf[7];
    FFTextReader tr;
    ff_text_init_buf(&tr, p->buf, p->buf_size);
    ff_text_read(&tr, buf, sizeof(buf));

    return !av_strncasecmp(buf, "<window", 7) ? AVPROBE_SCORE_EXTENSION : 0;
}

static int64_t read_ts(const char *s)
{
    int hh, mm, ss, ms;

    if (sscanf(s, "%u:%u:%u.%u", &hh, &mm, &ss, &ms) == 4) return (hh*3600LL + mm*60LL + ss) * 100LL + ms;
    if (sscanf(s, "%u:%u:%u"   , &hh, &mm, &ss     ) == 3) return (hh*3600LL + mm*60LL + ss) * 100LL;
    if (sscanf(s,    "%u:%u.%u",      &mm, &ss, &ms) == 3) return (            mm*60LL + ss) * 100LL + ms;
    if (sscanf(s,    "%u:%u"   ,      &mm, &ss     ) == 2) return (            mm*60LL + ss) * 100LL;
    if (sscanf(s,       "%u.%u",           &ss, &ms) == 2) return (                      ss) * 100LL + ms;
    return strtoll(s, NULL, 10) * 100ULL;
}

static int realtext_read_header(AVFormatContext *s)
{
    RealTextContext *rt = s->priv_data;
    AVStream *st = avformat_new_stream(s, NULL);
    AVBPrint buf;
    char c = 0;
    int res = 0, duration = read_ts("60"); // default duration is 60 seconds
    FFTextReader tr;
    ff_text_init_avio(s, &tr, s->pb);

    if (!st)
        return AVERROR(ENOMEM);
    avpriv_set_pts_info(st, 64, 1, 100);
    st->codecpar->codec_type = AVMEDIA_TYPE_SUBTITLE;
    st->codecpar->codec_id   = AV_CODEC_ID_REALTEXT;

    av_bprint_init(&buf, 0, AV_BPRINT_SIZE_UNLIMITED);

    while (!ff_text_eof(&tr)) {
        AVPacket *sub;
        const int64_t pos = ff_text_pos(&tr) - (c != 0);
        int n = ff_smil_extract_next_text_chunk(&tr, &buf, &c);

        if (n < 0) {
            res = n;
            goto end;
        }
        if (n == 0)
            break;

        if (!av_strncasecmp(buf.str, "<window", 7)) {
            /* save header to extradata */
            const char *p = ff_smil_get_attr_ptr(buf.str, "duration");

            if (st->codecpar->extradata) {
                res = AVERROR_INVALIDDATA;
                goto end;
            }
            if (p)
                duration = read_ts(p);
            st->codecpar->extradata = av_strdup(buf.str);
            if (!st->codecpar->extradata) {
                res = AVERROR(ENOMEM);
                goto end;
            }
            st->codecpar->extradata_size = buf.len + 1;
        } else {
            /* if we just read a <time> tag, introduce a new event, otherwise merge
             * with the previous one */
            int merge = !av_strncasecmp(buf.str, "<time", 5) ? 0 : 1;
            sub = ff_subtitles_queue_insert_bprint(&rt->q, &buf, merge);
            if (!sub) {
                res = AVERROR(ENOMEM);
                goto end;
            }
            if (!merge) {
                const char *begin = ff_smil_get_attr_ptr(buf.str, "begin");
                const char *end   = ff_smil_get_attr_ptr(buf.str, "end");
                int64_t endi = end ? read_ts(end) : 0;

                sub->pos      = pos;
                sub->pts      = begin ? read_ts(begin) : 0;
                sub->duration = (end && endi > sub->pts && endi - (uint64_t)sub->pts <= INT64_MAX) ? endi - sub->pts : duration;
            }
        }
        av_bprint_clear(&buf);
    }
    ff_subtitles_queue_finalize(s, &rt->q);

end:
    av_bprint_finalize(&buf, NULL);
    return res;
}

const FFInputFormat ff_realtext_demuxer = {
    .p.name         = "realtext",
    .p.long_name    = NULL_IF_CONFIG_SMALL("RealText subtitle format"),
    .p.extensions   = "rt",
    .priv_data_size = sizeof(RealTextContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = realtext_probe,
    .read_header    = realtext_read_header,
    .read_packet    = ff_subtitles_read_packet,
    .read_seek2     = ff_subtitles_read_seek,
    .read_close     = ff_subtitles_read_close,
};
