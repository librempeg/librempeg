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
 * WebVTT subtitle decoder
 * @see https://www.w3.org/TR/webvtt1/
 * @todo need to support extended markups and cue settings
 */

#include "avcodec.h"
#include "ass.h"
#include "codec_internal.h"
#include "libavutil/bprint.h"

static const struct {
    const char *from;
    const char *to;
} webvtt_tag_replace[] = {
    {"<i>", "{\\i1}"}, {"</i>", "{\\i0}"},
    {"<b>", "{\\b1}"}, {"</b>", "{\\b0}"},
    {"<u>", "{\\u1}"}, {"</u>", "{\\u0}"},
    {"{", "\\{{}"}, {"\\", "\\\xe2\x81\xa0"}, // escape to avoid ASS markup conflicts
    {"&gt;", ">"}, {"&lt;", "<"},
    {"&lrm;", "\xe2\x80\x8e"}, {"&rlm;", "\xe2\x80\x8f"},
    {"&amp;", "&"}, {"&nbsp;", "\\h"},
};

static int webvtt_event_to_ass(AVBPrint *buf, const char *p)
{
    int i, again = 0, skip = 0;

    while (*p) {

        for (i = 0; i < FF_ARRAY_ELEMS(webvtt_tag_replace); i++) {
            const char *from = webvtt_tag_replace[i].from;
            const size_t len = strlen(from);
            if (!strncmp(p, from, len)) {
                av_bprintf(buf, "%s", webvtt_tag_replace[i].to);
                p += len;
                again = 1;
                break;
            }
        }
        if (!*p)
            break;

        if (again) {
            again = 0;
            skip = 0;
            continue;
        }
        if (*p == '<')
            skip = 1;
        else if (*p == '>')
            skip = 0;
        else if (p[0] == '\n' && p[1])
            av_bprintf(buf, "\\N");
        else if (!skip && *p != '\r')
            av_bprint_chars(buf, *p, 1);
        p++;
    }
    return 0;
}

static int webvtt_decode_frame(AVCodecContext *avctx, AVSubtitle *sub,
                               int *got_sub_ptr, const AVPacket *avpkt)
{
    int ret = 0;
    const char *ptr = avpkt->data;
    FFASSDecoderContext *s = avctx->priv_data;
    AVBPrint buf;

    av_bprint_init(&buf, 0, AV_BPRINT_SIZE_UNLIMITED);
    if (ptr && avpkt->size > 0 && !webvtt_event_to_ass(&buf, ptr))
        ret = ff_ass_add_rect(sub, buf.str, s->readorder++, 0, NULL, NULL);
    av_bprint_finalize(&buf, NULL);
    if (ret < 0)
        return ret;
    *got_sub_ptr = sub->num_rects > 0;
    return avpkt->size;
}

const FFCodec ff_webvtt_decoder = {
    .p.name         = "webvtt",
    CODEC_LONG_NAME("WebVTT subtitle"),
    .p.type         = AVMEDIA_TYPE_SUBTITLE,
    .p.id           = AV_CODEC_ID_WEBVTT,
    FF_CODEC_DECODE_SUB_CB(webvtt_decode_frame),
    .init           = ff_ass_subtitle_header_default,
    .flush          = ff_ass_decoder_flush,
    .priv_data_size = sizeof(FFASSDecoderContext),
};
