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
 * SubViewer subtitle decoder
 * @see https://en.wikipedia.org/wiki/SubViewer
 */

#include "avcodec.h"
#include "ass.h"
#include "codec_internal.h"
#include "libavutil/bprint.h"

static int subviewer_event_to_ass(AVBPrint *buf, const char *p)
{
    while (*p) {
        if (!strncmp(p, "[br]", 4)) {
            av_bprintf(buf, "\\N");
            p += 4;
        } else {
            if (p[0] == '\n' && p[1])
                av_bprintf(buf, "\\N");
            else if (*p != '\n' && *p != '\r')
                av_bprint_chars(buf, *p, 1);
            p++;
        }
    }

    return 0;
}

static int subviewer_decode_frame(AVCodecContext *avctx, AVSubtitle *sub,
                                  int *got_sub_ptr, const AVPacket *avpkt)
{
    int ret = 0;
    const char *ptr = avpkt->data;
    FFASSDecoderContext *s = avctx->priv_data;
    AVBPrint buf;

    av_bprint_init(&buf, 0, AV_BPRINT_SIZE_UNLIMITED);
    if (ptr && avpkt->size > 0 && !subviewer_event_to_ass(&buf, ptr))
        ret = ff_ass_add_rect(sub, buf.str, s->readorder++, 0, NULL, NULL);
    av_bprint_finalize(&buf, NULL);
    if (ret < 0)
        return ret;
    *got_sub_ptr = sub->num_rects > 0;
    return avpkt->size;
}

const FFCodec ff_subviewer_decoder = {
    .p.name         = "subviewer",
    CODEC_LONG_NAME("SubViewer subtitle"),
    .p.type         = AVMEDIA_TYPE_SUBTITLE,
    .p.id           = AV_CODEC_ID_SUBVIEWER,
    FF_CODEC_DECODE_SUB_CB(subviewer_decode_frame),
    .init           = ff_ass_subtitle_header_default,
    .flush          = ff_ass_decoder_flush,
    .priv_data_size = sizeof(FFASSDecoderContext),
};
