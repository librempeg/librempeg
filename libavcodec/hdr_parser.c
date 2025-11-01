/*
 * Radiance HDR parser
 * Copyright (c) 2022 Paul B Mahol
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
 * Radiance HDR parser
 */

#include "libavutil/intreadwrite.h"
#include "parser.h"
#include "parser_internal.h"

typedef struct HDRParseContext {
    ParseContext pc;
} HDRParseContext;

static int hdr_parse(AVCodecParserContext *s, AVCodecContext *avctx,
                     const uint8_t **poutbuf, int *poutbuf_size,
                     const uint8_t *buf, int buf_size)
{
    HDRParseContext *ipc = s->priv_data;
    uint64_t state = ipc->pc.state64;
    int next = END_NOT_FOUND, i = 0;

    *poutbuf_size = 0;
    *poutbuf = NULL;

    if (s->flags & PARSER_FLAG_COMPLETE_FRAMES) {
        next = buf_size;
    } else {
        for (; i < buf_size; i++) {
            state = (state << 8) | buf[i];
            if (state == AV_RB64("ADIANCE\n") && (i > 10 || ipc->pc.index > 10)) {
                next = i - 10;
                break;
            }
        }

        ipc->pc.state64 = state;
        if (ff_combine_frame(&ipc->pc, next, &buf, &buf_size) < 0) {
            *poutbuf = NULL;
            *poutbuf_size = 0;
            return buf_size;
        }
    }

    *poutbuf      = buf;
    *poutbuf_size = buf_size;

    s->pict_type = AV_PICTURE_TYPE_I;
    s->key_frame = 1;
    s->duration  = 1;

    return next;
}

const AVCodecParser ff_hdr_parser = {
    PARSER_CODEC_LIST(AV_CODEC_ID_RADIANCE_HDR),
    .priv_data_size = sizeof(HDRParseContext),
    .parser_parse   = hdr_parse,
    .parser_close   = ff_parse_close,
};
