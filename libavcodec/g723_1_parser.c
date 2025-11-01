/*
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
 * G723_1 audio parser
 */

#include "parser.h"
#include "g723_1.h"
#include "parser_internal.h"

typedef struct G723_1ParseContext {
    ParseContext pc;
} G723_1ParseContext;

static int g723_1_parse(AVCodecParserContext *s1, AVCodecContext *avctx,
                        const uint8_t **poutbuf, int *poutbuf_size,
                        const uint8_t *buf, int buf_size)
{
    G723_1ParseContext *s = s1->priv_data;
    ParseContext *pc = &s->pc;
    int next = END_NOT_FOUND;

    if (buf_size > 0)
        next = frame_size[buf[0] & 3] * FFMAX(1, avctx->ch_layout.nb_channels);

    if (ff_combine_frame(pc, next, &buf, &buf_size) < 0 || !buf_size) {
        *poutbuf      = NULL;
        *poutbuf_size = 0;
        return buf_size;
    }

    s1->duration = 240;

    *poutbuf      = buf;
    *poutbuf_size = buf_size;
    return next;
}

const FFCodecParser ff_g723_1_parser = {
    PARSER_CODEC_LIST(AV_CODEC_ID_G723_1),
    .priv_data_size = sizeof(G723_1ParseContext),
    .parse          = g723_1_parse,
    .close          = ff_parse_close,
};
