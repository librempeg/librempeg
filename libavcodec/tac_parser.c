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
 * TAC audio parser
 *
 * Splits packets into individual blocks.
 */

#include "libavutil/intreadwrite.h"
#include "parser.h"

typedef struct TACParseContext {
    ParseContext pc;
    int size;
    int key;
    int pos;
    int skip;
} TACParseContext;

static int tac_parse(AVCodecParserContext *s1,
                     AVCodecContext *avctx,
                     const uint8_t **poutbuf, int *poutbuf_size,
                     const uint8_t *buf, int buf_size)
{
    TACParseContext *s = s1->priv_data;
    ParseContext *pc = &s->pc;
    uint32_t state = pc->state;
    int next = END_NOT_FOUND;

    *poutbuf      = NULL;
    *poutbuf_size = 0;

    if (s1->flags & PARSER_FLAG_COMPLETE_FRAMES) {
        next = buf_size;
    } else {
        for (int i = 0; i < buf_size; i++) {
            state = (state << 8) | buf[i];

            if (s->skip > 0) {
                s->skip--;
            } else if (s->size == 0) {
                s->pos++;
                if (s->pos >= 4) {
                    if (state == 0xFFFFFFFFu) {
                        s->skip = 0x4E000u - (s1->cur_offset%0x4E000) - 4;
                        s->size = 0;
                        s->key = 0;
                    } else {
                        s->size = av_bswap16(state & 0xFFFFu);
                        s->key = !(s->size & 0x8000u);
                        s->size &= 0x7FFFu;
                        s->size += 8;
                    }
                    s->pos = 0;
                }
            }

            if (s->size > 0 && s->skip == 0) {
                if (s->size + i - 3 < buf_size) {
                    next = s->size + i - 3;
                    s->size = 0;
                    s->pos = 0;
                    break;
                } else {
                    s->size -= buf_size - i;
                    break;
                }
            }
        }
        pc->state = state;

        if (ff_combine_frame(pc, next, &buf, &buf_size) < 0) {
            *poutbuf      = NULL;
            *poutbuf_size = 0;
            return buf_size;
        }

        s1->duration = 1024;
        s1->key_frame = s->key;
    }

    *poutbuf = buf;
    *poutbuf_size = buf_size;

    return next;
}

const AVCodecParser ff_tac_parser = {
    .codec_ids      = { AV_CODEC_ID_TAC },
    .priv_data_size = sizeof(TACParseContext),
    .parser_parse   = tac_parse,
    .parser_close   = ff_parse_close,
};
