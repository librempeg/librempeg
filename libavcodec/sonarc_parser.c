/*
 * Sonarc parser
 * Copyright (c) 2025 Paul B Mahol
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
 * Sonarc parser
 */

#include "libavutil/bswap.h"
#include "parser.h"
#include "parser_internal.h"

enum SonarcParseState {
    FRAME_START,
    FRAME_OTHER,
};

typedef struct SonarcParseContext {
    ParseContext pc;
    int channel;
    int size;
    int count;
    unsigned state;
} SonarcParseContext;

static int sonarc_parse(AVCodecParserContext *s, AVCodecContext *avctx,
                        const uint8_t **poutbuf, int *poutbuf_size,
                        const uint8_t *buf, int buf_size)
{
    SonarcParseContext *spc = s->priv_data;
    uint32_t state = spc->pc.state;
    int next = END_NOT_FOUND;

    *poutbuf_size = 0;
    *poutbuf = NULL;

    if (s->flags & PARSER_FLAG_COMPLETE_FRAMES) {
        next = buf_size;
    } else {
        for (int i = 0; i < buf_size; i++) {
            state = (state << 8) | buf[i];
            spc->count++;
            if (spc->state == FRAME_START && spc->count == 4) {
                spc->state = FRAME_OTHER;
                spc->count = 0;
                spc->size = av_bswap16(state >> 16);
                spc->size = FFMAX(spc->size, 4) - 3;
                s->duration = av_bswap16(state & 0xffff);
            } else if (spc->state == FRAME_OTHER && spc->size > 0 && spc->count == spc->size) {
                spc->state = FRAME_START;
                spc->channel++;
                if (spc->channel == avctx->ch_layout.nb_channels) {
                    spc->channel = 0;
                    spc->size = 0;
                    spc->count = 0;
                    next = i;
                    break;
                } else {
                    spc->count = 1;
                }
            }
        }

        spc->pc.state = state;
        if (ff_combine_frame(&spc->pc, next, &buf, &buf_size) < 0) {
            *poutbuf = NULL;
            *poutbuf_size = 0;
            return buf_size;
        }
    }

    *poutbuf      = buf;
    *poutbuf_size = buf_size;

    return next;
}

const FFCodecParser ff_sonarc_parser = {
    PARSER_CODEC_LIST(AV_CODEC_ID_SONARC),
    .priv_data_size = sizeof(SonarcParseContext),
    .parser_parse   = sonarc_parse,
    .parser_close   = ff_parse_close,
};
