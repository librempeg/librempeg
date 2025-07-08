/*
 * PCA parser
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
 * PCA parser
 */

#include "libavutil/bswap.h"
#include "parser.h"

enum PCAParseState {
    FRAME_START,
    FRAME_HISTORY,
    FRAME_OTHER,
};

typedef struct PCAParseContext {
    ParseContext pc;
    int key;
    int size;
    int count;
    unsigned state;
} PCAParseContext;

static int pca_parse(AVCodecParserContext *s, AVCodecContext *avctx,
                     const uint8_t **poutbuf, int *poutbuf_size,
                     const uint8_t *buf, int buf_size)
{
    const unsigned history_size = avctx->ch_layout.nb_channels * 16;
    const unsigned other_size = avctx->ch_layout.nb_channels * (4 + 1 + 1);
    PCAParseContext *ppc = s->priv_data;
    uint32_t state = ppc->pc.state;
    int next = END_NOT_FOUND;

    *poutbuf_size = 0;
    *poutbuf = NULL;

    if (s->flags & PARSER_FLAG_COMPLETE_FRAMES) {
        next = buf_size;
    } else {
        for (int i = 0; i < buf_size; i++) {
            state = (state << 8) | buf[i];
            ppc->count++;
            if (ppc->state == FRAME_START && state == 0xffffffff && ppc->count == 4) {
                ppc->key = 1;
                ppc->state = FRAME_HISTORY;
                ppc->count = 0;
            } else if (ppc->state == FRAME_HISTORY && ppc->count == history_size) {
                ppc->state = FRAME_START;
                ppc->count = 0;
            } else if (ppc->state == FRAME_START && ppc->count == 4) {
                ppc->count = 0;
                ppc->state = FRAME_OTHER;
                s->duration = av_bswap16(state >> 16);
                ppc->size = av_bswap16(state & 0xFFFF) + other_size + 1;
            } else if (ppc->state == FRAME_OTHER && ppc->count == ppc->size) {
                ppc->count = 0;
                ppc->state = FRAME_START;
                s->key_frame = ppc->key;
                ppc->key = 0;
                next = i;
                break;
            }
        }

        ppc->pc.state = state;
        if (ff_combine_frame(&ppc->pc, next, &buf, &buf_size) < 0) {
            *poutbuf = NULL;
            *poutbuf_size = 0;
            return buf_size;
        }
    }

    *poutbuf      = buf;
    *poutbuf_size = buf_size;

    return next;
}

const AVCodecParser ff_pca_parser = {
    .codec_ids      = { AV_CODEC_ID_PCA },
    .priv_data_size = sizeof(PCAParseContext),
    .parser_parse   = pca_parse,
    .parser_close   = ff_parse_close,
};
