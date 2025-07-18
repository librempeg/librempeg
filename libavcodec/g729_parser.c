/*
 * Copyright (c) 2015  Ganesh Ajjanagadde
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
 * G.729 audio parser
 *
 * Splits packets into individual blocks.
 */

#include "parser.h"
#include "g729.h"

typedef struct G729ParseContext {
    ParseContext pc;
    int block_size;
    int duration;
    int remaining;
} G729ParseContext;

static int g729_parse(AVCodecParserContext *s1, AVCodecContext *avctx,
                     const uint8_t **poutbuf, int *poutbuf_size,
                     const uint8_t *buf, int buf_size)
{
    G729ParseContext *s = s1->priv_data;
    ParseContext *pc = &s->pc;
    int next;

    if (!s->block_size) {
        /* FIXME: replace this heuristic block_size with more precise estimate */
        s->block_size = (avctx->bit_rate < 8000) ? G729D_6K4_BLOCK_SIZE : G729_8K_BLOCK_SIZE;
        if (avctx->codec_id == AV_CODEC_ID_ACELP_KELVIN)
            s->block_size++;
        // channels > 2 is invalid, we pass the packet on unchanged
        if (avctx->ch_layout.nb_channels > 2)
            s->block_size = 0;
        s->block_size *= avctx->ch_layout.nb_channels;
        s->duration   = avctx->frame_size;
    }

    if (!s->block_size) {
        *poutbuf      = buf;
        *poutbuf_size = buf_size;
        return buf_size;
    }

    if (!s->remaining)
        s->remaining = s->block_size;
    if (s->remaining <= buf_size) {
        next = s->remaining;
        s->remaining = 0;
    } else {
        next = END_NOT_FOUND;
        s->remaining -= buf_size;
    }

    if (ff_combine_frame(pc, next, &buf, &buf_size) < 0 || !buf_size) {
        *poutbuf      = NULL;
        *poutbuf_size = 0;
        return buf_size;
    }

    s1->duration = s->duration;

    *poutbuf      = buf;
    *poutbuf_size = buf_size;
    return next;
}

const AVCodecParser ff_g729_parser = {
    .codec_ids      = { AV_CODEC_ID_G729, AV_CODEC_ID_ACELP_KELVIN },
    .priv_data_size = sizeof(G729ParseContext),
    .parser_parse   = g729_parse,
    .parser_close   = ff_parse_close,
};
