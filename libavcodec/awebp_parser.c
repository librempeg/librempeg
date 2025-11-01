/*
 * Animation WebP parser
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
 * Animation WebP parser
 */

#include "libavutil/bswap.h"
#include "parser.h"
#include "parser_internal.h"

typedef enum AWebPParseStates {
    AWEBP_HEADER = 1,
    AWEBP_SKIP_CHUNK,
    AWEBP_CHUNK,
} awebp_states;

typedef struct AWebPParseContext {
    ParseContext pc;

    uint32_t chunk_type, chunk_size;
    int64_t pts, dts;
    int delay;
    int state;
    int index;
    int size;
    int first;
} AWebPParseContext;

static int awebp_find_frame_end(AVCodecParserContext *s, AVCodecContext *avctx,
                                AWebPParseContext *w, const uint8_t *buf,
                                int buf_size, void *logctx)
{
    ParseContext *pc = &w->pc;
    uint64_t state = pc->state64;
    int next = END_NOT_FOUND;

    for (int index = 0; index < buf_size; index++) {
        state = (state << 8) | buf[index];

        w->index++;

        if (!w->state) {
            w->state = AWEBP_HEADER;
            w->size = 8;
        }

        if (w->state == AWEBP_HEADER) {
            if (w->index >= w->size) {
                w->index = 0;
                w->chunk_type = state >> 32;
                w->chunk_size = av_bswap32(state & 0xffffffff);
                if (w->chunk_size & 1)
                    w->chunk_size++;

                w->size = w->chunk_size;
                w->state = AWEBP_SKIP_CHUNK;

                if (w->chunk_type == MKBETAG('R','I','F','F')) {
                    w->size = 4;
                    w->first = 0;
                }
            }
        } else if (w->state == AWEBP_SKIP_CHUNK) {
            if (w->chunk_type == MKBETAG('A','N','M','F')) {
                if (w->index == 16) {
                    w->delay = av_bswap32(state & 0xffffff00);
                    w->pts = s->last_pts + s->duration;
                    w->dts = s->last_dts + s->duration;
                }
            }
            if (w->index >= w->size) {
                w->index = 0;
                w->size = 8;
                w->state = AWEBP_CHUNK;
            }
        } else if (w->state == AWEBP_CHUNK) {
            if (w->index >= w->size) {
                w->index = 0;
                w->chunk_type = state >> 32;
                w->chunk_size = av_bswap32(state & 0xffffffff);
                if (w->chunk_size & 1)
                    w->chunk_size++;

                w->size = w->chunk_size;
                w->state = AWEBP_SKIP_CHUNK;

                if (w->chunk_type == MKBETAG('A','N','M','F')) {
                    if (w->first) {
                        next = index - 7;
                        w->index -= 8 - FFMAX(0, -next);

                        s->duration  = (w->delay > 0) ? w->delay : 100;
                        s->pts = w->pts;
                        s->dts = w->dts;
                        s->key_frame = s->pts == 0;
                        s->pict_type = s->key_frame ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;
                        w->delay = 0;

                        break;
                    }
                    w->first = 1;
                }
            }
        }
    }

    if (buf_size == 0) {
        s->duration  = (w->delay > 0) ? w->delay : 100;
        s->pts = w->pts;
        s->dts = w->dts;
        s->key_frame = s->pts == 0;
        s->pict_type = s->key_frame ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;
    }

    w->pc.state64 = state;

    return next;
}

static int awebp_parse(AVCodecParserContext *s, AVCodecContext *avctx,
                       const uint8_t **poutbuf, int *poutbuf_size,
                       const uint8_t *buf, int buf_size)
{
    AWebPParseContext *w = s->priv_data;
    int next;

    *poutbuf_size = 0;
    *poutbuf = NULL;

    if (s->flags & PARSER_FLAG_COMPLETE_FRAMES) {
        next = buf_size;
    } else {
        next = awebp_find_frame_end(s, avctx, w, buf, buf_size, avctx);
        if (ff_combine_frame(&w->pc, next, &buf, &buf_size) < 0) {
            *poutbuf      = NULL;
            *poutbuf_size = 0;
            return buf_size;
        }
    }

    *poutbuf      = buf;
    *poutbuf_size = buf_size;

    return next;
}

const FFCodecParser ff_awebp_parser = {
    PARSER_CODEC_LIST(AV_CODEC_ID_AWEBP),
    .priv_data_size = sizeof(AWebPParseContext),
    .parser_parse   = awebp_parse,
    .parser_close   = ff_parse_close,
};
