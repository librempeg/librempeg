/*
 * WwiseVorbis parser
 * Copyright (c) 2024 Paul B Mahol
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
 * WwiseVorbis parser
 */

#include "libavutil/intreadwrite.h"
#include "parser.h"
#include "parser_internal.h"

typedef struct WwVorbisParseContext {
    ParseContext pc;
    int left;
    int size;
} WwVorbisParseContext;

static int wwvorbis_parse(AVCodecParserContext *pc, AVCodecContext *avctx,
                          const uint8_t **poutbuf, int *poutbuf_size,
                          const uint8_t *buf, int buf_size)
{
    WwVorbisParseContext *s = pc->priv_data;
    uint32_t state = s->pc.state;
    int next = END_NOT_FOUND;

    *poutbuf_size = 0;
    *poutbuf = NULL;

    if (pc->flags & PARSER_FLAG_COMPLETE_FRAMES) {
        next = buf_size;
    } else {
        if (avctx->extradata &&
            avctx->extradata_size >= 26) {
            if (pc->cur_offset == AV_RL64(avctx->extradata))
                s->left = AV_RL32(avctx->extradata+22);
        }

        for (int i = 0; i < buf_size; i++) {
            state = (state << 8) | buf[i];

            s->size++;
            if (s->left >= 1) {
                s->left--;
                if (s->left == 0) {
                    next = i+1;
                    s->size = 0;
                    break;
                }
            } else if (s->size == 2) {
                uint32_t val = state & 0xFFFF;

                s->left = AV_RB16(&val);
            }
        }

        s->pc.state = state;
        if (ff_combine_frame(&s->pc, next, &buf, &buf_size) < 0) {
            *poutbuf = NULL;
            *poutbuf_size = 0;
            return buf_size;
        }
    }

    if (avctx->extradata_size >= 48)
        pc->duration = (1 << avctx->extradata[47]) / 2;

    *poutbuf      = buf;
    *poutbuf_size = buf_size;

    return next;
}

const FFCodecParser ff_wwvorbis_parser = {
    PARSER_CODEC_LIST(AV_CODEC_ID_WWVORBIS),
    .priv_data_size = sizeof(WwVorbisParseContext),
    .parse          = wwvorbis_parse,
    .close          = ff_parse_close,
};
