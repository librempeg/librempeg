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

#include "libavutil/intreadwrite.h"
#include "parser.h"

typedef struct FastAudioParseContext {
    ParseContext pc;
    int left;
} FastAudioParseContext;

static int fastaudio_parse(AVCodecParserContext *s1, AVCodecContext *avctx,
                           const uint8_t **poutbuf, int *poutbuf_size,
                           const uint8_t *buf, int buf_size)
{
    const int block_size = avctx->block_align;
    FastAudioParseContext *a = s1->priv_data;
    int next = END_NOT_FOUND;
    int size = a->left + buf_size;

    size = (size / block_size) * block_size;
    if (size == 0) {
        a->left += buf_size;
    } else {
        next = size - a->left;
        a->left = 0;
    }

    if (ff_combine_frame(&a->pc, next, &buf, &buf_size) < 0) {
        *poutbuf = NULL;
        *poutbuf_size = 0;
        return buf_size;
    }

    *poutbuf      = buf;
    *poutbuf_size = buf_size;

    return next;
}

const AVCodecParser ff_fastaudio_parser = {
    .codec_ids      = { AV_CODEC_ID_FASTAUDIO, AV_CODEC_ID_ADPCM_IMA_MO },
    .priv_data_size = sizeof(FastAudioParseContext),
    .parser_parse   = fastaudio_parse,
};
