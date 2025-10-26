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
 * ADPCM HVQM2 parser
 *
 * Determines the duration for each packet.
 */

#include "libavutil/intreadwrite.h"
#include "parser.h"
#include "parser_internal.h"

static int adpcm_hvqm2_parse(AVCodecParserContext *s1, AVCodecContext *avctx,
                             const uint8_t **poutbuf, int *poutbuf_size,
                             const uint8_t *buf, int buf_size)
{
    if (buf_size > 6) {
        s1->duration = AV_RB32(buf+2);
        s1->key_frame = !AV_RB16(buf);
    }

    /* always return the full packet. this parser isn't doing any splitting or
       combining, only packet analysis */
    *poutbuf      = buf;
    *poutbuf_size = buf_size;
    return buf_size;
}

const FFCodecParser ff_adpcm_hvqm2_parser = {
    PARSER_CODEC_LIST(AV_CODEC_ID_ADPCM_IMA_HVQM2),
    .parse = adpcm_hvqm2_parse,
};
