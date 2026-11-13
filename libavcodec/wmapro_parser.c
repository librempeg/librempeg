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
 * WMAPRO audio parser
 */

#include "libavutil/intreadwrite.h"
#include "avcodec.h"
#include "parser_internal.h"
#include "wma_common.h"

typedef struct WMAPROParserContext{
    int valid_extradata;
    int duration;
} WMAPROParserContext;

static int wmapro_parse(AVCodecParserContext *s1, AVCodecContext *avctx,
                        const uint8_t **poutbuf, int *poutbuf_size,
                        const uint8_t *buf, int buf_size)
{
    WMAPROParserContext *s = s1->priv_data;

    if (!s->valid_extradata && avctx->extradata && avctx->extradata_size >= 16) {
        const int decode_flags = AV_RL16(avctx->extradata + 14);
        const int bits = ff_wma_get_frame_len_bits(avctx->sample_rate, 3, decode_flags);

        s->duration = 1 << bits;
        s->valid_extradata = 1;
    }

    if (!s->valid_extradata)
        goto end;

    s1->duration = s->duration * (buf[0] & 15);

end:
    /* always return the full packet. this parser isn't doing any splitting or
       combining, only packet analysis */
    *poutbuf      = buf;
    *poutbuf_size = buf_size;
    return buf_size;
}

const FFCodecParser ff_wmapro_parser = {
    PARSER_CODEC_LIST(AV_CODEC_ID_WMAPRO),
    .priv_data_size = sizeof(WMAPROParserContext),
    .parse          = wmapro_parse,
};
