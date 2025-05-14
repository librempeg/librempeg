/*
 * VICAR image decoder
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <float.h>

#include "libavutil/intreadwrite.h"
#include "libavutil/intfloat.h"
#include "libavutil/dict.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "bytestream.h"

static void skip_value(GetByteContext *gb)
{
    while (bytestream2_get_bytes_left(gb) > 0) {
        if (bytestream2_peek_byte(gb) == ' ')
            break;
        bytestream2_skip(gb, 1);
    }
}

static int get_keyword(GetByteContext *gb, const uint8_t *keyword, const uint8_t **value)
{
    *value = NULL;

    while (bytestream2_get_bytes_left(gb) > 0) {
        if (bytestream2_peek_byte(gb) != ' ')
            break;
        bytestream2_skip(gb, 1);
    }

    if (!strncmp(gb->buffer, keyword, strlen(keyword))) {
        bytestream2_skip(gb, strlen(keyword));

        while (bytestream2_get_bytes_left(gb) > 0) {
            if (bytestream2_peek_byte(gb) == ' ')
                bytestream2_skip(gb, 1);

            if (bytestream2_peek_byte(gb) == '=') {
                bytestream2_skip(gb, 1);
                break;
            }
        }

        while (bytestream2_get_bytes_left(gb) > 0) {
            if (bytestream2_peek_byte(gb) != ' ')
                break;
            bytestream2_skip(gb, 1);
        }

        if (bytestream2_get_bytes_left(gb) > 0) {
            *value = gb->buffer;
            return 0;
        }
    }

    return AVERROR_INVALIDDATA;
}

static int get_any_keyword(GetByteContext *gb, uint8_t *keyword, const uint8_t **value)
{
    *value = NULL;

    while (bytestream2_get_bytes_left(gb) > 0) {
        if (bytestream2_peek_byte(gb) != ' ')
            break;
        bytestream2_skip(gb, 1);
    }

    for (int i = 0; i < 32; i++) {
        if (bytestream2_peek_byte(gb) == ' ' ||
            bytestream2_peek_byte(gb) == '=')
            break;
        keyword[i] = bytestream2_get_byte(gb);
    }

    while (bytestream2_get_bytes_left(gb) > 0) {
        if (bytestream2_get_byte(gb) == '=')
            break;
    }

    while (bytestream2_get_bytes_left(gb) > 0) {
        if (bytestream2_peek_byte(gb) != ' ')
            break;
        bytestream2_skip(gb, 1);
    }

    if (bytestream2_get_bytes_left(gb) > 0) {
        *value = gb->buffer;
        return 0;
    }

    return AVERROR_INVALIDDATA;
}

static void get_value(GetByteContext *gb, char *value, const int size)
{
    while (bytestream2_get_bytes_left(gb) > 0) {
        if (bytestream2_peek_byte(gb) != ' ')
            break;
        bytestream2_skip(gb, 1);
    }

    for (int n = 0; n < size; n++) {
        if (bytestream2_peek_byte(gb) == ' ') {
            value[n] = '\0';
            break;
        }
        value[n] = bytestream2_get_byte(gb);
    }
}

static int vicar_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                              int *got_frame, AVPacket *avpkt)
{
    int step, eol = 0, w = 0, h = 0, depth = 0, nbb = 0, nlb = 0, nb = 0, n1 = 0;
    char str_value[512] = { 0 };
    GetByteContext gbyte;
    GetByteContext *gb = &gbyte;
    const uint8_t *value;
    uint32_t data_pos;
    int ret;

    bytestream2_init(gb, avpkt->data, avpkt->size);

    ret = get_keyword(gb, "LBLSIZE", &value);
    if (ret < 0)
        return ret;

    get_value(gb, str_value, sizeof(str_value)-1);
    data_pos = atoi(str_value);
    skip_value(gb);

    while (bytestream2_tell(gb) < data_pos) {
        char keyword[33] = { 0 }, str_value[512] = { 0 };

        ret = get_any_keyword(gb, keyword, &value);
        if (ret == 0) {
            if (!strcmp("FORMAT", keyword)) {
                if (!strncmp(value, "'BYTE'", strlen("'BYTE'")))
                    depth = 8;
                skip_value(gb);
            } else if (!strcmp("TYPE", keyword)) {
                if (strncmp(value, "'IMAGE'", strlen("'IMAGE'")))
                    return AVERROR_INVALIDDATA;
                skip_value(gb);
            } else if (!strcmp("BUFSIZ", keyword)) {
                skip_value(gb);
            } else if (!strcmp("DIM", keyword)) {
                skip_value(gb);
            } else if (!strcmp("EOL", keyword)) {
                get_value(gb, str_value, sizeof(str_value)-1);
                eol = atoi(str_value);
            } else if (!strcmp("RECSIZE", keyword)) {
                skip_value(gb);
            } else if (!strcmp("ORG", keyword)) {
                skip_value(gb);
            } else if (!strcmp("NBB", keyword)) {
                get_value(gb, str_value, sizeof(str_value)-1);
                nbb = atoi(str_value);
            } else if (!strcmp("NLB", keyword)) {
                get_value(gb, str_value, sizeof(str_value)-1);
                nlb = atoi(str_value);
            } else if (!strcmp("NL", keyword)) {
                get_value(gb, str_value, sizeof(str_value)-1);
                h = atoi(str_value);
            } else if (!strcmp("NS", keyword)) {
                get_value(gb, str_value, sizeof(str_value)-1);
                w = atoi(str_value);
            } else if (!strcmp("NB", keyword)) {
                get_value(gb, str_value, sizeof(str_value)-1);
                nb = atoi(str_value);
            } else if (!strcmp("N1", keyword)) {
                get_value(gb, str_value, sizeof(str_value)-1);
                n1 = atoi(str_value);
            } else {
                skip_value(gb);
            }
        }
    }

    if (nb == 1) {
        switch (depth) {
        case 8:
            avctx->pix_fmt = AV_PIX_FMT_GRAY8;
            break;
        }
    }

    if (nb == 3) {
        switch (depth) {
        case 8:
            avctx->pix_fmt = AV_PIX_FMT_GBRP;
            break;
        }
    }

    step = (depth + 7) / 8;

    if ((ret = ff_set_dimensions(avctx, w, h)) < 0)
        return ret;

    bytestream2_seek(gb, data_pos, SEEK_SET);

    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    for (int p = 0; p < nb; p++) {
        uint8_t *dst = frame->data[p];

        for (int y = 0; y < h; y++) {
            if (nbb > 0)
                bytestream2_skip(gb, nbb);
            bytestream2_get_buffer(gb, dst, w * step);
            dst += frame->linesize[p];
        }
    }

    if (eol || nlb || n1) {
    }

    *got_frame = 1;

    return avpkt->size;
}

const FFCodec ff_vicar_decoder = {
    .p.name         = "vicar",
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_VICAR,
    .p.capabilities = AV_CODEC_CAP_DR1,
    CODEC_LONG_NAME("VICAR (Video Image Communication And Retrieval)"),
    FF_CODEC_DECODE_CB(vicar_decode_frame),
};
