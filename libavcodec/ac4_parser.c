/*
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
 * AC-4 audio parser
 *
 * Determines the duration for each packet.
 */

#include "libavutil/mem_internal.h"
#include "libavutil/rational.h"
#include "ac4dec_data.h"
#include "get_bits.h"
#include "parser.h"
#include "parser_internal.h"

static int variable_bits(GetBitContext *gb, int bits)
{
    int value = 0;
    int read_more;

    do {
        value += get_bits(gb, bits);
        read_more = get_bits1(gb);
        if (read_more) {
            value <<= bits;
            value += 1 << bits;
        }
    } while (read_more);

    return value;
}

static int ac4_parse(AVCodecParserContext *s1, AVCodecContext *avctx,
                     const uint8_t **poutbuf, int *poutbuf_size,
                     const uint8_t *buf, int buf_size)
{
    int version, frame_rate_index, ret, fs_index;
    AVRational resampling_ratio;
    GetBitContext gbc, *gb = &gbc;

    ret = init_get_bits8(gb, buf, buf_size);
    if (ret < 0)
        return ret;

    version = get_bits(gb, 2);
    if (version == 3)
        version += variable_bits(gb, 2);
    get_bits(gb, 10);

    if (get_bits1(gb)) {
        int nb_wait_frames = get_bits(gb, 3);

        if (nb_wait_frames > 0)
            skip_bits(gb, 2);
    }

    fs_index = get_bits1(gb);
    frame_rate_index = get_bits(gb, 4);
    resampling_ratio = resampling_ratios[frame_rate_index];
    s1->key_frame = get_bits1(gb);

    s1->duration = frame_len_base_48khz[frame_rate_index];
    avctx->sample_rate = fs_index ? 48000 : 44100;
    avctx->sample_rate = av_rescale(avctx->sample_rate,
                                    resampling_ratio.den,
                                    resampling_ratio.num);

    /* always return the full packet. this parser isn't doing any splitting or
       combining, only packet analysis */
    *poutbuf      = buf;
    *poutbuf_size = buf_size;
    return buf_size;
}

const FFCodecParser ff_ac4_parser = {
    PARSER_CODEC_LIST(AV_CODEC_ID_AC4),
    .parser_parse   = ac4_parse,
};
