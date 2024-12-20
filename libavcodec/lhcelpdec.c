/*
 * L&H CELP decoder
 * Copyright (c) 2024 Peter Ross
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

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#define BITSTREAM_READER_LE
#include "get_bits.h"
#include "lhcelpdata.h"

typedef struct LHCELPContext {
    int16_t previous[10];
    int16_t interim[148];
    int16_t filter1[11];
    int32_t filter2[11];
    int16_t output[202];
} LHCELPContext;

typedef struct Subframe {
    int position;
    int sf1_idx;
    int opcode;
    int sf2_idx;
} Subframe;

static av_cold int lhcelp_decode_init(AVCodecContext *avctx)
{
    LHCELPContext *s = avctx->priv_data;

    avctx->ch_layout.nb_channels = 1;
    avctx->sample_rate = 8000;

    for (int i = 0; i < 10; i++)
        s->previous[i] = lhcelp_init[i];

    avctx->sample_fmt = AV_SAMPLE_FMT_S16;

    return 0;
}

static void parse_bitstream(LHCELPContext *s, Subframe *subframe, int16_t *coeffs, const uint8_t *data)
{
    GetBitContext gb;
    uint8_t idx[10];

    init_get_bits8(&gb, data, 12);
    subframe[1].sf2_idx  = get_bits(&gb, 5);
    idx[2] = get_bits(&gb, 4);
    idx[1] = get_bits(&gb, 4);
    idx[0] = get_bits(&gb, 3);
    subframe[0].opcode   = get_bits(&gb, 8);
    subframe[2].sf2_idx  = get_bits(&gb, 5);
    idx[3] = get_bits(&gb, 3);
    idx[9] = get_bits1(&gb);
    idx[8] = get_bits(&gb, 3);
    idx[7] = get_bits(&gb, 2);
    idx[6] = get_bits(&gb, 3);
    idx[5] = get_bits(&gb, 3);
    idx[4] = get_bits(&gb, 4);
    subframe[0].sf2_idx  = get_bits(&gb, 5);
    subframe[0].sf1_idx  = get_bits(&gb, 4);
    subframe[0].position = get_bits(&gb, 7) + 20;
    subframe[1].opcode   = get_bits(&gb, 8);
    subframe[1].sf1_idx  = get_bits(&gb, 4);
    subframe[1].position = get_bits(&gb, 4) + subframe[0].position - 7;
    subframe[2].opcode   = get_bits(&gb, 8);
    subframe[2].sf1_idx  = get_bits(&gb, 4);
    subframe[2].position = get_bits(&gb, 4) + subframe[1].position - 7;

    for (int i = 0; i < 5; i++)
        coeffs[i*2] = lhcelp_cbs[i][idx[i*2]];

    /* for odd coeffs, build cb using delta of the adjacent even ceoffs */
    coeffs[10] = 0x7fff;
    for (int i = 0; i < 5; i++) {
        int16_t cb[16];
        int16_t *ptr = cb;
        int delta = coeffs[i*2 + 2] - coeffs[i*2];

        for (int j = 0; j < lhcelp_scale_length[i]; j++)
            *ptr++ = ((lhcelp_scale_a[i][j] * delta + 0x8000) >> 16) + coeffs[i*2];

        for (int j = 0; j < lhcelp_scale_length[i] - 2; j++)
            *ptr++ = ((lhcelp_scale_b[i][j] * -delta) >> 16) + coeffs[i*2 + 2];

        coeffs[i*2 + 1] = cb[idx[i*2 + 1]];
    }
}

static void process_coeffs2(int16_t *coeffs)
{
    int coeff, negative, steps, jump, offset, shift, num, den, new_coeff;
    const int16_t *table;

    for (int i = 0; i < 10 ; i++) {
        coeff = coeffs[i];
        negative = coeff >= 0x4000;
        if (negative)
            coeff = 0x7fff - coeff;

        if (coeff < 1849) {
            table = lhcelp_fast + 17;
            steps = 3;
            jump = 16;
            offset = 0x200;
            shift = 10;
        } else {
            table = lhcelp_slow + 129;
            steps = 6;
            jump = 128;
            offset = 0x40;
            shift = 7;
        }

        table += jump >> 1;
        for (int i = 0; i < steps; i++) {
            int n = jump >> (i + 2);
            table += coeff <= table[0] ? -n : n;
        }
        if (coeff <= table[0])
            table--;

        num = (int16_t)(coeff - table[0]) << 16;
        den = (table[1] - table[0]) << 1;
        new_coeff = table[-jump - 1] - ((int16_t)(num / den + offset) >> shift);

        coeffs[i] = negative ? -new_coeff : new_coeff;
    }
}

static void process_coeffs3(int16_t *coeffs)
{
    int coeff, width, ap, bp;
    int32_t stack[22];

    stack[0]  = 0x400000;
    stack[2]  = 0x400000;
    stack[11] = 0x400000;
    stack[13] = 0x400000;
    stack[1]  = coeffs[0] * -0x100;
    stack[12] = coeffs[1] * -0x100;

    width = 1;
    ap = 2;
    bp = 13;

    for (int i = 0; i < 10 - 2; i += 2) {
        stack[ap + 2] = 0x400000;
        coeff = coeffs[i + 2];
        stack[ap + 1] = stack[ap - 1] - coeff * 0x100;

        for (int j = 0; j < width; j++) {
            stack[ap] -= (stack[ap - 1] * (int64_t)coeff + 0x2000) >> 14;
            stack[ap] += stack[ap - 2];
            ap--;
        }

        stack[ap] -= (stack[ap - 1] * (int64_t)coeff + 0x2000) >> 14;

        stack[bp + 2] = 0x400000;
        coeff = coeffs[i + 3];
        stack[bp + 1] = stack[bp - 1] - coeff * 0x100;

        for (int j = 0; j < width; j++) {
            stack[bp] -= (stack[bp - 1] * (int64_t)coeff + 0x2000) >> 14;
            stack[bp] += stack[bp - 2];
            bp--;
        }

        stack[bp] -= (stack[ap - 1] * (int64_t)coeff + 0x2000) >> 14;

        width += 2;
        bp += width;
        ap += width;
    }

    coeffs[0] = 0x400;
    for (int i = 0; i < 10; i++)
        coeffs[i + 1] = (stack[i] + stack[i + 1] + stack[i + 12] - stack[i + 11] + 0x1000) >> 13;
}

static void mix(const int16_t *src1, const int16_t *src2, int16_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        int v = 2 * src1[i] + src2[i];
        dst[i] = (21845 * v + 0x8000) >> 16;
    }
}

static void process_coeffs(LHCELPContext *s, int16_t *initial_coeffs, int16_t final_coeffs[3][11])
{
    mix(s->previous, initial_coeffs, final_coeffs[0], 10);
    process_coeffs2(final_coeffs[0]);
    process_coeffs3(final_coeffs[0]);

    mix(initial_coeffs, s->previous, final_coeffs[1], 10);
    process_coeffs2(final_coeffs[1]);
    process_coeffs3(final_coeffs[1]);

    memcpy(final_coeffs[2], initial_coeffs, 2*10);
    process_coeffs2(final_coeffs[2]);
    process_coeffs3(final_coeffs[2]);

    memcpy(s->previous, initial_coeffs, 2*10);
}

static void memcpy_word_naive(int16_t *dst, const int16_t *src, int count)
{
    for (int i = 0; i < count; i++)
        dst[i] = src[i];
}

static void scale(const int16_t *src, int16_t *dst, int scale, int count)
{
    for (int i = 0; i < count; i++)
        dst[i] = (scale * src[i] + 0x1000) >> 13;
}

static void apply_sf1(LHCELPContext *s, const Subframe *subframe, int subframe_size, int sf1)
{
    int position = subframe->position;
    if (position < subframe_size) {
        memcpy(s->output, s->output + 201 - position, 2*position);
        memcpy_word_naive(s->output + position, s->output, subframe_size - position);  //must use naive copy
        scale(s->output, s->output, sf1, subframe_size);
    } else {
        scale(s->output + 201 - position, s->output, sf1, subframe_size);
    }
}

static void make_pulse(int16_t *pulse, int mask, int count)
{
    pulse[0] = 1;
    for (int i = 0; i < count - 1; i++)
        pulse[count - 1 - i] = mask & (1 << i) ? 1 : -1;
}

static void exciter(LHCELPContext *s, const Subframe *subframe, int subframe_size)
{
    int position = subframe->position;
    int sf2_idx = subframe->sf2_idx;
    int sf2 = sf2_idx < 16 ? lhcelp_sf2[sf2_idx] : -lhcelp_sf2[sf2_idx - 16];
    int opcode, offset, length, strip_size;
    int16_t pulse[8];

    opcode = subframe->opcode;
    if (opcode >= 54) {
        if (opcode >= 64) {
            if (opcode >= 128) {
                offset = 0;
                length = 8;
                strip_size = 7;
                make_pulse(pulse, opcode - 128, length);
            } else {
                offset = 3;
                length = 7;
                strip_size = 8;
                make_pulse(pulse, opcode - 64, length);
            }
        } else {
            length = opcode - 54;
            if (position >= subframe_size) {
                strip_size = position / 2;
                if (strip_size >= subframe_size)
                    strip_size = position / 3;
            } else {
                strip_size = subframe_size + 5;
            }
        }
    } else {
        strip_size = position;
        length = opcode;
    }

    if (opcode >= 64) {
        for (int i = 0; i < length; i++)
            s->output[offset + i*strip_size] += sf2 * pulse[i];
    } else {
        for (int i = length; i < subframe_size; i += strip_size)
            s->output[i] += sf2;
    }

    memmove(s->output + 54,                  s->output + 54 + subframe_size, 2*(148 - subframe_size));
    memcpy( s->output + 201 - subframe_size, s->output,                      2*subframe_size);
}

static void add_scaled(const int16_t *src, int16_t *dst, int scale, int count)
{
    for (int i = 0; i < count; i++)
        dst[i] += (scale * src[i] + 0x1000) >> 13;
}

static void lpc_filter(int16_t *filter, const int16_t *coeffs, const int16_t *src, int16_t *dst, int subframe_size, int width)
{
    for (int i = 0; i < subframe_size; i++) {
        int v = 0;
        filter[0] = -src[i];
        for (int j = 0; j < width + 1; j++) {
            int a = filter[width - j];
            filter[width - j + 1] = a;
            v -= coeffs[width - j] * a;
        }
        dst[i] = filter[1] = (v + 0x200) >> 10;
    }
}

static void synthesis(LHCELPContext *s, Subframe *subframe, int subframe_size, int sf1, const int16_t *coeffs)
{
    int position = subframe->position;

    if (sf1 >= 8192)
        sf1 = 2867;
    else if (sf1 <= -8192)
        sf1 = -2867;
    else
        sf1 = 35 * sf1 / 100;

    if (position < subframe_size) {
        add_scaled(s->interim + 148 - position, s->output,            sf1, position);
        add_scaled(s->output,                   s->output + position, sf1, subframe_size - position);
    } else {
        add_scaled(s->interim + 148 - position, s->output,             sf1, subframe_size);
    }

    memmove(s->interim,                      s->interim + subframe_size, 2*(148 - subframe_size));
    memcpy(s->interim + 148 - subframe_size, s->output,                  2*subframe_size);

    lpc_filter(s->filter1, coeffs, s->output, s->output, subframe_size, 10);
}

static void filter_output(int *filter, const int16_t *src, int16_t *dst, int subframe_size, int width)
{
    int end = 2 * width;
    int sample;
    for (int j = 0; j < subframe_size; j++) {
        int64_t v = 0;
        filter[0] = src[j];
        for (int k = 0; k < width; k++) {
            int a = filter[end + 1 - k];
            filter[end + 2 - k] = a;
            v -= lhcelp_output_filter[end - k] * (int64_t)a;
        }
        for (int i = 0; i < width + 1; i++) {
            int a = filter[end - width - i];
            filter[end - width + 1 - i] = a;
            v += lhcelp_output_filter[end - width - i] * (int64_t)a;
        }
        filter[6] = (v + 0x2000) >> 14;
        sample = filter[6] >> 14;
#if 1
        dst[j] = FFMIN(FFMAX(-32767, sample), 32767); //bit identical to reference decoder
#else
        dst[j] = av_clip_int16(sample);
#endif
    }
}

static int lhcelp_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                               int *got_frame_ptr, AVPacket *avpkt)
{
    LHCELPContext *s = avctx->priv_data;
    int16_t initial_coeffs[11], final_coeffs[3][11], *dst;
    Subframe subframe[3];
    int ret;

    if (avpkt->size < 12)
        return AVERROR_INVALIDDATA;

    frame->nb_samples = 160;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    parse_bitstream(s, subframe, initial_coeffs, avpkt->data);
    process_coeffs(s, initial_coeffs, final_coeffs);

    dst = (int16_t *)frame->data[0];
    for (int i = 0; i < 3; i++) {
        int subframe_size = i ? 53 : 54;
        int sf1_idx = subframe[i].sf1_idx;
        int sf1 = sf1_idx < 10 ? lhcelp_sf1[sf1_idx] : -lhcelp_sf1[sf1_idx - 10];
        apply_sf1(s, subframe + i, subframe_size, sf1);
        exciter(s, subframe + i, subframe_size);
        synthesis(s, subframe + i, subframe_size, sf1, final_coeffs[i]);
        filter_output(s->filter2, s->output, dst, subframe_size, 4);
        dst += subframe_size;
    }

    *got_frame_ptr = 1;

    return 12;
}

const FFCodec ff_lhcelp_decoder = {
    .p.name         = "lhcelp",
    CODEC_LONG_NAME("L&H CELP"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_LHCELP,
    .priv_data_size = sizeof(LHCELPContext),
    .init           = lhcelp_decode_init,
    FF_CODEC_DECODE_CB(lhcelp_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_CHANNEL_CONF,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
