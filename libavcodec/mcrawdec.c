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

#include <stdio.h>
#include <string.h>

#include "libavutil/avstring.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"

#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "thread.h"

#define ENCODING_BLOCK 64

static const uint8_t ENCODING_BLOCK_LENGTH[] = {
    0, 8, 16, 24, 32, 40, 48, 64, 64, 80, 80, 128, 128, 128, 128, 128, 128
};

typedef struct MotionCamRAWContext {
    uint16_t *bits;
    unsigned bits_size;
    uint16_t *refs;
    unsigned refs_size;
} MotionCamRAWContext;

static av_cold int decode_init(AVCodecContext *avctx)
{
    avctx->pix_fmt = AV_PIX_FMT_BAYER_GBRG16;
    avctx->color_trc = AVCOL_TRC_LINEAR;

    return 0;
}

static void decode_header(GetByteContext *gb, uint8_t *bits, uint16_t *reference)
{
    uint8_t b = bytestream2_get_byte(gb);

    *bits = (b & 0xF0) >> 4;
    *reference = (b & 0x0F) << 8 | bytestream2_get_byte(gb);
}

static void decode1(GetByteContext *gb, uint16_t *output)
{
    for (int i = 0; i < 8; i++) {
        uint8_t p = bytestream2_get_byte(gb);

        for (int j = 0; j < 8; j++) {
            output[j] = p & 1;
            p >>= 1;
        }

        output += 8;
    }
}

static void decode2_one(GetByteContext *gb, uint16_t *output)
{
    const unsigned N = 0x03;

    for (int i = 0; i < 8; i++) {
        const unsigned p = bytestream2_get_byte(gb);

        const unsigned r0 =  p & N;
        const unsigned r1 = (p & (N << 2)) >> 2;
        const unsigned r2 = (p & (N << 4)) >> 4;
        const unsigned r3 = (p & (N << 6)) >> 6;

        output[i   ] = r0;
        output[i+ 8] = r1;
        output[i+16] = r2;
        output[i+24] = r3;
    }
}

static void decode2(GetByteContext *gb, uint16_t *output)
{
    decode2_one(gb, output);
    decode2_one(gb, output+32);
}

static void decode3(GetByteContext *gb, uint16_t *output)
{
    const unsigned N = 0x07;
    const unsigned T = 0x03;
    const unsigned R = 0x01;

    GetByteContext gb1 = *gb;
    GetByteContext gb2 = *gb;

    bytestream2_skip(&gb1, 8);
    bytestream2_skip(&gb2, 16);

    for (int i = 0; i < 8; i++) {
        const unsigned p0 = bytestream2_get_byte(gb);
        const unsigned p1 = bytestream2_get_byte(&gb1);
        const unsigned p2 = bytestream2_get_byte(&gb2);

        const unsigned r0  =  p0 & N;
        const unsigned r1  = (p0 & (N << 3)) >> 3;
        const unsigned _r2 = (p0 & (T << 6)) >> 6;

        const unsigned r3  =  p1 & N;
        const unsigned r4  = (p1 & (N << 3)) >> 3;
        const unsigned _r5 = (p1 & (T << 6)) >> 6;

        const unsigned r6  =  p2 & N;
        const unsigned r7  = (p2 & (N << 3)) >> 3;

        const unsigned r2 = _r2 | (((p2 >> 6) & R) << 2);
        const unsigned r5 = _r5 | (((p2 >> 7) & R) << 2);

        output[i   ] = r0;
        output[i+ 8] = r1;
        output[i+16] = r2;
        output[i+24] = r3;
        output[i+32] = r4;
        output[i+40] = r5;
        output[i+48] = r6;
        output[i+56] = r7;
    }

    bytestream2_skip(gb, 16);
}

static void decode4_one(GetByteContext *gb, uint16_t *output)
{
    for (int i = 0; i < 8; i++) {
        const unsigned p = bytestream2_get_byte(gb);
        uint16_t r0 =  p & 0x0F;
        uint16_t r1 = (p & 0xF0) >> 4;

        output[i  ] = r0;
        output[i+8] = r1;
    }
}

static void decode4(GetByteContext *gb, uint16_t *output)
{
    decode4_one(gb, output);
    decode4_one(gb, output+16);
    decode4_one(gb, output+32);
    decode4_one(gb, output+48);
}

static void decode5(GetByteContext *gb, uint16_t *output)
{
    const unsigned N = 0x1F;
    const unsigned L = 0x07;
    const unsigned U = 0x03;
    const unsigned F = 0x01;

    GetByteContext gb1 = *gb;
    GetByteContext gb2 = *gb;
    GetByteContext gb3 = *gb;
    GetByteContext gb4 = *gb;

    bytestream2_skip(&gb1, 8);
    bytestream2_skip(&gb2, 16);
    bytestream2_skip(&gb3, 24);
    bytestream2_skip(&gb4, 32);

    for (int i = 0; i < 8; i++) {
        const unsigned p0 = bytestream2_get_byte(gb);
        const unsigned p1 = bytestream2_get_byte(&gb1);
        const unsigned p2 = bytestream2_get_byte(&gb2);
        const unsigned p3 = bytestream2_get_byte(&gb3);
        const unsigned p4 = bytestream2_get_byte(&gb4);

        const unsigned r0 = p0 & N;
        const unsigned r1 = p1 & N;
        const unsigned r2 = p2 & N;
        const unsigned r3 = p3 & N;
        const unsigned r4 = p4 & N;

        const unsigned r5 = (p0 >> 5) & L | (((p3 >> 5) & U) << 3);
        const unsigned r6 = (p1 >> 5) & L | (((p4 >> 5) & U) << 3);

        const unsigned tmp0 = (p2 >> 5) & L;
        const unsigned tmp1 = tmp0 | ((p3 >> 7) & F) << 3;

        const unsigned r7   = tmp1 | ((p4 >> 7) & F) << 4;

        output[i   ] = r0;
        output[i+ 8] = r1;
        output[i+16] = r2;
        output[i+24] = r3;
        output[i+32] = r4;
        output[i+40] = r5;
        output[i+48] = r6;
        output[i+56] = r7;
    }

    bytestream2_skip(gb, 32);
}

static void decode6(GetByteContext *gb, uint16_t *output)
{
    const unsigned N = 0x3F;
    const unsigned L = 0x03;

    GetByteContext gb1 = *gb;
    GetByteContext gb2 = *gb;
    GetByteContext gb3 = *gb;
    GetByteContext gb4 = *gb;
    GetByteContext gb5 = *gb;

    bytestream2_skip(&gb1, 8);
    bytestream2_skip(&gb2, 16);
    bytestream2_skip(&gb3, 24);
    bytestream2_skip(&gb4, 32);
    bytestream2_skip(&gb5, 40);

    for (int i = 0; i < 8; i++) {
        const unsigned p0 = bytestream2_get_byte(gb);
        const unsigned p1 = bytestream2_get_byte(&gb1);
        const unsigned p2 = bytestream2_get_byte(&gb2);
        const unsigned p3 = bytestream2_get_byte(&gb3);
        const unsigned p4 = bytestream2_get_byte(&gb4);
        const unsigned p5 = bytestream2_get_byte(&gb5);

        const unsigned r0 = p0 & N;
        const unsigned r1 = p1 & N;
        const unsigned r2 = p2 & N;
        const unsigned r3 = p3 & N;
        const unsigned r4 = p4 & N;
        const unsigned r5 = p5 & N;

        const unsigned r6 =
               ((p0 >> 6) & L)
            | (((p1 >> 6) & L) << 2)
            | (((p1 >> 6) & L) << 2)
            | (((p2 >> 6) & L) << 4);

        const unsigned r7 =
               ((p3 >> 6) & L)
            | (((p4 >> 6) & L) << 2)
            | (((p5 >> 6) & L) << 4);

        output[i   ] = r0;
        output[i+ 8] = r1;
        output[i+16] = r2;
        output[i+24] = r3;
        output[i+32] = r4;
        output[i+40] = r5;
        output[i+48] = r6;
        output[i+56] = r7;
    }

    bytestream2_skip(gb, 40);
}

static void decode8(GetByteContext *gb, uint16_t *output)
{
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++)
            output[j] = bytestream2_get_byte(gb);
        output += 8;
    }
}

static void decode10(GetByteContext *gb, uint16_t *output)
{
    const unsigned N = 0xFF;
    const unsigned L = 0x03;

    GetByteContext gb1 = *gb;
    GetByteContext gb2 = *gb;
    GetByteContext gb3 = *gb;
    GetByteContext gb4 = *gb;
    GetByteContext gb5 = *gb;
    GetByteContext gb6 = *gb;
    GetByteContext gb7 = *gb;
    GetByteContext gb8 = *gb;
    GetByteContext gb9 = *gb;

    bytestream2_skip(&gb1, 8);
    bytestream2_skip(&gb2, 16);
    bytestream2_skip(&gb3, 24);
    bytestream2_skip(&gb4, 32);
    bytestream2_skip(&gb5, 40);
    bytestream2_skip(&gb6, 48);
    bytestream2_skip(&gb7, 56);
    bytestream2_skip(&gb8, 64);
    bytestream2_skip(&gb9, 72);

    for (int i = 0; i < 8; i++) {
        const unsigned p0 = bytestream2_get_byte(gb);
        const unsigned p1 = bytestream2_get_byte(&gb1);
        const unsigned p2 = bytestream2_get_byte(&gb2);
        const unsigned p3 = bytestream2_get_byte(&gb3);
        const unsigned p4 = bytestream2_get_byte(&gb4);
        const unsigned p5 = bytestream2_get_byte(&gb5);
        const unsigned p6 = bytestream2_get_byte(&gb6);
        const unsigned p7 = bytestream2_get_byte(&gb7);
        const unsigned p8 = bytestream2_get_byte(&gb8);
        const unsigned p9 = bytestream2_get_byte(&gb9);

        const unsigned _r0 = p0 & N;
        const unsigned _r1 = p1 & N;
        const unsigned _r2 = p2 & N;
        const unsigned _r3 = p3 & N;

        const unsigned r0 = _r0 | ((p4 & L)        << 8);
        const unsigned r1 = _r1 | ((p4 & (L << 2)) << 6);
        const unsigned r2 = _r2 | ((p4 & (L << 4)) << 4);
        const unsigned r3 = _r3 | ((p4 & (L << 6)) << 2);

        const unsigned _r4 = p5 & N;
        const unsigned _r5 = p6 & N;
        const unsigned _r6 = p7 & N;
        const unsigned _r7 = p8 & N;

        const unsigned r4 = _r4 | ((p9 & L)        << 8);
        const unsigned r5 = _r5 | ((p9 & (L << 2)) << 6);
        const unsigned r6 = _r6 | ((p9 & (L << 4)) << 4);
        const unsigned r7 = _r7 | ((p9 & (L << 6)) << 2);

        output[i   ] = r0;
        output[i+ 8] = r1;
        output[i+16] = r2;
        output[i+24] = r3;
        output[i+32] = r4;
        output[i+40] = r5;
        output[i+48] = r6;
        output[i+56] = r7;
    }

    bytestream2_skip(gb, 72);
}

static void decode16(GetByteContext *gb, uint16_t *output)
{
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++)
            output[j] = bytestream2_get_le16(gb);
        output += 8;
    }
}

static int decode_block(GetByteContext *gb,
                        uint16_t *output, const uint16_t bits)
{
    if (ENCODING_BLOCK_LENGTH[bits] > bytestream2_get_bytes_left(gb))
        return AVERROR_INVALIDDATA;

    switch (bits) {
    case 0:
        memset(output, 0, sizeof(*output)*ENCODING_BLOCK);
        break;
    case 1:
        decode1(gb, output);
        break;
    case 2:
        decode2(gb, output);
        break;
    case 3:
        decode3(gb, output);
        break;
    case 4:
        decode4(gb, output);
        break;
    case 5:
        decode5(gb, output);
        break;
    case 6:
        decode6(gb, output);
        break;
    case 7:
    case 8:
        decode8(gb, output);
        break;
    case 9:
    case 10:
        decode10(gb, output);
        break;
    default:
    case 16:
        decode16(gb, output);
        break;
    }

    return 0;
}

static int decode_metadata(AVCodecContext *avctx,
                           GetByteContext *gb,
                           const uint32_t offset,
                           uint16_t **metadata,
                           unsigned *metadata_size)
{
    GetByteContext gb0 = *gb;
    uint32_t nb_blocks;
    uint16_t *data;

    bytestream2_seek(&gb0, offset, SEEK_SET);
    nb_blocks = bytestream2_get_le32(&gb0);

    av_fast_mallocz(metadata, metadata_size, nb_blocks * sizeof(*metadata));
    if (!*metadata)
        return AVERROR(ENOMEM);
    data = *metadata;

    for (int i = 0; i < nb_blocks; i += ENCODING_BLOCK) {
        uint16_t reference;
        uint8_t bits;
        int ret;

        decode_header(&gb0, &bits, &reference);
        ret = decode_block(&gb0, data, bits);
        if (ret < 0)
            return ret;

        for (int x = 0; x < ENCODING_BLOCK; x++)
            data[x] += reference;

        data += ENCODING_BLOCK;
    }

    return 0;
}

static int decode_mcraw(AVCodecContext *avctx, GetByteContext *gb,
                        AVFrame *frame)
{
    const ptrdiff_t linesize = frame->linesize[0] / 2;
    const ptrdiff_t linesize4 = 4 * linesize;
    MotionCamRAWContext *s = avctx->priv_data;
    uint16_t *row0, *row1, *row2, *row3;
    unsigned white_level = UINT16_MAX;
    uint32_t bits_offset, refs_offset;
    char *white_level_str = NULL;
    uint16_t p0[ENCODING_BLOCK];
    uint16_t p1[ENCODING_BLOCK];
    uint16_t p2[ENCODING_BLOCK];
    uint16_t p3[ENCODING_BLOCK];
    int shift, metadata_idx = 0;
    uint16_t *bits, *refs;

    bits_offset = bytestream2_get_le32(gb);
    refs_offset = bytestream2_get_le32(gb);

    decode_metadata(avctx, gb, bits_offset, &s->bits, &s->bits_size);
    decode_metadata(avctx, gb, refs_offset, &s->refs, &s->refs_size);

    row0 = (uint16_t *)frame->data[0];
    row1 = row0 + linesize;
    row2 = row1 + linesize;
    row3 = row2 + linesize;

    bits = s->bits;
    refs = s->refs;

    for (int y = 0; y < avctx->height; y += 4) {
        for (int x = 0; x < avctx->width; x += ENCODING_BLOCK) {
            uint16_t block_bits[4] = { bits[metadata_idx], bits[metadata_idx+1], bits[metadata_idx+2], bits[metadata_idx+3] };
            uint16_t block_refs[4] = { refs[metadata_idx], refs[metadata_idx+1], refs[metadata_idx+2], refs[metadata_idx+3] };

            decode_block(gb, &p0[0], block_bits[0]);
            decode_block(gb, &p1[0], block_bits[1]);
            decode_block(gb, &p2[0], block_bits[2]);
            decode_block(gb, &p3[0], block_bits[3]);

            for (int i = 0; i < ENCODING_BLOCK; i += 2) {
                row0[x+i]   = p0[i/2] + block_refs[0];
                row0[x+i+1] = p1[i/2] + block_refs[1];

                row1[x+i]   = p2[i/2] + block_refs[2];
                row1[x+i+1] = p3[i/2] + block_refs[3];

                row2[x+i]   = p0[ENCODING_BLOCK/2+i/2] + block_refs[0];
                row2[x+i+1] = p1[ENCODING_BLOCK/2+i/2] + block_refs[1];

                row3[x+i]   = p2[ENCODING_BLOCK/2+i/2] + block_refs[2];
                row3[x+i+1] = p3[ENCODING_BLOCK/2+i/2] + block_refs[3];
            }

            metadata_idx += 4;
        }

        row0 += linesize4;
        row1 += linesize4;
        row2 += linesize4;
        row3 += linesize4;
    }

    white_level_str = av_strnstr(gb->buffer, "WhiteLevel\":", bytestream2_get_bytes_left(gb));
    if (white_level_str)
        white_level = atoi(white_level_str + 12);
    shift = 16 - av_ceil_log2(white_level);

    row0 = (uint16_t *)frame->data[0];
    for (int y = 0; y < avctx->height; y++) {
        for (int x = 0; x < avctx->width; x++)
            row0[x] <<= shift;
        row0 += linesize;
    }

    return 0;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame, AVPacket *avpkt)
{
    int ret, width, height;
    GetByteContext gb;

    if (avpkt->size <= 16)
        return AVERROR_INVALIDDATA;

    bytestream2_init(&gb, avpkt->data, avpkt->size);
    width = bytestream2_get_le32(&gb);
    height = bytestream2_get_le32(&gb);

    ret = ff_set_dimensions(avctx, width, height);
    if (ret < 0)
        return ret;

    if (avctx->skip_frame >= AVDISCARD_ALL)
        return avpkt->size;

    if ((ret = ff_thread_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    if ((ret = decode_mcraw(avctx, &gb, frame)) < 0)
        return ret;

    frame->flags |= AV_FRAME_FLAG_KEY;
    frame->pict_type = AV_PICTURE_TYPE_I;

    *got_frame = 1;

    return avpkt->size;
}

static av_cold int decode_close(AVCodecContext *avctx)
{
    MotionCamRAWContext *s = avctx->priv_data;

    av_freep(&s->bits);
    av_freep(&s->refs);

    return 0;
}

const FFCodec ff_mcraw_decoder = {
    .p.name         = "mcraw",
    CODEC_LONG_NAME("MotionCam RAW"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_MCRAW,
    .priv_data_size = sizeof(MotionCamRAWContext),
    .init           = decode_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .close          = decode_close,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP |
                      FF_CODEC_CAP_SKIP_FRAME_FILL_PARAM,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_FRAME_THREADS,
};
