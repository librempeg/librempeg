/*
 * Microsoft Screen 1 (aka Windows Media Video V7 Screen) decoder
 * Copyright (c) 2012 Konstantin Shishkov
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
 * Microsoft Screen 1 (aka Windows Media Video V7 Screen) decoder
 */

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "mss12.h"

typedef struct MSS1Context {
    MSS12Context   ctx;
    AVFrame       *pic;
    SliceContext   sc;
} MSS1Context;

static void arith_normalise(ArithCoder *c)
{
    for (;;) {
        if (c->high >= 0x8000) {
            if (c->low < 0x8000) {
                if (c->low >= 0x4000 && c->high < 0xC000) {
                    c->value -= 0x4000;
                    c->low   -= 0x4000;
                    c->high  -= 0x4000;
                } else {
                    return;
                }
            } else {
                c->value -= 0x8000;
                c->low   -= 0x8000;
                c->high  -= 0x8000;
            }
        }
        c->value <<= 1;
        c->low   <<= 1;
        c->high  <<= 1;
        c->high   |= 1;
        if (get_bits_left(c->gbc.gb) < 1)
            c->overread++;
        c->value  |= get_bits1(c->gbc.gb);
    }
}

ARITH_GET_BIT(arith)

static int arith_get_bits(ArithCoder *c, int bits)
{
    int range = c->high - c->low + 1;
    int val   = (((c->value - c->low + 1) << bits) - 1) / range;
    int prob  = range * val;

    c->high   = ((prob + range) >> bits) + c->low - 1;
    c->low   += prob >> bits;

    arith_normalise(c);

    return val;
}

static int arith_get_number(ArithCoder *c, int mod_val)
{
    int range = c->high - c->low + 1;
    int val   = ((c->value - c->low + 1) * mod_val - 1) / range;
    int prob  = range * val;

    c->high   = (prob + range) / mod_val + c->low - 1;
    c->low   += prob / mod_val;

    arith_normalise(c);

    return val;
}

static int arith_get_prob(ArithCoder *c, int16_t *probs)
{
    int range = c->high - c->low + 1;
    int val   = ((c->value - c->low + 1) * probs[0] - 1) / range;
    int sym   = 1;

    while (probs[sym] > val)
        sym++;

    c->high = range * probs[sym - 1] / probs[0] + c->low - 1;
    c->low += range * probs[sym]     / probs[0];

    return sym;
}

ARITH_GET_MODEL_SYM(arith)

static void arith_init(ArithCoder *c, GetBitContext *gb)
{
    c->low           = 0;
    c->high          = 0xFFFF;
    c->value         = get_bits(gb, 16);
    c->overread      = 0;
    c->gbc.gb        = gb;
    c->get_model_sym = arith_get_model_sym;
    c->get_number    = arith_get_number;
}

static void decode_pal(MSS12Context *ctx, ArithCoder *acoder)
{
    int i, ncol, r, g, b;
    uint32_t *pal = ctx->pal + 256 - ctx->free_colours;

    if (!ctx->free_colours)
        return;

    ncol = arith_get_number(acoder, ctx->free_colours + 1);
    for (i = 0; i < ncol; i++) {
        r = arith_get_bits(acoder, 8);
        g = arith_get_bits(acoder, 8);
        b = arith_get_bits(acoder, 8);
        *pal++ = (0xFFU << 24) | (r << 16) | (g << 8) | b;
    }
}

static int mss1_decode_frame(AVCodecContext *avctx, AVFrame *rframe,
                             int *got_frame, AVPacket *avpkt)
{
    MSS1Context *ctx = avctx->priv_data;
    MSS12Context *c = &ctx->ctx;
    GetBitContext gb;
    ArithCoder acoder;
    int ret;

    if ((ret = init_get_bits8(&gb, avpkt->data, avpkt->size)) < 0)
        return ret;

    arith_init(&acoder, &gb);

    if ((ret = ff_reget_buffer(avctx, ctx->pic, 0)) < 0)
        return ret;

    c->pal_pic    =  ctx->pic->data[0] + ctx->pic->linesize[0] * (avctx->height - 1);
    c->pal_stride = -ctx->pic->linesize[0];
    c->keyframe   = !arith_get_bit(&acoder);
    if (c->keyframe) {
        c->corrupted = 0;
        ff_mss12_slicecontext_reset(&ctx->sc);
        decode_pal(c, &acoder);
        ctx->pic->flags |= AV_FRAME_FLAG_KEY;
        ctx->pic->pict_type = AV_PICTURE_TYPE_I;
    } else {
        if (c->corrupted)
            return AVERROR_INVALIDDATA;
        ctx->pic->flags &= ~AV_FRAME_FLAG_KEY;
        ctx->pic->pict_type = AV_PICTURE_TYPE_P;
    }
    c->corrupted = ff_mss12_decode_rect(&ctx->sc, &acoder, 0, 0,
                                        avctx->width, avctx->height);
    if (c->corrupted)
        return AVERROR_INVALIDDATA;
    memcpy(ctx->pic->data[1], c->pal, AVPALETTE_SIZE);

    if ((ret = av_frame_ref(rframe, ctx->pic)) < 0)
        return ret;

    *got_frame      = 1;

    /* always report that the buffer was completely consumed */
    return avpkt->size;
}

static av_cold int mss1_decode_init(AVCodecContext *avctx)
{
    MSS1Context * const c = avctx->priv_data;
    int ret;

    c->ctx.avctx       = avctx;

    c->pic = av_frame_alloc();
    if (!c->pic)
        return AVERROR(ENOMEM);

    ret = ff_mss12_decode_init(&c->ctx, 0, &c->sc, NULL);
    if (ret < 0)
        return ret;

    avctx->pix_fmt = AV_PIX_FMT_PAL8;

    return ret;
}

static av_cold int mss1_decode_end(AVCodecContext *avctx)
{
    MSS1Context * const ctx = avctx->priv_data;

    av_frame_free(&ctx->pic);
    ff_mss12_decode_end(&ctx->ctx);

    return 0;
}

const FFCodec ff_mss1_decoder = {
    .p.name         = "mss1",
    CODEC_LONG_NAME("MS Screen 1"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_MSS1,
    .priv_data_size = sizeof(MSS1Context),
    .init           = mss1_decode_init,
    .close          = mss1_decode_end,
    FF_CODEC_DECODE_CB(mss1_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
