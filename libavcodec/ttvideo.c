/*
 * Traveller's Tales Video decoder
 * Copyright (c) 2026 Paul B Mahol
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

#include "libavutil/attributes.h"
#include "libavutil/internal.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"

#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "get_bits.h"
#include "idctdsp.h"

static const uint8_t qtab[2][2][64] = {
    {
        {
            0x10, 0x10, 0x10, 0x10, 0x11, 0x12, 0x15, 0x18,
            0x10, 0x10, 0x10, 0x10, 0x11, 0x13, 0x16, 0x19,
            0x10, 0x10, 0x11, 0x12, 0x14, 0x16, 0x19, 0x1D,
            0x10, 0x10, 0x12, 0x15, 0x18, 0x1B, 0x1F, 0x24,
            0x11, 0x11, 0x14, 0x18, 0x1E, 0x23, 0x29, 0x2F,
            0x12, 0x13, 0x16, 0x1B, 0x23, 0x2C, 0x36, 0x41,
            0x15, 0x16, 0x19, 0x1F, 0x29, 0x36, 0x46, 0x58,
            0x18, 0x19, 0x1D, 0x24, 0x2F, 0x41, 0x58, 0x73
        },
        {
            0x11, 0x12, 0x18, 0x2F, 0x63, 0x63, 0x63, 0x63,
            0x12, 0x15, 0x1A, 0x42, 0x63, 0x63, 0x63, 0x63,
            0x18, 0x1A, 0x38, 0x63, 0x63, 0x63, 0x63, 0x63,
            0x2F, 0x42, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
            0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
            0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
            0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
            0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63
        },
    },
    {
        {
            0x10, 0x0B, 0x0A, 0x10, 0x18, 0x28, 0x33, 0x3D,
            0x0C, 0x0C, 0x0E, 0x13, 0x1A, 0x3A, 0x3C, 0x37,
            0x0E, 0x0D, 0x10, 0x18, 0x28, 0x39, 0x45, 0x38,
            0x0E, 0x11, 0x16, 0x1D, 0x33, 0x57, 0x50, 0x3E,
            0x12, 0x16, 0x25, 0x38, 0x44, 0x6D, 0x67, 0x4D,
            0x18, 0x23, 0x37, 0x40, 0x51, 0x68, 0x71, 0x5C,
            0x31, 0x40, 0x4E, 0x57, 0x67, 0x79, 0x78, 0x65,
            0x48, 0x5C, 0x5F, 0x62, 0x70, 0x64, 0x67, 0x63
        },
        {
            0x11, 0x12, 0x18, 0x2F, 0x63, 0x63, 0x63, 0x63,
            0x12, 0x15, 0x1A, 0x42, 0x63, 0x63, 0x63, 0x63,
            0x18, 0x1A, 0x38, 0x63, 0x63, 0x63, 0x63, 0x63,
            0x2F, 0x42, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
            0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
            0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
            0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
            0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63
        },
    }
};

typedef struct TTVideoContext {
    GetByteContext gb;
    PutByteContext pb;
    GetBitContext gbit;

    int qtab_idx;

    int nb_x_tiles;
    int nb_y_tiles;
    int frame_block_data_size;
    uint8_t *frame_block_data;

    int quality;
    uint16_t qtab[2][64];
    uint8_t scan[64];

    VLC vlcs[2][2];

    uint16_t block[6][64];

    AVFrame *last;

    IDCTDSPContext idsp;
} TTVideoContext;

static void gen_quant_tab(int quality, const uint8_t *base_tab, uint16_t *qtab)
{
    quality = av_clip(quality, 1, 100);

    int scale_factor;
    if (quality < 50) {
        scale_factor = 5000 / quality;
    } else {
        scale_factor = 200 - (quality * 2);
    }

    for (int i = 0; i < 64; i++) {
        int temp = (base_tab[i] * scale_factor + 50L) / 100L;

        qtab[i] = av_clip(temp, 1, 255);
    }
}

static int uncompress_rle(GetByteContext *gb, PutByteContext *pb, int size)
{
    const int runlen_sym = bytestream2_get_byte(gb);

    while (bytestream2_get_bytes_left_p(pb) > 0 &&
           bytestream2_tell(gb) < size + 2) {
        int next_sym = bytestream2_get_byte(gb);

        if (next_sym == runlen_sym) {
            int count = bytestream2_get_byte(gb);

            if (count <= 2) {
                bytestream2_set_buffer(pb, runlen_sym, count+1);
            } else {
                if (count & 0x80) {
                    next_sym = bytestream2_get_byte(gb);
                    count <<= 25;
                    count = next_sym + (count >> 17);
                }
                next_sym = bytestream2_get_byte(gb);
                bytestream2_set_buffer(pb, next_sym, count+1);
            }
        } else {
            if (bytestream2_get_bytes_left_p(pb) <= 0)
                return AVERROR_INVALIDDATA;

            bytestream2_put_byte(pb, next_sym);
        }
    }

    return 0;
}

static int build_huffman_codes(uint8_t *huff_size, GetByteContext *gb)
{
    int nb_codes = 0;

    for (int i = 1, j = 0; i <= 16; i++) {
        nb_codes += bytestream2_get_byte(gb);

        if (nb_codes > 256)
            return AVERROR_INVALIDDATA;

        for (; j < nb_codes; j++)
            huff_size[j] = i;
    }

    return nb_codes;
}

static int build_vlc(VLC *vlc, GetByteContext *gb, void *logctx)
{
    uint8_t huff_size[256];
    uint8_t huff_sym[256];
    int nb_codes = build_huffman_codes(huff_size, gb);

    if (nb_codes < 0)
        return nb_codes;

    for (int i = 0; i < nb_codes; i++)
        huff_sym[i] = bytestream2_get_byte(gb);

    return ff_vlc_init_from_lengths(vlc, 9, nb_codes, huff_size, 1,
                                    huff_sym, 1, 1, 0, 0, logctx);
}

static void copy_block16_mv(uint8_t *dst, const ptrdiff_t linesize,
                            const uint8_t *src, const ptrdiff_t slinesize,
                            int mv[2], const int w, const int h,
                            const int x, const int y)
{
    for (int yi = 0; yi < 16; yi++) {
        for (int xi = 0; xi < 16; xi++)
            dst[xi] = src[av_clip(mv[1]+yi+y, 0, h-1) * slinesize + av_clip(mv[0]+xi+x, 0, w-1)];

        dst += linesize;
    }
}

static void copy_block8_mv(uint8_t *dst, const ptrdiff_t linesize,
                           const uint8_t *src, const ptrdiff_t slinesize,
                           int mv[2], const int w, const int h,
                           const int x, const int y)
{
    for (int yi = 0; yi < 8; yi++) {
        for (int xi = 0; xi < 8; xi++)
            dst[xi] = src[av_clip((mv[1]+y+1)/2+yi, 0, h-1) * slinesize + av_clip((mv[0]+x+1)/2+xi, 0, w-1)];

        dst += linesize;
    }
}

static void copy_block16(uint8_t *dst, const ptrdiff_t linesize,
                         const uint8_t *src, const ptrdiff_t slinesize)
{
    for (int y = 0; y < 16; y++) {
        memcpy(dst, src, 16);

        src += slinesize;
        dst += linesize;
    }
}

static void copy_block8(uint8_t *dst, const ptrdiff_t linesize,
                         const uint8_t *src, const ptrdiff_t slinesize)
{
    for (int y = 0; y < 8; y++) {
        memcpy(dst, src, 8);

        src += slinesize;
        dst += linesize;
    }
}

static void fill_block16(uint8_t *dst, const ptrdiff_t linesize, const int fill)
{
    for (int y = 0; y < 16; y++) {
        memset(dst, fill, 16);

        dst += linesize;
    }
}

static void fill_block8(uint8_t *dst, const ptrdiff_t linesize, const int fill)
{
    for (int y = 0; y < 8; y++) {
        memset(dst, fill, 8);

        dst += linesize;
    }
}

static int decode_block(AVCodecContext *avctx, int *pred, uint16_t block[64],
                        GetBitContext *gb, const int idx)
{
    TTVideoContext *s = avctx->priv_data;
    const uint16_t *qtab = s->qtab[idx];
    const uint8_t *scan = s->scan;
    int a, v, m, i, b;

    if (get_bits_left(gb) <= 0)
        return AVERROR_INVALIDDATA;

    if (!s->vlcs[idx][0].table)
        return AVERROR_INVALIDDATA;

    a = get_vlc2(gb, s->vlcs[idx][0].table, 9, 2);
    if (a < 0)
        return AVERROR_INVALIDDATA;

    if (a) {
        v = get_bits(gb, a);
        m = 1 << (a - 1);

        if (v < m) {
            unsigned mask = 0xFFFFFFFF;

            mask = v + (mask << a);
            v = mask + 1;
        }
    } else {
        v = 0;
    }

    pred[0] += v * qtab[0];
    block[0] = pred[0] + 1024;

    i = 1;
    while (i < 64) {
        a = get_vlc2(gb, s->vlcs[idx][1].table, 9, 2);
        if (a < 0)
            return AVERROR_INVALIDDATA;

        b = a & 0xF;
        if (b) {
            i += a >> 4;
            v = get_bits(gb, b);
            m = 1 << (b - 1);

            if (v < m) {
                unsigned mask = 0xFFFFFFFF;

                mask = v + (mask << b);
                v = mask + 1;
            }

            block[scan[i]] = v * qtab[scan[i]];
            i++;
        } else if (a != 0xF0) {
            break;
        } else {
            i += 16;
        }
    }

    return 0;
}

static int decode(AVCodecContext *avctx, GetBitContext *gb, AVFrame *frame)
{
    TTVideoContext *s = avctx->priv_data;
    const uint8_t *frame_block = s->frame_block_data;
    const int nb_x_tiles = s->nb_x_tiles;
    const int h = avctx->height;
    const int w = avctx->width;
    const ptrdiff_t yplinesize = s->last->linesize[0];
    const ptrdiff_t uplinesize = s->last->linesize[1];
    const ptrdiff_t vplinesize = s->last->linesize[2];
    const ptrdiff_t ylinesize = frame->linesize[0];
    const ptrdiff_t ulinesize = frame->linesize[1];
    const ptrdiff_t vlinesize = frame->linesize[2];
    const uint8_t *yp = s->last->data[0];
    const uint8_t *up = s->last->data[1];
    const uint8_t *vp = s->last->data[2];
    int ret, pred[3] = { 0, 0, 0 };
    uint8_t *y = frame->data[0];
    uint8_t *u = frame->data[1];
    uint8_t *v = frame->data[2];

    for (int yi = 0; yi < h; yi += 16) {
        for (int xi = 0; xi < w; xi += 16) {
            int type = frame_block[(yi>>4) * nb_x_tiles + (xi>>4)];
            int fill[3], mv[2];

            switch (type) {
            case 0:
                if (!yp)
                    return AVERROR_INVALIDDATA;

                copy_block16(y + yi * ylinesize + xi, ylinesize,
                             yp + yi * yplinesize + xi, yplinesize);
                copy_block8(u + (yi/2) * ulinesize + xi/2, ulinesize,
                            up + (yi/2) * uplinesize + xi/2, uplinesize);
                copy_block8(v + (yi/2) * vlinesize + xi/2, vlinesize,
                            vp + (yi/2) * vplinesize + xi/2, vplinesize);
                break;
            case 1:
                if (!yp)
                    return AVERROR_INVALIDDATA;

                mv[0] = get_bits(gb, 5);
                mv[1] = get_bits(gb, 5);
                if (mv[0] > 15)
                    mv[0] -= 32;
                if (mv[1] > 15)
                    mv[1] -= 32;
                mv[0] = -mv[0];
                mv[1] = -mv[1];

                copy_block16_mv(y + yi * ylinesize + xi, ylinesize,
                                yp, yplinesize, mv, w, h, xi, yi);
                copy_block8_mv(u + (yi/2) * ulinesize + xi/2, ulinesize,
                               up, uplinesize, mv, w/2, h/2, xi, yi);
                copy_block8_mv(v + (yi/2) * vlinesize + xi/2, vlinesize,
                               vp, vplinesize, mv, w/2, h/2, xi, yi);
                break;
            case 2:
                fill[0] = get_bits(gb, 8);
                fill[1] = get_bits(gb, 8);
                fill[2] = get_bits(gb, 8);

                fill_block16(y + yi * ylinesize + xi, ylinesize, fill[0]);
                fill_block8(u + (yi/2) * ulinesize + xi/2, ulinesize, fill[1]);
                fill_block8(v + (yi/2) * vlinesize + xi/2, vlinesize, fill[2]);
                break;
            case 255:
                memset(s->block, 0, sizeof(s->block));

                for (int j = 0; j < 6; j++) {
                    ret = decode_block(avctx, &pred[0+(j>=4)+(j>=5)], s->block[j], gb, (j >= 4));
                    if (ret < 0)
                        return ret;
                }

                s->idsp.idct_put(y + yi * ylinesize + xi, ylinesize, s->block[0]);
                s->idsp.idct_put(y + yi * ylinesize + xi+8, ylinesize, s->block[1]);
                s->idsp.idct_put(y + (yi+8) * ylinesize + xi, ylinesize, s->block[2]);
                s->idsp.idct_put(y + (yi+8) * ylinesize + xi+8, ylinesize, s->block[3]);
                s->idsp.idct_put(u + (yi/2) * ulinesize + (xi/2), ulinesize, s->block[4]);
                s->idsp.idct_put(v + (yi/2) * vlinesize + (xi/2), vlinesize, s->block[5]);
                break;
            default:
                return AVERROR_INVALIDDATA;
            }
        }
    }

    return 0;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame, AVPacket *avpkt)
{
    const int key = avpkt->flags & AV_PKT_FLAG_KEY;
    TTVideoContext *s = avctx->priv_data;
    GetByteContext *gb = &s->gb;
    int ret;

    if (avpkt->size < 2)
        return AVERROR_INVALIDDATA;

    bytestream2_init(gb, avpkt->data, avpkt->size);
    if (key) {
        PutByteContext *pb = &s->pb;

        bytestream2_init_writer(pb, s->frame_block_data, s->frame_block_data_size);

        int length = bytestream2_get_le16(gb);

        ret = uncompress_rle(gb, pb, length);
        if (ret < 0)
            return ret;

        s->quality = bytestream2_get_byte(gb);

        if (!s->vlcs[0][0].table && avpkt->pts == 0) {
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 2; j++) {
                    if (bytestream2_get_bytes_left(gb) < 16)
                        return AVERROR_INVALIDDATA;

                    ff_vlc_free(&s->vlcs[i][j]);
                    ret = build_vlc(&s->vlcs[i][j], gb, avctx);
                    if (ret < 0)
                        return ret;
                }
            }
        } else if (avpkt->pts == 0) {
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 2; j++) {
                    int skip = 0;

                    if (bytestream2_get_bytes_left(gb) < 16)
                        return AVERROR_INVALIDDATA;

                    for (int k = 0; k < 16; k++)
                        skip += bytestream2_get_byte(gb);

                    bytestream2_skip(gb, skip);
                }
            }
        }

        gen_quant_tab(s->quality, qtab[s->qtab_idx][0], s->qtab[0]);
        gen_quant_tab(s->quality, qtab[s->qtab_idx][1], s->qtab[1]);

        ret = init_get_bits8(&s->gbit, avpkt->data + bytestream2_tell(gb), bytestream2_get_bytes_left(gb));
        if (ret < 0)
            return ret;
    } else {
        PutByteContext *pb = &s->pb;

        bytestream2_init_writer(pb, s->frame_block_data, s->frame_block_data_size);

        int length = bytestream2_get_le16(gb);

        ret = uncompress_rle(gb, pb, length);
        if (ret < 0)
            return ret;

        s->quality = bytestream2_get_byte(gb);

        gen_quant_tab(s->quality, qtab[s->qtab_idx][0], s->qtab[0]);
        gen_quant_tab(s->quality, qtab[s->qtab_idx][1], s->qtab[1]);

        ret = init_get_bits8(&s->gbit, avpkt->data + bytestream2_tell(gb), bytestream2_get_bytes_left(gb));
        if (ret < 0)
            return ret;
    }

    if ((ret = ff_get_buffer(avctx, frame, AV_GET_BUFFER_FLAG_REF)) < 0)
        return ret;

    ret = decode(avctx, &s->gbit, frame);
    if (ret < 0)
        return ret;

    if ((ret = av_frame_replace(s->last, frame)) < 0)
        return ret;

    frame->pict_type = key ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;
    if (key)
        frame->flags |= AV_FRAME_FLAG_KEY;
    else
        frame->flags &= ~AV_FRAME_FLAG_KEY;
    *got_frame = 1;

    return avpkt->size;
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    TTVideoContext *s = avctx->priv_data;

    avctx->pix_fmt = AV_PIX_FMT_YUV420P;

    s->nb_x_tiles = (avctx->width + 15) >> 4;
    s->nb_y_tiles = (avctx->height + 15) >> 4;
    s->frame_block_data_size = s->nb_x_tiles * s->nb_y_tiles;
    s->frame_block_data = av_calloc(s->nb_x_tiles, s->nb_y_tiles);
    if (!s->frame_block_data)
        return AVERROR(ENOMEM);

    s->last = av_frame_alloc();
    if (!s->last)
        return AVERROR(ENOMEM);

    ff_idctdsp_init(&s->idsp, avctx);
    ff_permute_scantable(s->scan, ff_zigzag_direct,
                         s->idsp.idct_permutation);

    if (avctx->extradata && avctx->extradata_size >= 2)
        s->qtab_idx = AV_RL16(avctx->extradata) != 0x113;

    return 0;
}

static av_cold void decode_flush(AVCodecContext *avctx)
{
    TTVideoContext *s = avctx->priv_data;

    memset(s->frame_block_data, 0, s->frame_block_data_size);
    av_frame_unref(s->last);
}

static av_cold int decode_close(AVCodecContext *avctx)
{
    TTVideoContext *s = avctx->priv_data;

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++)
            ff_vlc_free(&s->vlcs[i][j]);
    }

    av_freep(&s->frame_block_data);
    av_frame_free(&s->last);

    return 0;
}

const FFCodec ff_ttvideo_decoder = {
    .p.name         = "ttvideo",
    CODEC_LONG_NAME("Traveller's Tales Video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_TTVIDEO,
    .priv_data_size = sizeof(TTVideoContext),
    .init           = decode_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .flush          = decode_flush,
    .close          = decode_close,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
};
