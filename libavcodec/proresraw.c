/*
 * ProRes RAW decoder
 * Copyright (c) 2023-2025 Paul B Mahol
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

#include "libavutil/intreadwrite.h"
#include "libavutil/mem_internal.h"
#include "libavutil/mem.h"

#define CACHED_BITSTREAM_READER !ARCH_X86_32

#include "avcodec.h"
#include "blockdsp.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "get_bits.h"
#include "idctdsp.h"
#include "proresdata.h"
#include "proresdsp.h"
#include "thread.h"

typedef struct TileContext {
    GetByteContext gb;
    unsigned x, y;
} TileContext;

typedef struct ProResRAWContext {
    int nb_tiles;
    int tw, th;
    int nb_tw, nb_th;

    uint8_t qmat[64];

    TileContext *tiles;
    unsigned int tiles_size;

    uint8_t scan[64];

    ProresDSPContext prodsp;
    BlockDSPContext bdsp;
} ProResRAWContext;

static av_cold int decode_init(AVCodecContext *avctx)
{
    ProResRAWContext *s = avctx->priv_data;
    uint8_t idct_permutation[64];

    avctx->bits_per_raw_sample = 12;
    avctx->pix_fmt = AV_PIX_FMT_BAYER_RGGB16;
    avctx->color_trc = AVCOL_TRC_LINEAR;

    ff_blockdsp_init(&s->bdsp);
    ff_proresdsp_init(&s->prodsp, avctx->bits_per_raw_sample);

    ff_init_scantable_permutation(idct_permutation,
                                  s->prodsp.idct_permutation_type);

    ff_permute_scantable(s->scan, ff_prores_interlaced_scan, idct_permutation);

    return 0;
}

static uint32_t get_value(GetBitContext *gb, uint16_t codebook)
{
    const unsigned switch_bits = codebook >> 8;
    const unsigned rice_order = codebook & 0xf;
    const unsigned exp_order = (codebook >> 4) & 0xf;
    uint32_t b, bits, q;

    b = show_bits_long(gb, 32);
    if (!b)
        return 0;
    q = 31 - av_log2(b);

    if (b & 0x80000000u) {
        skip_bits_long(gb, 1u + rice_order);
        return (b & 0x7FFFFFFFu) >> (31u - rice_order);
    }

    if (q <= switch_bits) {
        skip_bits_long(gb, q + rice_order + 1u);
        return (q << rice_order) + (((b << (q+1)) >> 1u) >> (31u - rice_order));
    }

    bits = exp_order + (q<<1u) - switch_bits;
    skip_bits_long(gb, bits);
    return (b >> (32u - bits)) +
           ((switch_bits + 1) << rice_order) -
           (1u << exp_order);
}

#define TODCCODEBOOK(x) (((x) & 1) + (x) >> 1)

static const uint8_t align_tile_w[16] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
};

static const uint8_t dc_cb[13] = {
    16, 33, 50, 51, 51, 51, 68, 68, 68, 68, 68, 68, 118,
};

static const uint16_t ac_cb[95] = {
      0, 529, 273, 273, 546, 546, 546, 290, 290, 290, 563, 563,
    563, 563, 563, 563, 563, 563, 307, 307, 580, 580, 580, 580,
    580, 580, 580, 580, 580, 580, 580, 580, 580, 580, 580, 580,
    580, 580, 580, 580, 580, 580, 853, 853, 853, 853, 853, 853,
    853, 853, 853, 853, 853, 853, 853, 853, 853, 853, 853, 853,
    853, 853, 853, 853, 853, 853, 853, 853, 853, 853, 853, 853,
    853, 853, 853, 853, 853, 853, 853, 853, 853, 853, 853, 853,
    853, 853, 853, 853, 853, 853, 853, 853, 853, 853, 358
};

static const uint16_t rn_cb[28] = {
    512, 256, 0, 0, 529, 529, 273, 273, 17, 17, 33, 33, 546,
    34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 50, 50, 68,
};

static const uint16_t ln_cb[15] = {
    256, 273, 546, 546, 290, 290, 1075, 1075, 563, 563, 563, 563, 563, 563, 51
};

static int decode_tilec(AVCodecContext *avctx, TileContext *tile,
                        AVFrame *frame, int size, int component, const int16_t *qmat)
{
    ProResRAWContext *s = avctx->priv_data;
    const ptrdiff_t linesize = frame->linesize[0] / 2;
    uint16_t *dst = (uint16_t *)(frame->data[0] + tile->y * frame->linesize[0] + 2 * tile->x);
    const int w = FFMIN(s->tw, avctx->width - tile->x) / 2;
    const int nb_blocks = w / 8;
    const int log2_nb_blocks = av_log2(nb_blocks);
    const int block_mask = (1 << log2_nb_blocks)-1;
    const int nb_codes = 64 * nb_blocks;
    LOCAL_ALIGNED_32(int16_t, block, [128*8]);
    LOCAL_ALIGNED_32(int16_t, out, [64]);
    int prev_dc = 0, ret, sign = 0, dc_add = 0;
    GetByteContext *gb = &tile->gb;
    const uint8_t *scan = s->scan;
    uint16_t ln_codebook = 66;
    uint16_t ac_codebook = 49;
    uint16_t rn_codebook = 0;
    uint16_t ac, dc, rn, ln;
    uint16_t dc_codebook;
    GetBitContext gbit;

    if (bytestream2_get_bytes_left(gb) < size) {
        ret = AVERROR_INVALIDDATA;
        goto fail;
    }

    if (component > 1)
        dst += linesize;
    dst += component&1;

    if ((ret = init_get_bits8(&gbit, gb->buffer, size)) < 0)
        return ret;

    for (int n = 0; n < nb_blocks; n++) {
        s->bdsp.clear_block(block+n*64);

        if (get_bits_left(&gbit) <= 0) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }

        if ((n & 15) == 0) {
            dc_codebook = 700;
        } else if ((n & 15) == 1) {
            dc_codebook = 100;
        } else {
            dc_codebook = dc_cb[FFMIN(TODCCODEBOOK(dc), FF_ARRAY_ELEMS(dc_cb)-1)];
        }

        dc = get_value(&gbit, dc_codebook);
        if (n == 0) {
            prev_dc = ((dc&1) + (dc>>1) ^ -(int)(dc&1)) + (dc&1);
        } else {
            sign = sign ^ dc & 1;
            dc_add = (int)(-sign ^ TODCCODEBOOK(dc)) + sign;
            sign = dc_add < 0;
            prev_dc += dc_add;
        }

        block[n*64] = prev_dc;
    }

    for (int n = nb_blocks; n < nb_codes;) {
        if (get_bits_left(&gbit) <= 0)
            break;

        ln = get_value(&gbit, ln_codebook);
        for (int i = 0; i < ln; i++) {
            if (get_bits_left(&gbit) <= 0)
                break;

            if (n+i >= nb_codes)
                break;

            ac = get_value(&gbit, ac_codebook);
            ac_codebook = ac_cb[FFMIN(ac, FF_ARRAY_ELEMS(ac_cb)-1)];
            sign = -get_bits1(&gbit);
            block[scan[(n+i)>>log2_nb_blocks] + (((n+i)&block_mask)<<6)] = (((int)ac + 1) ^ sign) - sign;
        }

        n += ln;
        if (n >= nb_codes)
            break;

        rn = get_value(&gbit, rn_codebook);
        rn_codebook = rn_cb[FFMIN(rn, FF_ARRAY_ELEMS(rn_cb)-1)];

        n += rn + 1;
        if (n >= nb_codes)
            break;

        if (get_bits_left(&gbit) <= 0)
            break;

        ac = get_value(&gbit, ac_codebook);
        sign = -get_bits1(&gbit);
        block[scan[n>>log2_nb_blocks] + ((n&block_mask)<<6)] = (((int)ac + 1) ^ sign) - sign;

        ac_codebook = ac_cb[FFMIN(ac, FF_ARRAY_ELEMS(ac_cb)-1)];
        ln_codebook = ln_cb[FFMIN(ac, FF_ARRAY_ELEMS(ln_cb)-1)];

        n++;
    }

    for (int n = 0; n < nb_blocks; n++) {
        uint16_t *ptr = dst + n * 16;

        s->prodsp.idct_put(out, 16, block+n*64, qmat);
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++)
                ptr[j * 2] = out[8*i+j] * 16;
            ptr += 2 * linesize;
        }
    }

    if (get_bits_left(&gbit) < 0)
        av_log(avctx, AV_LOG_DEBUG, "overread\n");

    align_get_bits(&gbit);
    if (get_bits_left(&gbit) > 0)
        av_log(avctx, AV_LOG_DEBUG, "underread %d\n", get_bits_left(&gbit));

    bytestream2_skip(gb, size);

    return 0;
fail:
    av_log(avctx, AV_LOG_ERROR, "component %d decoding error\n", component);
    return ret;
}

static int decode_tile(AVCodecContext *avctx, TileContext *tile,
                       AVFrame *frame)
{
    LOCAL_ALIGNED_32(int16_t, qmat, [64]);
    GetByteContext *gb = &tile->gb;
    int ret, size[4];
    uint16_t qscale;

    if (tile->x >= avctx->width)
        return 0;

    qscale = bytestream2_get_be16(gb);
    for (int i = 0; i < 64; i++)
        qmat[i] = (qscale - 16384) >> 1;

    size[0] = bytestream2_get_be16(gb);
    size[1] = bytestream2_get_be16(gb);
    size[2] = bytestream2_get_be16(gb);
    size[3] = bytestream2_size(gb) - size[0] - size[1] - size[2] - 8;
    if (size[3] < 0)
        return AVERROR_INVALIDDATA;

    ret = decode_tilec(avctx, tile, frame, size[0], 2, qmat);
    if (ret < 0)
        goto fail;
    ret = decode_tilec(avctx, tile, frame, size[1], 1, qmat);
    if (ret < 0)
        goto fail;
    ret = decode_tilec(avctx, tile, frame, size[2], 3, qmat);
    if (ret < 0)
        goto fail;
    ret = decode_tilec(avctx, tile, frame, size[3], 0, qmat);
    if (ret < 0)
        goto fail;

    return 0;
fail:
    av_log(avctx, AV_LOG_ERROR, "tile %d/%d decoding error\n", tile->x, tile->y);
    return ret;
}

static int decode_tiles(AVCodecContext *avctx, void *arg,
                        int n, int thread_nb)
{
    ProResRAWContext *s = avctx->priv_data;
    TileContext *tile = &s->tiles[n];
    AVFrame *frame = arg;

    return decode_tile(avctx, tile, frame);
}

static int decode_frame(AVCodecContext *avctx,
                        AVFrame *frame, int *got_frame,
                        AVPacket *avpkt)
{
    int header_size, ret, version, w, h, aa, flags;
    ProResRAWContext *s = avctx->priv_data;
    GetByteContext gb;
    uint8_t qmat[64];
    uint32_t offset;

    switch (avctx->codec_tag) {
    case 0:
        break;
    case MKTAG('a','p','r','n'):
        avctx->profile = AV_PROFILE_PRORES_RAW;
        break;
    case MKTAG('a','p','r','h'):
        avctx->profile = AV_PROFILE_PRORES_RAW_HQ;
        break;
    default:
        avpriv_request_sample(avctx, "Profile %d", avctx->codec_tag);
        return AVERROR_PATCHWELCOME;
        break;
    }

    bytestream2_init(&gb, avpkt->data, avpkt->size);
    if (bytestream2_get_be32(&gb) != avpkt->size)
        return AVERROR_INVALIDDATA;

    if (bytestream2_get_le32(&gb) != MKTAG('p','r','r','f'))
        return AVERROR_INVALIDDATA;

    header_size = bytestream2_get_be16(&gb) + 8;
    version = bytestream2_get_be16(&gb);
    if (version > 1) {
        avpriv_request_sample(avctx, "Version %d", version);
        return AVERROR_PATCHWELCOME;
    }
    if (header_size < (version == 0 ? 144 : 96))
        return AVERROR_INVALIDDATA;
    bytestream2_skip(&gb, 4);

    w = bytestream2_get_be16(&gb);
    h = bytestream2_get_be16(&gb);

    if (w != avctx->width || h != avctx->height) {
        av_log(avctx, AV_LOG_WARNING, "picture resolution change: %dx%d -> %dx%d\n",
               avctx->width, avctx->height, w, h);
        if ((ret = ff_set_dimensions(avctx, w, h)) < 0)
            return ret;
    }

    avctx->coded_width = FFALIGN(w, 16);
    avctx->coded_height = FFALIGN(h, 16);

    switch (version) {
    case 0:
        bytestream2_skip(&gb, 1 * 4);
        bytestream2_skip(&gb, 2);
        bytestream2_skip(&gb, 2);
        bytestream2_skip(&gb, 4);
        bytestream2_skip(&gb, 4);
        bytestream2_skip(&gb, 3 * 3 * 4);
        bytestream2_skip(&gb, 4);
        bytestream2_skip(&gb, 2);

        flags = bytestream2_get_be16(&gb);
        aa = (flags >> 1) & 7;
        bytestream2_get_buffer(&gb, qmat, 64);
        break;
    case 1:
        bytestream2_skip(&gb, 10);
        bytestream2_skip(&gb, 48);

        memset(qmat, 1, 64);
        flags = bytestream2_get_be16(&gb);
        aa = (flags >> 1) & 7;
        bytestream2_skip(&gb, 16);
        break;
    }

    ff_permute_scantable(s->qmat, s->prodsp.idct_permutation, qmat);
    bytestream2_skip(&gb, header_size - bytestream2_tell(&gb));

    s->nb_tw = (w + 15) >> 4;
    s->nb_th = (h + 15) >> 4;
    s->nb_tw = (s->nb_tw >> aa) + align_tile_w[~(-1 * (1 << aa)) & s->nb_tw];
    s->nb_tiles = s->nb_tw * s->nb_th;
    av_log(avctx, AV_LOG_DEBUG, "nb tiles: %d\n", s->nb_tiles);

    s->tw = version == 0 ? 128 : 256;
    s->th = 16;
    av_log(avctx, AV_LOG_DEBUG, "tile size: %dx%d\n", s->tw, s->th);

    av_fast_mallocz(&s->tiles, &s->tiles_size, s->nb_tiles * sizeof(*s->tiles));
    if (!s->tiles)
        return AVERROR(ENOMEM);

    if (bytestream2_get_bytes_left(&gb) < s->nb_tiles * 2)
        return AVERROR_INVALIDDATA;

    offset = bytestream2_tell(&gb) + s->nb_tiles * 2;
    for (int n = 0; n < s->nb_tiles; n++) {
        TileContext *tile = &s->tiles[n];
        int size;

        size = bytestream2_get_be16(&gb);
        if (offset >= avpkt->size)
            return AVERROR_INVALIDDATA;
        if (size >= avpkt->size)
            return AVERROR_INVALIDDATA;
        if (offset > avpkt->size - size)
            return AVERROR_INVALIDDATA;

        bytestream2_init(&tile->gb, avpkt->data + offset, size);

        tile->y = (n / s->nb_tw) * s->th;
        tile->x = (n % s->nb_tw) * s->tw;

        offset += size;
    }

    ret = ff_thread_get_buffer(avctx, frame, 0);
    if (ret < 0)
        return ret;

    avctx->execute2(avctx, decode_tiles, frame, NULL, s->nb_tiles);

    frame->pict_type = AV_PICTURE_TYPE_I;
    frame->flags |= AV_FRAME_FLAG_KEY;

    *got_frame = 1;

    return avpkt->size;
}

static av_cold int decode_end(AVCodecContext *avctx)
{
    ProResRAWContext *s = avctx->priv_data;
    av_freep(&s->tiles);
    return 0;
}

const FFCodec ff_proresraw_decoder = {
    .p.name           = "proresraw",
    CODEC_LONG_NAME("ProRes RAW"),
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_PRORESRAW,
    .priv_data_size   = sizeof(ProResRAWContext),
    .init             = decode_init,
    .close            = decode_end,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities   = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_FRAME_THREADS |
                        AV_CODEC_CAP_SLICE_THREADS,
};
