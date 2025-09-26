/*
 * CFAST video decoder
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

#include "libavutil/display.h"
#include "libavutil/intreadwrite.h"
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "mathops.h"

typedef struct CFASTVideoContext {
    int nb_bit_planes;
    uint32_t base_colors[2];
} CFASTVideoContext;

static av_cold int cfast_decode_init(AVCodecContext *avctx)
{
    CFASTVideoContext *c = avctx->priv_data;

    avctx->pix_fmt = AV_PIX_FMT_PAL8;

    c->nb_bit_planes = avctx->extradata[0];
    if (c->nb_bit_planes == 0 || c->nb_bit_planes > 5)
        return AVERROR_INVALIDDATA;

    c->base_colors[0] = AV_RB24(avctx->extradata+1);
    c->base_colors[1] = AV_RB24(avctx->extradata+4);

    return 0;
}

static int cfast_decode_frame(AVCodecContext *avctx, AVFrame *p,
                              int *got_frame, AVPacket *avpkt)
{
    CFASTVideoContext *c = avctx->priv_data;
    const int nb_bit_planes = c->nb_bit_planes;
    const int w = FFALIGN(avctx->width, 16);
    const int h = avctx->height;
    GetByteContext gbc, *gb = &gbc;
    int extra_colors = 0;
    ptrdiff_t linesize;
    uint8_t *dst;
    int ret;

    bytestream2_init(gb, avpkt->data, avpkt->size);

    if ((ret = ff_get_buffer(avctx, p, 0)) < 0)
        return ret;

    linesize = p->linesize[0];
    dst = p->data[0];
    for (int y = 0; y < h; y++)
        memset(dst + y * linesize, 0, w);

    for (int plane = 0; plane < nb_bit_planes; plane++) {
        GetByteContext pgbc, *pgb = &pgbc;
        int32_t plane_size = bytestream2_get_be32(gb);
        int y = 0, x = 0;

        dst = p->data[0];
        if (plane_size < 0 || plane_size > bytestream2_get_bytes_left(gb))
            return AVERROR_INVALIDDATA;

        bytestream2_init(pgb, gb->buffer, plane_size);
        while (bytestream2_get_bytes_left(pgb) > 0) {
            int code = sign_extend(bytestream2_get_be16(pgb), 16);

            if (code < 0) {
                int run = -code + 1;
                unsigned fill = bytestream2_get_be16(pgb);

                for (int r = 0; r < run; r++) {
                    for (int px = 0; px < 16; px++)
                        dst[w-x-16+px] |= (!!(fill & (1 << px))) << plane;
                    dst += linesize;
                    y++;
                    if (y >= h) {
                        dst = p->data[0];
                        y = 0;
                        x += 16;
                        if (x >= w)
                            break;
                    }
                }
            } else {
                int run = code + 1;

                for (int r = 0; r < run; r++) {
                    unsigned fill = bytestream2_get_be16(pgb);

                    for (int px = 0; px < 16; px++)
                        dst[w-x-16+px] |= (!!(fill & (1 << px))) << plane;
                    dst += linesize;
                    y++;
                    if (y >= h) {
                        dst = p->data[0];
                        y = 0;
                        x += 16;
                        if (x >= w)
                            break;
                    }
                }
            }
            if (x >= w)
                break;
        }
        bytestream2_skip(gb, plane_size);
    }

    extra_colors = bytestream2_get_byte(gb);
    if (extra_colors > 254)
        return AVERROR_INVALIDDATA;

    AV_WB32(p->data[1]+0, (c->base_colors[0] << 8) | 0xFF);
    AV_WB32(p->data[1]+4, (c->base_colors[1] << 8) | 0xFF);
    for (int n = 0; n < extra_colors; n++)
        AV_WB32(p->data[1] + 8 + n*4, (bytestream2_get_be24(gb) << 8) | 0xFF);

    {
        AVFrameSideData *sd = NULL;
        int32_t *matrix;

        sd = av_frame_new_side_data(p, AV_FRAME_DATA_DISPLAYMATRIX, sizeof(int32_t) * 9);
        if (!sd)
            return AVERROR(ENOMEM);

        matrix = (int32_t *)sd->data;
        av_display_rotation_set(matrix, 0.0);
        av_display_matrix_flip(matrix, 1, 0);
    }

    *got_frame = 1;

    return avpkt->size;
}

const FFCodec ff_cfast_decoder = {
    .p.name         = "cfast",
    CODEC_LONG_NAME("CFAST (Disney Animation Studio)"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_CFAST,
    .priv_data_size = sizeof(CFASTVideoContext),
    .init           = cfast_decode_init,
    FF_CODEC_DECODE_CB(cfast_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
