/*
 * Sonic CD Video decoder
 * Copyright (c) 2025 smiRaphi
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

#include "libavutil/mem.h"
#include "libavutil/internal.h"
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"

typedef struct SONCDVIDContext {
    int tile_w;
    int tile_h;
} SONCDVIDContext;

static av_cold int soncdvid_decode_init(AVCodecContext *avctx)
{
    SONCDVIDContext *s = avctx->priv_data;
    s->tile_w = 32;
    s->tile_h = 14;

    avctx->width = 32 * 8;
    avctx->height = 14 * 8;
    avctx->pix_fmt = AV_PIX_FMT_PAL8;

    return 0;
}

static int soncdvid_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                                 int *got_frame, AVPacket *pkt)
{
    int res;
    uint16_t mode;
    SONCDVIDContext *s = avctx->priv_data;
    GetByteContext gbc, *gb = &gbc;

    bytestream2_init(gb, pkt->data, pkt->size);

    res = ff_get_buffer(avctx, frame, 0);
    if (res < 0)
        return res;

    printf("1.0\n");
    mode = bytestream2_get_be16u(gb);
    printf("1.1 mode: %04x\n", mode);
    if (mode > 0x1000) {
        bytestream2_skip(gb, 14);
        printf("2.0\n");
        switch (mode) {
        case 0x3038: // 08
        case 0x4E4F: // NO
            s->tile_w = 32;
            s->tile_h = 14;
            break;
        case 0x3135: // 15
            s->tile_w = 15;
            s->tile_h = 10;
            break;
        case 0x5253: // RS
        case 0x5352: // SR
        default:
            avpriv_request_sample(avctx, "mode %04x", mode);
            return AVERROR_PATCHWELCOME;
        }
    } else
        bytestream2_skip(gb, -2);

    printf("1.2\n");
    for (int f = 0; f < 8; f++) {
        printf("3.0\n");
        uint32_t *pal = (uint32_t *)frame->data[1];
        for (int i = 0; i < 16; i++) {
            uint8_t r, g, b;
            uint16_t clr = bytestream2_get_be16u(gb);
            b = (clr >> 8) & 0xF;
            g = (clr >> 4) & 0xF;
            r = (clr >> 0) & 0xF;
            *pal++ = (0xFF << 24) | (((r << 4) | r) << 16)
                   | (((g << 4) | g) << 8) | ((b << 4) | b);
        }
        printf("3.1\n");

        for (int ty = 0; ty < s->tile_h; ty++) {
            uint8_t *strip = frame->data[0] + ty * 8 * frame->linesize[0];

            for (int tx = 0; tx < s->tile_w; tx++) {
                int dpos = tx * 8;

                for (int row = 0; row < 8; row++) {
                    uint8_t *line = strip + row * frame->linesize[0] + dpos;

                    for (int i = 0; i < 4; i++) {
                        uint8_t b = bytestream2_get_byte(gb);
                        line[i*2]   = b >> 4;
                        line[i*2+1] = b & 0xF;
                    }
                }
            }
        }
        printf("3.2\n");
    }
    printf("1.3\n");

    *got_frame = 1;
    return pkt->size;
}

const FFCodec ff_soncdvid_decoder = {
    .p.name         = "soncdvid",
    CODEC_LONG_NAME("Sonic CD Video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_SONCDVID,
    .init           = soncdvid_decode_init,
    .priv_data_size = sizeof(SONCDVIDContext),
    FF_CODEC_DECODE_CB(soncdvid_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
