/*
 * Sonic CD STM video decoder
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
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"

typedef struct SonicContext {
    int width, height;
} SonicContext;

static av_cold int sonic_decode_init(AVCodecContext *avctx)
{
    SonicContext *s = avctx->priv_data;

    avctx->pix_fmt = AV_PIX_FMT_PAL8;
    avctx->width = s->width = 32*8;
    avctx->height = s->height = 14*8;

    return 0;
}

static int sonic_frame_size(const int mode, int *w, int *h)
{
    switch (mode) {
    case 0x5253:
    case 0x5352:
    case 0x3135:
        w[0] = 15 * 8;
        h[0] = 10 * 8;
        return w[0] * h[0] / 2 + 32 + 16;
    case 0x4E4F:
    case 0x3038:
        w[0] = 32 * 8;
        h[0] = 14 * 8;
        return w[0] * h[0] / 2 + 32 + 16;
    default:
        return w[0] * h[0] / 2 + 32;
    }
}

static int sonic_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                             int *got_frame, AVPacket *avpkt)
{
    SonicContext *s = avctx->priv_data;
    GetByteContext gbc, *gb = &gbc;
    int mode, ret, frame_size;
    ptrdiff_t linesize;
    uint8_t *dst;

    bytestream2_init(gb, avpkt->data, avpkt->size);
    mode = bytestream2_peek_be16u(gb);
    if (mode > 0x1000)
        bytestream2_skip(gb, 16);
    frame_size = sonic_frame_size(mode, &s->width, &s->height);

    if (avpkt->size < frame_size)
        return 0;

    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    for (int n = 0; n < 16; n++) {
        uint16_t color = bytestream2_get_be16u(gb);
        unsigned r = ((color >> 0) & 0xF) << 4;
        unsigned g = ((color >> 4) & 0xF) << 4;
        unsigned b = ((color >> 8) & 0xF) << 4;

        AV_WN32(frame->data[1] + 4 * n, (0xFFU << 24) | (r << 16) | (g << 8) | b);
    }

    dst = frame->data[0];
    linesize = frame->linesize[0];
    for (int y = 0; y < s->height; y += 8) {
        for (int x = 0; x < s->width; x += 8) {
            for (int yy = 0; yy < 8; yy++) {
                for (int xx = 0; xx < 8; xx += 2) {
                    uint16_t ic = bytestream2_get_byteu(gb);

                    dst[(x+xx+0) + yy * linesize] = ic >> 4;
                    dst[(x+xx+1) + yy * linesize] = ic & 0xf;
                }
            }
        }

        dst += 8 * linesize;
    }

    *got_frame = 1;

    return bytestream2_tell(gb);
}

const FFCodec ff_sonic_video_decoder = {
    .p.name         = "sonic",
    CODEC_LONG_NAME("Sonic CD video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_SONIC_VIDEO,
    .priv_data_size = sizeof(SonicContext),
    .init           = sonic_decode_init,
    FF_CODEC_DECODE_CB(sonic_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
