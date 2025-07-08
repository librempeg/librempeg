/*
 * ILDA Image Data Transfer Format decoder
 * Copyright (C) 2020 Peter Ross <pross@xvid.org>
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
 * ILDA Image Data Transfer Format decoder
 */

#include "libavutil/attributes.h"
#include "libavutil/common.h"
#include "libavutil/frame.h"
#include "libavutil/imgutils.h"
#include "libavutil/internal.h"
#include "libavutil/intreadwrite.h"
#include "avcodec.h"
#include "decode.h"
#include "codec_internal.h"
#include "internal.h"
#include "mathops.h"

static const uint8_t default_palette[64][3] = {
    {255,0,0},
    {255,16,0},
    {255,32,0},
    {255,48,0},
    {255,64,0},
    {255,80,0},
    {255,96,0},
    {255,112,0},
    {255,128,0},
    {255,144,0},
    {255,160,0},
    {255,176,0},
    {255,192,0},
    {255,208,0},
    {255,224,0},
    {255,240,0},
    {255,255,0},
    {224,255,0},
    {192,255,0},
    {160,255,0},
    {128,255,0},
    {96,255,0},
    {64,255,0},
    {32,255,0},
    {0,255,0},
    {0,255,36},
    {0,255,73},
    {0,255,109},
    {0,255,146},
    {0,255,182},
    {0,255,219},
    {0,255,255},
    {0,227,255},
    {0,198,255},
    {0,170,255},
    {0,142,255},
    {0,113,255},
    {0,85,255},
    {0,56,255},
    {0,28,255},
    {0,0,255},
    {32,0,255},
    {64,0,255},
    {96,0,255},
    {128,0,255},
    {160,0,255},
    {192,0,255},
    {224,0,255},
    {255,0,255},
    {255,32,255},
    {255,64,255},
    {255,96,255},
    {255,128,255},
    {255,160,255},
    {255,192,255},
    {255,224,255},
    {255,255,255},
    {255,224,224},
    {255,192,192},
    {255,160,160},
    {255,128,128},
    {255,96,96},
    {255,64,64},
    {255,32,32},
};

typedef struct ILDAContext {
    uint8_t palette[256][3];
} ILDAContext;

static av_cold int decode_init(AVCodecContext *avctx)
{
    ILDAContext *s = avctx->priv_data;

    if (!avctx->width || !avctx->height) {
        int ret = ff_set_dimensions(avctx, 1024, 1024);
        if (ret < 0)
            return ret;
    }

    avctx->pix_fmt = AV_PIX_FMT_RGB24;

    memcpy(s->palette, default_palette, sizeof(default_palette));
    memset(s->palette + FF_ARRAY_ELEMS(default_palette), 0xFF, sizeof(s->palette) - sizeof(default_palette));
    return 0;
};

static void draw_point(uint8_t *const pixels, int linesize, int width, int height,
                       int x, int y, const uint8_t *const rgbcolor)
{
    const int stop_y = FFMIN(y + 1, height);
    const int stop_x = FFMIN(x + 1, width);
    const int start_x = FFMAX(x - 1, 0);
    const int start_y = FFMAX(y - 1, 0);
    uint8_t *dst = pixels + start_y * linesize;

    for (int y = start_y; y < stop_y; y++) {
        for (int x = start_x; x < stop_x; x++)
            memcpy(dst + x * 3, rgbcolor, 3);
        dst += linesize;
    }
}

static void draw_line(uint8_t *const pixels, int linesize,
                      int x0, int y0, int x1, int y1, const uint8_t *const rgbcolor)
{
    int dx  = FFABS(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy  = FFABS(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2, e2;

    for (;;) {
        memcpy(&pixels[y0 * linesize + x0 * 3], rgbcolor, 3);

        if (x0 == x1 && y0 == y1)
            break;

        e2 = err;

        if (e2 >-dx) {
            err -= dy;
            x0 += sx;
        }

        if (e2 < dy) {
            err += dx;
            y0 += sy;
        }
    }
}

static int decode_indexed(AVCodecContext *avctx, AVFrame *frame, const uint8_t *buf,
                          const uint8_t *buf_end, int nb_entries, int zaxis, int truecolor)
{
    ILDAContext *s = avctx->priv_data;
    int ret, x0, y0;
    uint8_t *dst;

    ret = ff_get_buffer(avctx, frame, 0);
    if (ret < 0)
        return ret;

    dst = frame->data[0];
    for (int i = 0; i < avctx->height; i++) {
        memset(dst, 0, avctx->width * 3);
        dst += frame->linesize[0];
    }

    if (nb_entries < 2 || buf_end - buf < 6 + zaxis + truecolor)
        return 0;

#define coord(x)  sign_extend(AV_RB16(buf + x), 16)
#define get_x     ((32768 + coord(0)) * avctx->width) / 65536
#define get_y     (-(coord(2) - 32767) * avctx->height) / 65536

    x0 = get_x;
    y0 = get_y;
    buf += 6 + zaxis + truecolor;

    for (int i = 1; i < nb_entries && buf_end - buf >= 6 + zaxis + truecolor; i++) {
        int x1 = get_x;
        int y1 = get_y;
        uint8_t rgb[3];
        uint8_t *color;
        if (truecolor) {
            rgb[0] = buf[7 + zaxis];
            rgb[1] = buf[6 + zaxis];
            rgb[2] = buf[5 + zaxis];
            color = rgb;
        } else {
            color = s->palette[buf[5 + zaxis]];
        }

        if (!(buf[4 + zaxis] & 0x40)) {
            if (x0 != x1 || y0 != y1) {
                draw_line(frame->data[0], frame->linesize[0], x0, y0, x1, y1, color);
            } else {
                draw_point(frame->data[0], frame->linesize[0], avctx->width, avctx->height, x0, y0, color);
            }
        }

        x0 = x1;
        y0 = y1;

        buf += 6 + zaxis + truecolor;
    }

    return 0;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame, AVPacket *avpkt)
{
    ILDAContext *s = avctx->priv_data;
    uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    const uint8_t *buf_end   = buf + buf_size;
    int ret, format, nb_entries;

    if (buf_size < 32)
        return AVERROR_INVALIDDATA;

    format = buf[7];
    nb_entries = AV_RB16(buf + 24);

    buf += 32;

    switch (format) {
    case 0:
        ret = decode_indexed(avctx, frame, buf, buf_end, nb_entries, 2, 0);
        break;
    case 1:
        ret = decode_indexed(avctx, frame, buf, buf_end, nb_entries, 0, 0);
        break;
    case 2:
        memcpy(s->palette, buf, FFMIN((buf_end - buf) / 3, nb_entries) * 3);
        return buf_size;
    case 4:
        ret = decode_indexed(avctx, frame, buf, buf_end, nb_entries, 2, 2);
        break;
    case 5:
        ret = decode_indexed(avctx, frame, buf, buf_end, nb_entries, 0, 2);
        break;
    default:
        avpriv_request_sample(avctx, "format %d", format);
        return buf_size;
    }

    if (ret < 0)
        return ret;

    frame->flags |= AV_FRAME_FLAG_KEY;
    frame->pict_type = AV_PICTURE_TYPE_I;

    *got_frame = 1;

    return buf_size;
}

const FFCodec ff_ilda_decoder = {
    .p.name         = "ilda",
    .p.long_name    = NULL_IF_CONFIG_SMALL("ILDA Image Data Transfer Format"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_ILDA,
    .priv_data_size = sizeof(ILDAContext),
    .init           = decode_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
