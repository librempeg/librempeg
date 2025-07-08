/*
 * Mandsoft Screen Capture Codec decoder
 *
 * Copyright (c) 2017 Paul B Mahol
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

#include <stdio.h>
#include <string.h>

#include "libavutil/mem.h"
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "inflate.h"

typedef struct MSCCContext {
    unsigned          bpp;
    unsigned int      decomp_size;
    uint8_t          *decomp_buf;
    unsigned int      uncomp_size;
    uint8_t          *uncomp_buf;
    InflateContext    ic;

    uint32_t          pal[256];
} MSCCContext;

static int rle_uncompress(AVCodecContext *avctx, GetByteContext *gb, PutByteContext *pb)
{
    MSCCContext *s = avctx->priv_data;
    unsigned x = 0, y = 0;

    while (bytestream2_get_bytes_left(gb) > 0) {
        uint32_t fill;
        int j;
        unsigned run = bytestream2_get_byte(gb);

        if (run) {
            if (bytestream2_get_bytes_left_p(pb) < run * s->bpp)
                return AVERROR_INVALIDDATA;

            switch (avctx->bits_per_coded_sample) {
            case 8:
                fill = bytestream2_get_byte(gb);
                break;
            case 16:
                fill = bytestream2_get_le16(gb);
                break;
            case 24:
                fill = bytestream2_get_le24(gb);
                break;
            case 32:
                fill = bytestream2_get_le32(gb);
                break;
            }

            for (j = 0; j < run; j++) {
                switch (avctx->bits_per_coded_sample) {
                case 8:
                    bytestream2_put_byte(pb, fill);
                    break;
                case 16:
                    bytestream2_put_le16(pb, fill);
                    break;
                case 24:
                    bytestream2_put_le24(pb, fill);
                    break;
                case 32:
                    bytestream2_put_le32(pb, fill);
                    break;
                }
            }
            x += run;
        } else {
            unsigned copy = bytestream2_get_byte(gb);

            if (copy == 0) {
                x = 0;
                y++;
                bytestream2_seek_p(pb, y * avctx->width * s->bpp, SEEK_SET);
            } else if (copy == 1) {
                return 0;
            } else if (copy == 2) {

                x += bytestream2_get_byte(gb);
                y += bytestream2_get_byte(gb);

                bytestream2_seek_p(pb, y * avctx->width * s->bpp + x * s->bpp, SEEK_SET);
            } else {
                if (bytestream2_get_bytes_left_p(pb) < copy * s->bpp)
                    return AVERROR_INVALIDDATA;

                for (j = 0; j < copy; j++) {
                    switch (avctx->bits_per_coded_sample) {
                    case 8:
                        bytestream2_put_byte(pb, bytestream2_get_byte(gb));
                        break;
                    case 16:
                        bytestream2_put_le16(pb, bytestream2_get_le16(gb));
                        break;
                    case 24:
                        bytestream2_put_le24(pb, bytestream2_get_le24(gb));
                        break;
                    case 32:
                        bytestream2_put_le32(pb, bytestream2_get_le32(gb));
                        break;
                    }
                }

                if (s->bpp == 1 && (copy & 1))
                    bytestream2_skip(gb, 1);
                x += copy;
            }
        }
    }

    return AVERROR_INVALIDDATA;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame, AVPacket *avpkt)
{
    MSCCContext *s = avctx->priv_data;
    InflateContext *ic = &s->ic;
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    GetByteContext gb;
    PutByteContext pb;
    int ret;

    if (avpkt->size < 3)
        return buf_size;

    if (avctx->codec_id == AV_CODEC_ID_MSCC) {
        const uint8_t start = avpkt->data[2] ^ avpkt->data[0];

        if ((ret = av_packet_make_writable(avpkt)) < 0)
            return ret;

        avpkt->data[2] = start;
        buf += 2;
        buf_size -= 2;
    }
    ret = ff_inflate(ic, buf, buf_size, s->decomp_buf, 1, s->decomp_size, s->decomp_size);
    if (ret < 0)
        return ret;

    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    if (avctx->pix_fmt == AV_PIX_FMT_PAL8) {
        size_t size;
        const uint8_t *pal = av_packet_get_side_data(avpkt, AV_PKT_DATA_PALETTE, &size);

        if (pal && size == AVPALETTE_SIZE) {
            for (int j = 0; j < 256; j++)
                s->pal[j] = 0xFF000000 | AV_RL32(pal + j * 4);
        } else if (pal) {
            av_log(avctx, AV_LOG_ERROR,
                   "Palette size %"SIZE_SPECIFIER" is wrong\n", size);
        }
        memcpy(frame->data[1], s->pal, AVPALETTE_SIZE);
    }

    bytestream2_init(&gb, s->decomp_buf, ic->x);
    bytestream2_init_writer(&pb, s->uncomp_buf, s->uncomp_size);

    ret = rle_uncompress(avctx, &gb, &pb);
    if (ret)
        return ret;

    for (int j = 0; j < avctx->height; j++) {
        memcpy(frame->data[0] + (avctx->height - j - 1) * frame->linesize[0],
               s->uncomp_buf + s->bpp * j * avctx->width, s->bpp * avctx->width);
    }

    *got_frame = 1;

    return avpkt->size;
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    MSCCContext *s = avctx->priv_data;
    int stride;

    switch (avctx->bits_per_coded_sample) {
    case  8: avctx->pix_fmt = AV_PIX_FMT_PAL8;   break;
    case 16: avctx->pix_fmt = AV_PIX_FMT_RGB555; break;
    case 24: avctx->pix_fmt = AV_PIX_FMT_BGR24;  break;
    case 32: avctx->pix_fmt = AV_PIX_FMT_BGRA;   break;
    default:
        av_log(avctx, AV_LOG_ERROR, "Unsupported bitdepth %i\n", avctx->bits_per_coded_sample);
        return AVERROR_INVALIDDATA;
    }

    s->bpp = avctx->bits_per_coded_sample >> 3;
    stride = 4 * ((avctx->width * avctx->bits_per_coded_sample + 31) / 32);

    s->decomp_size = 2 * avctx->height * stride;
    if (!(s->decomp_buf = av_malloc(s->decomp_size)))
        return AVERROR(ENOMEM);

    s->uncomp_size = avctx->height * stride;
    if (!(s->uncomp_buf = av_malloc(s->uncomp_size)))
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold int decode_close(AVCodecContext *avctx)
{
    MSCCContext *s = avctx->priv_data;

    av_freep(&s->decomp_buf);
    s->decomp_size = 0;
    av_freep(&s->uncomp_buf);
    s->uncomp_size = 0;
    ff_inflate(&s->ic, NULL, 0, NULL, 0, 0, 0);

    return 0;
}

const FFCodec ff_mscc_decoder = {
    .p.name           = "mscc",
    CODEC_LONG_NAME("Mandsoft Screen Capture Codec"),
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_MSCC,
    .priv_data_size   = sizeof(MSCCContext),
    .init             = decode_init,
    .close            = decode_close,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities   = AV_CODEC_CAP_DR1,
    .caps_internal    = FF_CODEC_CAP_INIT_CLEANUP,
};

const FFCodec ff_srgc_decoder = {
    .p.name           = "srgc",
    CODEC_LONG_NAME("Screen Recorder Gold Codec"),
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_SRGC,
    .priv_data_size   = sizeof(MSCCContext),
    .init             = decode_init,
    .close            = decode_close,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities   = AV_CODEC_CAP_DR1,
    .caps_internal    = FF_CODEC_CAP_INIT_CLEANUP,
};
