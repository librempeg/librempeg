/*
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

/**
 * @file
 * Atrac3Plus frame skip bitstream filter.
 */

#include "libavutil/fifo.h"
#include "libavutil/intreadwrite.h"
#include "bsf.h"
#include "bsf_internal.h"

typedef struct AtracContext {
    AVPacket *in_pkt;
    AVPacket *out_pkt;
    AVFifo *fifo;
    int64_t pts, dts;
} AtracContext;

static int init(AVBSFContext *ctx)
{
    AtracContext *s = ctx->priv_data;
    AVRational sr = av_make_q(ctx->par_in->sample_rate, 1);

    ctx->par_out->block_align = ctx->par_in->block_align;
    ctx->time_base_in = av_inv_q(sr);
    ctx->time_base_out = av_inv_q(sr);

    s->fifo = av_fifo_alloc2(ctx->par_in->block_align, 1, 1);
    s->in_pkt = av_packet_alloc();
    s->out_pkt = av_packet_alloc();
    if (!s->fifo || !s->in_pkt || !s->out_pkt)
        return AVERROR(ENOMEM);

    return 0;
}

static int filter(AVBSFContext *ctx, AVPacket *pkt)
{
    AtracContext *s = ctx->priv_data;
    uint8_t tmp[8];
    int ret;

    if (!s->out_pkt->size) {
        ret = av_new_packet(s->out_pkt, ctx->par_in->block_align);
        if (ret < 0)
            return ret;
    }

    if (av_fifo_can_read(s->fifo) >= 8) {
        av_fifo_peek(s->fifo, tmp, 8, 0);
        if ((AV_RB64(tmp) & 0xff000000ffffffffULL ) == 0x0f00000000000000ULL)
            av_fifo_drain2(s->fifo, 8);
    }

    while (1) {
        if (av_fifo_can_read(s->fifo) < ctx->par_in->block_align) {
            ret = ff_bsf_get_packet_ref(ctx, s->in_pkt);
            if (ret < 0) {
                if (ret == AVERROR_EOF && av_fifo_can_read(s->fifo) > 0) {
                    s->pts = s->out_pkt->pts + av_fifo_can_read(s->fifo);
                    s->dts = s->out_pkt->dts + av_fifo_can_read(s->fifo);
                    goto flush;
                }
                return ret;
            }

            av_packet_rescale_ts(s->in_pkt, ctx->time_base_in, ctx->time_base_out);
            av_packet_copy_props(s->out_pkt, s->in_pkt);
            s->pts = s->out_pkt->pts;
            s->dts = s->out_pkt->dts;

            if (s->in_pkt->size > ctx->par_in->block_align)
                av_fifo_write(s->fifo, s->in_pkt->data+3, s->in_pkt->size-3);
            else
                av_fifo_write(s->fifo, s->in_pkt->data, s->in_pkt->size);
            av_packet_unref(s->in_pkt);
        }

        if (av_fifo_can_read(s->fifo) >= 8) {
            av_fifo_peek(s->fifo, tmp, 8, 0);
            if ((AV_RB64(tmp) & 0xff000000ffffffffULL ) == 0x0f00000000000000ULL)
                av_fifo_drain2(s->fifo, 8);
        }

        if (av_fifo_can_read(s->fifo) >= ctx->par_in->block_align) {
flush:
            ret = FFMIN(av_fifo_can_read(s->fifo), ctx->par_in->block_align);
            av_fifo_read(s->fifo, s->out_pkt->data, ret);

            av_packet_unref(pkt);
            av_packet_move_ref(pkt, s->out_pkt);

            pkt->pts = s->pts;
            pkt->dts = s->dts;
            pkt->size = ret;

            break;
        }
    }

    return 0;
}

static void flush(AVBSFContext *ctx)
{
    AtracContext *s = ctx->priv_data;

    av_fifo_reset2(s->fifo);
    av_packet_unref(s->in_pkt);
    av_packet_unref(s->out_pkt);
}

static void uninit(AVBSFContext *ctx)
{
    AtracContext *s = ctx->priv_data;

    av_fifo_freep2(&s->fifo);
    av_packet_free(&s->in_pkt);
    av_packet_free(&s->out_pkt);
}

const FFBitStreamFilter ff_atrac3plus_skip_bsf = {
    .p.name         = "atrac3plus_skip",
    .p.codec_ids    = (const enum AVCodecID []){ AV_CODEC_ID_ATRAC3P, AV_CODEC_ID_NONE },
    .priv_data_size = sizeof(AtracContext),
    .init           = init,
    .filter         = filter,
    .flush          = flush,
    .close          = uninit,
};
