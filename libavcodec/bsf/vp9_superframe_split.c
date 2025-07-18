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

/**
 * @file
 * This bitstream filter splits VP9 superframes into packets containing
 * just one frame.
 */

#include <stddef.h>

#include "bsf.h"
#include "bsf_internal.h"
#include "bytestream.h"
#include "get_bits.h"

typedef struct VP9SFSplitContext {
    AVPacket *buffer_pkt;

    int nb_frames;
    int next_frame;
    size_t next_frame_offset;
    int sizes[8];
} VP9SFSplitContext;

static int vp9_superframe_split_filter(AVBSFContext *ctx, AVPacket *out)
{
    VP9SFSplitContext *s = ctx->priv_data;
    AVPacket *in;
    int i, j, ret, marker;
    int is_superframe = !!s->buffer_pkt->data;

    if (!s->buffer_pkt->data) {
        ret = ff_bsf_get_packet_ref(ctx, s->buffer_pkt);
        if (ret < 0)
            return ret;
        in = s->buffer_pkt;

        if (!in->size)
            goto passthrough;

        marker = in->data[in->size - 1];
        if ((marker & 0xe0) == 0xc0) {
            int length_size = 1 + ((marker >> 3) & 0x3);
            int   nb_frames = 1 + (marker & 0x7);
            int    idx_size = 2 + nb_frames * length_size;

            if (in->size >= idx_size && in->data[in->size - idx_size] == marker) {
                GetByteContext bc;
                int64_t total_size = 0;

                bytestream2_init(&bc, in->data + in->size + 1 - idx_size,
                                 nb_frames * length_size);

                for (i = 0; i < nb_frames; i++) {
                    int frame_size = 0;
                    for (j = 0; j < length_size; j++)
                        frame_size |= bytestream2_get_byte(&bc) << (j * 8);

                    total_size += frame_size;
                    if (frame_size <= 0 || total_size > in->size - idx_size) {
                        av_log(ctx, AV_LOG_ERROR,
                               "Invalid frame size in a superframe: %d\n", frame_size);
                        ret = AVERROR(EINVAL);
                        goto fail;
                    }
                    s->sizes[i] = frame_size;
                }
                s->nb_frames         = nb_frames;
                s->next_frame        = 0;
                s->next_frame_offset = 0;
                is_superframe        = 1;
            }
        }
    }

    if (is_superframe) {
        GetBitContext gb;
        int profile, invisible = 0;

        ret = av_packet_ref(out, s->buffer_pkt);
        if (ret < 0)
            goto fail;

        out->data += s->next_frame_offset;
        out->size  = s->sizes[s->next_frame];

        s->next_frame_offset += out->size;
        s->next_frame++;

        if (s->next_frame >= s->nb_frames)
            av_packet_unref(s->buffer_pkt);

        ret = init_get_bits8(&gb, out->data, out->size);
        if (ret < 0)
            goto fail;

        get_bits(&gb, 2); // frame_marker
        profile  = get_bits1(&gb);
        profile |= get_bits1(&gb) << 1;
        if (profile == 3)
            get_bits1(&gb);
        if (!get_bits1(&gb)) {
            get_bits1(&gb);
            invisible = !get_bits1(&gb);
        }

        if (invisible)
            out->pts = AV_NOPTS_VALUE;

    } else {
passthrough:
        av_packet_move_ref(out, s->buffer_pkt);
    }

    return 0;
fail:
    if (ret < 0)
        av_packet_unref(out);
    av_packet_unref(s->buffer_pkt);
    return ret;
}

static int vp9_superframe_split_init(AVBSFContext *ctx)
{
    VP9SFSplitContext *s = ctx->priv_data;

    s->buffer_pkt = av_packet_alloc();
    if (!s->buffer_pkt)
        return AVERROR(ENOMEM);

    return 0;
}

static void vp9_superframe_split_flush(AVBSFContext *ctx)
{
    VP9SFSplitContext *s = ctx->priv_data;
    av_packet_unref(s->buffer_pkt);
}

static void vp9_superframe_split_uninit(AVBSFContext *ctx)
{
    VP9SFSplitContext *s = ctx->priv_data;
    av_packet_free(&s->buffer_pkt);
}

const FFBitStreamFilter ff_vp9_superframe_split_bsf = {
    .p.name         = "vp9_superframe_split",
    .p.codec_ids    = (const enum AVCodecID []){ AV_CODEC_ID_VP9, AV_CODEC_ID_NONE },
    .priv_data_size = sizeof(VP9SFSplitContext),
    .init           = vp9_superframe_split_init,
    .flush          = vp9_superframe_split_flush,
    .close          = vp9_superframe_split_uninit,
    .filter         = vp9_superframe_split_filter,
};
