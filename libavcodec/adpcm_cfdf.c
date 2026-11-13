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

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"

typedef struct CFDFADPCMState {
    unsigned sample;
    int pairs;
    int seeded;
} CFDFADPCMState;

typedef struct CFDFADPCMContext {
    CFDFADPCMState state;
    int framing; /* 0: continuous, 1: independent chunk, 2: D5 block */
} CFDFADPCMContext;

static av_cold int decode_init(AVCodecContext *avctx)
{
    CFDFADPCMContext *c = avctx->priv_data;

    if (avctx->ch_layout.nb_channels != 1 || avctx->extradata_size > 1)
        return AVERROR_INVALIDDATA;
    if (avctx->extradata_size) {
        c->framing = avctx->extradata[0];
        if (c->framing > 2)
            return AVERROR_INVALIDDATA;
    }
    avctx->sample_fmt = AV_SAMPLE_FMT_S16;
    return 0;
}

static void put_sample(int16_t *dst, int index, unsigned sample, int scale)
{
    int signed_sample = sample < 0x80 ? sample : (int)sample - 0x100;
    unsigned pcm = (unsigned)((signed_sample - 0x40) * scale) & 0xFFFF;

    if (dst)
        dst[index] = pcm < 0x8000 ? pcm : (int)pcm - 0x10000;
}

static int expand(CFDFADPCMState *state, const uint8_t *src, int size,
                  int framing, int16_t *dst)
{
    int pos = 0, count = 0;
    int scale = framing == 2 ? 0x200 : 0x100;

    if (framing)
        *state = (CFDFADPCMState){ 0 };
    if (!state->seeded && size) {
        state->sample = src[pos++];
        state->seeded = 1;
        if (framing != 2)
            put_sample(dst, count++, state->sample, scale);
    }
    while (pos < size) {
        unsigned code = src[pos++];

        if (count > INT_MAX - 0x80)
            return AVERROR_INVALIDDATA;
        if (state->pairs) {
            for (int shift = 4; shift >= 0; shift -= 4) {
                unsigned nibble = (code >> shift) & 0xF;
                int delta = (int)(nibble ^ 8) - 8;

                state->sample = (state->sample + delta) & 0xFF;
                put_sample(dst, count++, state->sample, scale);
            }
            state->pairs--;
        } else if (code < 0x80) {
            state->sample = code;
            put_sample(dst, count++, state->sample, scale);
        } else if (code < 0xC0) {
            state->pairs = (code & 0x3F) + 1;
        } else {
            int repeat = (code & 0x3F) + 1;

            for (int i = 0; i < repeat; i++)
                put_sample(dst, count++, state->sample, scale);
        }
    }
    if (framing && state->pairs)
        return AVERROR_INVALIDDATA;
    return count;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame, AVPacket *pkt)
{
    CFDFADPCMContext *c = avctx->priv_data;
    CFDFADPCMState next = c->state;
    int samples, ret;

    samples = expand(&next, pkt->data, pkt->size, c->framing, NULL);
    if (samples < 0)
        return samples;
    if (!samples) {
        c->state = next;
        return pkt->size;
    }
    frame->nb_samples = samples;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;
    expand(&c->state, pkt->data, pkt->size, c->framing,
           (int16_t *)frame->data[0]);
    *got_frame = 1;
    return pkt->size;
}

static void decode_flush(AVCodecContext *avctx)
{
    CFDFADPCMContext *c = avctx->priv_data;

    c->state = (CFDFADPCMState){ 0 };
}

const FFCodec ff_adpcm_cfdf_decoder = {
    .p.name         = "adpcm_cfdf",
    CODEC_LONG_NAME("ADPCM Cyberflix DreamFactory"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ADPCM_CFDF,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .priv_data_size = sizeof(CFDFADPCMContext),
    .init           = decode_init,
    .flush          = decode_flush,
    FF_CODEC_DECODE_CB(decode_frame),
};
