/*
 * ADPCM IMA XBOX encoder
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

#include <float.h>

#include "libavutil/opt.h"
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "encode.h"
#include "mathops.h"

#define BLOCK_SAMPLES 64
#define BLOCK_SIZE 36

typedef struct XboxState {
    int index;
    int16_t step_size;
    int predictor;
} XboxState;

typedef struct XboxContext {
    XboxState state[6];
} XboxContext;

static const int8_t index_tab[] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8,
};

static const int16_t step_tab[] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

static av_cold int xbox_encode_init(AVCodecContext *avctx)
{
    const int nb_channels = avctx->ch_layout.nb_channels;
    XboxContext *c = avctx->priv_data;

    if (nb_channels > FF_ARRAY_ELEMS(c->state))
        return AVERROR(EINVAL);

    if (avctx->frame_size <= 0)
        avctx->frame_size = BLOCK_SAMPLES;
    if ((avctx->frame_size % BLOCK_SAMPLES) > 0)
        avctx->frame_size = ((avctx->frame_size + BLOCK_SAMPLES-1) / BLOCK_SAMPLES) * BLOCK_SAMPLES;

    avctx->block_align = (avctx->frame_size / BLOCK_SAMPLES) * BLOCK_SIZE * nb_channels;

    return 0;
}

static int encode_nibble(XboxState *st, int val, int *error)
{
    int valpred = st->predictor;
    int step = st->step_size;

    int diff = val - valpred;
    int sign = (diff < 0) ? 8 : 0;
    if (sign != 0)
        diff = -diff;

    int delta = 0;
    int vpdiff = (step >> 3);

    if (diff >= step) {
        delta = 4;
        diff -= step;
        vpdiff += step;
    }
    step >>= 1;
    if (diff >= step) {
        delta |= 2;
        diff -= step;
        vpdiff += step;
    }
    step >>= 1;
    if (diff >= step) {
        delta |= 1;
        vpdiff += step;
    }

    if (sign != 0)
        valpred -= vpdiff;
    else
        valpred += vpdiff;

    valpred = av_clip_int16(valpred);
    if (error)
        error[0] = valpred - val;

    delta |= sign;

    st->index += index_tab[delta];
    st->index = av_clip(st->index, 0, 88);
    st->step_size = step_tab[st->index];
    st->predictor = valpred;

    return delta;
}

#define SQR(x) ((x)*(x))

static int xbox_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                             const AVFrame *frame, int *got_packet_ptr)
{
    const int nb_channels = avctx->ch_layout.nb_channels;
    XboxContext *c = avctx->priv_data;
    PutByteContext pbc;
    PutByteContext *pb = &pbc;
    int out_size, ret, nb_blocks;

    nb_blocks = frame->nb_samples / BLOCK_SAMPLES;
    out_size = nb_blocks * BLOCK_SIZE * nb_channels;
    if ((ret = ff_get_encode_buffer(avctx, avpkt, out_size, 0)) < 0)
        return ret;

    bytestream2_init_writer(pb, avpkt->data, avpkt->size);

    for (int blk = 0; blk < nb_blocks; blk++) {
        for (int ch = 0; ch < nb_channels; ch++) {
            const int16_t *src = (const int16_t *)frame->extended_data[ch];
            XboxState *state = &c->state[ch];
            XboxState xenc_state;
            XboxState *enc_state = &xenc_state;
            int start = blk * BLOCK_SAMPLES;
            const int16_t predictor = src[start];
            double best_diff = DBL_MAX;
            int best_index = 0;

            for (int index = 0; index < 89; index++) {
                double diff = 0.0;

                enc_state->predictor = predictor;
                enc_state->index = index;
                enc_state->step_size = step_tab[index];

                for (int group = 0; group < 8; group++) {
                    for (int i = 0; i < 8; i++) {
                        int sample_index = blk * BLOCK_SAMPLES + group * 8 + i + 1;
                        int sample = src[sample_index];
                        int error;

                        encode_nibble(enc_state, sample, &error);

                        if (group == 7 && i == 7)
                            continue;

                        diff += SQR(error);
                    }
                }

                if (diff < best_diff) {
                    best_diff = diff;
                    best_index = index;
                }
            }

            state->predictor = predictor;
            state->index = best_index;
            state->step_size = step_tab[best_index];

            bytestream2_put_le16(pb, predictor);
            bytestream2_put_byte(pb, best_index);
            bytestream2_put_byte(pb, 0);
        }

        for (int group = 0; group < 8; group++) {
            for (int ch = 0; ch < nb_channels; ch++) {
                const int16_t *src = (const int16_t *)frame->extended_data[ch];
                XboxState *state = &c->state[ch];
                unsigned pack = 0;
                int shift = 0;

                for (int i = 0; i < 8; i++) {
                    int sample_index = blk * BLOCK_SAMPLES + group * 8 + i + 1;
                    int sample = src[sample_index];
                    int code = encode_nibble(state, sample, NULL);

                    pack |= ((unsigned)(code & 0x0F)) << shift;
                    shift += 4;
                }

                bytestream2_put_le32(pb, pack);
            }
        }
    }

    *got_packet_ptr = 1;

    return 0;
}

const FFCodec ff_adpcm_ima_xbox_encoder = {
    .p.name         = "adpcm_ima_xbox",
    CODEC_LONG_NAME("ADPCM IMA Xbox"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ADPCM_IMA_XBOX,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .priv_data_size = sizeof(XboxContext),
    .init           = xbox_encode_init,
    FF_CODEC_ENCODE_CB(xbox_encode_frame),
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16P),
};
