/*
 * ADPCM XA encoder
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

#include "libavutil/opt.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "encode.h"
#include "mathops.h"

#define BLOCK_SAMPLES 28

static const int8_t filter_k1[] = { 0, 60, 115, 98 };
static const int8_t filter_k2[] = { 0, 0, -52, -55 };

typedef struct XAState {
    uint64_t mse;
    int prev1, prev2;
} XAState;

typedef struct XAContext {
    AVClass *class;

    int search;
    XAState state[2];
} XAContext;

static av_cold int xa_encode_init(AVCodecContext *avctx)
{
    const int nb_channels = avctx->ch_layout.nb_channels;
    XAContext *c = avctx->priv_data;

    if (nb_channels > FF_ARRAY_ELEMS(c->state))
        return AVERROR(EINVAL);

    avctx->frame_size = 4032 / nb_channels;

    return 0;
}

#define SQR(x) ((x)*(x))

static uint8_t do_encode(XAState *outstate,
                         const XAState *instate,
                         const int16_t *samples,
                         int sample_limit,
                         uint8_t *data,
                         int filter,
                         int sample_shift,
                         int shift_range)
{
    const uint8_t nondata_mask = 0xF0;
    int min_shift = sample_shift;
    int k1 = filter_k1[filter];
    int k2 = filter_k2[filter];
    uint8_t hdr = (min_shift & 0x0F) | (filter << 4);

    if (outstate != instate)
        memcpy(outstate, instate, sizeof(XAState));

    outstate->mse = 0;

    for (int i = 0; i < BLOCK_SAMPLES; i++) {
        int32_t sample = (i >= sample_limit) ? 0 : samples[i];
        int32_t prev = (k1*outstate->prev1 + k2*outstate->prev2 + (1<<5))>>6;
        int32_t sample_enc = sample - prev;

        sample_enc *= 1 << min_shift;
        sample_enc += 1 << (shift_range-1);
        sample_enc >>= shift_range;

        if (sample_enc < (INT16_MIN >> shift_range))
            sample_enc = INT16_MIN >> shift_range;
        if (sample_enc > (INT16_MAX >> shift_range))
            sample_enc = INT16_MAX >> shift_range;

        sample_enc &= 0xF;

        int32_t sample_dec = (int16_t)((sample_enc & 0xF) << shift_range);
        sample_dec >>= min_shift;
        sample_dec += prev;
        sample_dec = av_clip_int16(sample_dec);
        int64_t sample_error = sample_dec - sample;

        data[i] = (data[i] & nondata_mask) | sample_enc;
        outstate->mse += sample_error * sample_error;

        outstate->prev2 = outstate->prev1;
        outstate->prev1 = sample_dec;
    }

    return hdr;
}

static int find_min_shift(const XAState *state,
                          const int16_t *samples, int sample_limit, int filter,
                          int shift_range)
{
    const int k1 = filter_k1[filter];
    const int k2 = filter_k2[filter];
    int prev1 = state->prev1;
    int prev2 = state->prev2;
    int s_min = 0;
    int s_max = 0;
    int shift = 0;

    for (int i = 0; i < BLOCK_SAMPLES; i++) {
        int raw_sample = (i >= sample_limit) ? 0 : samples[i];
        int prev = (k1 * prev1 + k2 * prev2 + (1 << 5)) >> 6;
        int sample = raw_sample - prev;

        s_min = FFMIN(s_min, sample);
        s_max = FFMAX(s_max, sample);

        prev2 = prev1;
        prev1 = raw_sample;
    }

    while (shift < shift_range && (s_max >> shift) > (INT16_MAX >> shift_range)) {
        shift++;
    }

    while (shift < shift_range && (s_min >> shift) < (INT16_MIN >> shift_range)) {
        shift++;
    }

    return shift_range - shift;
}

static uint8_t encode(XAState *state, const int16_t *samples, int sample_limit,
                      uint8_t *data, const int search)
{
    int64_t best_mse = INT64_MAX;
    int best_sample_shift = 0;
    int best_filter = 0;
    XAState tmp;

    for (int filter = 0; filter < FF_ARRAY_ELEMS(filter_k1); filter++) {
        int true_min_shift = find_min_shift(state, samples, sample_limit, filter, 12);
        int min_shift = FFMAX(true_min_shift - search, 0);
        int max_shift = FFMIN(true_min_shift + search, 12);

        for (int sample_shift = min_shift; sample_shift <= max_shift; sample_shift++) {
            do_encode(&tmp, state, samples, sample_limit, data, filter, sample_shift, 12);

            if (best_mse > tmp.mse) {
                best_mse = tmp.mse;
                best_filter = filter;
                best_sample_shift = sample_shift;
            }
        }
    }

    return do_encode(state, state,
                     samples, sample_limit,
                     data, best_filter, best_sample_shift, 12);
}

static int xa_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                           const AVFrame *frame, int *got_packet_ptr)
{
    const int nb_channels = avctx->ch_layout.nb_channels;
    const int nb_subblocks = (3 - nb_channels) * 4;
    const int samples_per_block = BLOCK_SAMPLES * nb_subblocks;
    XAContext *c = avctx->priv_data;
    int ret, nb_blocks;
    uint8_t *dst;

    nb_blocks = nb_channels * frame->nb_samples / 224;
    if ((ret = ff_get_encode_buffer(avctx, avpkt, nb_blocks * 128, 0)) < 0)
        return ret;

    dst = avpkt->data;
    memset(dst, 0, avpkt->size);
    for (int blk = 0, sample_offset = 0; blk < nb_blocks; blk++, sample_offset += samples_per_block) {
        for (int sb = 0, start = 0; sb < nb_subblocks; sb++, start += BLOCK_SAMPLES) {
            for (int ch = 0; ch < nb_channels; ch++) {
                const int16_t *src = ((const int16_t *)frame->extended_data[ch]) + sample_offset + start;
                XAState *state = &c->state[ch];
                uint8_t buf[BLOCK_SAMPLES];

                dst[4 + sb * nb_channels + ch] = encode(state, src, frame->nb_samples - sample_offset - start, buf, c->search);

                if (nb_channels == 1) {
                    if (!(sb & 1)) {
                        for (int i = 0; i < BLOCK_SAMPLES; i++)
                            dst[16 + sb/2 + i * 4] |= buf[i] & 0xF;
                    } else {
                        for (int i = 0; i < BLOCK_SAMPLES; i++)
                            dst[16 + sb/2 + i * 4] |= (buf[i] & 0xF) << 4;
                    }
                } else if (ch == 0) {
                    for (int i = 0; i < BLOCK_SAMPLES; i++)
                        dst[16 + sb + i * 4] |= buf[i] & 0xF;
                } else {
                    for (int i = 0; i < BLOCK_SAMPLES; i++)
                        dst[16 + sb + i * 4] |= (buf[i] & 0xF) << 4;
                }
            }
        }

        dst += 128;
    }

    *got_packet_ptr = 1;

    return 0;
}

#define FLAGS AV_OPT_FLAG_ENCODING_PARAM | AV_OPT_FLAG_AUDIO_PARAM
static const AVOption options[] = {
    { "search", "set the shift search", offsetof(XAContext, search), AV_OPT_TYPE_INT, {.i64 = 1 }, 0, 4, FLAGS },
    { NULL },
};

static const AVClass adpcm_xa_encoder_class = {
    .class_name = "ADPCM XA encoder",
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_adpcm_xa_encoder = {
    .p.name         = "adpcm_xa",
    CODEC_LONG_NAME("ADPCM CDROM XA"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ADPCM_XA,
    .p.capabilities = AV_CODEC_CAP_DR1,
    .priv_data_size = sizeof(XAContext),
    .init           = xa_encode_init,
    FF_CODEC_ENCODE_CB(xa_encode_frame),
    .p.priv_class   = &adpcm_xa_encoder_class,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16P),
    CODEC_SAMPLERATES(18900, 37800),
    CODEC_CH_LAYOUTS(AV_CHANNEL_LAYOUT_MONO,
                     AV_CHANNEL_LAYOUT_STEREO),
};
