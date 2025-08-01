/*
 * ADPCM THP codecs
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

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "encode.h"
#include "mathops.h"

#define BLOCK_SAMPLES 14
#define BLOCK_SIZE 8

typedef struct THPChannel {
    int16_t input[BLOCK_SAMPLES+2];
    int16_t table[16];
} THPChannel;

typedef struct THPContext {
    AVClass *class;

    uint8_t *coeffs;
    int coeffs_len;

    int coded_nb_samples;
    int le;

    THPChannel chs[8];
} THPContext;

static av_cold int thp_encode_init(AVCodecContext *avctx)
{
    const int nb_channels = avctx->ch_layout.nb_channels;
    THPContext *c = avctx->priv_data;

    if (nb_channels > FF_ARRAY_ELEMS(c->chs))
        return AVERROR(EINVAL);

    c->le = avctx->codec_id == AV_CODEC_ID_ADPCM_THP_LE || avctx->codec_id == AV_CODEC_ID_ADPCM_NDSP_LE;
    c->coded_nb_samples = avctx->codec_id == AV_CODEC_ID_ADPCM_THP || avctx->codec_id == AV_CODEC_ID_ADPCM_THP_LE;

    if ((avctx->frame_size % BLOCK_SAMPLES) > 0)
        avctx->frame_size = ((avctx->frame_size + BLOCK_SAMPLES-1) / BLOCK_SAMPLES) * BLOCK_SAMPLES;
    if (avctx->frame_size <= 0)
        avctx->frame_size = BLOCK_SAMPLES;
    avctx->block_align = (avctx->frame_size / BLOCK_SAMPLES) * BLOCK_SIZE * nb_channels;

    if (!c->coded_nb_samples) {
        avctx->extradata = av_calloc(nb_channels, 32);
        if (!avctx->extradata)
            return AVERROR(ENOMEM);
        avctx->extradata_size = nb_channels * 32;

        for (int ch = 0; ch < nb_channels; ch++) {
            if (c->coeffs_len >= 32 * nb_channels) {
                for (int n = 0; n < 16; n++)
                    c->chs[ch].table[n] = sign_extend(AV_RL16(c->coeffs + n*2 + 32*ch), 16);
            }

            if (c->le) {
                for (int n = 0; n < 16; n++)
                    AV_WL16(avctx->extradata + n*2 + 32*ch, c->chs[ch].table[n]);
            } else {
                for (int n = 0; n < 16; n++)
                    AV_WB16(avctx->extradata + n*2 + 32*ch, c->chs[ch].table[n]);
            }
        }
    }

    return 0;
}

static int thp_encode(THPChannel *chs, uint8_t *dst,
                      const int16_t *samples, const int nb_samples)
{
    const int16_t *coefs = chs->table;
    int in_samples[8][BLOCK_SAMPLES+2];
    int out_samples[8][BLOCK_SAMPLES];
    int16_t *input = chs->input;
    int64_t min, error_sum[8];
    int best_index = 0;
    int scale[8];

    for (int n = 0; n < nb_samples; n++)
        input[n + 2] = samples[n];

    for (int i = 0 ; i < 8; i++) {
        const int16_t *coefs_in = coefs + i * 2;
        int distance, index;
        int v1, v2, v3;

        in_samples[i][0] = input[0];
        in_samples[i][1] = input[1];

        distance = 0;
        for (int s = 0; s < nb_samples; s++) {
            in_samples[i][s + 2] = v1 = ((input[s] * coefs_in[1]) + (input[s + 1] * coefs_in[0])) / 2048;
            v2 = input[s + 2] - v1;
            v3 = av_clip_int16(v2);
            if (FFABS(v3) > FFABS(distance))
                distance = v3;
        }

        for (scale[i] = 0; (scale[i] <= 12) && ((distance > 7) || (distance <- 8)); scale[i]++, distance /= 2);

        scale[i] = (scale[i] <= 1) ? -1 : scale[i] - 2;

        do {
            scale[i]++;
            error_sum[i] = 0;
            index = 0;

            for (int s = 0; s < nb_samples; s++) {
                v1 = (in_samples[i][s] * coefs_in[1]) + (in_samples[i][s + 1] * coefs_in[0]);
                v2 = ((input[s + 2] * (1 << 11)) - v1) / 2048;
                v3 = (v2 > 0) ? (v2 + (1 << scale[i])/2) / (1 << scale[i]) : (v2 - (1 << scale[i])/2) / (1 << scale[i]);

                if (v3 < -8) {
                    if (index < (v3 = -8 - v3))
                        index = v3;
                    v3 = -8;
                } else if (v3 > 7) {
                    if (index < (v3 -= 7))
                        index = v3;
                    v3 = 7;
                }

                out_samples[i][s] = v3;

                v1 = (v1 + ((v3 * (1 << scale[i])) * (1 << 11)) + 1024) >> 11;
                in_samples[i][s + 2] = v2 = av_clip_int16(v1);
                v3 = input[s + 2] - v2;
                error_sum[i] += v3 * (int64_t)v3;
            }

            for (int x = index + 8; x > 256; x >>= 1)
                if (++scale[i] >= 12)
                    scale[i] = 11;
        } while ((scale[i] < 12) && (index > 1));
    }

    min = INT64_MAX;
    for (int i = 0; i < 8; i++) {
        if (error_sum[i] < min) {
            min = error_sum[i];
            best_index = i;
        }
    }

    for (int s = 0; s < nb_samples; s++)
        input[s + 2] = in_samples[best_index][s + 2];

    dst[0] = (best_index << 4) | (scale[best_index] & 0xF);
    for (int s = nb_samples; s < BLOCK_SAMPLES; s++)
        out_samples[best_index][s] = 0;

    for (int y = 0; y < BLOCK_SIZE-1; y++)
        dst[y + 1] = ((out_samples[best_index][y * 2] & 0xF) << 4) | (out_samples[best_index][y * 2 + 1] & 0xF);

    input[0] = input[BLOCK_SAMPLES];
    input[1] = input[BLOCK_SAMPLES+1];

    return 0;
}

typedef struct ThreadData {
    const AVFrame *frame;
    uint8_t *dst;
    int nb_blocks;
} ThreadData;

static int encode_channel(AVCodecContext *avctx, void *arg, int ch, int threadnr)
{
    THPContext *c = avctx->priv_data;
    ThreadData *td = arg;
    const int nb_blocks = td->nb_blocks;
    const AVFrame *frame = td->frame;
    uint8_t *dst = td->dst + nb_blocks * BLOCK_SIZE * ch;
    const int16_t *samples = (const int16_t *)frame->extended_data[ch];

    for (int n = 0; n < nb_blocks; n++) {
        thp_encode(&c->chs[ch], dst, samples + BLOCK_SAMPLES * n,
                   FFMIN(frame->nb_samples - n * BLOCK_SAMPLES, BLOCK_SAMPLES));

        dst += BLOCK_SIZE;
    }

    return 0;
}

static int thp_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                            const AVFrame *frame, int *got_packet_ptr)
{
    const int nb_channels = avctx->ch_layout.nb_channels;
    THPContext *c = avctx->priv_data;
    int out_size, ret, nb_blocks;
    ThreadData td;
    uint8_t *dst;

    nb_blocks = ((frame->nb_samples + BLOCK_SAMPLES-1) / BLOCK_SAMPLES);
    out_size = nb_blocks * BLOCK_SIZE * nb_channels;
    if (c->coded_nb_samples)
        out_size += (32 + 4) * nb_channels + 8;

    if ((ret = ff_get_encode_buffer(avctx, avpkt, out_size, 0)) < 0)
        return ret;
    dst = avpkt->data;

    if (c->coded_nb_samples) {
        if (c->le) {
            bytestream_put_le32(&dst, nb_blocks * BLOCK_SIZE * nb_channels);
            bytestream_put_le32(&dst, frame->nb_samples);

            for (int ch = 0; ch < nb_channels; ch++) {
                for (int n = 0; n < 16; n++)
                    bytestream_put_le16(&dst, c->chs[ch].table[n]);
            }

            for (int ch = 0; ch < nb_channels; ch++) {
                bytestream_put_le16(&dst, c->chs[ch].input[0]);
                bytestream_put_le16(&dst, c->chs[ch].input[1]);
            }
        } else {
            bytestream_put_be32(&dst, nb_blocks * BLOCK_SIZE * nb_channels);
            bytestream_put_be32(&dst, frame->nb_samples);

            for (int ch = 0; ch < nb_channels; ch++) {
                for (int n = 0; n < 16; n++)
                    bytestream_put_be16(&dst, c->chs[ch].table[n]);
            }

            for (int ch = 0; ch < nb_channels; ch++) {
                bytestream_put_be16(&dst, c->chs[ch].input[0]);
                bytestream_put_be16(&dst, c->chs[ch].input[1]);
            }
        }
    }

    td.nb_blocks = nb_blocks;
    td.frame = frame;
    td.dst = dst;

    avctx->execute2(avctx, encode_channel, &td, NULL, nb_channels);

    *got_packet_ptr = 1;

    return 0;
}

#define OFFSET(x) offsetof(THPContext, x)
#define FLAGS AV_OPT_FLAG_ENCODING_PARAM | AV_OPT_FLAG_AUDIO_PARAM

static const AVOption options[] = {
    { "coeffs", "channel coefficients table", OFFSET(coeffs), AV_OPT_TYPE_BINARY,
        {.str="f80f00f8fc0700fcfe0300feff0100ffff0080ff7f00c0ff3f00e0ff1f00f0fff80f00f8fc0700fcfe0300feff0100ffff0080ff7f00c0ff3f00e0ff1f00f0ff"},
        .flags = FLAGS },
    { NULL },
};

static const AVClass adpcm_thp_encoder_class = {
    .class_name = "ADPCM THP/NDSP encoder",
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_adpcm_thp_encoder = {
    .p.name         = "adpcm_thp",
    CODEC_LONG_NAME("ADPCM Nintendo THP"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ADPCM_THP,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_SMALL_LAST_FRAME |
                      AV_CODEC_CAP_SLICE_THREADS |
                      AV_CODEC_CAP_ENCODER_REORDERED_OPAQUE,
    .priv_data_size = sizeof(THPContext),
    .p.priv_class   = &adpcm_thp_encoder_class,
    .init           = thp_encode_init,
    FF_CODEC_ENCODE_CB(thp_encode_frame),
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16P),
};

const FFCodec ff_adpcm_thp_le_encoder = {
    .p.name         = "adpcm_thp_le",
    CODEC_LONG_NAME("ADPCM Nintendo THP (little-endian)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ADPCM_THP_LE,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_SMALL_LAST_FRAME |
                      AV_CODEC_CAP_SLICE_THREADS |
                      AV_CODEC_CAP_ENCODER_REORDERED_OPAQUE,
    .priv_data_size = sizeof(THPContext),
    .p.priv_class   = &adpcm_thp_encoder_class,
    .init           = thp_encode_init,
    FF_CODEC_ENCODE_CB(thp_encode_frame),
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16P),
};

const FFCodec ff_adpcm_ndsp_encoder = {
    .p.name         = "adpcm_ndsp",
    CODEC_LONG_NAME("ADPCM Nintendo DSP"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ADPCM_NDSP,
    .p.capabilities = AV_CODEC_CAP_DR1 |
                      AV_CODEC_CAP_SLICE_THREADS |
                      AV_CODEC_CAP_ENCODER_REORDERED_OPAQUE,
    .priv_data_size = sizeof(THPContext),
    .p.priv_class   = &adpcm_thp_encoder_class,
    .init           = thp_encode_init,
    FF_CODEC_ENCODE_CB(thp_encode_frame),
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16P),
};

const FFCodec ff_adpcm_ndsp_le_encoder = {
    .p.name         = "adpcm_ndsp_le",
    CODEC_LONG_NAME("ADPCM Nintendo DSP (little-endian)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_ADPCM_NDSP_LE,
    .p.capabilities = AV_CODEC_CAP_DR1 |
                      AV_CODEC_CAP_SLICE_THREADS |
                      AV_CODEC_CAP_ENCODER_REORDERED_OPAQUE,
    .priv_data_size = sizeof(THPContext),
    .p.priv_class   = &adpcm_thp_encoder_class,
    .init           = thp_encode_init,
    FF_CODEC_ENCODE_CB(thp_encode_frame),
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16P),
};
