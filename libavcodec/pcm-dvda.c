/*
 * LPCM codec for PCM formats found in DVD-Audio streams
 * Copyright (c) 2026 Kacper Michajłow
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * LPCM codec for PCM formats found in DVD-Audio (AOB) streams.
 *
 * Samples are grouped in sets of 2 samples over all channels. Channels are
 * split in up to two channel groups, which may use different quantization.
 * Within each set the second channel group's data comes first. A group's
 * data consists of big-endian 16-bit most significant sample parts, followed
 * by the remaining 4 or 8 bits of all samples for 20 or 24-bit quantization.
 *
 * Packets begin with the LPCM private stream header (following the substream
 * ID), which carries the audio format and the header length.
 */

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"

#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"

#define HEADER_SIZE 11

typedef struct PCMDVDAContext {
    uint32_t last_header;    // Cached header to avoid reparsing
    int block_size;          // Size of a set of 2 samples over all channels
    int channels;
    int group_channels[2];   // Channels in group 1 / group 2 (0 = unused)
    int group_bits[2];       // Quantization of group 1 / group 2
    uint8_t group_map[2][6]; // Stream channel -> native layout position
} PCMDVDAContext;

#define CH_F   AV_CH_FRONT_CENTER
#define CH_L   AV_CH_FRONT_LEFT
#define CH_R   AV_CH_FRONT_RIGHT
#define CH_LF  AV_CH_LOW_FREQUENCY
#define CH_S   AV_CH_BACK_CENTER
#define CH_LS  AV_CH_BACK_LEFT
#define CH_RS  AV_CH_BACK_RIGHT

static const struct {
    uint8_t group1_channels;
    uint8_t group2_channels;
    uint64_t channels[6];   // in stream order, group 1 first
} channel_assignments[21] = {
    [ 0] = { 1, 0, { CH_F } },
    [ 1] = { 2, 0, { CH_L, CH_R } },
    [ 2] = { 2, 1, { CH_L, CH_R, CH_S } },
    [ 3] = { 2, 2, { CH_L, CH_R, CH_LS, CH_RS } },
    [ 4] = { 2, 1, { CH_L, CH_R, CH_LF } },
    [ 5] = { 2, 2, { CH_L, CH_R, CH_LF, CH_S } },
    [ 6] = { 2, 3, { CH_L, CH_R, CH_LF, CH_LS, CH_RS } },
    [ 7] = { 2, 1, { CH_L, CH_R, CH_F } },
    [ 8] = { 2, 2, { CH_L, CH_R, CH_F, CH_S } },
    [ 9] = { 2, 3, { CH_L, CH_R, CH_F, CH_LS, CH_RS } },
    [10] = { 2, 2, { CH_L, CH_R, CH_F, CH_LF } },
    [11] = { 2, 3, { CH_L, CH_R, CH_F, CH_LF, CH_S } },
    [12] = { 2, 4, { CH_L, CH_R, CH_F, CH_LF, CH_LS, CH_RS } },
    [13] = { 3, 1, { CH_L, CH_R, CH_F, CH_S } },
    [14] = { 3, 2, { CH_L, CH_R, CH_F, CH_LS, CH_RS } },
    [15] = { 3, 1, { CH_L, CH_R, CH_F, CH_LF } },
    [16] = { 3, 2, { CH_L, CH_R, CH_F, CH_LF, CH_S } },
    [17] = { 3, 3, { CH_L, CH_R, CH_F, CH_LF, CH_LS, CH_RS } },
    [18] = { 4, 1, { CH_L, CH_R, CH_LS, CH_RS, CH_LF } },
    [19] = { 4, 1, { CH_L, CH_R, CH_LS, CH_RS, CH_F } },
    [20] = { 4, 2, { CH_L, CH_R, CH_LS, CH_RS, CH_F, CH_LF } },
};

static av_cold int pcm_dvda_decode_init(AVCodecContext *avctx)
{
    PCMDVDAContext *s = avctx->priv_data;

    /* Invalid header to force parsing of the first header */
    s->last_header = -1;

    return 0;
}

static int pcm_dvda_parse_header(AVCodecContext *avctx, const uint8_t *header)
{
    PCMDVDAContext *s = avctx->priv_data;
    /*
     * header[0]  continuity counter
     * header[1]  header size (2 bytes, size following this field)
     * header[3]  byte pointer to the start of the first audio frame (2 bytes)
     * header[5]  unknown, observed 0x10 for stereo and 0x00 for surround
     * header[6]  quantization, group 1 (4) / group 2 (4)
     * header[7]  sample rate, group 1 (4) / group 2 (4)
     * header[8]  unknown
     * header[9]  channel group assignment
     * header[10] unknown
     */
    uint32_t header_int = header[6] | header[7] << 8 | header[9] << 16;
    int assignment = header[9];
    int bits[2], rate[2];
    uint64_t mask = 0;

    /* early exit if the header didn't change */
    if (s->last_header == header_int)
        return 0;
    s->last_header = -1;

    if (avctx->debug & FF_DEBUG_PICT_INFO)
        av_log(avctx, AV_LOG_DEBUG, "pcm_dvda_parse_header: header = %02x%02x%02x\n",
               header[6], header[7], header[9]);

    if (assignment > 20) {
        av_log(avctx, AV_LOG_ERROR, "invalid channel group assignment %d\n",
               assignment);
        return AVERROR_INVALIDDATA;
    }

    s->group_channels[0] = channel_assignments[assignment].group1_channels;
    s->group_channels[1] = channel_assignments[assignment].group2_channels;

    for (int i = 0; i < 2; i++) {
        int quant = i ? header[6] & 0xf : header[6] >> 4;
        int freq  = i ? header[7] & 0xf : header[7] >> 4;

        /* 0xf marks an absent channel group */
        if (i && (quant == 0xf || freq == 0xf))
            s->group_channels[1] = 0;
        if (!s->group_channels[i]) {
            bits[i] = rate[i] = 0;
            continue;
        }

        if (quant > 2 || (freq & 7) > 2) {
            av_log(avctx, AV_LOG_ERROR,
                   "invalid group %d quantization %#x or sample rate %#x\n",
                   i + 1, quant, freq);
            return AVERROR_INVALIDDATA;
        }
        bits[i] = 16 + 4 * quant;
        rate[i] = (freq & 8 ? 44100 : 48000) << (freq & 7);

        if (bits[i] == 20) {
            avpriv_request_sample(avctx, "20-bit group %d quantization", i + 1);
            return AVERROR_PATCHWELCOME;
        }
    }

    if (s->group_channels[1] && rate[1] != rate[0]) {
        avpriv_request_sample(avctx, "Mixed group sample rates (%d, %d)",
                              rate[0], rate[1]);
        return AVERROR_PATCHWELCOME;
    }

    s->group_bits[0] = bits[0];
    s->group_bits[1] = bits[1];
    s->channels      = s->group_channels[0] + s->group_channels[1];
    s->block_size    = 2 * (s->group_channels[0] * bits[0] +
                            s->group_channels[1] * bits[1]) / 8;

    /* map the stream channel order to the native layout order */
    for (int i = 0; i < s->channels; i++)
        mask |= channel_assignments[assignment].channels[i];
    for (int i = 0; i < s->channels; i++) {
        uint64_t ch = channel_assignments[assignment].channels[i];
        int pos = av_popcount64(mask & (ch - 1));
        if (i < s->group_channels[0])
            s->group_map[0][i] = pos;
        else
            s->group_map[1][i - s->group_channels[0]] = pos;
    }

    avctx->sample_fmt = FFMAX(bits[0], bits[1]) == 16 ? AV_SAMPLE_FMT_S16
                                                      : AV_SAMPLE_FMT_S32;
    avctx->bits_per_raw_sample = FFMAX(bits[0], bits[1]);
    avctx->sample_rate = rate[0];
    av_channel_layout_uninit(&avctx->ch_layout);
    av_channel_layout_from_mask(&avctx->ch_layout, mask);
    avctx->bit_rate = (int64_t)s->block_size * rate[0] * 8 / 2;

    if (avctx->debug & FF_DEBUG_PICT_INFO)
        ff_dlog(avctx,
                "pcm_dvda_parse_header: %d channels, %d+%d bits per sample, "
                "%d Hz, %"PRId64" bit/s\n",
                s->channels, bits[0], bits[1], avctx->sample_rate,
                avctx->bit_rate);

    s->last_header = header_int;

    return 0;
}

static void pcm_dvda_decode_samples(AVCodecContext *avctx, GetByteContext *gb,
                                    void *dst, int blocks)
{
    PCMDVDAContext *s = avctx->priv_data;
    int16_t *dst16    = dst;
    int32_t *dst32    = dst;

    while (blocks--) {
        /* the second channel group's data comes first in each set */
        for (int g = 1; g >= 0; g--) {
            const int ch          = s->group_channels[g];
            const int bits        = s->group_bits[g];
            const uint8_t *map    = s->group_map[g];

            if (!ch)
                continue;

            for (int n = 0; n < 2; n++) {
                for (int j = 0; j < ch; j++) {
                    unsigned v = bytestream2_get_be16u(gb);
                    if (avctx->sample_fmt == AV_SAMPLE_FMT_S16)
                        dst16[n * s->channels + map[j]] = v;
                    else
                        dst32[n * s->channels + map[j]] = v << 16;
                }
            }
            if (bits == 24) {
                for (int n = 0; n < 2; n++)
                    for (int j = 0; j < ch; j++)
                        dst32[n * s->channels + map[j]] |=
                            bytestream2_get_byteu(gb) << 8;
            }
        }
        dst16 += 2 * s->channels;
        dst32 += 2 * s->channels;
    }
}

static int pcm_dvda_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                                 int *got_frame_ptr, AVPacket *avpkt)
{
    PCMDVDAContext *s  = avctx->priv_data;
    int buf_size       = avpkt->size;
    GetByteContext gb;
    int header_size;
    int retval;
    int blocks;

    if (buf_size < HEADER_SIZE) {
        av_log(avctx, AV_LOG_ERROR, "PCM packet too small\n");
        return AVERROR_INVALIDDATA;
    }

    /* skip the header including its padding */
    header_size = 3 + AV_RB16(avpkt->data + 1);
    if (header_size < HEADER_SIZE || header_size > buf_size) {
        av_log(avctx, AV_LOG_ERROR, "invalid PCM header size %d\n",
               header_size);
        return AVERROR_INVALIDDATA;
    }

    if ((retval = pcm_dvda_parse_header(avctx, avpkt->data)))
        return retval;

    buf_size -= header_size;
    blocks    = buf_size / s->block_size;
    if (buf_size % s->block_size)
        av_log(avctx, AV_LOG_DEBUG, "ignoring %d leftover bytes\n",
               buf_size % s->block_size);
    if (!blocks) {
        *got_frame_ptr = 0;
        return avpkt->size;
    }

    /* get output buffer */
    frame->nb_samples = blocks * 2;
    if ((retval = ff_get_buffer(avctx, frame, 0)) < 0)
        return retval;

    bytestream2_init(&gb, avpkt->data + header_size, blocks * s->block_size);
    pcm_dvda_decode_samples(avctx, &gb, frame->data[0], blocks);

    *got_frame_ptr = 1;

    return avpkt->size;
}

const FFCodec ff_pcm_dvda_decoder = {
    .p.name         = "pcm_dvda",
    CODEC_LONG_NAME("PCM signed 16|20|24-bit big-endian for DVD-Audio media"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_PCM_DVDA,
    .priv_data_size = sizeof(PCMDVDAContext),
    .init           = pcm_dvda_decode_init,
    FF_CODEC_DECODE_CB(pcm_dvda_decode_frame),
    .p.capabilities = AV_CODEC_CAP_CHANNEL_CONF |
                      AV_CODEC_CAP_DR1,
};
