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

#include <stdint.h>

#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"
#include "mathops.h"

static const int16_t step_tab[13][16] =
{
    { -14, -10, -7, -5, -3, -2, -1, 0, 0, 1, 2, 3, 5, 7, 10, 14 },
    { -28, -20, -14, -10, -7, -5, -3, -1, 1, 3, 5, 7, 10, 14, 20, 28 },
    { -56, -40, -28, -20, -14, -10, -6, -2, 2, 6, 10, 14, 20, 28, 40, 56 },
    { -112, -80, -56, -40, -28, -20, -12, -4, 4, 12, 20, 28, 40, 56, 80, 112 },
    { -224, -160, -112, -80, -56, -40, -24, -8, 8, 24, 40, 56, 80, 112, 160, 224 },
    { -448, -320, -224, -160, -112, -80, -48, -16, 16, 48, 80, 112, 160, 224, 320, 448 },
    { -896, -640, -448, -320, -224, -160, -96, -32, 32, 96, 160, 224, 320, 448, 640, 896 },
    { -1792, -1280, -896, -640, -448, -320, -192, -64, 64, 192, 320, 448, 640, 896, 1280, 1792 },
    { -3584, -2560, -1792, -1280, -896, -640, -384, -128, 128, 384, 640, 896, 1280, 1792, 2560, 3584 },
    { -7168, -5120, -3584, -2560, -1792, -1280, -768, -256, 256, 768, 1280, 1792, 2560, 3584, 5120, 7168 },
    { -14336, -10240, -7168, -5120, -3584, -2560, -1536, -512, 512, 1536, 2560, 3584, 5120, 7168, 10240, 14336 },
    { -28672, -20480, -14336, -10240, -7168, -5120, -3072, -1024, 1024, 3072, 5120, 7168, 10240, 14336, 20480, 28672 },
    /* rest is 0s (uses up to index 12) */
};

static const uint8_t index_tab[13][16] =
{
    { 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2 },
    { 3, 3, 2, 2, 1, 1, 1, 0, 0, 1, 1, 1, 2, 2, 3, 3 },
    { 4, 4, 3, 3, 2, 2, 2, 1, 1, 2, 2, 2, 3, 3, 4, 4 },
    { 5, 5, 4, 4, 3, 3, 3, 2, 2, 3, 3, 3, 4, 4, 5, 5 },
    { 6, 6, 5, 5, 4, 4, 4, 3, 3, 4, 4, 4, 5, 5, 6, 6 },
    { 7, 7, 6, 6, 5, 5, 5, 4, 4, 5, 5, 5, 6, 6, 7, 7 },
    { 8, 8, 7, 7, 6, 6, 6, 5, 5, 6, 6, 6, 7, 7, 8, 8 },
    { 9, 9, 8, 8, 7, 7, 7, 6, 6, 7, 7, 7, 8, 8, 9, 9 },
    {10,10, 9, 9, 8, 8, 8, 7, 7, 8, 8, 8, 9, 9,10,10 },
    {11,11,10,10, 9, 9, 9, 8, 8, 9, 9, 9,10,10,11,11 },
    {11,11,11,11,10,10,10, 9, 9,10,10,10,11,11,11,11 },
    {11,11,11,11,11,11,11,10,10,11,11,11,11,11,11,11 },
    /* rest is 0s (uses up to index 12) */
};

static av_cold int decode_init(AVCodecContext *avctx)
{
    avctx->sample_fmt = AV_SAMPLE_FMT_S16P;

    if (avctx->ch_layout.nb_channels <= 0)
        return AVERROR_INVALIDDATA;

    if ((avctx->block_align / avctx->ch_layout.nb_channels) < 6)
        return AVERROR_INVALIDDATA;

    return 0;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame_ptr, AVPacket *avpkt)
{
    const int nb_channels = avctx->ch_layout.nb_channels;
    const int blocks = avpkt->size / avctx->block_align;
    const int block_samples = 2 + (avctx->block_align/nb_channels - 5) * 2;
    GetByteContext gbc, *gb = &gbc;
    int ret;

    if (avpkt->size <= 5)
        return AVERROR_INVALIDDATA;

    bytestream2_init(gb, avpkt->data, avpkt->size);

    frame->nb_samples = blocks * block_samples;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    for (int b = 0; b < blocks; b++) {
        for (int ch = 0; ch < nb_channels; ch++) {
            int16_t *samples = ((int16_t *)frame->extended_data[ch]) + b * block_samples;
            int hist1, hist2, index;
            uint8_t nibbles;

            hist2 = sign_extend(bytestream2_get_le16(gb), 16);
            hist1 = sign_extend(bytestream2_get_le16(gb), 16);
            index = bytestream2_get_byte(gb);

            index = FFMIN(index, 12);

            samples[0] = hist2;
            samples[1] = hist1;

            for (int i = 2; i < block_samples; i++) {
                int sample, nibble, step;

                if (!(i & 1)) {
                    nibbles = bytestream2_get_byte(gb);
                    nibble = nibbles & 0xF;
                } else {
                    nibble = nibbles >> 4;
                }

                step  = step_tab[index][nibble];
                index = index_tab[index][nibble];
                sample = av_clip_int16(step + 2 * hist1 - hist2);

                samples[i] = sample;

                hist2 = hist1;
                hist1 = sample;
            }
        }
    }

    *got_frame_ptr = 1;

    return avpkt->size;
}

const FFCodec ff_adpcm_pt_decoder = {
    .p.name       = "adpcm_pt",
    CODEC_LONG_NAME("ADPCM Wwise Platinum"),
    .p.type       = AVMEDIA_TYPE_AUDIO,
    .p.id         = AV_CODEC_ID_ADPCM_PT,
    .init         = decode_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
};
