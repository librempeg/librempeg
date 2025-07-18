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

#include "libavutil/fifo.h"
#include "avcodec.h"
#include "bswapdsp.h"
#include "codec_internal.h"
#include "decode.h"

typedef struct PCMHDMVContext {
    AVFifo *fifo;
    int block_size;
    int channels;

    BswapDSPContext bbdsp;
} PCMHDMVContext;

static av_cold int pcm_hdmv_decode_init(AVCodecContext *avctx)
{
    PCMHDMVContext *s = avctx->priv_data;

    ff_bswapdsp_init(&s->bbdsp);

    s->fifo = av_fifo_alloc2(2048, 1, 1);
    if (!s->fifo)
        return AVERROR(ENOMEM);

    return 0;
}

static void pcm_hdmv_parse_header(AVCodecContext *avctx, const uint8_t *header)
{
    PCMHDMVContext *s = avctx->priv_data;

    avctx->sample_fmt = AV_SAMPLE_FMT_S16;
    avctx->bits_per_coded_sample = 16;
    avctx->bits_per_raw_sample = 16;
    avctx->sample_rate = 48000;

    s->channels = 1 + !!(header[0] & 0x20);
    s->block_size = 2 * s->channels;

    av_channel_layout_uninit(&avctx->ch_layout);
    av_channel_layout_default(&avctx->ch_layout, s->channels);

    avctx->bit_rate = s->channels *
                      avctx->sample_rate *
                      avctx->bits_per_coded_sample;
}

static int pcm_hdmv_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                                 int *got_frame_ptr, AVPacket *avpkt)
{
    PCMHDMVContext *s = avctx->priv_data;
    const uint8_t *src = avpkt->data;
    int skip, ret, buf_size = avpkt->size;
    void *dst;

    if (buf_size < 3) {
        av_log(avctx, AV_LOG_ERROR, "PCM packet too small\n");
        return AVERROR_INVALIDDATA;
    }

    pcm_hdmv_parse_header(avctx, src);

    skip      = 3;
    src      += skip;
    buf_size -= skip;
    av_fifo_write(s->fifo, src, buf_size);

    frame->nb_samples = av_fifo_can_read(s->fifo) / s->block_size;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;
    dst = frame->data[0];
    av_fifo_read(s->fifo, dst, s->block_size * frame->nb_samples);
    s->bbdsp.bswap16_buf(dst, dst, (s->block_size * frame->nb_samples) / 2);

    *got_frame_ptr = 1;

    return avpkt->size;
}

static av_cold int pcm_hdmv_decode_close(AVCodecContext *avctx)
{
    PCMHDMVContext *s = avctx->priv_data;

    av_fifo_freep2(&s->fifo);

    return 0;
}

const FFCodec ff_pcm_hdmv_decoder = {
    .p.name         = "pcm_hdmv",
    CODEC_LONG_NAME("PCM signed 16-bit big-endian for HDMV media"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_PCM_HDMV,
    .priv_data_size = sizeof(PCMHDMVContext),
    .init           = pcm_hdmv_decode_init,
    FF_CODEC_DECODE_CB(pcm_hdmv_decode_frame),
    .close          = pcm_hdmv_decode_close,
    .p.capabilities = AV_CODEC_CAP_CHANNEL_CONF |
                      AV_CODEC_CAP_DR1,
    CODEC_SAMPLEFMTS(AV_SAMPLE_FMT_S16),
};
