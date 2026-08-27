/*
 * CompressWave decoder
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

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/thread.h"

#define BITSTREAM_READER_LE
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "get_bits.h"

typedef struct CompressWaveContext {
    AVCodecContext *avctx;
    GetBitContext gb;
    uint8_t *bitstream;
    int max_framesize;
    uint64_t max_samples;
    int bitstream_size;
    int bitstream_index;
    int skip;

    int32_t state[2];
    int32_t fstate[2];

    VLC vlc;

    uint32_t freq[256];
    int32_t values[256];
} CompressWaveContext;

typedef struct Node {
    int64_t count;
    int16_t sym;
    int16_t l, r;
} Node;

typedef enum { nsEmpty, nsBranch, nsLeaf, nsRoot } NodeState;

static void get_tree_codes(uint32_t *bits, int16_t *lens, uint8_t *xlat,
                           const uint32_t *counts, const Node *nodes, int i, uint32_t code, int len,
                           int *nb_codes)
{
    if (nodes[i].sym < 0) {
        get_tree_codes(bits, lens, xlat, counts, nodes, nodes[i].l, (code << 1)|0, len+1, nb_codes);
        get_tree_codes(bits, lens, xlat, counts, nodes, nodes[i].r, (code << 1)|1, len+1, nb_codes);
        return;
    }

    if (counts[nodes[i].sym] > 0) {
        xlat[nb_codes[0]] = nodes[i].sym;
        bits[nb_codes[0]] = code;
        lens[nb_codes[0]] = len;
        nb_codes[0]++;
    }
}

static int build_vlc(AVCodecContext *avctx)
{
    CompressWaveContext *s = avctx->priv_data;
    int16_t lens[256] = { 0 };
    Node nodes[512] = { 0 };
    int node[257] = { 0 };
    uint32_t bits[256];
    uint8_t xlat[256];
    int nb_codes = 0;

    ff_vlc_free(&s->vlc);

    for (int i = 0; i < 256; i++) {
        nodes[i].count = s->freq[i];
        nodes[i].sym   = i;
        nodes[i].l     = i;
        nodes[i].r     = i;
        node[i]        = i;
    }

    int cur_node = 256;
    for (int i = 0; i < 256-1; i++) {
        int64_t min_count = INT64_MAX;
        int second_node = -1;
        int first_node = -1;
        int64_t nd, st;

        nodes[cur_node].count = -1;

        for (int k = 0; k < 256; k++) {
            if (node[k] < 0)
                continue;

            if (nodes[node[k]].count < min_count) {
                first_node = k;
                min_count = nodes[node[k]].count;
            }
        }

        min_count = INT64_MAX;
        for (int k = 0; k < 256; k++) {
            if (node[k] < 0)
                continue;

            if (k == first_node)
                continue;

            if (FFABS(nodes[node[k]].count - nodes[node[first_node]].count) < min_count) {
                second_node = k;
                min_count = nodes[node[k]].count;
            }
        }

        if (first_node == -1 || second_node == -1)
            break;

        nd = nodes[node[second_node]].count;
        st = nodes[node[first_node]].count;
        nodes[node[second_node]].count = -1;
        nodes[node[first_node]].count  = -1;
        if (nd >= INT64_MAX - st) {
            av_log(avctx, AV_LOG_ERROR, "count overflow\n");
            return AVERROR_INVALIDDATA;
        }
        nodes[cur_node].count = nd + st;
        nodes[cur_node].sym = -1;
        nodes[cur_node].l = node[first_node];
        nodes[cur_node].r = node[second_node];
        node[first_node] = -1;
        node[second_node] = -1;
        node[first_node] = cur_node;
        cur_node++;
    }

    get_tree_codes(bits, lens, xlat, s->freq, nodes, cur_node-1, 0, 0, &nb_codes);

    return ff_vlc_init_sparse(&s->vlc, 10, nb_codes, lens, 2, 2,
                              bits, 4, 4, xlat, 1, 1, VLC_INIT_OUTPUT_LE);
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    CompressWaveContext *s = avctx->priv_data;

    s->avctx = avctx;
    if (!avctx->extradata || avctx->extradata_size < 0x940)
        return AVERROR_INVALIDDATA;

    avctx->ch_layout.nb_channels = AV_RL32(avctx->extradata);
    if (avctx->ch_layout.nb_channels < 1 ||
        avctx->ch_layout.nb_channels > 2)
        return AVERROR_INVALIDDATA;

    avctx->sample_rate = AV_RL32(avctx->extradata + 4);
    if (avctx->sample_rate <= 0)
        return AVERROR_INVALIDDATA;

    s->max_samples = AV_RL64(avctx->extradata + 0x410) / 4;
    if (s->max_samples == 0)
        s->max_samples = UINT64_MAX;
    s->max_framesize = 1024;

    s->bitstream = av_calloc(s->max_framesize + AV_INPUT_BUFFER_PADDING_SIZE / sizeof(*s->bitstream) + 1, sizeof(*s->bitstream));
    if (!s->bitstream)
        return AVERROR(ENOMEM);

    avctx->sample_fmt = AV_SAMPLE_FMT_S16;

    for (int i = 0; i < FF_ARRAY_ELEMS(s->values); i++)
        s->values[i] = AV_RL32(avctx->extradata + i * 4 + 12);

    if (avctx->extradata[0x530] != 3)
        return AVERROR_INVALIDDATA;

    for (int i = 0; i < FF_ARRAY_ELEMS(s->freq); i++)
        s->freq[i] = AV_RL32(avctx->extradata + i * 4 + 0x538);

    return build_vlc(avctx);
}

static int decode_block(AVCodecContext *avctx, AVFrame *frame, int eof)
{
    const int nb_channels = avctx->ch_layout.nb_channels;
    int16_t *samples = (int16_t *)frame->data[0];
    CompressWaveContext *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    const int32_t *values = s->values;
    int32_t *fstate = s->fstate;
    int32_t *state = s->state;
    const int left = eof ? 0 : 32;

    for (int n = 0, i = 0; n < frame->nb_samples; n++) {
        if (get_bits_left(gb) <= left) {
            frame->nb_samples = n;
            break;
        }

        for (int ch = 0; ch < nb_channels; ch++, i++) {
            int v = get_vlc2(gb, s->vlc.table, 10, 3);
            if (v < 0)
                return AVERROR_INVALIDDATA;

            state[ch] += values[v];
            fstate[ch] += state[ch];

            if (fstate[ch] > 32760) {
                fstate[ch] = 32760;
                state[ch] = 0;
            }

            if (fstate[ch] < -32760) {
                fstate[ch] = -32760;
                state[ch] = 0;
            }

            samples[i] = fstate[ch] * 255 / 256;
        }
    }

    return 0;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame_ptr, AVPacket *pkt)
{
    CompressWaveContext *s = avctx->priv_data;
    int ret, n, buf_size, input_buf_size;
    GetBitContext *gb = &s->gb;
    const uint8_t *buf;

    if (!pkt->size && !s->bitstream_size) {
        *got_frame_ptr = 0;
        return 0;
    }

    buf_size = FFMIN(pkt->size, s->max_framesize - s->bitstream_size);
    input_buf_size = buf_size;
    if (s->bitstream_index + s->bitstream_size + buf_size > s->max_framesize) {
        memmove(s->bitstream, &s->bitstream[s->bitstream_index], s->bitstream_size);
        s->bitstream_index = 0;
    }
    if (pkt->data)
        memcpy(&s->bitstream[s->bitstream_index + s->bitstream_size], pkt->data, buf_size);
    buf = &s->bitstream[s->bitstream_index];
    buf_size += s->bitstream_size;
    s->bitstream_size = buf_size;
    if (buf_size < s->max_framesize && pkt->data) {
        *got_frame_ptr = 0;
        return input_buf_size;
    }

    if ((ret = init_get_bits8(gb, buf, buf_size)) < 0)
        return ret;

    frame->nb_samples = FFMIN(8192, s->max_samples);
    if (frame->nb_samples <= 0) {
        *got_frame_ptr = 0;
        return pkt->size;
    }

    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    skip_bits(gb, s->skip);
    ret = decode_block(avctx, frame, !pkt->data);
    if (ret < 0)
        return ret;

    if (frame->nb_samples <= 0) {
        *got_frame_ptr = 0;
        return pkt->size;
    }

    s->max_samples -= frame->nb_samples;

    *got_frame_ptr = 1;
    s->skip = get_bits_count(gb) - 8 * (get_bits_count(gb) / 8);
    n = get_bits_count(gb) / 8;

    if (n > buf_size && pkt->data) {
        s->bitstream_size = 0;
        s->bitstream_index = 0;
        return AVERROR_INVALIDDATA;
    }

    if (s->bitstream_size > 0) {
        s->bitstream_index += n;
        s->bitstream_size  -= FFMIN(s->bitstream_size, n);
        return input_buf_size;
    }
    return n;
}

static void decode_flush(AVCodecContext *avctx)
{
    CompressWaveContext *s = avctx->priv_data;

    memset(s->state, 0, sizeof(s->state));
    memset(s->fstate, 0, sizeof(s->fstate));

    s->bitstream_size = 0;
    s->bitstream_index = 0;
}

static av_cold int decode_close(AVCodecContext *avctx)
{
    CompressWaveContext *s = avctx->priv_data;

    ff_vlc_free(&s->vlc);
    av_freep(&s->bitstream);
    s->bitstream_size = 0;

    return 0;
}

const FFCodec ff_compresswave_decoder = {
    .p.name         = "compresswave",
    CODEC_LONG_NAME("CompressWave"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_COMPRESSWAVE,
    .init           = decode_init,
    .flush          = decode_flush,
    .close          = decode_close,
    FF_CODEC_DECODE_CB(decode_frame),
    .p.capabilities = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP,
    .priv_data_size = sizeof(CompressWaveContext),
};
