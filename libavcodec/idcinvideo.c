/*
 * id Quake II CIN Video Decoder
 * Copyright (C) 2003 The FFmpeg project
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
 * id Quake II Cin Video Decoder by Dr. Tim Ferguson
 * For more information about the id CIN format, visit:
 *   http://www.csse.monash.edu.au/~timf/
 *
 * This video decoder outputs PAL8 colorspace data. Interacting with this
 * decoder is a little involved. During initialization, the demuxer must
 * transmit the 65536-byte Huffman table(s) to the decoder via extradata.
 * Then, whenever a palette change is encountered while demuxing the file,
 * the demuxer must use the same extradata space to transmit an
 * AVPaletteControl structure.
 *
 * id CIN video is purely Huffman-coded, intraframe-only codec. It achieves
 * a little more compression by exploiting the fact that adjacent pixels
 * tend to be similar.
 *
 * Note that this decoder could use libavcodec's optimized VLC facilities
 * rather than naive, tree-based Huffman decoding. However, there are 256
 * Huffman tables. Plus, the VLC bit coding order is right -> left instead
 * or left -> right, so all of the bits would have to be reversed. Further,
 * the original Quake II implementation likely used a similar naive
 * decoding algorithm and it worked fine on much lower spec machines.
 */

#include <stddef.h>
#include <string.h>

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "libavutil/internal.h"

#define HUFFMAN_TABLE_SIZE 64 * 1024
#define HUF_TOKENS 256
#define PALETTE_COUNT 256

typedef struct hnode {
  int count;
  unsigned char used;
  int children[2];
} hnode;

typedef struct IdcinContext {

    AVCodecContext *avctx;

    const unsigned char *buf;
    int size;

    hnode huff_nodes[256][HUF_TOKENS*2];
    int num_huff_nodes[256];

    uint32_t pal[256];
} IdcinContext;

/**
 * Find the lowest probability node in a Huffman table, and mark it as
 * being assigned to a higher probability.
 * @return the node index of the lowest unused node, or -1 if all nodes
 * are used.
 */
static int huff_smallest_node(hnode *hnodes, int num_hnodes) {
    int i;
    int best, best_node;

    best = 99999999;
    best_node = -1;
    for(i = 0; i < num_hnodes; i++) {
        if(hnodes[i].used)
            continue;
        if(!hnodes[i].count)
            continue;
        if(hnodes[i].count < best) {
            best = hnodes[i].count;
            best_node = i;
        }
    }

    if(best_node == -1)
        return -1;
    hnodes[best_node].used = 1;
    return best_node;
}

/*
 * Build the Huffman tree using the generated/loaded probabilities histogram.
 *
 * On completion:
 *  huff_nodes[prev][i < HUF_TOKENS] - are the nodes at the base of the tree.
 *  huff_nodes[prev][i >= HUF_TOKENS] - are used to construct the tree.
 *  num_huff_nodes[prev] - contains the index to the root node of the tree.
 *    That is: huff_nodes[prev][num_huff_nodes[prev]] is the root node.
 */
static av_cold void huff_build_tree(IdcinContext *s, int prev) {
    hnode *node, *hnodes;
     int num_hnodes, i;

    num_hnodes = HUF_TOKENS;
    hnodes = s->huff_nodes[prev];
    for(i = 0; i < HUF_TOKENS * 2; i++)
        hnodes[i].used = 0;

    while (1) {
        node = &hnodes[num_hnodes];             /* next free node */

        /* pick two lowest counts */
        node->children[0] = huff_smallest_node(hnodes, num_hnodes);
        if(node->children[0] == -1)
            break;      /* reached the root node */

        node->children[1] = huff_smallest_node(hnodes, num_hnodes);
        if(node->children[1] == -1)
            break;      /* reached the root node */

        /* combine nodes probability for new node */
        node->count = hnodes[node->children[0]].count +
        hnodes[node->children[1]].count;
        num_hnodes++;
    }

    s->num_huff_nodes[prev] = num_hnodes - 1;
}

static av_cold int idcin_decode_init(AVCodecContext *avctx)
{
    IdcinContext *s = avctx->priv_data;
    int i, j, histogram_index = 0;
    unsigned char *histograms;

    s->avctx = avctx;
    avctx->pix_fmt = AV_PIX_FMT_PAL8;

    /* make sure the Huffman tables make it */
    if (s->avctx->extradata_size != HUFFMAN_TABLE_SIZE) {
        av_log(s->avctx, AV_LOG_ERROR, "  id CIN video: expected extradata size of %d\n", HUFFMAN_TABLE_SIZE);
        return -1;
    }

    /* build the 256 Huffman decode trees */
    histograms = (unsigned char *)s->avctx->extradata;
    for (i = 0; i < 256; i++) {
        for(j = 0; j < HUF_TOKENS; j++)
            s->huff_nodes[i][j].count = histograms[histogram_index++];
        huff_build_tree(s, i);
    }

    return 0;
}

static int idcin_decode_vlcs(IdcinContext *s, AVFrame *frame)
{
    hnode *hnodes;
    long x, y;
    int prev;
    unsigned char v = 0;
    int bit_pos, node_num, dat_pos;

    prev = bit_pos = dat_pos = 0;
    for (y = 0; y < (frame->linesize[0] * s->avctx->height);
        y += frame->linesize[0]) {
        for (x = y; x < y + s->avctx->width; x++) {
            node_num = s->num_huff_nodes[prev];
            hnodes = s->huff_nodes[prev];

            while(node_num >= HUF_TOKENS) {
                if(!bit_pos) {
                    if(dat_pos >= s->size) {
                        av_log(s->avctx, AV_LOG_ERROR, "Huffman decode error.\n");
                        return -1;
                    }
                    bit_pos = 8;
                    v = s->buf[dat_pos++];
                }

                node_num = hnodes[node_num].children[v & 0x01];
                v = v >> 1;
                bit_pos--;
            }

            frame->data[0][x] = node_num;
            prev = node_num;
        }
    }

    return 0;
}

static int idcin_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                              int *got_frame, AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    IdcinContext *s = avctx->priv_data;
    int ret;

    s->buf = buf;
    s->size = buf_size;

    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    if (idcin_decode_vlcs(s, frame))
        return AVERROR_INVALIDDATA;

    ff_copy_palette(s->pal, avpkt, avctx);
    /* make the palette available on the way out */
    memcpy(frame->data[1], s->pal, AVPALETTE_SIZE);

    *got_frame = 1;

    /* report that the buffer was completely consumed */
    return buf_size;
}

static const FFCodecDefault idcin_defaults[] = {
    { "max_pixels", "320*240" },
    { NULL },
};

const FFCodec ff_idcin_decoder = {
    .p.name         = "idcinvideo",
    CODEC_LONG_NAME("id Quake II CIN video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_IDCIN,
    .priv_data_size = sizeof(IdcinContext),
    .init           = idcin_decode_init,
    FF_CODEC_DECODE_CB(idcin_decode_frame),
    .p.capabilities = AV_CODEC_CAP_DR1,
    .defaults       = idcin_defaults,
};
