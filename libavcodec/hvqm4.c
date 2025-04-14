/*
 * HVQM4 Video decoder
 * Copyright (c) 2018-2020 Tillmann Karras
 * Copyright (c) 2022 Paul B Mahol
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

#include <inttypes.h>

#include "libavutil/mem.h"
#include "libavutil/thread.h"
#include "libavutil/qsort.h"

#include "avcodec.h"
#include "bytestream.h"
#include "codec_internal.h"
#include "decode.h"

#define CACHED_BITSTREAM_READER !ARCH_X86_32

#include "get_bits.h"
#include "internal.h"

#define PLANE_COUNT 3
#define LUMA_CHROMA 2
#define LUMA_IDX 0
#define CHROMA_IDX 1

enum FrameType {
    I_FRAME = 0x10,
    P_FRAME = 0x20,
    B_FRAME = 0x30,
};

typedef struct BlockData {
    uint8_t value;
    uint8_t type;
} BlockData;

typedef struct StackState {
    uint32_t plane_idx;
    BlockData const *line_prev;
    BlockData const *line_curr;
    BlockData const *line_next;
    BlockData next;
    BlockData curr;
    uint8_t value_prev;
} StackState;

typedef struct GBCWithVLC {
    GetBitContext gb;
    VLC *vlc;
    uint32_t value;
} GBCWithVLC;

typedef struct HVQPlaneDesc {
    BlockData *border; // 0-3 beginning of the plane including the border
    BlockData *payload; // 4-7 beginning of the non-border plane data
    uint16_t h_blocks;
    uint16_t v_blocks;
    uint16_t h_blocks_safe;
    uint16_t v_blocks_safe;
    // offsets of PBs within one MCB
    // +---+---+
    // | 0 | 3 |
    // +---+---+
    // | 1 | 2 |
    // +---+---+
    uint16_t mcb_offset[4];
    uint32_t px_offset[4];
    uint32_t py_offset[4];
    uint8_t width_shift;
    uint8_t height_shift;
    uint8_t pb_per_mcb_x;
    uint8_t pb_per_mcb_y;
    uint8_t blocks_per_mcb;
} HVQPlaneDesc;

typedef void (*motion_comp_func)(uint8_t *dst, ptrdiff_t dst_linesize,
                                 const uint8_t *src, ptrdiff_t src_linesize);

static void motion_comp_00(uint8_t *dst, ptrdiff_t dst_stride, uint8_t const *src, ptrdiff_t src_stride)
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            dst[j] = src[j];
        dst += dst_stride;
        src += src_stride;
    }
}

// offset vertically by half a sample
static void motion_comp_01(uint8_t *dst, ptrdiff_t dst_stride, uint8_t const *src, ptrdiff_t src_stride)
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            dst[j] = (src[j] +
                      src[src_stride + j] + 1) / 2;
        dst += dst_stride;
        src += src_stride;
    }
}

// offset horizontally by half a sample
static void motion_comp_10(uint8_t *dst, ptrdiff_t dst_stride, uint8_t const *src, ptrdiff_t src_stride)
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            dst[j] = (src[j + 0] +
                      src[j + 1] + 1) / 2;
        dst += dst_stride;
        src += src_stride;
    }
}

// offset by half a sample in both directions
static void motion_comp_11(uint8_t *dst, ptrdiff_t dst_stride, uint8_t const *src, ptrdiff_t src_stride)
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            dst[j] = (src[j + 0] +
                      src[j + 1] +
                      src[src_stride + j + 0] +
                      src[src_stride + j + 1] + 2) >> 2;
        dst += dst_stride;
        src += src_stride;
    }
}

typedef struct VideoState {
    HVQPlaneDesc planes[PLANE_COUNT];
    VLC vlc[6];
    GBCWithVLC dc_values[PLANE_COUNT]; // DC values
    GBCWithVLC dc_rle[PLANE_COUNT]; // DC run lengths
    GBCWithVLC bufTree0[PLANE_COUNT];
    GBCWithVLC basis_num[LUMA_CHROMA];
    GBCWithVLC basis_num_run[LUMA_CHROMA];
    GetBitContext fixvl[PLANE_COUNT]; // uncompressed high-entropy data
    GBCWithVLC mv_h; // horizontal motion vectors
    GBCWithVLC mv_v; // vertical motion vectors
    GBCWithVLC mcb_proc; // macroblock proc
    GBCWithVLC mcb_type; // macroblock type
    int h_nest_size;
    int v_nest_size;
    int is_landscape; // FIXME: check what happens for square video
    uint8_t nest_data[70 * 38];
    int dc_max;
    int dc_min;
    int unk_shift;
    int dc_shift;
    int version;
    // number of residual bits to read from mv_h/mv_v,
    // one setting for each of past and future
    int mc_residual_bits_h[3];
    int mc_residual_bits_v[3];

    motion_comp_func motion_comp_func[2][2];
} VideoState;

typedef struct SeqObj {
    VideoState *state;
    uint16_t width;
    uint16_t height;
    uint8_t h_samp;
    uint8_t v_samp;
} SeqObj;

typedef struct MCPlane {
    uint32_t rle;
    uint32_t pb_dc;
    BlockData *payload_cur_blk;
    BlockData *payload_cur_row;
    uint8_t *present;
    ptrdiff_t present_stride;
    uint8_t *top;
    ptrdiff_t top_stride;
    uint8_t *target;
    ptrdiff_t target_stride;
    uint8_t *past;
    ptrdiff_t past_stride;
    uint8_t *future;
    ptrdiff_t future_stride;
    ptrdiff_t h_mcb_stride;
    ptrdiff_t v_mcb_stride;
    uint32_t pb_per_mcb_x;
    ptrdiff_t stride;
} MCPlane;

struct RLDecoder {
    uint32_t value;
    uint32_t count;
};

typedef struct OrderFrame {
    uint32_t disp_id;
    AVFrame *frame;
} OrderFrame;

typedef struct HVQM4Context {
    int version;
    int eof;
    AVPacket *pkt;
    AVFrame *frame[3];

    int last_frame_type;
    int64_t last_i_pts;
    unsigned queued_frames;
    unsigned flushed_frames;
    OrderFrame *frames;

    SeqObj seqobj;
    VideoState state;
    uint8_t *buffer;

    GetBitContext gb;
} HVQM4Context;

static int32_t div_tab[16];
static int32_t mcdiv_tab[512];

static av_cold void hvqm4_init_static(void)
{
    div_tab[0] = 0;
    mcdiv_tab[0] = 0;

    for (int i = 1; i < 16; i++)
        div_tab[i] = 0x1000 / (i * 16) * 16;
    for (int i = 1; i < 512; i++)
        mcdiv_tab[i] = 0x1000 / i;
}

static void set_plane_desc(SeqObj *seqobj, uint8_t plane_idx, uint8_t h_samp, uint8_t v_samp)
{
    HVQPlaneDesc *plane = &seqobj->state->planes[plane_idx];

    plane->width_shift = h_samp == 2 ? 1 : 0;
    plane->height_shift = v_samp == 2 ? 1 : 0;
    // pixels per 2x2 block
    plane->pb_per_mcb_x = 2 >> plane->width_shift; // 1..2
    plane->pb_per_mcb_y = 2 >> plane->height_shift; // 1..2
    plane->blocks_per_mcb = plane->pb_per_mcb_x * plane->pb_per_mcb_y; // 1..4
    // number of 4x4 blocks
    plane->h_blocks = seqobj->width / (h_samp * 4);
    plane->v_blocks = seqobj->height / (v_samp * 4);
    // number of 4x4 blocks + border
    plane->h_blocks_safe = plane->h_blocks + 2;
    plane->v_blocks_safe = plane->v_blocks + 2;
    // offset of blocks in MCB
    plane->mcb_offset[0] = 0;
    plane->mcb_offset[1] = plane->h_blocks_safe;
    plane->mcb_offset[2] = plane->h_blocks_safe + 1;
    plane->mcb_offset[3] = 1;
    plane->px_offset[0] = 0;
    plane->px_offset[1] = 0;
    plane->px_offset[2] = 4;
    plane->px_offset[3] = 4;
    plane->py_offset[0] = 0;
    plane->py_offset[1] = 4;
    plane->py_offset[2] = 4;
    plane->py_offset[3] = 0;
}

static void set_border(BlockData *dst)
{
    dst->value = 127;
    dst->type = 255;
}

static void set_buffer(SeqObj *seqobj, void *workbuff, uint8_t *buffer, int version)
{
    VideoState *state = workbuff;
    BlockData *plane_data;

    seqobj->state = state;
    state->version = version;
    set_plane_desc(seqobj, 0, 1, 1);
    set_plane_desc(seqobj, 1, 2, 2);
    set_plane_desc(seqobj, 2, 2, 2);

    state->is_landscape = seqobj->width >= seqobj->height;
    if (state->is_landscape) {
        state->h_nest_size = 70;
        state->v_nest_size = 38;
    } else {
        state->h_nest_size = 38;
        state->v_nest_size = 70;
    }

    state->basis_num[0].vlc = &state->vlc[3];
    state->basis_num[1].vlc = &state->vlc[3];

    state->basis_num_run[0].vlc = &state->vlc[1];
    state->basis_num_run[1].vlc = &state->vlc[1];

    state->dc_values[0].vlc = &state->vlc[0];
    state->dc_values[1].vlc = &state->vlc[0];
    state->dc_values[2].vlc = &state->vlc[0];

    state->dc_rle[0].vlc = &state->vlc[1]; // reuse!
    state->dc_rle[1].vlc = &state->vlc[1]; //
    state->dc_rle[2].vlc = &state->vlc[1]; //

    state->bufTree0[0].vlc = &state->vlc[2];
    state->bufTree0[1].vlc = &state->vlc[2];
    state->bufTree0[2].vlc = &state->vlc[2];

    state->mv_h.vlc = &state->vlc[4];
    state->mv_v.vlc = &state->vlc[4];

    state->mcb_proc.vlc = &state->vlc[5];
    state->mcb_type.vlc = &state->vlc[5];

    state->motion_comp_func[0][0] = motion_comp_00;
    state->motion_comp_func[0][1] = motion_comp_01;
    state->motion_comp_func[1][0] = motion_comp_10;
    state->motion_comp_func[1][1] = motion_comp_11;

    plane_data = (BlockData *)buffer;
    for (int i = 0; i < PLANE_COUNT; i++) {
        HVQPlaneDesc *plane = &state->planes[i];
        ptrdiff_t stride = plane->h_blocks_safe;
        BlockData *ptr;

        plane->border = plane_data;
        // skip top border (stride) and left border (1)
        plane->payload = plane_data + stride + 1;
        plane_data += plane->h_blocks_safe * plane->v_blocks_safe;

        // set horizontal borders
        ptr = plane->border;
        for (int i = plane->h_blocks_safe; i > 0; i--) {
            set_border(ptr);
            ptr++;
        }

        ptr = plane_data;
        for (int i = plane->h_blocks_safe; i > 0; i--) {
            ptr--;
            set_border(ptr);
        }

        // set vertical borders
        ptr = plane->border + stride;
        for (int i = plane->v_blocks_safe - 2; i > 0; i--) {
            set_border(ptr);
            ptr += stride;
        }

        ptr = plane->border + stride * 2 - 1;
        for (int i = plane->v_blocks_safe - 2; i > 0; i--) {
            set_border(ptr);
            ptr += stride;
        }
    }
}

static uint32_t hvqm4_buffsize(SeqObj *seqobj)
{
    uint32_t h_blocks = seqobj->width / 4;
    uint32_t v_blocks = seqobj->height / 4;
    uint32_t y_blocks = (h_blocks + 2) * (v_blocks + 2);

    uint32_t uv_h_blocks = seqobj->h_samp == 2 ? h_blocks / 2 : h_blocks;
    uint32_t uv_v_blocks = seqobj->v_samp == 2 ? v_blocks / 2 : v_blocks;
    uint32_t uv_blocks = (uv_h_blocks + 2) * (uv_v_blocks + 2);

    uint32_t total = (y_blocks + uv_blocks * 2) * sizeof(uint16_t);

    return total;
}

static av_cold int hvqm4_init(AVCodecContext *avctx)
{
    static AVOnce init_static_once = AV_ONCE_INIT;
    HVQM4Context *s = avctx->priv_data;

    s->pkt = avctx->internal->in_pkt;
    avctx->pix_fmt = AV_PIX_FMT_YUV420P;

    avctx->coded_width  = FFALIGN(avctx->width , 8) + 32;
    avctx->coded_height = FFALIGN(avctx->height, 8) + 32;

    for (int i = 0; i < 3; i++) {
        s->frame[i] = av_frame_alloc();
        if (!s->frame[i])
            return AVERROR(ENOMEM);
    }

    ff_thread_once(&init_static_once, hvqm4_init_static);

    s->seqobj.width = avctx->width;
    s->seqobj.height = avctx->height;
    s->seqobj.h_samp = 2;
    s->seqobj.v_samp = 2;
    if (avctx->extradata_size >= 1)
        s->version = (avctx->extradata[0] - 48) == 5;

    s->buffer = av_calloc(hvqm4_buffsize(&s->seqobj), 1);
    if (!s->buffer)
        return AVERROR(ENOMEM);

    set_buffer(&s->seqobj, &s->state, s->buffer, s->version);

    return 0;
}

typedef struct HuffTree {
    int nb_codes;
    int max_bits;
    uint8_t bits[256];
    uint32_t codes[256];
    int16_t symbols[256];
} HuffTree;

static int read_trees(uint32_t length,
                      uint16_t code,
                      HuffTree *huff,
                      GetBitContext *gb,
                      const uint32_t tree_signed,
                      const uint32_t tree_scale)
{
    int ret;

    if (get_bits_left(gb) < 1 ||
        get_bits1(gb) == 0) { // leaf node
        uint8_t byte = get_bits(gb, 8);
        int16_t symbol = (tree_signed && byte > 0x7f) ? (int8_t)byte : byte;

        symbol *= 1 << tree_scale;
        if (huff->nb_codes >= FF_ARRAY_ELEMS(huff->bits))
            return AVERROR_INVALIDDATA;
        huff->bits[huff->nb_codes] = FFMAX(length, 1);
        huff->codes[huff->nb_codes] = code;
        huff->symbols[huff->nb_codes] = symbol;
        huff->max_bits = FFMAX3(huff->max_bits, length, 1);
        huff->nb_codes++;
        return 0;
    } else { // recurse
        if (length >= 16)
            return AVERROR_INVALIDDATA;
        code <<= 1;
        ret = read_trees(length + 1, code|0, huff, gb, tree_signed, tree_scale);
        if (ret < 0)
            return ret;
        return read_trees(length + 1, code|1, huff, gb, tree_signed, tree_scale);
    }
}

static int build_huff(GBCWithVLC *buf, int is_signed, uint32_t scale)
{
    const int tree_signed = is_signed;
    const uint32_t tree_scale = scale;
    VLC *vlc = buf->vlc;
    HuffTree huff;
    int ret;

    huff.nb_codes = 0;
    huff.max_bits = 0;

    ret = read_trees(0, 0, &huff, &buf->gb, tree_signed, tree_scale);
    if (ret < 0)
        return ret;

    if (huff.nb_codes > 0) {
        ff_vlc_free(vlc);
        return ff_vlc_init_sparse(vlc, FFMIN(huff.max_bits, 12),
                                  huff.nb_codes,
                                  huff.bits, 1, 1, huff.codes, 4, 4,
                                  huff.symbols, 2, 2, 0);
    }

    return 0;
}

static int get_code(GetBitContext *new_gb, GetBitContext *gb, int skip)
{
    GetBitContext tmp_gb = *gb;
    uint32_t new_size, offset = get_bits_long(gb, 32);
    int ret;

    if (offset >= INT32_MAX - skip - 4)
        return AVERROR_INVALIDDATA;

    if ((get_bits_left(gb) >> 3) < offset)
        return AVERROR_INVALIDDATA;
    offset += skip;

    ret = init_get_bits8(&tmp_gb, gb->buffer + offset, (gb->size_in_bits >> 3) - offset);
    if (ret < 0)
        return ret;

    new_size = get_bits_long(&tmp_gb, 32);

    if (new_size >= INT32_MAX - 4)
        return AVERROR_INVALIDDATA;

    if ((get_bits_left(&tmp_gb) >> 3) < new_size)
        return AVERROR_INVALIDDATA;

    return init_get_bits8(new_gb, tmp_gb.buffer + 4, new_size);
}

static int decode_huff(GBCWithVLC *buf, uint32_t *value)
{
    av_assert2(&buf->gb);
    if (!buf->vlc->bits) {
        *value = 0;
        return AVERROR_INVALIDDATA;
    }
    if (get_bits_left(&buf->gb) < 0) {
        *value = buf->value;
        return 0;
    }
    av_assert2(buf->vlc->table);
    *value = buf->value = get_vlc2(&buf->gb, buf->vlc->table, buf->vlc->bits, 3);
    return 0;
}

static int iframe_basis_numdec(VideoState *state)
{
    BlockData *luma_dst = state->planes[LUMA_IDX].payload;
    const uint32_t luma_h_blocks = state->planes[LUMA_IDX].h_blocks;
    const uint32_t luma_v_blocks = state->planes[LUMA_IDX].v_blocks;
    const uint32_t chroma_h_blocks = state->planes[CHROMA_IDX].h_blocks;
    const uint32_t chroma_v_blocks = state->planes[CHROMA_IDX].v_blocks;
    BlockData *u_dst = state->planes[CHROMA_IDX].payload;
    BlockData *v_dst = state->planes[CHROMA_IDX+1].payload;
    uint32_t rle = 0;

    for (int y = 0; y < luma_v_blocks; y++) {
        for (int x = 0; x < luma_h_blocks; x++) {
            if (rle) {
                luma_dst->type = 0;
                rle--;
            } else {
                int num, ret = decode_huff(&state->basis_num[LUMA_IDX], &num);

                if (ret < 0)
                    return ret;
                if (num == 0) {
                    ret = decode_huff(&state->basis_num_run[LUMA_IDX], &rle);
                    if (ret < 0)
                        return ret;
                }

                luma_dst->type = num & 0xFF;
            }
            luma_dst++;
        }
        // skip borders
        luma_dst += 2;
    }

    rle = 0;
    for (int y = 0; y < chroma_v_blocks; y++) {
        for (int x = 0; x < chroma_h_blocks; x++) {
            if (rle) {
                u_dst->type = 0;
                v_dst->type = 0;
                rle--;
            } else {
                int num, ret = decode_huff(&state->basis_num[CHROMA_IDX], &num);

                if (ret < 0)
                    return ret;
                if (num == 0) {
                    ret = decode_huff(&state->basis_num_run[CHROMA_IDX], &rle);
                    if (ret < 0)
                        return ret;
                }

                u_dst->type = (num >> 0) & 0xF;
                v_dst->type = (num >> 4) & 0xF;
            }
            u_dst++;
            v_dst++;
        }
        u_dst += 2;
        v_dst += 2;
    }

    return 0;
}

static int decode_sovf_sym(GBCWithVLC *buf, int32_t min, int32_t max, int32_t *r)
{
    int sum = 0, value;

    do {
        int ret = decode_huff(buf, &value);
        if (ret < 0)
            return ret;
        sum += value;
    } while (value <= min || value >= max);

    *r = sum;

    return 0;
}

static int decode_uovf_sym(GBCWithVLC *buf, int max, int32_t *r)
{
    int sum = 0, value;

    do {
        int ret = decode_huff(buf, &value);
        if (ret < 0)
            return ret;
        sum += value;
    } while (value >= max);

    *r = sum;

    return 0;
}

static int get_delta_dc(VideoState *state, uint32_t plane_idx, uint32_t *rle, int32_t *r)
{
    if (rle[0] == 0) {
        int32_t delta;
        int ret = decode_sovf_sym(&state->dc_values[plane_idx], state->dc_min, state->dc_max, &delta);

        if (ret < 0)
            return ret;

        if (delta == 0) { // successive zeroes are run-length encoded
            ret = decode_huff(&state->dc_rle[plane_idx], rle);
            if (ret < 0)
                return ret;
        }
        *r = delta;
        return 0;
    } else {
        rle[0]--;
        *r = 0;
        return 0;
    }
}

static int iframe_dc_decode(VideoState *state)
{
    for (int plane_idx = 0; plane_idx < PLANE_COUNT; plane_idx++) {
        HVQPlaneDesc *plane = &state->planes[plane_idx];
        const uint32_t v_blocks = plane->v_blocks;
        BlockData *curr = plane->payload;
        uint32_t rle = 0;

        for (int y = 0; y < v_blocks; y++) {
            // pointer to previous line
            BlockData const *prev = curr - plane->h_blocks_safe;
            // first prediction on a line is only the previous line's value
            uint8_t value = prev->value;
            for (int x = 0; x < plane->h_blocks; x++) {
                int32_t delta;
                int ret = get_delta_dc(state, plane_idx, &rle, &delta);

                if (ret < 0)
                    return ret;
                value += delta;

                curr->value = value;
                curr++;
                prev++;
                // next prediction on this line is the mean of left (current) and top values
                // +---+---+
                // |   | T |
                // +---+---+
                // | L | P |
                // +---+---+
                value = (value + prev->value + 1) / 2;
            }
            // skip right border of this line and left border of next line
            curr += 2;
        }
    }

    return 0;
}

static void make_nest(VideoState *state, uint16_t nest_x, uint16_t nest_y)
{
    int32_t v_empty, h_empty, v_nest_blocks, h_nest_blocks, v_mirror, h_mirror;
    HVQPlaneDesc *y_plane = &state->planes[0];
    BlockData const *ptr = y_plane->payload + y_plane->h_blocks_safe * nest_y + nest_x;
    uint8_t const *nest2;
    uint8_t *nest;

    if (y_plane->h_blocks < state->h_nest_size) {
        // special case if the video is less than 280 pixels wide (assuming landscape mode)
        h_nest_blocks = y_plane->h_blocks;
        h_mirror = state->h_nest_size - y_plane->h_blocks;
        if (h_mirror > y_plane->h_blocks)
            h_mirror = y_plane->h_blocks;
        h_empty = state->h_nest_size - (h_nest_blocks + h_mirror);
    } else {
        h_nest_blocks = state->h_nest_size;
        h_empty = 0;
        h_mirror = 0;
    }

    if (y_plane->v_blocks < state->v_nest_size) {
        // special case if the video is less than 152 pixels high
        v_nest_blocks = y_plane->v_blocks;
        v_mirror = state->v_nest_size - y_plane->v_blocks;
        if (v_mirror > y_plane->v_blocks)
            v_mirror = y_plane->v_blocks;
        v_empty = state->v_nest_size - (v_nest_blocks + v_mirror);
    } else {
        v_nest_blocks = state->v_nest_size;
        v_empty = 0;
        v_mirror = 0;
    }

    nest = state->nest_data;
    for (int i = 0; i < v_nest_blocks; i++) {
        BlockData const *p = ptr;
        for (int j = 0; j < h_nest_blocks; j++) {
            nest[0] = (p->value >> 4) & 0xF;
            nest++;
            p++;
        }
        // if the video is too small, mirror it
        for (int j = 0; j < h_mirror; j++) {
            p--;
            nest[0] = (p->value >> 4) & 0xF;
            nest++;
        }
        // if it is still too small, null out the rest
        for (int j = 0; j < h_empty; j++) {
            nest[0] = 0;
            nest++;
        }
        ptr += y_plane->h_blocks_safe;
    }

    // handle vertical mirroring
    nest2 = nest - state->h_nest_size;
    for (int i = 0; i < v_mirror; i++) {
        for (int j = 0; j < state->h_nest_size; j++) {
            nest[0] = nest2[j];
            nest++;
        }
        nest2 -= state->h_nest_size;
    }

    // and vertical nulling
    for (int i = 0; i < v_empty; i++) {
        for (int j = 0; j < state->h_nest_size; j++) {
            nest[0] = 0;
            nest++;
        }
    }
}

static uint8_t sat_mean8(uint32_t u)
{
    return av_clip_uint8((u + 4) / 8);
}

static void weight_im_block(uint8_t *dst, ptrdiff_t stride, uint8_t value,
                            uint8_t top, uint8_t bottom, uint8_t left, uint8_t right)
{
    /*
    +---+---+---+
    |   | T |   |
    +---+---+---+
    | L | D | R |
    +---+---+---+
    |   | B |   |
    +---+---+---+
     */
    int32_t tmb = top - bottom;
    int32_t lmr = left - right;
    int32_t vph = tmb + lmr;
    int32_t vmh = tmb - lmr;

    int32_t v2 = value * 2;
    int32_t v8 = value * 8;

    int32_t tpl = (top + left) - v2;
    int32_t tpr = (top + right) - v2;
    int32_t bpr = (bottom + right) - v2;
    int32_t bpl = (bottom + left) - v2;

    int32_t tml = top - left;
    int32_t tmr = top - right;
    int32_t bmr = bottom - right;
    int32_t bml = bottom - left;

    // V:
    // 6  8  8 6
    // 8 10 10 8
    // 8 10 10 8
    // 6  8  8 6
    //
    // T:
    //  2  2  2  2
    //  0  0  0  0
    // -1 -1 -1 -1
    // -1 -1 -1 -1
    //
    // B/L/R: like T but rotated accordingly

    // (6*V + 2*T - B + 2*L -   R + 4) / 8
    // (8*V + 2*T - B       -   R + 4) / 8
    // (8*V + 2*T - B -   L       + 4) / 8
    // (6*V + 2*T - B -   L + 2*R + 4) / 8

    dst[0] = sat_mean8(v8 + vph + tpl);
    dst[1] = sat_mean8(v8 + vph + tml);
    dst[2] = sat_mean8(v8 + vmh + tmr);
    dst[3] = sat_mean8(v8 + vmh + tpr);

    dst += stride;

    // ( 8*V - B + 2*L -   R + 4) / 8
    // (10*V - B       -   R + 4) / 8
    // (10*V - B -   L       + 4) / 8
    // ( 8*V - B -   L + 2*R + 4) / 8

    dst[0] = sat_mean8(v8 + vph - tml);
    dst[1] = sat_mean8(v8 - bpr);
    dst[2] = sat_mean8(v8 - bpl);
    dst[3] = sat_mean8(v8 + vmh - tmr);

    dst += stride;

    // ( 8*V - T + 2*L - R + 4) / 8
    // (10*V - T       - R + 4) / 8
    // (10*V - T - L

    dst[0] = sat_mean8(v8 - vmh - bml);
    dst[1] = sat_mean8(v8 - tpr);
    dst[2] = sat_mean8(v8 - tpl);
    dst[3] = sat_mean8(v8 - vph - bmr);

    dst += stride;

    dst[0] = sat_mean8(v8 - vmh + bpl);
    dst[1] = sat_mean8(v8 - vmh + bml);
    dst[2] = sat_mean8(v8 - vph + bmr);
    dst[3] = sat_mean8(v8 - vph + bpr);
}

static void dc_block(uint8_t *dst, ptrdiff_t stride, uint8_t value)
{
    for (int y = 0; y < 4; y++) {
        memset(dst, value, 4);
        dst += stride;
    }
}

static int get_aot_basis(VideoState *state, uint8_t basis_out[4][4],
                         int32_t *sum, uint8_t const *nest_data,
                         ptrdiff_t nest_stride, uint32_t plane_idx,
                         uint32_t *r)
{
    GetBitContext *gb = &state->fixvl[plane_idx];
    uint16_t bits = get_bits(gb, 16);
    ptrdiff_t x_stride, y_stride;
    uint32_t offset70 = bits & 0x3F;
    uint32_t offset38 = (bits >> 6) & 0x1F;
    ptrdiff_t stride70 = (bits >> 11) & 1;
    ptrdiff_t stride38 = (bits >> 12) & 1;
    int32_t inverse, offset;
    uint8_t min, max;
    uint32_t value;
    int ret;

    if (state->is_landscape) {
        nest_data += nest_stride * offset38 + offset70;
        x_stride = 1 << stride70;
        y_stride = nest_stride << stride38;
    } else {
        nest_data += nest_stride * offset70 + offset38;
        x_stride = 1 << stride38;
        y_stride = nest_stride << stride70;
    }

    // copy basis vector from the nest
    min = nest_data[0];
    max = nest_data[0];
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
            uint8_t nest_value = nest_data[y * y_stride + x * x_stride];
            basis_out[y][x] = nest_value;
            min = nest_value < min ? nest_value : min;
            max = nest_value > max ? nest_value : max;
        }
    }
    av_assert1(plane_idx < 3);
    ret = decode_huff(&state->bufTree0[plane_idx], &value);
    if (ret < 0)
        return ret;
    sum[0] += value;
    inverse = div_tab[max - min];
    if (bits & 0x8000)
        inverse = -inverse;
    offset = (bits >> 13) & 3;
    *r = (sum[0] + offset) * inverse;

    return 0;
}

static int get_aot_sum(VideoState *state, int32_t result[4][4],
                       uint8_t num_bases, uint8_t const *nest_data,
                       ptrdiff_t nest_stride, uint32_t plane_idx, int32_t *r)
{
    int32_t temp, sum;
    uint8_t basis[4][4];

    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++)
            result[y][x] = 0;
    temp = 0;

    for (int k = 0; k < num_bases; k++) {
        uint32_t factor;
        int ret = get_aot_basis(state, basis, &temp, nest_data, nest_stride, plane_idx, &factor);

        if (ret < 0)
            return ret;

        for (int y = 0; y < 4; y++)
            for (int x = 0; x < 4; x++)
                result[y][x] += factor * basis[y][x];
    }

    sum = 0;
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++)
            sum += result[y][x];
    *r = sum >> 4;
    return 0;
}

static void read_block(VideoState *state, uint8_t *dst, ptrdiff_t stride, uint32_t plane_idx)
{
    GetBitContext *gb = &state->fixvl[plane_idx];

    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++)
            dst[x] = get_bits(gb, 8);
        dst += stride;
    }
}

static int intra_aot_block(VideoState *state, uint8_t *dst, ptrdiff_t stride,
                            uint8_t target_average, uint8_t block_type, uint32_t plane_idx)
{
    int32_t result[4][4], aot_average, delta;
    int ret;

    if (block_type == 6) {
        read_block(state, dst, stride, plane_idx);
        return 0;
    }

    // block types 1..5 serve as number of bases to use, 9..15 are unused
    ret = get_aot_sum(state, result, block_type, state->nest_data, state->h_nest_size, plane_idx, &aot_average);
    if (ret < 0)
        return ret;
    delta = (target_average << state->unk_shift) - aot_average;
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
            int32_t value = ((result[y][x] + delta) >> state->unk_shift);
            dst[x] = av_clip_uint8(value);
        }
        dst += stride;
    }

    return 0;
}

static int decode_iframe_block(VideoState *state, uint8_t *dst, ptrdiff_t stride, StackState *stack_state)
{
    int ret;

    if (stack_state->curr.type == 0) {
        uint8_t top = stack_state->line_prev->type & 0x77 ? stack_state->curr.value : stack_state->line_prev->value;
        uint8_t bottom = stack_state->line_next->type & 0x77 ? stack_state->curr.value : stack_state->line_next->value;
        uint8_t right = stack_state->next.type & 0x77 ? stack_state->curr.value : stack_state->next.value;
        // the left value is tracked manually, the logic is equivalent with the other surrounding values
        uint8_t left = stack_state->value_prev;

        weight_im_block(dst, stride, stack_state->curr.value, top, bottom, left, right);
        stack_state->value_prev = stack_state->curr.value;
    } else if (stack_state->curr.type == 8) {
        dc_block(dst, stride, stack_state->curr.value);
        stack_state->value_prev = stack_state->curr.value;
    } else {
        ret = intra_aot_block(state, dst, stride, stack_state->curr.value, stack_state->curr.type, stack_state->plane_idx);
        if (ret < 0)
            return ret;
        // don't use the current DC value to predict the next one
        stack_state->value_prev = stack_state->next.value;
    }
    // next block
    stack_state->line_prev++;
    stack_state->line_next++;

    return 0;
}

static int iframe_line(VideoState *state, uint8_t *dst, ptrdiff_t stride, StackState *stack_state, uint16_t h_blocks)
{
    int ret;

    stack_state->next = stack_state->line_curr[0];
    stack_state->value_prev = stack_state->line_curr[0].value;

    while (--h_blocks > 0) {
        stack_state->curr = stack_state->next;
        stack_state->line_curr++;
        stack_state->next = stack_state->line_curr[0];
        ret = decode_iframe_block(state, dst, stride, stack_state);
        if (ret < 0)
            return ret;
        // next block on same line
        dst += 4;
    }

    stack_state->curr = stack_state->next;
    ret = decode_iframe_block(state, dst, stride, stack_state);
    if (ret < 0)
        return ret;

    // skip current, right border on same line, and left border on next line
    stack_state->line_curr += 3;

    // these have already been advanced to the right border in decode_iframe_block
    stack_state->line_prev += 2;
    stack_state->line_next += 2;

    return 0;
}

static int decode_iframe_plane(VideoState *state, int plane_idx, AVFrame *frame)
{
    HVQPlaneDesc *plane = &state->planes[plane_idx];
    const int border = 32 >> (!!plane_idx);
    ptrdiff_t linesize = frame->linesize[plane_idx];
    uint8_t *dst = frame->data[plane_idx] + linesize * border + border;
    StackState stack_state;
    int16_t v_blocks;
    int ret;

    stack_state.plane_idx = plane_idx;
    stack_state.line_prev = plane->payload;
    stack_state.line_curr = plane->payload;
    stack_state.line_next = plane->payload + plane->h_blocks_safe;
    v_blocks = plane->v_blocks;

    // first line
    if (v_blocks > 0) {
        ret = iframe_line(state, dst, linesize, &stack_state, plane->h_blocks);
        if (ret < 0)
            return ret;
        // blocks are 4x4 so advance dst by 4 lines
        dst += linesize * 4;
        v_blocks--;
    }
    // middle lines
    stack_state.line_prev = plane->payload;
    while (v_blocks > 1) {
        ret = iframe_line(state, dst, linesize, &stack_state, plane->h_blocks);
        if (ret < 0)
            return ret;
        dst += linesize * 4;
        v_blocks--;
    }
    // last line
    if (v_blocks > 0) {
        stack_state.line_next = stack_state.line_curr;
        ret = iframe_line(state, dst, linesize, &stack_state, plane->h_blocks);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int decode_iframe(SeqObj *seqobj, GetBitContext *gb, AVFrame *frame)
{
    VideoState *state = seqobj->state;
    int dc_shift = get_bits(gb, 8);
    uint16_t nest_x, nest_y;
    int ret;

    state->unk_shift = get_bits(gb, 8);
    skip_bits(gb, 16);
    nest_x = get_bits(gb, 16);
    nest_y = get_bits(gb, 16);

    for (int i = 0; i < LUMA_CHROMA; i++) {
        ret = get_code(&state->basis_num[i].gb, gb, 78);
        if (ret < 0)
            return ret;
        ret = get_code(&state->basis_num_run[i].gb, gb, 78);
        if (ret < 0)
            return ret;
    }

    for (int i = 0; i < PLANE_COUNT; i++) {
        ret = get_code(&state->dc_values[i].gb, gb, 78);
        if (ret < 0)
            return ret;
        ret = get_code(&state->bufTree0[i].gb, gb, 78);
        if (ret < 0)
            return ret;
        ret = get_code(&state->fixvl[i], gb, 78);
        if (ret < 0)
            return ret;
    }

    for (int i = 0; i < PLANE_COUNT; i++) {
        ret = get_code(&state->dc_rle[i].gb, gb, 78);
        if (ret < 0)
            return ret;
    }

    ret = build_huff(&state->basis_num[0], 0, 0);
    if (ret < 0)
        return ret;
    ret = build_huff(&state->basis_num_run[0], 0, 0);
    if (ret < 0)
        return ret;
    ret = build_huff(&state->dc_values[0], 1, dc_shift);
    if (ret < 0)
        return ret;
    ret = build_huff(&state->bufTree0[0], 0, 2);
    if (ret < 0)
        return ret;

    state->dc_max =   127 << dc_shift;
    state->dc_min = -(128 << dc_shift);

    // 4x4 block types
    ret = iframe_basis_numdec(state);
    if (ret < 0)
        return ret;
    // 4x4 block DC values
    ret = iframe_dc_decode(state);
    if (ret < 0)
        return ret;
    // 70x38 nest copied from upper 4 bits of DC values somewhere in the luma plane
    make_nest(state, nest_x, nest_y);

    for (int i = 0; i < PLANE_COUNT; i++) {
        ret = decode_iframe_plane(state, i, frame);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static void init_mc_handler(VideoState *state,
                            MCPlane *mcplanes,
                            AVFrame *present, AVFrame *past, AVFrame *future)
{
    for (int i = 0; i < PLANE_COUNT; i++) {
        const int border = 32 >> (!!i);

        MCPlane *mcplane = &mcplanes[i];
        HVQPlaneDesc *plane = &state->planes[i];

        mcplane->rle = 0;
        mcplane->pb_dc = 127;
        mcplane->present = present->data[i] + border * present->linesize[i] + border;
        mcplane->present_stride = present->linesize[i];
        mcplane->past    = past->data[i] + border * past->linesize[i] + border;
        mcplane->past_stride = past->linesize[i];
        mcplane->future  = future->data[i] + border * future->linesize[i] + border;
        mcplane->future_stride = future->linesize[i];
        mcplane->payload_cur_blk = plane->payload;
        mcplane->payload_cur_row = plane->payload;
        mcplane->h_mcb_stride = 8 >> plane->width_shift;
        mcplane->v_mcb_stride = present->linesize[i] * (8 >> plane->height_shift);
        mcplane->pb_per_mcb_x = plane->pb_per_mcb_x;
        mcplane->stride = plane->h_blocks_safe * plane->pb_per_mcb_y;
    }
}

static int initMCBproc(GBCWithVLC *buf, struct RLDecoder *proc)
{
    if (buf->gb.buffer) {
        int ret;

        proc->value = get_bits1(&buf->gb);
        ret = decode_uovf_sym(buf, 255, &proc->count);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int initMCBtype(GBCWithVLC *buf, struct RLDecoder *type)
{
    if (buf->gb.buffer) {
        int ret;

        type->value = get_bits(&buf->gb, 2);
        ret = decode_uovf_sym(buf, 255, &type->count);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static void setMCTop(MCPlane *mcplanes)
{
    for (int i = 0; i < PLANE_COUNT; i++) {
        mcplanes[i].top = mcplanes[i].present;
        mcplanes[i].top_stride = mcplanes[i].present_stride;
    }
}

static const uint8_t mcbtypetrans[2][3] = {
    { 1, 2, 0 },
    { 2, 0, 1 },
};

static int getMCBtype(GBCWithVLC *buftree, struct RLDecoder *type)
{
    if (type->count == 0) {
        // only three possible values, so when the value changes,
        // a single bit decides which other value to select
        // bit == 0 -> increment
        // bit == 1 -> decrement
        // then wrap to range 0..2
        int ret, bit = get_bits1(&buftree->gb);

        type->value = mcbtypetrans[bit][type->value];
        ret = decode_uovf_sym(buftree, 255, &type->count);
        if (ret < 0)
            return ret;
    }

    if (type->count == 0)
        return AVERROR_INVALIDDATA;
    type->count--;
    return 0;
}

static int decode_PB_dc(VideoState *state, MCPlane *mcplanes)
{
    for (int i = 0; i < PLANE_COUNT; i++) {
        HVQPlaneDesc *plane = &state->planes[i];
        MCPlane *mcplane = &mcplanes[i];

        for (int j = 0; j < plane->blocks_per_mcb; j++) {
            BlockData *payload;
            int ret, value;

            ret = decode_sovf_sym(&state->dc_values[i], state->dc_min, state->dc_max, &value);
            if (ret < 0)
                return ret;
            mcplane->pb_dc += value;
            payload = mcplane->payload_cur_blk;
            payload[plane->mcb_offset[j]].value = mcplane->pb_dc;
        }
    }

    return 0;
}

static int decode_PB_cc(VideoState *state, MCPlane *mcplanes, uint32_t proc, uint32_t type)
{
    uint32_t block_type = (type << 5) | (proc << 4);
    if (proc == 1) {
        for (int i = 0; i < PLANE_COUNT; i++) {
            BlockData *payload = mcplanes[i].payload_cur_blk;
            HVQPlaneDesc *plane = &state->planes[i];
            for (int j = 0; j < plane->blocks_per_mcb; j++)
                payload[plane->mcb_offset[j]].type = block_type;
        }
    } else {
        HVQPlaneDesc *planeY = &state->planes[0];
        HVQPlaneDesc *planeU = &state->planes[1];
        MCPlane *mcplaneY = &mcplanes[0];
        MCPlane *mcplaneU = &mcplanes[1];
        MCPlane *mcplaneV = &mcplanes[2];
        for (int i = 0; i < planeY->blocks_per_mcb; i++) {
            BlockData *ptr = mcplaneY->payload_cur_blk;
            if (mcplaneY->rle) {
                ptr[planeY->mcb_offset[i]].type = block_type;
                mcplaneY->rle--;
            } else {
                int huff, ret = decode_huff(&state->basis_num[LUMA_IDX], &huff);

                if (ret < 0)
                    return ret;
                if (huff) {
                    ptr[planeY->mcb_offset[i]].type = block_type | huff;
                } else {
                    ptr[planeY->mcb_offset[i]].type = block_type;
                    ret = decode_huff(&state->basis_num_run[LUMA_IDX], &mcplaneY->rle);
                    if (ret < 0)
                        return ret;
                }
            }
        }
        for (int i = 0; i < planeU->blocks_per_mcb; i++) { // chroma
            BlockData *ptrU = mcplaneU->payload_cur_blk;
            BlockData *ptrV = mcplaneV->payload_cur_blk;
            if (mcplaneU->rle) {
                ptrU[planeU->mcb_offset[i]].type = block_type;
                ptrV[planeU->mcb_offset[i]].type = block_type;
                mcplaneU->rle--;
            } else {
                int huff, ret = decode_huff(&state->basis_num[CHROMA_IDX], &huff);

                if (ret < 0)
                    return ret;
                if (huff) {
                    ptrU[planeU->mcb_offset[i]].type = block_type | ((huff >> 0) & 0xF);
                    ptrV[planeU->mcb_offset[i]].type = block_type | ((huff >> 4) & 0xF);
                } else {
                    ptrU[planeU->mcb_offset[i]].type = block_type;
                    ptrV[planeU->mcb_offset[i]].type = block_type;
                    ret = decode_huff(&state->basis_num_run[CHROMA_IDX], &mcplaneU->rle);
                    if (ret < 0)
                        return ret;
                }
            }
        }
    }

    return 0;
}

static void reset_PB_dc(MCPlane *mcplanes)
{
    for (int i = 0; i < PLANE_COUNT; i++)
        mcplanes[i].pb_dc = 127;
}

static int getMCBproc(GBCWithVLC *buf, struct RLDecoder *proc, uint32_t *r)
{
    if (proc->count == 0) {
        int ret;

        proc->value ^= 1;
        ret = decode_uovf_sym(buf, 255, &proc->count);
        if (ret < 0)
            return ret;
    }

    if (proc->count == 0) {
        *r = 0;
        return AVERROR_INVALIDDATA;
    }

    proc->count--;
    *r = proc->value;

    return 0;
}

static void setMCNextBlk(MCPlane *mcplanes)
{
    for (int i = 0; i < PLANE_COUNT; i++) {
        mcplanes[i].top += mcplanes[i].h_mcb_stride;
        mcplanes[i].payload_cur_blk += mcplanes[i].pb_per_mcb_x;
    }
}

static void setMCDownBlk(MCPlane *mcplanes)
{
    for (int i = 0; i < PLANE_COUNT; i++) {
        MCPlane *mcplane = &mcplanes[i];
        BlockData *first_block_on_next_row = mcplane->payload_cur_row + mcplane->stride;

        mcplane->present += mcplane->v_mcb_stride;
        mcplane->payload_cur_blk = first_block_on_next_row;
        mcplane->payload_cur_row = first_block_on_next_row;
    }
}

static int spread_PB_descMap(SeqObj *seqobj, MCPlane *mcplanes)
{
    struct RLDecoder proc, type;
    VideoState *state = seqobj->state;
    int ret;

    ret = initMCBproc(&state->mcb_proc, &proc);
    if (ret < 0)
        return ret;
    ret = initMCBtype(&state->mcb_type, &type);
    if (ret < 0)
        return ret;

    for (int i = 0; i < seqobj->height; i += 8) {
        setMCTop(mcplanes);
        for (int j = 0; j < seqobj->width; j += 8) {
            ret = getMCBtype(&state->mcb_type, &type);
            if (ret < 0)
                return ret;

            if (type.value == 0) {
                ret = decode_PB_dc(state, mcplanes);
                if (ret < 0)
                    return ret;
                ret = decode_PB_cc(state, mcplanes, 0, type.value);
            } else {
                uint32_t new_proc;

                reset_PB_dc(mcplanes);
                ret = getMCBproc(&state->mcb_proc, &proc, &new_proc);
                if (ret < 0)
                    return ret;
                ret = decode_PB_cc(state, mcplanes, new_proc, type.value);
            }

            if (ret < 0)
                return ret;
            setMCNextBlk(mcplanes);
                // for all planes
                //     top             += h_mcb_stride
                //     payload_cur_blk += pb_per_mcb_x
        }
        setMCDownBlk(mcplanes);
            // for all planes
            //     present += v_mcb_stride
            //     payload_cur_row += stride;
            //     payload_cur_blk = payload_cur_row
    }

    return 0;
}

static void resetMCHandler(VideoState *state, MCPlane *mcplanes, AVFrame *frame)
{
    for (int i = 0; i < PLANE_COUNT; i++) {
        const int border = 32 >> (!!i);
        ptrdiff_t linesize = frame->linesize[i];
        uint8_t *dst = frame->data[i] + linesize * border + border;

        mcplanes[i].present = dst;
        mcplanes[i].payload_cur_blk = state->planes[i].payload;
        mcplanes[i].payload_cur_row = state->planes[i].payload;
    }
}

static int MCBlockDecDCNest(VideoState *state, MCPlane *mcplanes)
{
    for (int plane_idx = 0; plane_idx < PLANE_COUNT; plane_idx++) {
        BlockData const *ptr = mcplanes[plane_idx].payload_cur_blk;
        HVQPlaneDesc *plane = &state->planes[plane_idx];
        ptrdiff_t stride = mcplanes[plane_idx].top_stride;
        int32_t line = plane->h_blocks_safe;

        for (int j = 0; j < plane->blocks_per_mcb; j++) {
            // dst is a 4x4 region
            uint8_t *dst = mcplanes[plane_idx].top + plane->px_offset[j] + plane->py_offset[j] * mcplanes[plane_idx].top_stride;
            int32_t block_idx = plane->mcb_offset[j];
            uint32_t value = ptr[block_idx].value;
            // block type:
            // 0: weighted
            // 6: literal block
            // 8: single value
            uint32_t type = ptr[block_idx].type & 0xF;
            // see also IpicBlockDec
            if (type == 0) {
                uint8_t top    = ptr[block_idx - line].type & 0x77 ? value : ptr[block_idx - line].value;
                uint8_t left   = ptr[block_idx -    1].type & 0x77 ? value : ptr[block_idx -    1].value;
                uint8_t right  = ptr[block_idx +    1].type & 0x77 ? value : ptr[block_idx +    1].value;
                uint8_t bottom = ptr[block_idx + line].type & 0x77 ? value : ptr[block_idx + line].value;
                weight_im_block(dst, stride, value, top, bottom, left, right);
            } else if (type == 8) {
                dc_block(dst, stride, value);
            } else {
                int ret = intra_aot_block(state, dst, stride, value, type, plane_idx);
                if (ret < 0)
                    return ret;
            }
        }
    }

    return 0;
}

static void setMCTarget(MCPlane *mcplanes, int reference_frame)
{
    if (reference_frame == 0) {
        for (int i = 0; i < PLANE_COUNT; i++) {
            mcplanes[i].target = mcplanes[i].past;
            mcplanes[i].target_stride = mcplanes[i].past_stride;
        }
    } else {
        for (int i = 0; i < PLANE_COUNT; i++) {
            mcplanes[i].target = mcplanes[i].future;
            mcplanes[i].target_stride = mcplanes[i].future_stride;
        }
    }
}

static int get_mv_vector(int *result, GBCWithVLC *buf, int residual_bits)
{
    int32_t max_val_plus_1 = 1 << (residual_bits + 5);
    int ret, value;

    ret = decode_huff(buf, &value); // quantized value
    if (ret < 0)
        return ret;

    value *= (1 << residual_bits);
    // residual bits
    value += get_bits_long(&buf->gb, residual_bits);
    value += result[0];
    // signed wrap to -max_val_plus_1 .. max_val_plus_1-1
    if (value >= max_val_plus_1)
        value -= max_val_plus_1 << 1;
    else if (value < -max_val_plus_1)
        value += max_val_plus_1 << 1;
    result[0] = value;

    return 0;
}

static int get_mc_aot_basis(VideoState *state, uint8_t basis_out[4][4],
                            int32_t *sum, uint8_t const *nest_data,
                            ptrdiff_t nest_stride, uint32_t plane_idx,
                            uint32_t *r)
{
    // the only difference to GetAotBasis() seems to be the ">> 4 & 0xF"
    GetBitContext *gb = &state->fixvl[plane_idx];
    uint16_t bits = get_bits(gb, 16);
    uint32_t step, stride;
    uint32_t big = bits & 0x3F;
    uint32_t small = (bits >> 6) & 0x1F;
    int32_t inverse, foo;
    uint8_t min, max;
    int ret, value;

    if (state->is_landscape) {
        nest_data += nest_stride * small + big;
        step   =           1 << ((bits >> 11) & 1);
        stride = nest_stride << ((bits >> 12) & 1);
    } else {
        nest_data += nest_stride * big + small;
        step   =           1 << ((bits >> 12) & 1);
        stride = nest_stride << ((bits >> 11) & 1);
    }
    min = max = (nest_data[0] >> 4) & 0xF; // !
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            uint8_t nest_value = (nest_data[i * stride + j * step] >> 4) & 0xF; // !

            basis_out[i][j] = nest_value;
            min = nest_value < min ? nest_value : min;
            max = nest_value > max ? nest_value : max;
        }
    }
    ret = decode_huff(&state->bufTree0[plane_idx], &value);
    if (ret < 0)
        return ret;

    sum[0] += value;
    inverse = div_tab[max - min];
    if (bits & 0x8000)
        inverse = -inverse;
    foo = (bits >> 13) & 3;
    *r = (sum[0] + foo) * inverse;
    return 0;
}

static int get_mc_aot_sum(VideoState *state, int32_t result[4][4], uint8_t num_bases,
                          uint8_t const *nest_data, ptrdiff_t nest_stride, uint32_t plane_idx,
                          int32_t *r)
{
    uint8_t byte_result[4][4];
    int32_t sum, temp = 0;

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            result[i][j] = 0;

    for (int k = 0; k < num_bases; k++) {
        uint32_t factor;
        int ret = get_mc_aot_basis(state, byte_result, &temp, nest_data, nest_stride, plane_idx, &factor);

        if (ret < 0)
            return ret;

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                result[i][j] += factor * byte_result[i][j];
    }

    sum = 0;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            sum += result[i][j];
    *r = sum >> 4;

    return 0;
}

static int PrediAotBlock(VideoState *state, uint8_t *dst, ptrdiff_t dst_stride, uint8_t const *src, ptrdiff_t src_stride, uint8_t block_type,
                         uint8_t *nest_data, ptrdiff_t h_nest_size, uint32_t plane_idx, uint32_t hpel_dx, uint32_t hpel_dy)
{
    int32_t result[4][4], mean, diff[4][4], min, max;
    uint32_t addend, factor, aot_sum;
    uint8_t mdst[4][4];
    int value, ret;

    ret = get_mc_aot_sum(state, result, block_type - 1, nest_data, h_nest_size, plane_idx, &aot_sum);
    if (ret < 0)
        return ret;

    state->motion_comp_func[hpel_dx][hpel_dy]((uint8_t *)mdst, 4, src, src_stride);
    mean = 8;
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++)
            mean += mdst[y][x];
    mean /= 16;
    min = max = mdst[0][0] - mean;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            int32_t value = diff[i][j] = mdst[i][j] - mean;

            min = value < min ? value : min;
            max = value > max ? value : max;
        }
    }
    ret = decode_sovf_sym(&state->dc_values[plane_idx], state->dc_min, state->dc_max, &value);
    if (ret < 0)
        return ret;

    value >>= state->dc_shift;
    addend = (value * (1 << state->unk_shift)) - aot_sum;
    ret = decode_sovf_sym(&state->dc_values[plane_idx], state->dc_min, state->dc_max, &value);
    if (ret < 0)
        return ret;

    factor = value >> state->dc_shift;
    factor *= mcdiv_tab[max - min];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            result[i][j] += addend + diff[i][j] * factor;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            uint32_t value = (result[i][j] >> state->unk_shift) + mdst[i][j];

            dst[j] = av_clip_uint8(value);
        }
        dst += dst_stride;
    }

    return 0;
}

static int MCBlockDecMCNest(VideoState *state, MCPlane *mcplanes, int x, int y)
{
    int hpel_dx = x & 1, hpel_dy = y & 1;
    void *nest_data;

    if (state->is_landscape)
        nest_data = mcplanes[0].target + x/2 + (y/2 - 16)*mcplanes[0].target_stride - 32;
    else
        nest_data = mcplanes[0].target + x/2 + (y/2 - 32)*mcplanes[0].target_stride - 16;
    for (int plane_idx = 0; plane_idx < PLANE_COUNT; plane_idx++) {
        MCPlane *mcplane = &mcplanes[plane_idx];
        HVQPlaneDesc *plane = &state->planes[plane_idx];
        for (int i = 0; i < plane->blocks_per_mcb; i++) {
            BlockData const *ptr = mcplane->payload_cur_blk;
            uint8_t block_type = ptr[plane->mcb_offset[i]].type & 0xF;
            uint8_t *dst = mcplane->top + plane->px_offset[i] + plane->py_offset[i] * mcplane->top_stride;
            ptrdiff_t stride = mcplane->top_stride;

            if (block_type == 6) {
                read_block(state, dst, stride, plane_idx);
            } else {
                int plane_dx = x >> plane->width_shift;
                int plane_dy = y >> plane->height_shift;
                uint8_t const *src = mcplane->target + (plane_dy >> 1) * mcplane->target_stride + (plane_dx >> 1) +
                                     plane->px_offset[i] + plane->py_offset[i] * mcplane->target_stride;

                if (state->version) {
                    hpel_dx = plane_dx & 1;
                    hpel_dy = plane_dy & 1;
                }

                if (block_type == 0) {
                    state->motion_comp_func[hpel_dx][hpel_dy](dst, stride, src, mcplane->target_stride);
                } else {
                    ptrdiff_t strideY = mcplanes[0].target_stride;
                    int ret = PrediAotBlock(state, dst, stride, src, mcplane->target_stride, block_type, nest_data, strideY, plane_idx, hpel_dx, hpel_dy);
                    if (ret < 0)
                        return ret;
                }
            }
        }
    }

    return 0;
}

static int motion_comp(VideoState *state, MCPlane *mcplanes,
                       int dx, int dy, int width, int height)
{
    uint32_t hpel_dx = dx & 1;
    uint32_t hpel_dy = dy & 1;

    for (int i = 0; i < PLANE_COUNT; i++) {
        MCPlane *mcplane = &mcplanes[i];
        HVQPlaneDesc *plane = &state->planes[i];
        int plane_dx = dx >> plane->width_shift;
        int plane_dy = dy >> plane->height_shift;
        uint8_t *ptr = mcplane->target;
        int mb_x = plane_dx >> 1;
        int mb_y = plane_dy >> 1;

        if (state->version) {
            hpel_dx = plane_dx & 1;
            hpel_dy = plane_dy & 1;
        }

        if (mb_y < 0 || mb_y >= height >> plane->height_shift)
            return AVERROR_INVALIDDATA;
        if (mb_x < 0 || mb_x >= width >> plane->width_shift)
            return AVERROR_INVALIDDATA;

        ptr += mb_y * mcplane->target_stride + mb_x;
        for (int j = 0; j < plane->blocks_per_mcb; j++) {
            state->motion_comp_func[hpel_dx][hpel_dy](mcplane->top + plane->px_offset[j] + plane->py_offset[j] * mcplane->top_stride,
                                                      mcplane->top_stride,
                                                      ptr + plane->px_offset[j] + plane->py_offset[j] * mcplane->target_stride,
                                                      mcplane->target_stride);
        }
    }

    return 0;
}

static int decode_bframe_plane(SeqObj *seqobj, AVFrame *present, AVFrame *past, AVFrame *future)
{
    MCPlane mcplanes[PLANE_COUNT];
    VideoState *state = seqobj->state;
    int reference_frame = -1;
    int mv_h, mv_v, ret;

    init_mc_handler(state, mcplanes, present, past, future);
    ret = spread_PB_descMap(seqobj, mcplanes);
    if (ret < 0)
        return ret;
    resetMCHandler(state, mcplanes, present);
    for (int y = 0; y < seqobj->height; y += 8) { // MC blocks are 8x8 pixels
        setMCTop(mcplanes);
        for (int x = 0; x < seqobj->width; x += 8) {
            uint8_t bits = mcplanes[0].payload_cur_blk->type;
            // 0: intra
            // 1: inter - past
            // 2: inter - future
            // see getMCBtype()
            int new_reference_frame = (bits >> 5) & 3;
            if (new_reference_frame == 0) {
                // intra
                ret = MCBlockDecDCNest(state, mcplanes);
                if (ret < 0)
                    return ret;
            } else {
                uint32_t mcb_proc, ref_x, ref_y;

                new_reference_frame--;
                // check if we need to update the reference frame pointers
                if (new_reference_frame != reference_frame) {
                    reference_frame = new_reference_frame;
                    setMCTarget(mcplanes, reference_frame);
                    mv_h = 0;
                    mv_v = 0;
                }

                ret = get_mv_vector(&mv_h, &state->mv_h, state->mc_residual_bits_h[reference_frame]);
                if (ret < 0)
                    return ret;
                ret = get_mv_vector(&mv_v, &state->mv_v, state->mc_residual_bits_v[reference_frame]);
                if (ret < 0)
                    return ret;

                // compute half-pixel position of reference macroblock
                ref_x = x * 2 + mv_h;
                ref_y = y * 2 + mv_v;

                // see getMCBproc()
                mcb_proc = (bits >> 4) & 1;
                if (mcb_proc == 0) {
                    ret = MCBlockDecMCNest(state, mcplanes, ref_x, ref_y);
                } else {
                    ret = motion_comp(state, mcplanes, ref_x, ref_y, seqobj->width, seqobj->height);
                }
                if (ret < 0)
                    return ret;
            }
            setMCNextBlk(mcplanes);
        }
        setMCDownBlk(mcplanes);
    }

    return 0;
}

static int decode_bframe(SeqObj *seqobj, GetBitContext *gb, AVFrame *present, AVFrame *past, AVFrame *future)
{
    VideoState *state = seqobj->state;
    int ret;

    state->dc_shift = get_bits(gb, 8);
    state->unk_shift = get_bits(gb, 8);
    state->mc_residual_bits_h[0] = get_bits(gb, 8);
    state->mc_residual_bits_v[0] = get_bits(gb, 8);
    state->mc_residual_bits_h[1] = get_bits(gb, 8);
    state->mc_residual_bits_v[1] = get_bits(gb, 8);
    state->mc_residual_bits_h[2] = get_bits(gb, 8);
    state->mc_residual_bits_v[2] = get_bits(gb, 8);

    for (int i = 0; i < LUMA_CHROMA; i++) {
        ret = get_code(&state->basis_num[i].gb, gb, 82);
        if (ret < 0)
            return ret;
        ret = get_code(&state->basis_num_run[i].gb, gb, 82);
        if (ret < 0)
            return ret;
    }

    for (int i = 0; i < PLANE_COUNT; i++) {
        ret = get_code(&state->dc_values[i].gb, gb, 82);
        if (ret < 0)
            return ret;
        ret = get_code(&state->bufTree0[i].gb, gb, 82);
        if (ret < 0)
            return ret;
        ret = get_code(&state->fixvl[i], gb, 82);
        if (ret < 0)
            return ret;
    }

    ret = get_code(&state->mv_h.gb, gb, 82);
    if (ret < 0)
        return ret;
    ret = get_code(&state->mv_v.gb, gb, 82);
    if (ret < 0)
        return ret;
    ret = get_code(&state->mcb_type.gb, gb, 82);
    if (ret < 0)
        return ret;
    ret = get_code(&state->mcb_proc.gb, gb, 82);
    if (ret < 0)
        return ret;

    ret = build_huff(&state->basis_num[0], 0, 0);
    if (ret < 0)
        return ret;
    ret = build_huff(&state->basis_num_run[0], 0, 0);
    if (ret < 0)
        return ret;
    ret = build_huff(&state->dc_values[0], 1, state->dc_shift);
    if (ret < 0)
        return ret;
    ret = build_huff(&state->bufTree0[0], 0, 2);
    if (ret < 0)
        return ret;
    ret = build_huff(&state->mv_h, 1, 0);
    if (ret < 0)
        return ret;
    ret = build_huff(&state->mcb_type, 0, 0);
    if (ret < 0)
        return ret;

    state->dc_max =   127 << state->dc_shift;
    state->dc_min = -(128 << state->dc_shift);

    return decode_bframe_plane(seqobj, present, past, future);
}

static int hvqm4_decode(AVCodecContext *avctx, AVFrame *avframe,
                        int *got_frame, AVPacket *pkt)
{
    HVQM4Context *s = avctx->priv_data;
    GetBitContext *gb = &s->gb;
    AVFrame *frame = s->frame[0];
    int frame_type, ret;
    uint32_t disp_id;

    if ((ret = init_get_bits8(gb, pkt->data, pkt->size)) < 0)
        return ret;

    frame_type = get_bits(gb, 16);
    if (frame_type != B_FRAME)
        FFSWAP(AVFrame *, s->frame[1], s->frame[2]);

    if ((ret = ff_reget_buffer(avctx, frame, 0)) < 0)
        return ret;

    disp_id = get_bits_long(gb, 32);
    switch (frame_type) {
    case I_FRAME:
        s->last_i_pts = av_rescale_q(pkt->pts, pkt->time_base, avctx->time_base);
        frame->pict_type = AV_PICTURE_TYPE_I;
        frame->flags |= AV_FRAME_FLAG_KEY;
        ret = decode_iframe(&s->seqobj, gb, frame);
        break;
    case P_FRAME:
        frame->pict_type = AV_PICTURE_TYPE_P;
        if (!s->frame[1]->data[0])
            return AVERROR_INVALIDDATA;
        ret = decode_bframe(&s->seqobj, gb, frame, s->frame[1], frame);
        break;
    case B_FRAME:
        frame->pict_type = AV_PICTURE_TYPE_B;
        if (!s->frame[1]->data[0] ||
            !s->frame[2]->data[0])
            return AVERROR_INVALIDDATA;
        ret = decode_bframe(&s->seqobj, gb, frame, s->frame[1], s->frame[2]);
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    if (ret < 0)
        return ret;

    if (frame_type != I_FRAME) {
        s->frames = av_realloc_f(s->frames, sizeof(*s->frames), s->queued_frames+1);
        if (!s->frames)
            return AVERROR(ENOMEM);

        frame->pts = s->last_i_pts + disp_id * avctx->time_base.num;
        frame->duration = avctx->time_base.num;
        s->frames[s->queued_frames].frame = av_frame_clone(frame);
        if (ret < 0)
            return ret;

        s->frames[s->queued_frames].disp_id = disp_id;
        s->queued_frames++;

        if (frame_type != B_FRAME)
            FFSWAP(AVFrame *, s->frame[0], s->frame[2]);

        *got_frame = 0;

        return AVERROR(EAGAIN);
    }

    frame->pts = s->last_i_pts;

    ret = av_frame_ref(avframe, frame);
    if (ret < 0)
        return ret;

    if (frame_type != B_FRAME)
        FFSWAP(AVFrame *, s->frame[0], s->frame[2]);

    avframe->data[0] += 32 * avframe->linesize[0] + 32;
    avframe->data[1] += 16 * avframe->linesize[1] + 16;
    avframe->data[2] += 16 * avframe->linesize[2] + 16;

    *got_frame = 1;

    return 0;
}

static int compare_by_disp_id(const void *a, const void *b)
{
    const OrderFrame *a2 = a;
    const OrderFrame *b2 = b;

    return a2->disp_id - b2->disp_id;
}

static int flush_item(AVCodecContext *avctx, AVFrame *avframe)
{
    HVQM4Context *s = avctx->priv_data;
    int ret;

    if (s->flushed_frames == 0)
        AV_QSORT(s->frames, s->queued_frames, OrderFrame, compare_by_disp_id);

    {
        AVFrame *frame = s->frames[s->flushed_frames].frame;

        s->flushed_frames++;
        if (s->flushed_frames == s->queued_frames) {
            s->flushed_frames = 0;
        }

        ret = av_frame_ref(avframe, frame);
        if (ret < 0)
            return ret;

        avframe->data[0] += 32 * avframe->linesize[0] + 32;
        avframe->data[1] += 16 * avframe->linesize[1] + 16;
        avframe->data[2] += 16 * avframe->linesize[2] + 16;

        if (s->flushed_frames == 0) {
            for (int i = 0; i < s->queued_frames; i++)
                av_frame_free(&s->frames[i].frame);
            s->queued_frames = 0;
        }

        return 0;
    }
}

static int hvqm4_receive_frame(AVCodecContext *avctx, AVFrame *avframe)
{
    HVQM4Context *s = avctx->priv_data;
    int got_frame = 0, ret;

    if (s->eof && s->queued_frames > 0)
        return flush_item(avctx, avframe);

    if (s->flushed_frames == 0 && !s->pkt->data) {
        ret = ff_decode_get_packet(avctx, s->pkt);
        if (ret < 0) {
            if (ret == AVERROR_EOF && s->queued_frames > 0) {
                s->eof = 1;
                return flush_item(avctx, avframe);
            }
            return ret;
        }

        if (s->pkt->size <= 2) {
            av_packet_unref(s->pkt);
            return AVERROR_INVALIDDATA;
        }

        s->last_frame_type = AV_RB16(s->pkt->data);
    }

    if (s->last_frame_type == I_FRAME && s->queued_frames > 0)
        return flush_item(avctx, avframe);

    ret = hvqm4_decode(avctx, avframe, &got_frame, s->pkt);

    av_packet_unref(s->pkt);

    return ret;
}

static void hvqm4_flush(AVCodecContext *avctx)
{
    HVQM4Context *s = avctx->priv_data;

    for (int i = 0; i < 3; i++)
        av_frame_unref(s->frame[i]);
}

static av_cold int hvqm4_close(AVCodecContext *avctx)
{
    HVQM4Context *s = avctx->priv_data;

    for (int i = 0; i < s->queued_frames && s->frames; i++)
        av_frame_free(&s->frames[i].frame);
    av_freep(&s->frames);

    av_freep(&s->buffer);
    for (int i = 0; i < 6; i++)
        ff_vlc_free(&s->state.vlc[i]);
    for (int i = 0; i < 3; i++)
        av_frame_free(&s->frame[i]);

    return 0;
}

const FFCodec ff_hvqm4_decoder = {
    .p.name         = "hvqm4",
    CODEC_LONG_NAME("HVQM4 Video"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_HVQM4,
    .priv_data_size = sizeof(HVQM4Context),
    .init           = hvqm4_init,
    FF_CODEC_RECEIVE_FRAME_CB(hvqm4_receive_frame),
    .flush          = hvqm4_flush,
    .close          = hvqm4_close,
    .p.capabilities = AV_CODEC_CAP_DR1,
};
