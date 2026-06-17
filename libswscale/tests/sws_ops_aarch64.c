/*
 * Copyright (C) 2026 Ramiro Polla
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

#include <stdio.h>

#include "libavutil/avassert.h"
#include "libavutil/bprint.h"
#include "libavutil/mem.h"
#include "libavutil/tree.h"
#include "libswscale/graph.h"
#include "libswscale/ops.h"
#include "libswscale/ops_chain.h"
#include "libswscale/op_list_gen_template.c"
#include "libswscale/ops_dispatch.h"

#include "libswscale/aarch64/ops_impl_conv.c"

#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif

/*********************************************************************/
static uint16_t clear_to_mask(const SwsClearUOp *clear)
{
    uint16_t mask = 0;
    for (int i = 0; i < 4; i++) {
        if (clear->zero & SWS_COMP(i)) {
            /* no-op */
        } else if (clear->one & SWS_COMP(i)) {
            NIBBLE_SET(mask, i, 1);
        } else {
            NIBBLE_SET(mask, i, 0xf);
        }
    }
    return mask;
}

static uint64_t move_to_mask(const SwsMoveUOp *move)
{
    uint64_t mask = 0;
    for (int i = 0; i < move->num_moves; i++) {
        uint8_t dst = move->dst[i] < 0 ? 0xf : move->dst[i];
        uint8_t src = move->src[i] < 0 ? 0xf : move->src[i];
        uint64_t pair = src | (dst << 4);
        mask |= pair << (i * 8);
    }
    return mask;
}

static uint16_t pack_to_mask(const SwsPackUOp *pack)
{
    uint16_t mask = 0;
    for (int i = 0; i < 4; i++)
        NIBBLE_SET(mask, i, pack->pattern[i]);
    return mask;
}

static uint64_t linear_to_mask(const SwsLinearUOp *linear)
{
    uint64_t mask = 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 5; j++) {
            int jj = (j == 0) ? 4 : (j - 1);
            if (linear->one & SWS_MASK(i, jj))
                mask |= 1ULL << (2 * ((5 * i + j)));
            else if (!(linear->zero & SWS_MASK(i, jj)))
                mask |= 3ULL << (2 * ((5 * i + j)));
        }
    }
    return mask;
}

static uint16_t dither_to_mask(const SwsAArch64OpImplParams *p, const SwsDitherUOp *dither)
{
    uint16_t mask = 0;
    for (int i = 0; i < 4; i++) {
        if (p->mask & SWS_COMP(i)) {
            NIBBLE_SET(mask, i, dither->y_offset[i]);
        } else {
            NIBBLE_SET(mask, i, 0xf);
        }
    }
    return mask;
}

static int aarch64_op_impl_cmp(const void *a, const void *b)
{
    const SwsAArch64OpImplParams *pa = (const SwsAArch64OpImplParams *) a;
    const SwsAArch64OpImplParams *pb = (const SwsAArch64OpImplParams *) b;

    if (pa->uop != pb->uop)
        return (int) pa->uop - pb->uop;

    switch (pa->uop) {
    case SWS_UOP_PERMUTE:
    case SWS_UOP_COPY: {
        uint64_t ia = move_to_mask(&pa->move);
        uint64_t ib = move_to_mask(&pb->move);
        if (ia != ib)
            return (int64_t) (ia - ib) < 0 ? -1 : 1;
        break;
    }
    case SWS_UOP_UNPACK:
    case SWS_UOP_PACK: {
        uint16_t ia = pack_to_mask(&pa->pack);
        uint16_t ib = pack_to_mask(&pb->pack);
        if (ia != ib)
            return (int) ia - ib;
        break;
    }
    case SWS_UOP_LSHIFT:
    case SWS_UOP_RSHIFT:
        if (pa->shift.amount != pb->shift.amount)
            return (int) pa->shift.amount - pb->shift.amount;
        break;
    case SWS_UOP_CLEAR: {
        uint16_t ia = clear_to_mask(&pa->clear);
        uint16_t ib = clear_to_mask(&pb->clear);
        if (ia != ib)
            return (int) ia - ib;
        break;
    }
    case SWS_UOP_LINEAR:
    case SWS_UOP_LINEAR_FMA: {
        uint64_t ia = linear_to_mask(&pa->linear);
        uint64_t ib = linear_to_mask(&pb->linear);
        if (ia != ib)
            return (int64_t) (ia - ib) < 0 ? -1 : 1;
        break;
    }
    case SWS_UOP_DITHER: {
        uint16_t ia = dither_to_mask(pa, &pa->dither);
        uint16_t ib = dither_to_mask(pb, &pb->dither);
        if (ia != ib)
            return (int) ia - ib;
        if (pa->dither.size_log2 != pb->dither.size_log2)
            return (int) pa->dither.size_log2 - pb->dither.size_log2;
        break;
    }
    }

    if (pa->block_size != pb->block_size)
        return (int) pa->block_size - pb->block_size;
    if (pa->type != pb->type)
        return (int) pa->type - pb->type;
    if (pa->mask != pb->mask)
        return (int) pa->mask - pb->mask;

    return 0;
}

/*********************************************************************/
/* Insert the SwsAArch64OpImplParams structure into the AVTreeNode. */
static int aarch64_collect_op(const SwsAArch64OpImplParams *params, struct AVTreeNode **root)
{
    int ret = 0;

    struct AVTreeNode *node = av_tree_node_alloc();
    SwsAArch64OpImplParams *copy = av_memdup(params, sizeof(*params));
    if (!node || !copy) {
        ret = AVERROR(ENOMEM);
        goto error;
    }
    av_tree_insert(root, copy, aarch64_op_impl_cmp, &node);
    if (!node)
        copy = NULL;

error:
    av_free(node);
    av_free(copy);
    return ret;
}

static int collect_ops_compile(SwsContext *ctx, const SwsOpList *ops,
                               SwsCompiledOp *out)
{
    struct AVTreeNode **root = (struct AVTreeNode **) ctx->opaque;
    int ret;

    /* Use at most two full vregs during the widest precision section */
    int block_size = (ff_sws_op_list_max_size(ops) == 4) ? 8 : 16;

    for (int i = 0; i < ops->num_ops; i++) {
        SwsAArch64OpImplParams params = { 0 };
        ret = convert_to_aarch64_impl(ctx, ops, i, block_size, &params);
        if (ret == AVERROR(ENOTSUP))
            continue;
        if (ret < 0)
            goto end;
        ret = aarch64_collect_op(&params, root);
        if (ret < 0)
            goto end;
        if (params.uop == SWS_UOP_LINEAR_FMA) {
            /**
             * Generate both sets of linear op functions that do use
             * and do not use fmla (selected by SWS_BITEXACT).
             */
            params.uop = SWS_UOP_LINEAR;
            ret = aarch64_collect_op(&params, root);
            if (ret < 0)
                goto end;
        }
    }

    *out = (SwsCompiledOp) { 0 };
    ret = 0;

end:
    return ret;
}

static const SwsOpBackend backend_collect = {
    .name    = "collect_ops",
    .compile = collect_ops_compile,
};

/*********************************************************************/
static int register_op(SwsContext *ctx, void *opaque, SwsOpList *ops)
{
    /* Skip ops lists which include filtering, since this is still not
     * supported. */
    for (int i = 0; i < ops->num_ops; i++) {
        const SwsOp *op = &ops->ops[i];
        switch (op->op) {
        case SWS_OP_READ:
        case SWS_OP_WRITE:
            if (op->rw.filter.op)
                return 0;
            break;
        case SWS_OP_FILTER_H:
        case SWS_OP_FILTER_V:
            return 0;
        }
    }

    /* ff_sws_compile_pass() takes over ownership of `ops` */
    SwsOpList *copy = ff_sws_op_list_duplicate(ops);
    if (!copy)
        return AVERROR(ENOMEM);

    const int flags = SWS_OP_FLAG_DRY_RUN | SWS_OP_FLAG_SPLIT_MEMCPY;
    return ff_sws_compile_pass(opaque, &backend_collect, &copy, flags, NULL, NULL);
}

/*********************************************************************/
static const char op_type_names[SWS_UOP_TYPE_NB][16] = {
    [SWS_UOP_READ_BIT      ] = "read_bit",
    [SWS_UOP_READ_NIBBLE   ] = "read_nibble",
    [SWS_UOP_READ_PACKED   ] = "read_packed",
    [SWS_UOP_READ_PLANAR   ] = "read_planar",
    [SWS_UOP_WRITE_BIT     ] = "write_bit",
    [SWS_UOP_WRITE_NIBBLE  ] = "write_nibble",
    [SWS_UOP_WRITE_PACKED  ] = "write_packed",
    [SWS_UOP_WRITE_PLANAR  ] = "write_planar",
    [SWS_UOP_SWAP_BYTES    ] = "swap_bytes",
    [SWS_UOP_PERMUTE       ] = "permute",
    [SWS_UOP_COPY          ] = "copy",
    [SWS_UOP_UNPACK        ] = "unpack",
    [SWS_UOP_PACK          ] = "pack",
    [SWS_UOP_LSHIFT        ] = "lshift",
    [SWS_UOP_RSHIFT        ] = "rshift",
    [SWS_UOP_CLEAR         ] = "clear",
    [SWS_UOP_TO_U8         ] = "to_u8",
    [SWS_UOP_TO_U16        ] = "to_u16",
    [SWS_UOP_TO_U32        ] = "to_u32",
    [SWS_UOP_TO_F32        ] = "to_f32",
    [SWS_UOP_EXPAND_PAIR   ] = "expand_pair",
    [SWS_UOP_EXPAND_QUAD   ] = "expand_quad",
    [SWS_UOP_MIN           ] = "min",
    [SWS_UOP_MAX           ] = "max",
    [SWS_UOP_SCALE         ] = "scale",
    [SWS_UOP_LINEAR        ] = "linear",
    [SWS_UOP_LINEAR_FMA    ] = "linear_fma",
    [SWS_UOP_DITHER        ] = "dither",
};

static const char pixel_type_names[SWS_PIXEL_TYPE_NB][4] = {
    [SWS_PIXEL_U8 ] = "u8",
    [SWS_PIXEL_U16] = "u16",
    [SWS_PIXEL_U32] = "u32",
    [SWS_PIXEL_F32] = "f32",
};

static void impl_func_name(AVBPrint *bp, const SwsAArch64OpImplParams *params)
{
    av_bprintf(bp, "ff_sws_%s", op_type_names[params->uop]);
    switch (params->uop) {
    case SWS_UOP_PERMUTE:
    case SWS_UOP_COPY:
        av_bprintf(bp, "_%012" PRIx64, move_to_mask(&params->move));
        break;
    case SWS_UOP_UNPACK:
    case SWS_UOP_PACK:
        av_bprintf(bp, "_%04x", pack_to_mask(&params->pack));
        break;
    case SWS_UOP_LSHIFT:
    case SWS_UOP_RSHIFT:
        av_bprintf(bp, "_%u", params->shift.amount);
        break;
    case SWS_UOP_CLEAR:
        av_bprintf(bp, "_%04x", clear_to_mask(&params->clear));
        break;
    case SWS_UOP_LINEAR:
    case SWS_UOP_LINEAR_FMA:
        av_bprintf(bp, "_%010" PRIx64, linear_to_mask(&params->linear));
        break;
    case SWS_UOP_DITHER:
        av_bprintf(bp, "_%04x_%u", dither_to_mask(params, &params->dither), params->dither.size_log2);
        break;
    }
    av_bprintf(bp, "_%u_%s_%04x_neon", params->block_size, pixel_type_names[params->type], nibble_mask(params->mask));
}

static const char op_types[SWS_UOP_TYPE_NB][32] = {
    [SWS_UOP_READ_BIT      ] = "SWS_UOP_READ_BIT",
    [SWS_UOP_READ_NIBBLE   ] = "SWS_UOP_READ_NIBBLE",
    [SWS_UOP_READ_PACKED   ] = "SWS_UOP_READ_PACKED",
    [SWS_UOP_READ_PLANAR   ] = "SWS_UOP_READ_PLANAR",
    [SWS_UOP_WRITE_BIT     ] = "SWS_UOP_WRITE_BIT",
    [SWS_UOP_WRITE_NIBBLE  ] = "SWS_UOP_WRITE_NIBBLE",
    [SWS_UOP_WRITE_PACKED  ] = "SWS_UOP_WRITE_PACKED",
    [SWS_UOP_WRITE_PLANAR  ] = "SWS_UOP_WRITE_PLANAR",
    [SWS_UOP_SWAP_BYTES    ] = "SWS_UOP_SWAP_BYTES",
    [SWS_UOP_PERMUTE       ] = "SWS_UOP_PERMUTE",
    [SWS_UOP_COPY          ] = "SWS_UOP_COPY",
    [SWS_UOP_UNPACK        ] = "SWS_UOP_UNPACK",
    [SWS_UOP_PACK          ] = "SWS_UOP_PACK",
    [SWS_UOP_LSHIFT        ] = "SWS_UOP_LSHIFT",
    [SWS_UOP_RSHIFT        ] = "SWS_UOP_RSHIFT",
    [SWS_UOP_CLEAR         ] = "SWS_UOP_CLEAR",
    [SWS_UOP_TO_U8         ] = "SWS_UOP_TO_U8",
    [SWS_UOP_TO_U16        ] = "SWS_UOP_TO_U16",
    [SWS_UOP_TO_U32        ] = "SWS_UOP_TO_U32",
    [SWS_UOP_TO_F32        ] = "SWS_UOP_TO_F32",
    [SWS_UOP_EXPAND_PAIR   ] = "SWS_UOP_EXPAND_PAIR",
    [SWS_UOP_EXPAND_QUAD   ] = "SWS_UOP_EXPAND_QUAD",
    [SWS_UOP_MIN           ] = "SWS_UOP_MIN",
    [SWS_UOP_MAX           ] = "SWS_UOP_MAX",
    [SWS_UOP_SCALE         ] = "SWS_UOP_SCALE",
    [SWS_UOP_LINEAR        ] = "SWS_UOP_LINEAR",
    [SWS_UOP_LINEAR_FMA    ] = "SWS_UOP_LINEAR_FMA",
    [SWS_UOP_DITHER        ] = "SWS_UOP_DITHER",
};

static const char pixel_types[SWS_PIXEL_TYPE_NB][32] = {
    [SWS_PIXEL_U8 ] = "SWS_PIXEL_U8",
    [SWS_PIXEL_U16] = "SWS_PIXEL_U16",
    [SWS_PIXEL_U32] = "SWS_PIXEL_U32",
    [SWS_PIXEL_F32] = "SWS_PIXEL_F32",
};

static void serialize_op(AVBPrint *bp, const SwsAArch64OpImplParams *params)
{
    av_bprintf(bp, "ENTRY(");
    impl_func_name(bp, params);
    av_bprintf(bp, ", { .uop = %s", op_types[params->uop]);
    switch (params->uop) {
    case SWS_UOP_PERMUTE:
    case SWS_UOP_COPY:
        av_bprintf(bp, ", .move = { .num_moves = %d, .dst = {%d, %d, %d, %d, %d, %d}, .src = {%d, %d, %d, %d, %d, %d} }",
                   params->move.num_moves,
                   params->move.dst[0], params->move.dst[1], params->move.dst[2],
                   params->move.dst[3], params->move.dst[4], params->move.dst[5],
                   params->move.src[0], params->move.src[1], params->move.src[2],
                   params->move.src[3], params->move.src[4], params->move.src[5]);
        break;
    case SWS_UOP_UNPACK:
    case SWS_UOP_PACK:
        av_bprintf(bp, ", .pack = { .pattern = {%d, %d, %d, %d} }",
                   params->pack.pattern[0], params->pack.pattern[1],
                   params->pack.pattern[2], params->pack.pattern[3]);
        break;
    case SWS_UOP_LSHIFT:
    case SWS_UOP_RSHIFT:
        av_bprintf(bp, ", .shift = { .amount = %u }", params->shift.amount);
        break;
    case SWS_UOP_CLEAR:
        av_bprintf(bp, ", .clear = { .one = 0x%0x, .zero = 0x%0x }", params->clear.one, params->clear.zero);
        break;
    case SWS_UOP_LINEAR:
    case SWS_UOP_LINEAR_FMA:
        av_bprintf(bp, ", .linear = { .one = 0x%x, .zero = 0x%x }", params->linear.one, params->linear.zero);
        break;
    case SWS_UOP_DITHER:
        av_bprintf(bp, ", .dither = { .y_offset = {%u, %u, %u, %u}, .size_log2 = %u }",
                   params->dither.y_offset[0], params->dither.y_offset[1],
                   params->dither.y_offset[2], params->dither.y_offset[3],
                   params->dither.size_log2);
        break;
    }
    av_bprintf(bp, ", .block_size = %u, .type = %s, .mask = 0x%x })", params->block_size, pixel_types[params->type], params->mask);
}

/* Serialize SwsAArch64OpImplParams for one function. */
static int print_op(void *opaque, void *elem)
{
    SwsAArch64OpImplParams *params = (SwsAArch64OpImplParams *) elem;
    FILE *fp = (FILE *) opaque;

    AVBPrint bp;
    av_bprint_init(&bp, 0, AV_BPRINT_SIZE_UNLIMITED);
    serialize_op(&bp, params);
    fprintf(fp, "%s\n", bp.str);
    av_bprint_finalize(&bp, NULL);

    av_free(params);

    return 0;
}

/*********************************************************************/
int main(int argc, char *argv[])
{
    struct AVTreeNode *root = NULL;
    int ret = 1;

#ifdef _WIN32
    _setmode(_fileno(stdout), _O_BINARY);
#endif

    SwsContext *ctx = sws_alloc_context();
    if (!ctx)
        goto fail;

    SwsGraph *graph = ff_sws_graph_alloc();
    if (!graph)
        goto fail;

    graph->ctx = ctx;
    ctx->opaque = &root;

    ret = ff_sws_enum_op_lists(ctx, graph, AV_PIX_FMT_NONE, AV_PIX_FMT_NONE,
                               register_op);

    /**
     * Generate a C file with all the unique function parameter entries
     * collected by aarch64_enum_ops().
     */
    printf("/*\n");
    printf(" * This file is automatically generated. Do not edit manually.\n");
    printf(" * To regenerate, run: make fate-sws-ops-entries-aarch64 GEN=1\n");
    printf(" */\n");
    printf("\n");
    av_tree_enumerate(root, stdout, NULL, print_op);

    ff_sws_graph_free(&graph);

fail:
    av_tree_destroy(root);
    sws_free_context(&ctx);
    return ret;
}
