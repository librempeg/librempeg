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
static int aarch64_op_impl_cmp(const void *a, const void *b)
{
    const SwsAArch64OpImplParams *pa = (const SwsAArch64OpImplParams *) a;
    const SwsAArch64OpImplParams *pb = (const SwsAArch64OpImplParams *) b;

    if (pa->op != pb->op)
        return (int) pa->op - pb->op;

    switch (pa->op) {
    case AARCH64_SWS_OP_PERMUTE:
    case AARCH64_SWS_OP_COPY:
        if (pa->move != pb->move)
            return (int64_t) (pa->move - pb->move) < 0 ? -1 : 1;
        break;
    case AARCH64_SWS_OP_UNPACK:
    case AARCH64_SWS_OP_PACK:
        if (pa->pack != pb->pack)
            return (int) pa->pack - pb->pack;
        break;
    case AARCH64_SWS_OP_LSHIFT:
    case AARCH64_SWS_OP_RSHIFT:
        if (pa->shift != pb->shift)
            return (int) pa->shift - pb->shift;
        break;
    case AARCH64_SWS_OP_CLEAR:
        if (pa->clear != pb->clear)
            return (int) pa->clear - pb->clear;
        break;
    case AARCH64_SWS_OP_LINEAR:
    case AARCH64_SWS_OP_LINEAR_FMA:
        if (pa->linear.mask != pb->linear.mask)
            return (int64_t) (pa->linear.mask - pb->linear.mask) < 0 ? -1 : 1;
        break;
    case AARCH64_SWS_OP_DITHER:
        if (pa->dither.y_offset != pb->dither.y_offset)
            return (int) pa->dither.y_offset - pb->dither.y_offset;
        if (pa->dither.size_log2 != pb->dither.size_log2)
            return (int) pa->dither.size_log2 - pb->dither.size_log2;
        break;
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
        if (params.op == AARCH64_SWS_OP_LINEAR_FMA) {
            /**
             * Generate both sets of linear op functions that do use
             * and do not use fmla (selected by SWS_BITEXACT).
             */
            params.op = AARCH64_SWS_OP_LINEAR;
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
static const char op_type_names[AARCH64_SWS_OP_TYPE_NB][16] = {
    [AARCH64_SWS_OP_READ_BIT      ] = "read_bit",
    [AARCH64_SWS_OP_READ_NIBBLE   ] = "read_nibble",
    [AARCH64_SWS_OP_READ_PACKED   ] = "read_packed",
    [AARCH64_SWS_OP_READ_PLANAR   ] = "read_planar",
    [AARCH64_SWS_OP_WRITE_BIT     ] = "write_bit",
    [AARCH64_SWS_OP_WRITE_NIBBLE  ] = "write_nibble",
    [AARCH64_SWS_OP_WRITE_PACKED  ] = "write_packed",
    [AARCH64_SWS_OP_WRITE_PLANAR  ] = "write_planar",
    [AARCH64_SWS_OP_SWAP_BYTES    ] = "swap_bytes",
    [AARCH64_SWS_OP_PERMUTE       ] = "permute",
    [AARCH64_SWS_OP_COPY          ] = "copy",
    [AARCH64_SWS_OP_UNPACK        ] = "unpack",
    [AARCH64_SWS_OP_PACK          ] = "pack",
    [AARCH64_SWS_OP_LSHIFT        ] = "lshift",
    [AARCH64_SWS_OP_RSHIFT        ] = "rshift",
    [AARCH64_SWS_OP_CLEAR         ] = "clear",
    [AARCH64_SWS_OP_TO_U8         ] = "to_u8",
    [AARCH64_SWS_OP_TO_U16        ] = "to_u16",
    [AARCH64_SWS_OP_TO_U32        ] = "to_u32",
    [AARCH64_SWS_OP_TO_F32        ] = "to_f32",
    [AARCH64_SWS_OP_EXPAND_PAIR   ] = "expand_pair",
    [AARCH64_SWS_OP_EXPAND_QUAD   ] = "expand_quad",
    [AARCH64_SWS_OP_MIN           ] = "min",
    [AARCH64_SWS_OP_MAX           ] = "max",
    [AARCH64_SWS_OP_SCALE         ] = "scale",
    [AARCH64_SWS_OP_LINEAR        ] = "linear",
    [AARCH64_SWS_OP_LINEAR_FMA    ] = "linear_fma",
    [AARCH64_SWS_OP_DITHER        ] = "dither",
};

static const char pixel_type_names[AARCH64_PIXEL_TYPE_NB][4] = {
    [AARCH64_PIXEL_U8 ] = "u8",
    [AARCH64_PIXEL_U16] = "u16",
    [AARCH64_PIXEL_U32] = "u32",
    [AARCH64_PIXEL_F32] = "f32",
};

static void impl_func_name(AVBPrint *bp, const SwsAArch64OpImplParams *params)
{
    av_bprintf(bp, "ff_sws_%s", op_type_names[params->op]);
    switch (params->op) {
    case AARCH64_SWS_OP_PERMUTE:
    case AARCH64_SWS_OP_COPY:
        av_bprintf(bp, "_%012" PRIx64, params->move);
        break;
    case AARCH64_SWS_OP_UNPACK:
    case AARCH64_SWS_OP_PACK:
        av_bprintf(bp, "_%04x", params->pack);
        break;
    case AARCH64_SWS_OP_LSHIFT:
    case AARCH64_SWS_OP_RSHIFT:
        av_bprintf(bp, "_%u", params->shift);
        break;
    case AARCH64_SWS_OP_CLEAR:
        av_bprintf(bp, "_%04x", params->clear);
        break;
    case AARCH64_SWS_OP_LINEAR:
    case AARCH64_SWS_OP_LINEAR_FMA:
        av_bprintf(bp, "_%010" PRIx64, params->linear.mask);
        break;
    case AARCH64_SWS_OP_DITHER:
        av_bprintf(bp, "_%04x_%u", params->dither.y_offset, params->dither.size_log2);
        break;
    }
    av_bprintf(bp, "_%u_%s_%04x_neon", params->block_size, pixel_type_names[params->type], params->mask);
}

static const char op_types[AARCH64_SWS_OP_TYPE_NB][32] = {
    [AARCH64_SWS_OP_READ_BIT      ] = "AARCH64_SWS_OP_READ_BIT",
    [AARCH64_SWS_OP_READ_NIBBLE   ] = "AARCH64_SWS_OP_READ_NIBBLE",
    [AARCH64_SWS_OP_READ_PACKED   ] = "AARCH64_SWS_OP_READ_PACKED",
    [AARCH64_SWS_OP_READ_PLANAR   ] = "AARCH64_SWS_OP_READ_PLANAR",
    [AARCH64_SWS_OP_WRITE_BIT     ] = "AARCH64_SWS_OP_WRITE_BIT",
    [AARCH64_SWS_OP_WRITE_NIBBLE  ] = "AARCH64_SWS_OP_WRITE_NIBBLE",
    [AARCH64_SWS_OP_WRITE_PACKED  ] = "AARCH64_SWS_OP_WRITE_PACKED",
    [AARCH64_SWS_OP_WRITE_PLANAR  ] = "AARCH64_SWS_OP_WRITE_PLANAR",
    [AARCH64_SWS_OP_SWAP_BYTES    ] = "AARCH64_SWS_OP_SWAP_BYTES",
    [AARCH64_SWS_OP_PERMUTE       ] = "AARCH64_SWS_OP_PERMUTE",
    [AARCH64_SWS_OP_COPY          ] = "AARCH64_SWS_OP_COPY",
    [AARCH64_SWS_OP_UNPACK        ] = "AARCH64_SWS_OP_UNPACK",
    [AARCH64_SWS_OP_PACK          ] = "AARCH64_SWS_OP_PACK",
    [AARCH64_SWS_OP_LSHIFT        ] = "AARCH64_SWS_OP_LSHIFT",
    [AARCH64_SWS_OP_RSHIFT        ] = "AARCH64_SWS_OP_RSHIFT",
    [AARCH64_SWS_OP_CLEAR         ] = "AARCH64_SWS_OP_CLEAR",
    [AARCH64_SWS_OP_TO_U8         ] = "AARCH64_SWS_OP_TO_U8",
    [AARCH64_SWS_OP_TO_U16        ] = "AARCH64_SWS_OP_TO_U16",
    [AARCH64_SWS_OP_TO_U32        ] = "AARCH64_SWS_OP_TO_U32",
    [AARCH64_SWS_OP_TO_F32        ] = "AARCH64_SWS_OP_TO_F32",
    [AARCH64_SWS_OP_EXPAND_PAIR   ] = "AARCH64_SWS_OP_EXPAND_PAIR",
    [AARCH64_SWS_OP_EXPAND_QUAD   ] = "AARCH64_SWS_OP_EXPAND_QUAD",
    [AARCH64_SWS_OP_MIN           ] = "AARCH64_SWS_OP_MIN",
    [AARCH64_SWS_OP_MAX           ] = "AARCH64_SWS_OP_MAX",
    [AARCH64_SWS_OP_SCALE         ] = "AARCH64_SWS_OP_SCALE",
    [AARCH64_SWS_OP_LINEAR        ] = "AARCH64_SWS_OP_LINEAR",
    [AARCH64_SWS_OP_LINEAR_FMA    ] = "AARCH64_SWS_OP_LINEAR_FMA",
    [AARCH64_SWS_OP_DITHER        ] = "AARCH64_SWS_OP_DITHER",
};

static const char pixel_types[AARCH64_PIXEL_TYPE_NB][32] = {
    [AARCH64_PIXEL_U8 ] = "AARCH64_PIXEL_U8",
    [AARCH64_PIXEL_U16] = "AARCH64_PIXEL_U16",
    [AARCH64_PIXEL_U32] = "AARCH64_PIXEL_U32",
    [AARCH64_PIXEL_F32] = "AARCH64_PIXEL_F32",
};

static void serialize_op(AVBPrint *bp, const SwsAArch64OpImplParams *params)
{
    av_bprintf(bp, "ENTRY(");
    impl_func_name(bp, params);
    av_bprintf(bp, ", { .op = %s", op_types[params->op]);
    switch (params->op) {
    case AARCH64_SWS_OP_PERMUTE:
    case AARCH64_SWS_OP_COPY:
        av_bprintf(bp, ", .move = 0x%012" PRIx64 "ULL", params->move);
        break;
    case AARCH64_SWS_OP_UNPACK:
    case AARCH64_SWS_OP_PACK:
        av_bprintf(bp, ", .pack = 0x%04x", params->pack);
        break;
    case AARCH64_SWS_OP_LSHIFT:
    case AARCH64_SWS_OP_RSHIFT:
        av_bprintf(bp, ", .shift = %u", params->shift);
        break;
    case AARCH64_SWS_OP_CLEAR:
        av_bprintf(bp, ", .clear = 0x%04x", params->clear);
        break;
    case AARCH64_SWS_OP_LINEAR:
    case AARCH64_SWS_OP_LINEAR_FMA:
        av_bprintf(bp, ", .linear.mask = 0x%010" PRIx64 "ULL", params->linear.mask);
        break;
    case AARCH64_SWS_OP_DITHER:
        av_bprintf(bp, ", .dither.y_offset = 0x%04x, .dither.size_log2 = %u", params->dither.y_offset, params->dither.size_log2);
        break;
    }
    av_bprintf(bp, ", .block_size = %u, .type = %s, .mask = 0x%04x })", params->block_size, pixel_types[params->type], params->mask);
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
