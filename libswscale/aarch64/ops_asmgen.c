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

#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif

/**
 * This file is compiled as a standalone build-time tool and must not depend
 * on internal FFmpeg libraries. The necessary utils are redefined below using
 * standard C equivalents.
 */

#define AVUTIL_AVASSERT_H
#define AVUTIL_LOG_H
#define AVUTIL_MACROS_H
#define AVUTIL_MEM_H
#define av_assert0(cond) assert(cond)
#define av_malloc(s)     malloc(s)
#define av_mallocz(s)    calloc(1, s)
#define av_realloc(p, s) realloc(p, s)
#define av_strdup(s)     strdup(s)
#define av_free(p)       free(p)
#define FFMAX(a,b) ((a) > (b) ? (a) : (b))
#define FFMIN(a,b) ((a) > (b) ? (b) : (a))
#define MKTAG(a,b,c,d) ((a) | ((b) << 8) | ((c) << 16) | ((unsigned)(d) << 24))

static void av_freep(void *ptr)
{
    void **pptr = (void **) ptr;
    if (pptr) {
        ptr = *pptr;
        if (ptr)
            free(ptr);
        *pptr = NULL;
    }
}

static void *av_memdup(const void *p, size_t size)
{
    void *ptr = NULL;
    if (p) {
        ptr = av_malloc(size);
        if (ptr)
            memcpy(ptr, p, size);
    }
    return ptr;
}

#include "libavutil/dynarray.h"

static void *av_dynarray2_add(void **tab_ptr, int *nb_ptr, size_t elem_size,
                              const uint8_t *elem_data)
{
    uint8_t *tab_elem_data = NULL;

    FF_DYNARRAY_ADD(INT_MAX, elem_size, *tab_ptr, *nb_ptr, {
        tab_elem_data = (uint8_t *)*tab_ptr + (*nb_ptr) * elem_size;
        if (elem_data)
            memcpy(tab_elem_data, elem_data, elem_size);
    }, {
        av_freep(tab_ptr);
        *nb_ptr = 0;
    });
    return tab_elem_data;
}

#include "libavutil/bprint.c"

/*********************************************************************/
#include "rasm.c"
#include "rasm_print.c"
#include "ops_impl.h"

/**
 * Implementation parameters for all exported functions. This list is
 * compiled by performing a dummy run of all conversions in sws_ops and
 * collecting all functions that need to be generated. This is achieved
 * by running:
 *   make fate-sws-ops-entries-aarch64 GEN=1
 */
typedef struct SwsAArch64OpEntry {
    const char *name;
    SwsAArch64OpImplParams params;
} SwsAArch64OpEntry;

static const SwsAArch64OpEntry ops_entries[] = {
#define ENTRY(fname, ...) { .name = #fname, .params = __VA_ARGS__ },
#include "ops_entries.c"
#undef ENTRY
    { NULL }
};

/*********************************************************************/
typedef struct SwsAArch64Context {
    RasmContext *rctx;

    /* SwsOpFunc arguments. */
    RasmOp exec;
    RasmOp impl;
    RasmOp bx_start;
    RasmOp y_start;
    RasmOp bx_end;
    RasmOp y_end;

    /* Loop iterator variables. */
    RasmOp bx;
    RasmOp y;

    /* Scratch registers. */
    RasmOp tmp0;
    RasmOp tmp1;

    /* CPS-related variables. */
    RasmOp op0_func;
    RasmOp op1_impl;
    RasmOp cont;
    RasmNode *load_cont_node;

    /* Vector registers. Two banks (low and high) are used. */
    RasmOp vl[ 4];
    RasmOp vh[ 4];
    RasmOp vt[12];

    /* Read/Write data pointers and padding. */
    RasmOp in[4];
    RasmOp out[4];
    RasmOp in_bump[4];
    RasmOp out_bump[4];

    /* Vector register dimensions. */
    size_t el_size;
    size_t el_count;
    size_t vec_size;
    bool use_vh;
} SwsAArch64Context;

/*********************************************************************/
/* Helpers functions. */

/* Looping when s->use_vh is set. */
#define LOOP_VH(s, mask, idx) if (s->use_vh) LOOP(mask, idx)
#define LOOP_MASK_VH(s, p, idx) if (s->use_vh) LOOP_MASK(p, idx)
#define LOOP_MASK_BWD_VH(s, p, idx) if (s->use_vh) LOOP_MASK_BWD(p, idx)

/* Inline rasm comments. */
#define CMT(comment)   rasm_annotate(r, comment)
#define CMTF(fmt, ...) rasm_annotatef(r, (char[128]){0}, 128, fmt, __VA_ARGS__)

/* Reshape all vector registers for current SwsOp. */
static void reshape_all_vectors(SwsAArch64Context *s, int el_count, int el_size)
{
    s->vl[ 0] = a64op_make_vec( 0, el_count, el_size);
    s->vl[ 1] = a64op_make_vec( 1, el_count, el_size);
    s->vl[ 2] = a64op_make_vec( 2, el_count, el_size);
    s->vl[ 3] = a64op_make_vec( 3, el_count, el_size);
    s->vh[ 0] = a64op_make_vec( 4, el_count, el_size);
    s->vh[ 1] = a64op_make_vec( 5, el_count, el_size);
    s->vh[ 2] = a64op_make_vec( 6, el_count, el_size);
    s->vh[ 3] = a64op_make_vec( 7, el_count, el_size);
    s->vt[ 0] = a64op_make_vec(16, el_count, el_size);
    s->vt[ 1] = a64op_make_vec(17, el_count, el_size);
    s->vt[ 2] = a64op_make_vec(18, el_count, el_size);
    s->vt[ 3] = a64op_make_vec(19, el_count, el_size);
    s->vt[ 4] = a64op_make_vec(20, el_count, el_size);
    s->vt[ 5] = a64op_make_vec(21, el_count, el_size);
    s->vt[ 6] = a64op_make_vec(22, el_count, el_size);
    s->vt[ 7] = a64op_make_vec(23, el_count, el_size);
    s->vt[ 8] = a64op_make_vec(24, el_count, el_size);
    s->vt[ 9] = a64op_make_vec(25, el_count, el_size);
    s->vt[10] = a64op_make_vec(26, el_count, el_size);
    s->vt[11] = a64op_make_vec(27, el_count, el_size);
}

/*********************************************************************/
/* Function frame */

static unsigned clobbered_frame_size(unsigned n)
{
    return ((n + 1) >> 1) * 16;
}

static void asmgen_prologue(SwsAArch64Context *s, const RasmOp *regs, unsigned n)
{
    RasmContext *r = s->rctx;
    RasmOp sp = a64op_sp();
    unsigned frame_size = clobbered_frame_size(n);
    RasmOp sp_pre = a64op_pre(sp, -frame_size);

    rasm_add_comment(r, "prologue");
    if (n == 0) {
        /* no-op */
    } else if (n == 1) {
        i_str(r, regs[0], sp_pre);
    } else {
        i_stp(r, regs[0], regs[1], sp_pre);
        for (unsigned i = 2; i + 1 < n; i += 2)
            i_stp(r, regs[i],     regs[i + 1], a64op_off(sp, i * sizeof(uint64_t)));
        if (n & 1)
            i_str(r, regs[n - 1],              a64op_off(sp, (n - 1) * sizeof(uint64_t)));
    }
}

static void asmgen_epilogue(SwsAArch64Context *s, const RasmOp *regs, unsigned n)
{
    RasmContext *r = s->rctx;
    RasmOp sp = a64op_sp();
    unsigned frame_size = clobbered_frame_size(n);
    RasmOp sp_post = a64op_post(sp, frame_size);

    rasm_add_comment(r, "epilogue");
    if (n == 0) {
        /* no-op */
    } else if (n == 1) {
        i_ldr(r, regs[0], sp_post);
    } else {
        if (n & 1)
            i_ldr(r, regs[n - 1],              a64op_off(sp, (n - 1) * sizeof(uint64_t)));
        for (unsigned i = (n & ~1u) - 2; i >= 2; i -= 2)
            i_ldp(r, regs[i],     regs[i + 1], a64op_off(sp, i * sizeof(uint64_t)));
        i_ldp(r, regs[0], regs[1], sp_post);
    }
}

/*********************************************************************/
/* Callee-saved registers (r19-r28, fp, and lr). */
#define MAX_SAVED_REGS 12

static void clobber_gpr(RasmOp regs[MAX_SAVED_REGS], unsigned *count,
                        RasmOp gpr)
{
    const int n = a64op_gpr_n(gpr);
    if (n >= 19 && n <= 30)
        regs[(*count)++] = gpr;
}

static unsigned clobbered_gprs(const SwsAArch64Context *s,
                               SwsCompMask mask,
                               RasmOp regs[MAX_SAVED_REGS])
{
    unsigned count = 0;
    clobber_gpr(regs, &count, a64op_lr());
    LOOP(mask, i) {
        clobber_gpr(regs, &count, s->in[i]);
        clobber_gpr(regs, &count, s->out[i]);
        clobber_gpr(regs, &count, s->in_bump[i]);
        clobber_gpr(regs, &count, s->out_bump[i]);
    }
    return count;
}

static void asmgen_process(SwsAArch64Context *s, SwsCompMask mask)
{
    RasmContext *r = s->rctx;
    char func_name[128];
    char buf[64];

    /**
     * The process function for aarch64 works similarly to the x86 backend.
     * The description in x86/ops_include.asm mostly holds as well here.
     */

    snprintf(func_name, sizeof(func_name), "ff_sws_process_%04x_neon", nibble_mask(mask));

    rasm_func_begin(r, func_name, true, false);

    /* Function prologue */
    RasmOp saved_regs[MAX_SAVED_REGS];
    unsigned nsaved = clobbered_gprs(s, mask, saved_regs);
    if (nsaved)
        asmgen_prologue(s, saved_regs, nsaved);

    /* Load values from impl. */
    i_ldr(r, s->op0_func, a64op_off(s->impl, offsetof_impl_cont));  CMT("SwsFuncPtr op0_func = impl->cont;");
    i_add(r, s->op1_impl, s->impl, IMM(sizeof_impl));               CMT("SwsOpImpl *op1_impl = impl + 1;");

    /* Load values from exec. */
    LOOP(mask, i) {
        rasm_annotate_nextf(r, buf, sizeof(buf), "in[%u] = exec->in[%u];", i, i);
        i_ldr(r, s->in[i],       a64op_off(s->exec, offsetof_exec_in       + (i * sizeof(uint8_t *))));
    }
    LOOP(mask, i) {
        rasm_annotate_nextf(r, buf, sizeof(buf), "out[%u] = exec->out[%u];", i, i);
        i_ldr(r, s->out[i],      a64op_off(s->exec, offsetof_exec_out      + (i * sizeof(uint8_t *))));
    }
    LOOP(mask, i) {
        rasm_annotate_nextf(r, buf, sizeof(buf), "in_bump[%u] = exec->in_bump[%u];", i, i);
        i_ldr(r, s->in_bump[i],  a64op_off(s->exec, offsetof_exec_in_bump  + (i * sizeof(ptrdiff_t))));
    }
    LOOP(mask, i) {
        rasm_annotate_nextf(r, buf, sizeof(buf), "out_bump[%u] = exec->out_bump[%u];", i, i);
        i_ldr(r, s->out_bump[i], a64op_off(s->exec, offsetof_exec_out_bump + (i * sizeof(ptrdiff_t))));
    }

    int first_row  = rasm_new_label(r, NULL);
    int next_row   = rasm_new_label(r, NULL);
    int next_block = rasm_new_label(r, NULL);

    /* Jump to first row (skips padding). */
    i_b  (r, rasm_op_label(first_row));     CMT("goto first_row;");

    /* Perform padding, preparing for next row. */
    rasm_add_label(r, next_row);            CMT("next_row:");
    LOOP(mask, i) { i_add(r, s->in[i],  s->in[i],  s->in_bump[i]);  CMTF("in[%u] += in_bump[%u];", i, i); }
    LOOP(mask, i) { i_add(r, s->out[i], s->out[i], s->out_bump[i]); CMTF("out[%u] += out_bump[%u];", i, i); }

    /* First row (reset x). */
    rasm_add_label(r, first_row);           CMT("first_row:");
    i_mov(r, s->bx, s->bx_start);           CMT("bx = bx_start;");

    /* Reset impl and call first kernel. */
    rasm_add_label(r, next_block);          CMT("next_block:");
    i_mov(r, s->impl, s->op1_impl);         CMT("impl = op1_impl;");
    i_blr(r, s->op0_func);                  CMT("op0_func();");

    /* Perform horizontal loop. */
    i_add(r, s->bx, s->bx, IMM(1));         CMT("bx += 1;");
    i_cmp(r, s->bx, s->bx_end);             CMT("if (bx != bx_end)");
    i_bne(r, next_block);                   CMT("    goto next_block;");

    /* Perform vertical loop. */
    i_add(r, s->y, s->y, IMM(1));           CMT("y += 1;");
    i_cmp(r, s->y, s->y_end);               CMT("if (y != y_end)");
    i_bne(r, next_row);                     CMT("    goto next_row;");

    /* Function epilogue */
    if (nsaved)
        asmgen_epilogue(s, saved_regs, nsaved);

    i_ret(r);
}

/*********************************************************************/
/**
 * Set node where the continuation address will be loaded and impl will
 * be incremented. This should be done right after impl->priv has been
 * used.
 */
static void asmgen_set_load_cont_node(SwsAArch64Context *s)
{
    RasmContext *r = s->rctx;
    s->load_cont_node = rasm_get_current_node(r);
}

/*********************************************************************/
/* gather raw pixels from planes */
/* SWS_UOP_READ_BIT */
/* SWS_UOP_READ_NIBBLE */
/* SWS_UOP_READ_PACKED */
/* SWS_UOP_READ_PLANAR */

static void asmgen_op_read_bit(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    RasmOp bitmask_vec = s->vt[1];
    RasmOp wtmp = a64op_w(s->tmp0);
    AArch64VecViews vl[1];
    AArch64VecViews vtmp;
    AArch64VecViews shift_vec;

    a64op_vec_views(s->vt[0], &shift_vec);
    a64op_vec_views(s->vl[0], &vl[0]);
    a64op_vec_views(s->vt[2], &vtmp);

    /* Note that shift_vec has negative values, so that using it with
     * ushl actually performs a right shift. */
    rasm_annotate_next(r, "v128 shift_vec = impl->priv.v128;");
    i_ldr(r, shift_vec.q, a64op_off(s->impl, offsetof_impl_priv));
    asmgen_set_load_cont_node(s);

    if (p->block_size == 16) {
        i_ldrh(r, wtmp,        a64op_post(s->in[0], 2));    CMT("uint16_t tmp = *in[0]++;");
        i_movi(r, bitmask_vec, IMM(1));                     CMT("v128 bitmask_vec = {1 <repeats 16 times>};");
        i_dup (r, vl[0].b8,    wtmp);                       CMT("vl[0].lo = broadcast(tmp);");
        i_lsr (r, wtmp,        wtmp, IMM(8));               CMT("tmp >>= 8;");
        i_dup (r, vtmp.b8,     wtmp);                       CMT("vtmp.lo = broadcast(tmp);");
        i_ins (r, vl[0].de[1], vtmp.de[0]);                 CMT("vl[0].hi = vtmp.lo;");
        i_ushl(r, vl[0].b16,   vl[0].b16, shift_vec.b16);   CMT("vl[0] <<= shift_vec;");
        i_and (r, vl[0].b16,   vl[0].b16, bitmask_vec);     CMT("vl[0] &= bitmask_vec;");
    } else {
        i_ldrb(r, wtmp,        a64op_post(s->in[0], 1));    CMT("uint8_t tmp = *in[0]++;");
        i_movi(r, bitmask_vec, IMM(1));                     CMT("v128 bitmask_vec = {1 <repeats 8 times>, 0 <repeats 8 times>};");
        i_dup (r, vl[0].b8,    wtmp);                       CMT("vl[0].lo = broadcast(tmp);");
        i_ushl(r, vl[0].b8,    vl[0].b8,  shift_vec.b8);    CMT("vl[0] <<= shift_vec;");
        i_and (r, vl[0].b8,    vl[0].b8,  bitmask_vec);     CMT("vl[0] &= bitmask_vec;");
    }
}

static void asmgen_op_read_nibble(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    RasmOp nibble_mask = v_8b(s->vt[0]);
    AArch64VecViews vl[1];
    AArch64VecViews vtmp;

    a64op_vec_views(s->vl[0], &vl[0]);
    a64op_vec_views(s->vt[1], &vtmp);

    rasm_annotate_next(r, "v128 nibble_mask = {0xf <repeats 8 times>, 0x0 <repeats 8 times>};");
    i_movi(r, nibble_mask, IMM(0x0f));

    if (p->block_size == 8) {
        i_ldr (r, vl[0].s,   a64op_post(s->in[0], 4));  CMT("vl[0] = *in[0]++;");
        i_ushr(r, vtmp.b8,   vl[0].b8, IMM(4));         CMT("vtmp.lo = vl[0] >> 4;");
        i_and (r, vl[0].b8,  vl[0].b8, nibble_mask);    CMT("vl[0].lo &= nibble_mask;");
        i_zip1(r, vl[0].b8,  vtmp.b8,  vl[0].b8);       CMT("interleave");
    } else {
        i_ldr (r, vl[0].d,   a64op_post(s->in[0], 8));  CMT("vl[0] = *in[0]++;");
        i_ushr(r, vtmp.b8,   vl[0].b8, IMM(4));         CMT("vtmp.lo = vl[0] >> 4;");
        i_and (r, vl[0].b8,  vl[0].b8, nibble_mask);    CMT("vl[0].lo &= nibble_mask;");
        i_zip1(r, vl[0].b16, vtmp.b16, vl[0].b16);      CMT("interleave");
    }
}

static void asmgen_op_read_packed_n(SwsAArch64Context *s, const SwsAArch64OpImplParams *p, RasmOp *vx)
{
    RasmContext *r = s->rctx;

    switch (p->mask) {
    case SWS_COMP_ELEMS(2): i_ld2(r, vv_2(vx[0], vx[1]),               a64op_post(s->in[0], s->vec_size * 2)); break;
    case SWS_COMP_ELEMS(3): i_ld3(r, vv_3(vx[0], vx[1], vx[2]),        a64op_post(s->in[0], s->vec_size * 3)); break;
    case SWS_COMP_ELEMS(4): i_ld4(r, vv_4(vx[0], vx[1], vx[2], vx[3]), a64op_post(s->in[0], s->vec_size * 4)); break;
    }
}

static void asmgen_op_read_packed(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    av_assert0(p->mask != 0x0001);
    asmgen_op_read_packed_n(s, p, s->vl);
    if (s->use_vh)
        asmgen_op_read_packed_n(s, p, s->vh);
}

static void asmgen_op_read_planar(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    AArch64VecViews vl[4];
    AArch64VecViews vh[4];

    for (int i = 0; i < 4; i++) {
        a64op_vec_views(s->vl[i], &vl[i]);
        a64op_vec_views(s->vh[i], &vh[i]);
    }

    LOOP_MASK(p, i) {
        switch ((s->use_vh ? 0x100 : 0) | s->vec_size) {
        case 0x008: i_ldr(r, vl[i].d,          a64op_post(s->in[i], s->vec_size * 1)); break;
        case 0x010: i_ldr(r, vl[i].q,          a64op_post(s->in[i], s->vec_size * 1)); break;
        case 0x108: i_ldp(r, vl[i].d, vh[i].d, a64op_post(s->in[i], s->vec_size * 2)); break;
        case 0x110: i_ldp(r, vl[i].q, vh[i].q, a64op_post(s->in[i], s->vec_size * 2)); break;
        }
    }
}

/*********************************************************************/
/* write raw pixels to planes */
/* SWS_UOP_WRITE_BIT */
/* SWS_UOP_WRITE_NIBBLE */
/* SWS_UOP_WRITE_PACKED */
/* SWS_UOP_WRITE_PLANAR */

static void asmgen_op_write_bit(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    AArch64VecViews vl[1];
    AArch64VecViews shift_vec;
    AArch64VecViews vtmp0;
    AArch64VecViews vtmp1;

    a64op_vec_views(s->vl[0], &vl[0]);
    a64op_vec_views(s->vt[0], &shift_vec);
    a64op_vec_views(s->vt[1], &vtmp0);
    a64op_vec_views(s->vt[2], &vtmp1);

    rasm_annotate_next(r, "v128 shift_vec = impl->priv.v128;");
    i_ldr(r, shift_vec.q, a64op_off(s->impl, offsetof_impl_priv));
    asmgen_set_load_cont_node(s);

    if (p->block_size == 8) {
        i_ushl(r, vl[0].b8,    vl[0].b8,   shift_vec.b8);   CMT("vl[0] <<= shift_vec;");
        i_addv(r, vtmp0.b,     vl[0].b8);                   CMT("vtmp0[0] = add_across(vl[0].lo);");
        i_str (r, vtmp0.b,     a64op_post(s->out[0], 1));   CMT("*out[0]++ = vtmp0;");
    } else {
        i_ushl(r, vl[0].b16,   vl[0].b16,  shift_vec.b16);  CMT("vl[0] <<= shift_vec;");
        i_addv(r, vtmp0.b,     vl[0].b8);                   CMT("vtmp0[0] = add_across(vl[0].lo);");
        i_ins (r, vtmp1.de[0], vl[0].de[1]);                CMT("vtmp1.lo = vl[0].hi;");
        i_addv(r, vtmp1.b,     vtmp1.b8);                   CMT("vtmp1[0] = add_across(vtmp1);");
        i_ins (r, vtmp0.be[1], vtmp1.be[0]);                CMT("vtmp0[1] = vtmp1[0];");
        i_str (r, vtmp0.h,     a64op_post(s->out[0], 2));   CMT("*out[0]++ = vtmp0;");
    }
}

static void asmgen_op_write_nibble(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    AArch64VecViews vl[4];
    AArch64VecViews vtmp0;
    AArch64VecViews vtmp1;

    for (int i = 0; i < 4; i++)
        a64op_vec_views(s->vl[i], &vl[i]);
    a64op_vec_views(s->vt[0], &vtmp0);
    a64op_vec_views(s->vt[1], &vtmp1);

    if (p->block_size == 8) {
        i_shl (r, vtmp0.h4,  vl[0].h4,  IMM(4));
        i_ushr(r, vtmp1.h4,  vl[0].h4,  IMM(8));
        i_orr (r, vl[0].b8,  vtmp0.b8,  vtmp1.b8);
        i_xtn (r, vtmp0.b8,  vl[0].h8);
        i_str (r, vtmp0.s,   a64op_post(s->out[0], 4));
    } else {
        i_shl (r, vtmp0.h8,  vl[0].h8,  IMM(4));
        i_ushr(r, vtmp1.h8,  vl[0].h8,  IMM(8));
        i_orr (r, vl[0].b16, vtmp0.b16, vtmp1.b16);
        i_xtn (r, vtmp0.b8,  vl[0].h8);
        i_str (r, vtmp0.d,   a64op_post(s->out[0], 8));
    }
}

static void asmgen_op_write_packed_n(SwsAArch64Context *s, const SwsAArch64OpImplParams *p, RasmOp *vx)
{
    RasmContext *r = s->rctx;

    switch (p->mask) {
    case SWS_COMP_ELEMS(2): i_st2(r, vv_2(vx[0], vx[1]),               a64op_post(s->out[0], s->vec_size * 2)); break;
    case SWS_COMP_ELEMS(3): i_st3(r, vv_3(vx[0], vx[1], vx[2]),        a64op_post(s->out[0], s->vec_size * 3)); break;
    case SWS_COMP_ELEMS(4): i_st4(r, vv_4(vx[0], vx[1], vx[2], vx[3]), a64op_post(s->out[0], s->vec_size * 4)); break;
    }
}

static void asmgen_op_write_packed(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    av_assert0(p->mask != 0x0001);
    asmgen_op_write_packed_n(s, p, s->vl);
    if (s->use_vh)
        asmgen_op_write_packed_n(s, p, s->vh);
}

static void asmgen_op_write_planar(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    AArch64VecViews vl[4];
    AArch64VecViews vh[4];

    for (int i = 0; i < 4; i++) {
        a64op_vec_views(s->vl[i], &vl[i]);
        a64op_vec_views(s->vh[i], &vh[i]);
    }

    LOOP_MASK(p, i) {
        switch ((s->use_vh ? 0x100 : 0) | s->vec_size) {
        case 0x008: i_str(r, vl[i].d,          a64op_post(s->out[i], s->vec_size * 1)); break;
        case 0x010: i_str(r, vl[i].q,          a64op_post(s->out[i], s->vec_size * 1)); break;
        case 0x108: i_stp(r, vl[i].d, vh[i].d, a64op_post(s->out[i], s->vec_size * 2)); break;
        case 0x110: i_stp(r, vl[i].q, vh[i].q, a64op_post(s->out[i], s->vec_size * 2)); break;
        }
    }
}

/*********************************************************************/
/* swap byte order (for differing endianness) */
/* SWS_UOP_SWAP_BYTES */

static void asmgen_op_swap_bytes(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    AArch64VecViews vl[4];
    AArch64VecViews vh[4];

    for (int i = 0; i < 4; i++) {
        a64op_vec_views(s->vl[i], &vl[i]);
        a64op_vec_views(s->vh[i], &vh[i]);
    }

    switch (ff_sws_pixel_type_size(p->type)) {
    case sizeof(uint16_t):
        LOOP_MASK      (p, i) i_rev16(r, vl[i].b16, vl[i].b16);
        LOOP_MASK_VH(s, p, i) i_rev16(r, vh[i].b16, vh[i].b16);
        break;
    case sizeof(uint32_t):
        LOOP_MASK      (p, i) i_rev32(r, vl[i].b16, vl[i].b16);
        LOOP_MASK_VH(s, p, i) i_rev32(r, vh[i].b16, vh[i].b16);
        break;
    }
}

/*********************************************************************/
/* rearrange channel order, or duplicate channels */
/* SWS_UOP_PERMUTE */
/* SWS_UOP_COPY */

static const char *print_swizzle_v(char buf[8], int8_t n, uint8_t vh)
{
    if (n == -1)
        snprintf(buf, sizeof(char[8]), "vtmp%c", vh ? 'h' : 'l');
    else
        snprintf(buf, sizeof(char[8]), "v%c[%u]", vh ? 'h' : 'l', n);
    return buf;
}
#define PRINT_SWIZZLE_V(n, vh) print_swizzle_v((char[8]){ 0 }, n, vh)

static RasmOp swizzle_a64op(SwsAArch64Context *s, int8_t n, uint8_t vh)
{
    if (n == -1)
        return s->vt[vh];
    return vh ? s->vh[n] : s->vl[n];
}

static void swizzle_emit(SwsAArch64Context *s, int8_t dst, int8_t src)
{
    RasmContext *r = s->rctx;
    RasmOp src_op[2] = { swizzle_a64op(s, src, 0), swizzle_a64op(s, src, 1) };
    RasmOp dst_op[2] = { swizzle_a64op(s, dst, 0), swizzle_a64op(s, dst, 1) };

    i_mov    (r, dst_op[0], src_op[0]); CMTF("%s = %s;", PRINT_SWIZZLE_V(dst, 0), PRINT_SWIZZLE_V(src, 0));
    if (s->use_vh) {
        i_mov(r, dst_op[1], src_op[1]); CMTF("%s = %s;", PRINT_SWIZZLE_V(dst, 1), PRINT_SWIZZLE_V(src, 1));
    }
}

static void asmgen_op_move(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    for (int i = 0; i < p->par.move.num_moves; i++)
        swizzle_emit(s, p->par.move.dst[i], p->par.move.src[i]);
}

/*********************************************************************/
/* split tightly packed data into components */
/* SWS_UOP_UNPACK */

static void asmgen_op_unpack(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    RasmOp *vl = s->vl;
    RasmOp *vh = s->vh;
    RasmOp *vt = s->vt;
    RasmOp mask_gpr = a64op_w(s->tmp0);
    uint32_t mask_val[4] = { 0 };
    uint8_t mask_idx[4] = { 0 };
    uint8_t cur_vt = 0;

    const int offsets[4] = {
        p->par.pack.pattern[3] + p->par.pack.pattern[2] + p->par.pack.pattern[1],
        p->par.pack.pattern[3] + p->par.pack.pattern[2],
        p->par.pack.pattern[3],
        0
    };

    /* Generate masks. */
    rasm_add_comment(r, "generate masks");
    LOOP_MASK(p, i) {
        uint32_t val = (1u << p->par.pack.pattern[i]) - 1;
        for (int j = 0; j < 4; j++) {
            if (mask_val[j] == val) {
                mask_val[i] = mask_val[j];
                mask_idx[i] = mask_idx[j];
                break;
            }
        }
        if (!mask_val[i]) {
            /**
             * All-one values in movi only work up to 8-bit, and then
             * at full 16- or 32-bit, but not for intermediate values
             * like 10-bit. In those cases, we use mov + dup instead.
             */
            if (val <= 0xff || val == 0xffff) {
                i_movi(r, vt[cur_vt], IMM(val));
            } else {
                i_mov (r, mask_gpr,   IMM(val));
                i_dup (r, vt[cur_vt], mask_gpr);
            }
            mask_val[i] = val;
            mask_idx[i] = cur_vt++;
        }
    }

    /* Loop backwards to avoid clobbering component 0. */
    LOOP_MASK_BWD      (p, i) {
        if (offsets[i]) {
            i_ushr  (r, vl[i], vl[0], IMM(offsets[i])); CMTF("vl[%u] >>= %u;", i, offsets[i]);
        } else if (i) {
            i_mov16b(r, vl[i], vl[0]);                  CMTF("vl[%u] = vl[0];", i);
        }
    }
    LOOP_MASK_BWD_VH(s, p, i) {
        if (offsets[i]) {
            i_ushr  (r, vh[i], vh[0], IMM(offsets[i])); CMTF("vh[%u] >>= %u;", i, offsets[i]);
        } else if (i) {
            i_mov16b(r, vh[i], vh[0]);                  CMTF("vh[%u] = vh[0];", i);
        }
    }

    /* Apply masks. */
    reshape_all_vectors(s, 16, 1);
    LOOP_MASK_BWD      (p, i) { i_and(r, vl[i], vl[i], vt[mask_idx[i]]); CMTF("vl[%u] &= 0x%x;", i, mask_val[i]); }
    LOOP_MASK_BWD_VH(s, p, i) { i_and(r, vh[i], vh[i], vt[mask_idx[i]]); CMTF("vh[%u] &= 0x%x;", i, mask_val[i]); }
}

/*********************************************************************/
/* compress components into tightly packed data */
/* SWS_UOP_PACK */

static void asmgen_op_pack(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    RasmOp *vl = s->vl;
    RasmOp *vh = s->vh;

    const int offsets[4] = {
        p->par.pack.pattern[3] + p->par.pack.pattern[2] + p->par.pack.pattern[1],
        p->par.pack.pattern[3] + p->par.pack.pattern[2],
        p->par.pack.pattern[3],
        0
    };
    SwsCompMask offset_mask = 0;
    LOOP_MASK(p, i) {
        if (offsets[i])
            offset_mask |= SWS_COMP(i);
    }

    /* Perform left shift. */
    LOOP      (offset_mask, i) { i_shl(r, vl[i], vl[i], IMM(offsets[i])); CMTF("vl[%u] <<= %u;", i, offsets[i]); }
    LOOP_VH(s, offset_mask, i) { i_shl(r, vh[i], vh[i], IMM(offsets[i])); CMTF("vh[%u] <<= %u;", i, offsets[i]); }

    /* Combine components. */
    reshape_all_vectors(s, 16, 1);
    LOOP_MASK      (p, i) {
        if (i != 0) {
            i_orr    (r, vl[0], vl[0], vl[i]); CMTF("vl[0] |= vl[%u];", i);
            if (s->use_vh) {
                i_orr(r, vh[0], vh[0], vh[i]); CMTF("vh[0] |= vh[%u];", i);
            }
        }
    }
}

/*********************************************************************/
/* logical left shift of raw pixel values */
/* SWS_UOP_LSHIFT */

static void asmgen_op_lshift(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    uint8_t shift = p->par.shift.amount;
    RasmContext *r = s->rctx;
    RasmOp *vl = s->vl;
    RasmOp *vh = s->vh;

    LOOP_MASK      (p, i) { i_shl(r, vl[i], vl[i], IMM(shift)); CMTF("vl[%u] <<= %u;", i, shift); }
    LOOP_MASK_VH(s, p, i) { i_shl(r, vh[i], vh[i], IMM(shift)); CMTF("vh[%u] <<= %u;", i, shift); }
}

/*********************************************************************/
/* right shift of raw pixel values */
/* SWS_UOP_RSHIFT */

static void asmgen_op_rshift(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    uint8_t shift = p->par.shift.amount;
    RasmContext *r = s->rctx;
    RasmOp *vl = s->vl;
    RasmOp *vh = s->vh;

    LOOP_MASK      (p, i) { i_ushr(r, vl[i], vl[i], IMM(shift)); CMTF("vl[%u] >>= %u;", i, shift); }
    LOOP_MASK_VH(s, p, i) { i_ushr(r, vh[i], vh[i], IMM(shift)); CMTF("vh[%u] >>= %u;", i, shift); }
}

/*********************************************************************/
/* clear pixel values */
/* SWS_UOP_CLEAR */

static void emit_clear(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                       RasmOp *vx, int i, const char *vx_str)
{
    RasmContext *r = s->rctx;
    RasmOp clear_vec = s->vt[0];
    if (p->par.clear.zero & SWS_COMP(i)) {
        i_movi(r, vx[i], IMM(0));                   CMTF("%s[%u] = 0;", vx_str, i);
    } else if (p->par.clear.one & SWS_COMP(i)) {
        if (p->block_size * ff_sws_pixel_type_size(p->type) == 8) {
            i_movi(r, v_8b (vx[i]), IMM(0xff));
        } else {
            i_movi(r, v_16b(vx[i]), IMM(0xff));
        }
        CMTF("%s[%u] = UINT_MAX;", vx_str, i);
    } else {
        i_dup (r, vx[i], a64op_elem(clear_vec, i)); CMTF("%s[%u] = broadcast(clear_vec[%u]);", vx_str, i, i);
    }
}

static void asmgen_op_clear(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    RasmOp clear_vec = s->vt[0];

    /**
     * TODO
     * - pack elements in impl->priv and perform smaller loads
     * - if only 1 element and not vh, load directly with ld1r
     */

    bool load_priv = false;
    LOOP_MASK(p, i) {
        if (!((p->par.clear.zero | p->par.clear.one) & SWS_COMP(i)))
            load_priv = true;
    }
    if (load_priv) {
        i_ldr(r, v_q(clear_vec), a64op_off(s->impl, offsetof_impl_priv));   CMT("v128 clear_vec = impl->priv.v128;");
        asmgen_set_load_cont_node(s);
    }

    LOOP_MASK      (p, i) { emit_clear(s, p, s->vl, i, "vl"); }
    LOOP_MASK_VH(s, p, i) { emit_clear(s, p, s->vh, i, "vh"); }
}

/*********************************************************************/
/* convert (cast) between formats */
/* SWS_UOP_TO_U8 */
/* SWS_UOP_TO_U16 */
/* SWS_UOP_TO_U32 */
/* SWS_UOP_TO_F32 */

static void asmgen_op_convert(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    AArch64VecViews vl[4];
    AArch64VecViews vh[4];

    /**
     * Since each instruction in the convert operation needs specific
     * element types, it is simpler to use arrangement specifiers for
     * each operand instead of reshaping all vectors.
     */

    for (int i = 0; i < 4; i++) {
        a64op_vec_views(s->vl[i], &vl[i]);
        a64op_vec_views(s->vh[i], &vh[i]);
    }

    size_t src_el_size = s->el_size;
    SwsPixelType to_type;
    switch (p->uop) {
    case SWS_UOP_TO_U8:  to_type = SWS_PIXEL_U8;  break;
    case SWS_UOP_TO_U16: to_type = SWS_PIXEL_U16; break;
    case SWS_UOP_TO_U32: to_type = SWS_PIXEL_U32; break;
    case SWS_UOP_TO_F32: to_type = SWS_PIXEL_F32; break;
    default:
        av_assert0(!"Invalid uop!");
        break;
    }
    size_t dst_el_size = ff_sws_pixel_type_size(to_type);

    /**
     * This function assumes block_size is either 8 or 16, and that
     * we're always using the most amount of vector registers possible.
     * Therefore, u32 always uses the high vector bank.
     */
    if (p->type == SWS_PIXEL_F32) {
        rasm_add_comment(r, "f32 -> u32");
        LOOP_MASK(p, i) i_fcvtzu(r, vl[i].s4, vl[i].s4);
        LOOP_MASK(p, i) i_fcvtzu(r, vh[i].s4, vh[i].s4);
    }

    if (p->block_size == 8) {
        if (src_el_size == 1 && dst_el_size > src_el_size) {
            rasm_add_comment(r, "u8 -> u16");
            LOOP_MASK(p, i) i_uxtl (r, vl[i].h8,    vl[i].b8);
            src_el_size = 2;
        } else if (src_el_size == 4 && dst_el_size < src_el_size) {
            rasm_add_comment(r, "u32 -> u16");
            LOOP_MASK(p, i) i_xtn  (r, vl[i].h4,    vl[i].s4);
            LOOP_MASK(p, i) i_xtn  (r, vh[i].h4,    vh[i].s4);
            LOOP_MASK(p, i) i_ins  (r, vl[i].de[1], vh[i].de[0]);
            src_el_size = 2;
        }
        if (src_el_size == 2 && dst_el_size == 4) {
            rasm_add_comment(r, "u16 -> u32");
            LOOP_MASK(p, i) i_uxtl2(r, vh[i].s4,    vl[i].h8);
            LOOP_MASK(p, i) i_uxtl (r, vl[i].s4,    vl[i].h4);
            src_el_size = 4;
        } else if (src_el_size == 2 && dst_el_size == 1) {
            rasm_add_comment(r, "u16 -> u8");
            LOOP_MASK(p, i) i_xtn  (r, vl[i].b8,    vl[i].h8);
            src_el_size = 1;
        }
    } else /* if (p->block_size == 16) */ {
        if (src_el_size == 1 && dst_el_size == 2) {
            rasm_add_comment(r, "u8 -> u16");
            LOOP_MASK(p, i) i_uxtl2(r, vh[i].h8,    vl[i].b16);
            LOOP_MASK(p, i) i_uxtl (r, vl[i].h8,    vl[i].b8);
        } else if (src_el_size == 2 && dst_el_size == 1) {
            rasm_add_comment(r, "u16 -> u8");
            LOOP_MASK(p, i) i_xtn  (r, vl[i].b8,    vl[i].h8);
            LOOP_MASK(p, i) i_xtn  (r, vh[i].b8,    vh[i].h8);
            LOOP_MASK(p, i) i_ins  (r, vl[i].de[1], vh[i].de[0]);
        }
    }

    /* See comment above for high vector bank usage for u32. */
    if (to_type == SWS_PIXEL_F32) {
        rasm_add_comment(r, "u32 -> f32");
        LOOP_MASK(p, i) i_ucvtf(r, vl[i].s4, vl[i].s4);
        LOOP_MASK(p, i) i_ucvtf(r, vh[i].s4, vh[i].s4);
    }
}

/*********************************************************************/
/* expand integers to the full range */
/* SWS_UOP_EXPAND_PAIR */
/* SWS_UOP_EXPAND_QUAD */

static void asmgen_op_expand(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    RasmOp *vl = s->vl;
    RasmOp *vh = s->vh;

    size_t src_el_size = s->el_size;
    SwsPixelType to_type;
    switch (p->uop) {
    case SWS_UOP_EXPAND_PAIR: to_type = SWS_PIXEL_U16; break;
    case SWS_UOP_EXPAND_QUAD: to_type = SWS_PIXEL_U32; break;
    default:
        av_assert0(!"Invalid uop!");
        break;
    }
    size_t dst_el_size = ff_sws_pixel_type_size(to_type);
    size_t dst_total_size = p->block_size * dst_el_size;
    size_t dst_vec_size = FFMIN(dst_total_size, 16);

    if (!s->use_vh)
        s->use_vh = (dst_vec_size != dst_total_size);

    if (src_el_size == 1) {
        rasm_add_comment(r, "u8 -> u16");
        reshape_all_vectors(s, 16, 1);
        LOOP_MASK_VH(s, p, i) i_zip2(r, vh[i], vl[i], vl[i]);
        LOOP_MASK      (p, i) i_zip1(r, vl[i], vl[i], vl[i]);
    }
    if (dst_el_size == 4) {
        rasm_add_comment(r, "u16 -> u32");
        reshape_all_vectors(s, 8, 2);
        LOOP_MASK_VH(s, p, i) i_zip2(r, vh[i], vl[i], vl[i]);
        LOOP_MASK      (p, i) i_zip1(r, vl[i], vl[i], vl[i]);
    }
}

/*********************************************************************/
/* numeric minimum */
/* SWS_UOP_MIN */

static void asmgen_op_min(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    RasmOp *vl = s->vl;
    RasmOp *vh = s->vh;
    RasmOp *vt = s->vt;
    RasmOp min_vec = s->vt[4];

    i_ldr(r, v_q(min_vec), a64op_off(s->impl, offsetof_impl_priv)); CMT("v128 min_vec = impl->priv.v128;");
    asmgen_set_load_cont_node(s);
    LOOP_MASK(p, i) { i_dup(r, vt[i], a64op_elem(min_vec, i));      CMTF("v128 vmin%u = min_vec[%u];", i, i); }

    if (p->type == SWS_PIXEL_F32) {
        LOOP_MASK      (p, i) { i_fmin(r, vl[i], vl[i], vt[i]);     CMTF("vl[%u] = min(vl[%u], vmin%u);", i, i, i); }
        LOOP_MASK_VH(s, p, i) { i_fmin(r, vh[i], vh[i], vt[i]);     CMTF("vh[%u] = min(vh[%u], vmin%u);", i, i, i); }
    } else {
        LOOP_MASK      (p, i) { i_umin(r, vl[i], vl[i], vt[i]);     CMTF("vl[%u] = min(vl[%u], vmin%u);", i, i, i); }
        LOOP_MASK_VH(s, p, i) { i_umin(r, vh[i], vh[i], vt[i]);     CMTF("vh[%u] = min(vh[%u], vmin%u);", i, i, i); }
    }
}

/*********************************************************************/
/* numeric maximum */
/* SWS_UOP_MAX */

static void asmgen_op_max(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    RasmOp *vl = s->vl;
    RasmOp *vh = s->vh;
    RasmOp *vt = s->vt;
    RasmOp max_vec = s->vt[4];

    i_ldr(r, v_q(max_vec), a64op_off(s->impl, offsetof_impl_priv)); CMT("v128 max_vec = impl->priv.v128;");
    asmgen_set_load_cont_node(s);
    LOOP_MASK(p, i) { i_dup(r, vt[i], a64op_elem(max_vec, i));      CMTF("v128 vmax%u = max_vec[%u];", i, i); }

    if (p->type == SWS_PIXEL_F32) {
        LOOP_MASK      (p, i) { i_fmax(r, vl[i], vl[i], vt[i]);     CMTF("vl[%u] = max(vl[%u], vmax%u);", i, i, i); }
        LOOP_MASK_VH(s, p, i) { i_fmax(r, vh[i], vh[i], vt[i]);     CMTF("vh[%u] = max(vh[%u], vmax%u);", i, i, i); }
    } else {
        LOOP_MASK      (p, i) { i_umax(r, vl[i], vl[i], vt[i]);     CMTF("vl[%u] = max(vl[%u], vmax%u);", i, i, i); }
        LOOP_MASK_VH(s, p, i) { i_umax(r, vh[i], vh[i], vt[i]);     CMTF("vh[%u] = max(vh[%u], vmax%u);", i, i, i); }
    }
}

/*********************************************************************/
/* multiplication by scalar */
/* SWS_UOP_SCALE */

static void asmgen_op_scale(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    RasmOp *vl = s->vl;
    RasmOp *vh = s->vh;
    RasmOp priv_ptr = s->tmp0;
    RasmOp scale_vec = s->vt[0];

    i_add (r, priv_ptr, s->impl, IMM(offsetof_impl_priv));          CMT("v128 *scale_vec_ptr = &impl->priv;");
    asmgen_set_load_cont_node(s);
    i_ld1r(r, vv_1(scale_vec), a64op_base(priv_ptr));               CMT("v128 scale_vec = broadcast(*scale_vec_ptr);");

    if (p->type == SWS_PIXEL_F32) {
        LOOP_MASK      (p, i) { i_fmul(r, vl[i], vl[i], scale_vec); CMTF("vl[%u] *= scale_vec;", i); }
        LOOP_MASK_VH(s, p, i) { i_fmul(r, vh[i], vh[i], scale_vec); CMTF("vh[%u] *= scale_vec;", i); }
    } else {
        LOOP_MASK      (p, i) { i_mul (r, vl[i], vl[i], scale_vec); CMTF("vl[%u] *= scale_vec;", i); }
        LOOP_MASK_VH(s, p, i) { i_mul (r, vh[i], vh[i], scale_vec); CMTF("vh[%u] *= scale_vec;", i); }
    }
}

/*********************************************************************/
/* generalized linear affine transform */
/* SWS_UOP_LINEAR */
/* SWS_UOP_LINEAR_FMA */

/**
 * Performs one pass of the linear transform over a single vector bank
 * (low or high).
 */
static void linear_pass(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                        RasmOp *vt, RasmOp *vc,
                        SwsCompMask save_mask, bool vh_pass)
{
    RasmContext *r = s->rctx;
    /**
     * The intermediate registers for fmul+fadd (for when SWS_BITEXACT
     * is set) start from temp vector 4.
     */
    RasmOp *vtmp = &vt[4];
    RasmOp *vx = vh_pass ? s->vh : s->vl;
    char cvh = vh_pass ? 'h' : 'l';

    if (vh_pass && !s->use_vh)
        return;

    /**
     * Save rows that need to be used as input after they have been already
     * written to.
     */
    RasmOp src_vx[4] = { vx[0], vx[1], vx[2], vx[3] };
    if (save_mask) {
        for (int i = 0; i < 4; i++) {
            if (save_mask & SWS_COMP(i)) {
                src_vx[i] = vt[i];
                i_mov16b(r, vt[i], vx[i]);  CMTF("vsrc[%u] = v%c[%u];", i, cvh, i);
            }
        }
    }

    /**
     * The non-zero coefficients have been packed in aarch64_setup_linear()
     * in sequential order into the individual lanes of the coefficient
     * vector registers. We must follow the same order of execution here.
     */
    int i_coeff = 0;
    LOOP_MASK(p, i) {
        bool first = true;
        RasmNode *pre_mul = rasm_get_current_node(r);
        for (int j = 0; j < 5; j++) {
            bool is_offset = (j == 0);
            int src_j = is_offset ? 4 : (j - 1);
            if (p->par.lin.zero & SWS_MASK(i, src_j))
                continue;
            RasmOp vsrc = src_vx[src_j];
            uint8_t vc_i = i_coeff / 4;
            uint8_t vc_j = i_coeff & 3;
            RasmOp vcoeff = a64op_elem(vc[vc_i], vc_j);
            i_coeff++;
            if (first && is_offset) {
                i_dup (r, vx[i], vcoeff);               CMTF("v%c[%u]  = broadcast(vc[%u][%u]);", cvh, i, vc_i, vc_j);
            } else if (first && !is_offset) {
                if (p->par.lin.one & SWS_MASK(i, src_j)) {
                    i_mov16b(r, vx[i], vsrc);           CMTF("v%c[%u]  = vsrc[%u];", cvh, i, src_j);
                } else {
                    i_fmul  (r, vx[i], vsrc, vcoeff);   CMTF("v%c[%u]  = vsrc[%u] * vc[%u][%u];", cvh, i, src_j, vc_i, vc_j);
                }
            } else if (p->uop == SWS_UOP_LINEAR_FMA) {
                /**
                 * Most modern aarch64 cores have a fastpath for sequences
                 * of fmla instructions. This means that even if the coefficient
                 * is 1, it is still faster to use fmla by 1 instead of fadd.
                 */
                i_fmla(r, vx[i], vsrc, vcoeff);         CMTF("v%c[%u] += vsrc[%u] * vc[%u][%u];", cvh, i, src_j, vc_i, vc_j);
            } else {
                /**
                 * Split the multiply-accumulate into fmul+fadd. All
                 * multiplications are performed first into temporary
                 * registers, and only then added to the destination,
                 * to reduce the dependency chain.
                 * There is no need to perform multiplications by 1.
                 */
                if (!(p->par.lin.one & SWS_MASK(i, src_j))) {
                    pre_mul = rasm_set_current_node(r, pre_mul);
                    i_fmul(r, vtmp[vc_j], vsrc, vcoeff);    CMTF("vtmp[%u] = vsrc[%u] * vc[%u][%u];", vc_j, src_j, vc_i, vc_j);
                    pre_mul = rasm_set_current_node(r, pre_mul);
                    i_fadd(r, vx[i], vx[i], vtmp[vc_j]);    CMTF("v%c[%u] += vtmp[%u];", cvh, i, vc_j);
                } else {
                    i_fadd(r, vx[i], vx[i], vsrc);          CMTF("v%c[%u] += vsrc[%u];", cvh, i, vc_j);
                }
            }
            first = false;
        }
    }
}

static void asmgen_op_linear(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    RasmOp *vt = s->vt;
    RasmOp *vc = &vt[8]; /* The coefficients are loaded starting from temp vector 8 */
    RasmOp ptr = s->tmp0;
    RasmOp coeff_veclist;

    /* Preload coefficients from impl->priv. */
    const int num_vregs = linear_num_vregs(p);
    av_assert0(num_vregs <= 4);
    switch (num_vregs) {
    case 1: coeff_veclist = vv_1(vc[0]);                      break;
    case 2: coeff_veclist = vv_2(vc[0], vc[1]);               break;
    case 3: coeff_veclist = vv_3(vc[0], vc[1], vc[2]);        break;
    case 4: coeff_veclist = vv_4(vc[0], vc[1], vc[2], vc[3]); break;
    }
    i_ldr(r, ptr, a64op_off(s->impl, offsetof_impl_priv));  CMT("v128 *vcoeff_ptr = impl->priv.ptr;");
    asmgen_set_load_cont_node(s);
    i_ld1(r, coeff_veclist, a64op_base(ptr));               CMT("coeff_veclist = *vcoeff_ptr;");

    /* Compute mask for rows that must be saved before being overwritten. */
    SwsCompMask save_mask = 0;
    bool overwritten[4] = { false, false, false, false };
    LOOP_MASK(p, i) {
        for (int j = 0; j < 5; j++) {
            bool is_offset = (j == 0);
            int src_j = is_offset ? 4 : (j - 1);
            if (p->par.lin.zero & SWS_MASK(i, src_j))
                continue;
            if (!is_offset && overwritten[src_j])
                save_mask |= SWS_COMP(src_j);
            overwritten[i] = true;
        }
    }

    /* Perform linear passes for low and high vector banks. */
    linear_pass(s, p, vt, vc, save_mask, false);
    linear_pass(s, p, vt, vc, save_mask, true);
}

/*********************************************************************/
/* add dithering noise */
/* SWS_UOP_DITHER */

static void asmgen_op_dither(SwsAArch64Context *s, const SwsAArch64OpImplParams *p)
{
    RasmContext *r = s->rctx;
    RasmOp *vl = s->vl;
    RasmOp *vh = s->vh;
    RasmOp ptr = s->tmp0;
    RasmOp tmp1 = s->tmp1;
    RasmOp wtmp1 = a64op_w(tmp1);
    RasmOp dither_vl = s->vt[0];
    RasmOp dither_vh = s->vt[1];
    RasmOp bx64 = a64op_x(s->bx);
    RasmOp y64 = a64op_x(s->y);

    /**
     * For a description of the matrix buffer layout, read the comments
     * in aarch64_setup_dither() in aarch64/ops.c.
     */

    /**
     * Sort components by y_offset value so that we can start dithering
     * with the smallest value, and increment the pointer upwards for
     * each new offset. The dither matrix is over-allocated and may be
     * over-read at the top, but it cannot be over-read before the start
     * of the buffer. Since we only mask the y offset once, this would
     * be an issue if we tried to subtract a value larger than the
     * initial y_offset.
     */
    int sorted[4];
    int n_comps = 0;
    /* Very cheap bucket sort. */
    int max_offset = 0;
    LOOP_MASK(p, i)
        max_offset = FFMAX(max_offset, p->par.dither.y_offset[i]);
    for (int y_off = 0; y_off <= max_offset; y_off++) {
        LOOP_MASK(p, i) {
            if (p->par.dither.y_offset[i] == y_off)
                sorted[n_comps++] = i;
        }
    }

    i_ldr(r, ptr, a64op_off(s->impl, offsetof_impl_priv));  CMT("void *ptr = impl->priv.ptr;");
    asmgen_set_load_cont_node(s);

    /**
     * We use ubfiz to mask and shift left in one single instruction:
     *   ubfiz <Wd>, <Wn>, #<lsb>, #<width>
     *   Wd = (Wn & ((1 << width) - 1)) << lsb;
     *
     * Given:
     *  block_size    =  8, log2(block_size)    = 3
     *  dither_size   = 16, log2(dither_size)   = 4, dither_mask = 0b1111
     *  sizeof(float) =  4, log2(sizeof(float)) = 2
     *
     * Suppose we have bx = 0bvvvv. To get x, we left shift by
     * log2(block_size) and end up with 0bvvvv000. Then we mask against
     * dither_mask, and end up with 0bv000. Finally we multiply by
     * sizeof(float), which is the same as shifting left by
     * log2(sizeof(float)). The result is 0bv00000.
     *
     * Therefore:
     *  width = log2(dither_size) - log2(block_size)
     *  lsb   = log2(block_size) + log2(sizeof(float))
     */
    const int block_size_log2   = (p->block_size == 16) ? 4 : 3;
    const int dither_size_log2  = p->par.dither.size_log2;
    const int sizeof_float_log2 = 2;
    if (dither_size_log2 != block_size_log2) {
        RasmOp lsb   = IMM(block_size_log2 + sizeof_float_log2);
        RasmOp width = IMM(dither_size_log2 - block_size_log2);
        i_ubfiz(r, tmp1, bx64, lsb, width); CMT("tmp1 = (bx & ((dither_size / block_size) - 1)) * block_size * sizeof(float);");
        i_add  (r, ptr,  ptr,  tmp1);       CMT("ptr += tmp1;");
    }

    int last_y_off = -1;
    int prev_i = 0;
    for (int sorted_i = 0; sorted_i < n_comps; sorted_i++) {
        int i = sorted[sorted_i];
        uint8_t y_off = p->par.dither.y_offset[i];
        bool do_load = (y_off != last_y_off);

        if (last_y_off < 0) {
            /* On the first run, calculate pointer inside dither_matrix. */
            RasmOp lsb   = IMM(dither_size_log2 + sizeof_float_log2);
            RasmOp width = IMM(dither_size_log2);
            /**
             * The ubfiz instruction for the y offset performs masking
             * by the dither matrix size and shifts by the stride.
             */
            if (y_off == 0) {
                i_ubfiz(r, tmp1,  y64,  lsb, width);        CMT("tmp1 = (y & (dither_size - 1)) * dither_size * sizeof(float);");
            } else {
                i_add  (r, wtmp1, s->y, IMM(y_off));        CMTF("tmp1 = y + y_off[%u];", i);
                i_ubfiz(r, tmp1,  tmp1, lsb, width);        CMT("tmp1 = (tmp1 & (dither_size - 1)) * dither_size * sizeof(float);");
            }
            i_add(r, ptr, ptr, tmp1);                       CMT("ptr += tmp1;");
        } else if (do_load) {
            /**
             * On subsequent runs, just increment the pointer.
             * The matrix is over-allocated, so we don't risk
             * overreading.
             */
            int delta = (y_off - last_y_off) * (1 << dither_size_log2) * sizeof(float);
            i_add(r, ptr, ptr, IMM(delta));                 CMTF("ptr += (y_off[%u] - y_off[%u]) * dither_size * sizeof(float);", i, prev_i);
        }

        if (do_load) {
            RasmOp dither_vlq = v_q(dither_vl);
            RasmOp dither_vhq = v_q(dither_vh);
            i_ldp (r, dither_vlq, dither_vhq, a64op_base(ptr)); CMT("{ ditherl, ditherh } = *ptr;");
        }

        i_fadd    (r, vl[i], vl[i], dither_vl);             CMTF("vl[%u] += vditherl;", i);
        if (s->use_vh) {
            i_fadd(r, vh[i], vh[i], dither_vh);             CMTF("vh[%u] += vditherh;", i);
        }

        last_y_off = y_off;
        prev_i = i;
    }
}

/*********************************************************************/
static void asmgen_op_cps(SwsAArch64Context *s, const SwsAArch64OpEntry *entry)
{
    const SwsAArch64OpImplParams *p = &entry->params;
    RasmContext *r = s->rctx;

    bool is_read = false;
    bool is_write = false;
    switch (p->uop) {
    case SWS_UOP_READ_BIT:
    case SWS_UOP_READ_NIBBLE:
    case SWS_UOP_READ_PACKED:
    case SWS_UOP_READ_PLANAR:
        is_read = true;
        break;
    case SWS_UOP_WRITE_BIT:
    case SWS_UOP_WRITE_NIBBLE:
    case SWS_UOP_WRITE_PACKED:
    case SWS_UOP_WRITE_PLANAR:
        is_write = true;
        break;
    default:
        break;
    }

    rasm_func_begin(r, entry->name, true, !is_read);

    /**
     * Set up vector register dimensions and reshape all vectors
     * accordingly.
     */
    size_t el_size = ff_sws_pixel_type_size(p->type);
    size_t total_size = p->block_size * el_size;

    s->vec_size = FFMIN(total_size, 16);
    s->use_vh = (s->vec_size != total_size);

    s->el_size = el_size;
    s->el_count = s->vec_size / el_size;
    reshape_all_vectors(s, s->el_count, el_size);

    /* Common start for continuation-passing style (CPS) functions. */
    asmgen_set_load_cont_node(s);

    switch (p->uop) {
    case SWS_UOP_READ_BIT:     asmgen_op_read_bit(s, p);     break;
    case SWS_UOP_READ_NIBBLE:  asmgen_op_read_nibble(s, p);  break;
    case SWS_UOP_READ_PACKED:  asmgen_op_read_packed(s, p);  break;
    case SWS_UOP_READ_PLANAR:  asmgen_op_read_planar(s, p);  break;
    case SWS_UOP_WRITE_BIT:    asmgen_op_write_bit(s, p);    break;
    case SWS_UOP_WRITE_NIBBLE: asmgen_op_write_nibble(s, p); break;
    case SWS_UOP_WRITE_PACKED: asmgen_op_write_packed(s, p); break;
    case SWS_UOP_WRITE_PLANAR: asmgen_op_write_planar(s, p); break;
    case SWS_UOP_SWAP_BYTES:   asmgen_op_swap_bytes(s, p);   break;
    case SWS_UOP_PERMUTE:      asmgen_op_move(s, p);         break;
    case SWS_UOP_COPY:         asmgen_op_move(s, p);         break;
    case SWS_UOP_UNPACK:       asmgen_op_unpack(s, p);       break;
    case SWS_UOP_PACK:         asmgen_op_pack(s, p);         break;
    case SWS_UOP_LSHIFT:       asmgen_op_lshift(s, p);       break;
    case SWS_UOP_RSHIFT:       asmgen_op_rshift(s, p);       break;
    case SWS_UOP_CLEAR:        asmgen_op_clear(s, p);        break;
    case SWS_UOP_TO_U8:        asmgen_op_convert(s, p);      break;
    case SWS_UOP_TO_U16:       asmgen_op_convert(s, p);      break;
    case SWS_UOP_TO_U32:       asmgen_op_convert(s, p);      break;
    case SWS_UOP_TO_F32:       asmgen_op_convert(s, p);      break;
    case SWS_UOP_EXPAND_PAIR:  asmgen_op_expand(s, p);       break;
    case SWS_UOP_EXPAND_QUAD:  asmgen_op_expand(s, p);       break;
    case SWS_UOP_MIN:          asmgen_op_min(s, p);          break;
    case SWS_UOP_MAX:          asmgen_op_max(s, p);          break;
    case SWS_UOP_SCALE:        asmgen_op_scale(s, p);        break;
    case SWS_UOP_LINEAR:       asmgen_op_linear(s, p);       break;
    case SWS_UOP_LINEAR_FMA:   asmgen_op_linear(s, p);       break;
    case SWS_UOP_DITHER:       asmgen_op_dither(s, p);       break;
    /* TODO implement SWS_UOP_SHUFFLE */
    default:
        break;
    }

    if (is_write) {
        /* Write functions return directly. */
        i_ret(r);
    } else {
        /* Load continuation address and increment impl pointer. */
        RasmNode *node = rasm_set_current_node(r, s->load_cont_node);
        RasmOp impl_post = a64op_post(s->impl, sizeof_impl);
        i_ldr(r, s->cont, impl_post);                   CMT("SwsFuncPtr cont = (impl++)->cont;");
        rasm_set_current_node(r, node);
        /* Common end for remaining CPS functions. */
        i_br (r, s->cont);                              CMT("jump to cont");
    }
}

/*********************************************************************/

/* Generate all functions described by ops_entries.c */
static int asmgen(void)
{
    RasmContext *rctx = rasm_alloc();
    if (!rctx)
        return AVERROR(ENOMEM);

    SwsAArch64Context s = { .rctx = rctx };
    AVBPrint bp;
    int ret;

    av_bprint_init(&bp, 0, AV_BPRINT_SIZE_UNLIMITED);

    /**
     * The entry point of the SwsOpFunc is the `process` function. The
     * first kernel function is called from `process`, and subsequent
     * kernel functions are chained by directly branching to the next
     * operation, using a continuation-passing style design. The last
     * operation must be a write operation, which returns from the call
     * to the `process` function.
     *
     * The GPRs used by the entire call-chain are listed below.
     *
     * Function arguments are passed in r0-r5. After the parameters
     * from `exec` have been read, r0 is reused to branch to the
     * continuation functions. After the original parameters from
     * `impl` have been computed, r1 is reused as the `impl` pointer
     * for each operation.
     *
     * Loop iterators are r6 for `bx` and r3 for `y`, reused from
     * `y_start`, which doesn't need to be preserved.
     *
     * The intra-procedure-call temporary registers (r16 and r17) are
     * used as scratch registers. They may be used by call veneers and
     * PLT code inserted by the linker, so we cannot expect them to
     * persist across branches between functions.
     *
     * The Platform Register (r18) is not used.
     *
     * The read/write data pointers and padding values first use up the
     * remaining free caller-saved registers, and only then are the
     * caller-saved registers (r19-r28) used.
     *
     * The Link Register (r30) is used when calling the first kernel,
     * so it must be saved.
     */

    /* SwsOpFunc arguments. */
    s.exec      = a64op_gpx(0); // const SwsOpExec *exec
    s.impl      = a64op_gpx(1); // const void *priv
    s.bx_start  = a64op_gpw(2); // int bx_start
    s.y_start   = a64op_gpw(3); // int y_start
    s.bx_end    = a64op_gpw(4); // int bx_end
    s.y_end     = a64op_gpw(5); // int y_end

    /* Loop iterator variables. */
    s.bx        = a64op_gpw(6);
    s.y         = s.y_start;    /* Reused from SwsOpFunc argument. */

    /* Scratch registers. */
    s.tmp0      = a64op_gpx(16); /* IP0 */
    s.tmp1      = a64op_gpx(17); /* IP1 */

    /* CPS-related variables. */
    s.op0_func  = a64op_gpx(7);
    s.op1_impl  = a64op_gpx(8);
    s.cont      = s.exec;       /* Reused from SwsOpFunc argument. */

    /* Read/Write data pointers and padding. */
    s.in      [0] = a64op_gpx(9);
    s.out     [0] = a64op_gpx(10);
    s.in_bump [0] = a64op_gpx(11);
    s.out_bump[0] = a64op_gpx(12);
    s.in      [1] = a64op_gpx(13);
    s.out     [1] = a64op_gpx(14);
    s.in_bump [1] = a64op_gpx(15);
    s.out_bump[1] = a64op_gpx(19);
    s.in      [2] = a64op_gpx(20);
    s.out     [2] = a64op_gpx(21);
    s.in_bump [2] = a64op_gpx(22);
    s.out_bump[2] = a64op_gpx(23);
    s.in      [3] = a64op_gpx(24);
    s.out     [3] = a64op_gpx(25);
    s.in_bump [3] = a64op_gpx(26);
    s.out_bump[3] = a64op_gpx(27);

    /* Generate all process functions using rasm. */
    asmgen_process(&s, SWS_COMP_ELEMS(1));
    asmgen_process(&s, SWS_COMP_ELEMS(2));
    asmgen_process(&s, SWS_COMP_ELEMS(3));
    asmgen_process(&s, SWS_COMP_ELEMS(4));

    /* Generate all functions from ops_entries.c using rasm. */
    const SwsAArch64OpEntry *entries = ops_entries;
    while (entries->name) {
        asmgen_op_cps(&s, entries++);
        if (rctx->error) {
            ret = rctx->error;
            goto error;
        }
    }

    /* Print all rasm functions to stdout. */
    printf("#include \"libavutil/aarch64/asm.S\"\n");
    printf("\n");
    ret = rasm_print(s.rctx, &bp);
    if (ret < 0)
        goto error;
    fputs(bp.str, stdout);

error:
    av_bprint_finalize(&bp, NULL);
    rasm_free(&s.rctx);
    return ret;
}

/*********************************************************************/
int main(int argc, char *argv[])
{
#ifdef _WIN32
    _setmode(_fileno(stdout), _O_BINARY);
#endif

    return asmgen();
}
