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
#define FF_ARRAY_ELEMS(a) (sizeof(a) / sizeof((a)[0]))
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

#define IMPL_PRIV(s) a64op_off(s->impl, offsetof_impl_priv)

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
typedef struct SwsAArch64OpRegs {
    RasmOp sl[ 4]; /* input vector registers (low bank) */
    RasmOp sh[ 4]; /* input vector registers (high bank) */
    RasmOp dl[ 4]; /* output vector registers (low bank) */
    RasmOp dh[ 4]; /* output vector registers (high bank) */
    RasmOp vt[12]; /* temp vector registers */
    RasmOp vk[ 4]; /* constant data (may be gprs) */

    /* Op-specific registers. */
    union {
        RasmOp dither_ptr;
        RasmOp linear_vcoeff[4][5];
    };
} SwsAArch64OpRegs;

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
    SwsAArch64OpRegs regs;

    /* Read/Write data pointers and padding. */
    RasmOp in[4];
    RasmOp out[4];
    RasmOp in_bump[4];
    RasmOp out_bump[4];

    /* Process function. */
    RasmNode *setup;
    RasmNode *loop;

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

/* Reshape input/output vector registers for current SwsOp. */
static void reshape_io_vectors(SwsAArch64OpRegs *regs, int el_count, int el_size)
{
    for (int i = 0; i < 4; i++) {
        regs->sl[i] = a64op_make_vec(a64op_vec_n(regs->sl[i]), el_count, el_size);
        regs->sh[i] = a64op_make_vec(a64op_vec_n(regs->sh[i]), el_count, el_size);
        regs->dl[i] = a64op_make_vec(a64op_vec_n(regs->dl[i]), el_count, el_size);
        regs->dh[i] = a64op_make_vec(a64op_vec_n(regs->dh[i]), el_count, el_size);
    }
}

/* Reshape temp vector registers for current SwsOp. */
static void reshape_temp_vectors(SwsAArch64OpRegs *regs, int el_count, int el_size)
{
    for (int i = 0; i < FF_ARRAY_ELEMS(regs->vt); i++)
        regs->vt[i] = a64op_make_vec(a64op_vec_n(regs->vt[i]), el_count, el_size);
}

/* Reshape const vector registers for current SwsOp. */
static void reshape_const_vectors(SwsAArch64OpRegs *regs, int el_count, int el_size)
{
    for (int i = 0; i < FF_ARRAY_ELEMS(regs->vk); i++)
        regs->vk[i] = a64op_make_vec(a64op_vec_n(regs->vk[i]), el_count, el_size);
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
                               SwsCompMask imask, SwsCompMask omask,
                               RasmOp regs[MAX_SAVED_REGS])
{
    unsigned count = 0;
    clobber_gpr(regs, &count, a64op_lr());
    LOOP(imask, i) {
        clobber_gpr(regs, &count, s->in[i]);
        clobber_gpr(regs, &count, s->in_bump[i]);
    }
    LOOP(omask, i) {
        clobber_gpr(regs, &count, s->out[i]);
        clobber_gpr(regs, &count, s->out_bump[i]);
    }
    return count;
}

static void asmgen_process(SwsAArch64Context *s, SwsCompMask imask, SwsCompMask omask)
{
    RasmContext *r = s->rctx;

    /**
     * The process function for aarch64 works similarly to the x86 backend.
     * The description in x86/ops_include.asm mostly holds as well here.
     */

    /* Function prologue */
    RasmOp saved_regs[MAX_SAVED_REGS];
    unsigned nsaved = clobbered_gprs(s, imask, omask, saved_regs);
    if (nsaved)
        asmgen_prologue(s, saved_regs, nsaved);

    /* Load values from exec. */
    RasmOp exec_in[4];
    RasmOp exec_in_bump[4];
    RasmOp exec_out[4];
    RasmOp exec_out_bump[4];
    LOOP(imask, i) { exec_in      [i] = a64op_off(s->exec, offsetof_exec_in       + (i * sizeof(uint8_t *))); }
    LOOP(imask, i) { exec_in_bump [i] = a64op_off(s->exec, offsetof_exec_in_bump  + (i * sizeof(uint8_t *))); }
    LOOP(omask, i) { exec_out     [i] = a64op_off(s->exec, offsetof_exec_out      + (i * sizeof(uint8_t *))); }
    LOOP(omask, i) { exec_out_bump[i] = a64op_off(s->exec, offsetof_exec_out_bump + (i * sizeof(uint8_t *))); }
    LOOP(imask, i) { i_ldr(r, s->in[i],       exec_in [i]);         CMTF("in[%u] = exec->in[%u];", i, i); }
    LOOP(omask, i) { i_ldr(r, s->out[i],      exec_out[i]);         CMTF("out[%u] = exec->out[%u];", i, i); }
    LOOP(imask, i) { i_ldr(r, s->in_bump[i],  exec_in_bump[i]);     CMTF("in_bump[%u] = exec->in_bump[%u];", i, i); }
    LOOP(omask, i) { i_ldr(r, s->out_bump[i], exec_out_bump[i]);    CMTF("out_bump[%u] = exec->out_bump[%u];", i, i); }

    /* Setup. */
    s->setup = rasm_get_current_node(r);

    int first_row  = rasm_new_label(r, NULL);
    int next_row   = rasm_new_label(r, NULL);
    int next_block = rasm_new_label(r, NULL);

    /* Jump to first row (skips padding). */
    i_b  (r, rasm_op_label(first_row));     CMT("goto first_row;");

    /* Perform padding, preparing for next row. */
    rasm_add_label(r, next_row);            CMT("next_row:");
    LOOP(imask, i) { i_add(r, s->in[i],  s->in[i],  s->in_bump[i]);  CMTF("in[%u] += in_bump[%u];", i, i); }
    LOOP(omask, i) { i_add(r, s->out[i], s->out[i], s->out_bump[i]); CMTF("out[%u] += out_bump[%u];", i, i); }

    /* First row (reset x). */
    rasm_add_label(r, first_row);           CMT("first_row:");
    i_mov(r, s->bx, s->bx_start);           CMT("bx = bx_start;");

    /* Main loop. */
    rasm_add_label(r, next_block);          CMT("next_block:");
    s->loop = rasm_get_current_node(r);

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

static void asmgen_setup_read_bit(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                  SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    AArch64VecViews shift_vec   = a64op_vec_views(regs->vk[0]);
    AArch64VecViews bitmask_vec = a64op_vec_views(regs->vk[1]);

    rasm_annotate_next(r, "v128 shift_vec = impl->priv.v128;");
    i_ldr(r, shift_vec.q, IMPL_PRIV(s));
    asmgen_set_load_cont_node(s);
    if (p->block_size == 16) {
        i_movi(r, bitmask_vec.b16, IMM(1));                 CMT("v128 bitmask_vec = {1 <repeats 16 times>};");
    } else {
        i_movi(r, bitmask_vec.b8,  IMM(1));                 CMT("v128 bitmask_vec = {1 <repeats 8 times>, 0 <repeats 8 times>};");
    }
}

static void asmgen_op_read_bit(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                               SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    AArch64VecViews dl[1]       = { a64op_vec_views(regs->dl[0]) };
    AArch64VecViews shift_vec   = a64op_vec_views(regs->vk[0]);
    AArch64VecViews bitmask_vec = a64op_vec_views(regs->vk[1]);

    AArch64VecViews vtmp = a64op_vec_views(regs->vt[0]);
    RasmOp wtmp          = a64op_w(s->tmp0);

    /* Note that shift_vec has negative values, so that using it with
     * ushl actually performs a right shift. */
    if (p->block_size == 16) {
        i_ldrh(r, wtmp,        a64op_post(s->in[0], 2));    CMT("uint16_t tmp = *in[0]++;");
        i_dup (r, dl[0].b8,    wtmp);                       CMT("vl[0].lo = broadcast(tmp);");
        i_lsr (r, wtmp,        wtmp, IMM(8));               CMT("tmp >>= 8;");
        i_dup (r, vtmp.b8,     wtmp);                       CMT("vtmp.lo = broadcast(tmp);");
        i_ins (r, dl[0].de[1], vtmp.de[0]);                 CMT("vl[0].hi = vtmp.lo;");
        i_ushl(r, dl[0].b16,   dl[0].b16, shift_vec.b16);   CMT("vl[0] <<= shift_vec;");
        i_and (r, dl[0].b16,   dl[0].b16, bitmask_vec.b16); CMT("vl[0] &= bitmask_vec;");
    } else {
        i_ldrb(r, wtmp,        a64op_post(s->in[0], 1));    CMT("uint8_t tmp = *in[0]++;");
        i_dup (r, dl[0].b8,    wtmp);                       CMT("vl[0].lo = broadcast(tmp);");
        i_ushl(r, dl[0].b8,    dl[0].b8,  shift_vec.b8);    CMT("vl[0] <<= shift_vec;");
        i_and (r, dl[0].b8,    dl[0].b8,  bitmask_vec.b8);  CMT("vl[0] &= bitmask_vec;");
    }
}

static void asmgen_setup_read_nibble(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                     SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    AArch64VecViews nibble_mask = a64op_vec_views(regs->vk[0]);

    rasm_annotate_next(r, "v128 nibble_mask = {0xf <repeats 8 times>, 0x0 <repeats 8 times>};");
    i_movi(r, nibble_mask.b8, IMM(0x0f));
}

static void asmgen_op_read_nibble(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                  SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    AArch64VecViews dl[1]       = { a64op_vec_views(regs->dl[0]) };
    AArch64VecViews nibble_mask = a64op_vec_views(regs->vk[0]);

    AArch64VecViews vtmp  = a64op_vec_views(regs->vt[0]);

    if (p->block_size == 8) {
        i_ldr (r, dl[0].s,   a64op_post(s->in[0], 4));  CMT("vl[0] = *in[0]++;");
        i_ushr(r, vtmp.b8,   dl[0].b8, IMM(4));         CMT("vtmp.lo = vl[0] >> 4;");
        i_and (r, dl[0].b8,  dl[0].b8, nibble_mask.b8); CMT("vl[0].lo &= nibble_mask;");
        i_zip1(r, dl[0].b8,  vtmp.b8,  dl[0].b8);       CMT("interleave");
    } else {
        i_ldr (r, dl[0].d,   a64op_post(s->in[0], 8));  CMT("vl[0] = *in[0]++;");
        i_ushr(r, vtmp.b8,   dl[0].b8, IMM(4));         CMT("vtmp.lo = vl[0] >> 4;");
        i_and (r, dl[0].b8,  dl[0].b8, nibble_mask.b8); CMT("vl[0].lo &= nibble_mask;");
        i_zip1(r, dl[0].b16, vtmp.b16, dl[0].b16);      CMT("interleave");
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

static void asmgen_op_read_packed(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                  SwsAArch64OpRegs *regs)
{
    av_assert0(p->mask != 0x0001);
    asmgen_op_read_packed_n(s, p, regs->dl);
    if (s->use_vh)
        asmgen_op_read_packed_n(s, p, regs->dh);
}

static void asmgen_op_read_planar(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                  SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    AArch64VecViews dl[4] = A64OP_VEC_VIEWS4(regs->dl);
    AArch64VecViews dh[4] = A64OP_VEC_VIEWS4(regs->dh);

    LOOP_MASK(p, i) {
        switch ((s->use_vh ? 0x100 : 0) | s->vec_size) {
        case 0x008: i_ldr(r, dl[i].d,          a64op_post(s->in[i], s->vec_size * 1)); break;
        case 0x010: i_ldr(r, dl[i].q,          a64op_post(s->in[i], s->vec_size * 1)); break;
        case 0x108: i_ldp(r, dl[i].d, dh[i].d, a64op_post(s->in[i], s->vec_size * 2)); break;
        case 0x110: i_ldp(r, dl[i].q, dh[i].q, a64op_post(s->in[i], s->vec_size * 2)); break;
        }
    }
}

/*********************************************************************/
/* write raw pixels to planes */
/* SWS_UOP_WRITE_BIT */
/* SWS_UOP_WRITE_NIBBLE */
/* SWS_UOP_WRITE_PACKED */
/* SWS_UOP_WRITE_PLANAR */

static void asmgen_setup_write_bit(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                   SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    AArch64VecViews shift_vec = a64op_vec_views(regs->vk[0]);

    rasm_annotate_next(r, "v128 shift_vec = impl->priv.v128;");
    i_ldr(r, shift_vec.q, IMPL_PRIV(s));
    asmgen_set_load_cont_node(s);
}

static void asmgen_op_write_bit(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    AArch64VecViews sl[1]     = { a64op_vec_views(regs->sl[0]) };
    AArch64VecViews shift_vec = a64op_vec_views(regs->vk[0]);

    AArch64VecViews vtmp0 = a64op_vec_views(regs->vt[0]);
    AArch64VecViews vtmp1 = a64op_vec_views(regs->vt[1]);

    if (p->block_size == 8) {
        i_ushl(r, sl[0].b8,    sl[0].b8,   shift_vec.b8);   CMT("vl[0] <<= shift_vec;");
        i_addv(r, vtmp0.b,     sl[0].b8);                   CMT("vtmp0[0] = add_across(vl[0].lo);");
        i_str (r, vtmp0.b,     a64op_post(s->out[0], 1));   CMT("*out[0]++ = vtmp0;");
    } else {
        i_ushl(r, sl[0].b16,   sl[0].b16,  shift_vec.b16);  CMT("vl[0] <<= shift_vec;");
        i_addv(r, vtmp0.b,     sl[0].b8);                   CMT("vtmp0[0] = add_across(vl[0].lo);");
        i_ins (r, vtmp1.de[0], sl[0].de[1]);                CMT("vtmp1.lo = vl[0].hi;");
        i_addv(r, vtmp1.b,     vtmp1.b8);                   CMT("vtmp1[0] = add_across(vtmp1);");
        i_ins (r, vtmp0.be[1], vtmp1.be[0]);                CMT("vtmp0[1] = vtmp1[0];");
        i_str (r, vtmp0.h,     a64op_post(s->out[0], 2));   CMT("*out[0]++ = vtmp0;");
    }
}

static void asmgen_op_write_nibble(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                   SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    AArch64VecViews sl[4] = A64OP_VEC_VIEWS4(regs->sl);
    AArch64VecViews vtmp0 = a64op_vec_views(regs->vt[0]);
    AArch64VecViews vtmp1 = a64op_vec_views(regs->vt[1]);

    if (p->block_size == 8) {
        i_shl (r, vtmp0.h4,  sl[0].h4,  IMM(4));
        i_ushr(r, vtmp1.h4,  sl[0].h4,  IMM(8));
        i_orr (r, sl[0].b8,  vtmp0.b8,  vtmp1.b8);
        i_xtn (r, vtmp0.b8,  sl[0].h8);
        i_str (r, vtmp0.s,   a64op_post(s->out[0], 4));
    } else {
        i_shl (r, vtmp0.h8,  sl[0].h8,  IMM(4));
        i_ushr(r, vtmp1.h8,  sl[0].h8,  IMM(8));
        i_orr (r, sl[0].b16, vtmp0.b16, vtmp1.b16);
        i_xtn (r, vtmp0.b8,  sl[0].h8);
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

static void asmgen_op_write_packed(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                   SwsAArch64OpRegs *regs)
{
    av_assert0(p->mask != 0x0001);
    asmgen_op_write_packed_n(s, p, regs->sl);
    if (s->use_vh)
        asmgen_op_write_packed_n(s, p, regs->sh);
}

static void asmgen_op_write_planar(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                   SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    AArch64VecViews sl[4] = A64OP_VEC_VIEWS4(regs->sl);
    AArch64VecViews sh[4] = A64OP_VEC_VIEWS4(regs->sh);

    LOOP_MASK(p, i) {
        switch ((s->use_vh ? 0x100 : 0) | s->vec_size) {
        case 0x008: i_str(r, sl[i].d,          a64op_post(s->out[i], s->vec_size * 1)); break;
        case 0x010: i_str(r, sl[i].q,          a64op_post(s->out[i], s->vec_size * 1)); break;
        case 0x108: i_stp(r, sl[i].d, sh[i].d, a64op_post(s->out[i], s->vec_size * 2)); break;
        case 0x110: i_stp(r, sl[i].q, sh[i].q, a64op_post(s->out[i], s->vec_size * 2)); break;
        }
    }
}

/*********************************************************************/
/* swap byte order (for differing endianness) */
/* SWS_UOP_SWAP_BYTES */

static void asmgen_op_swap_bytes(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                 SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    AArch64VecViews sl[4] = A64OP_VEC_VIEWS4(regs->sl);
    AArch64VecViews sh[4] = A64OP_VEC_VIEWS4(regs->sh);
    AArch64VecViews dl[4] = A64OP_VEC_VIEWS4(regs->dl);
    AArch64VecViews dh[4] = A64OP_VEC_VIEWS4(regs->dh);

    switch (ff_sws_pixel_type_size(p->type)) {
    case sizeof(uint16_t):
        LOOP_MASK      (p, i) i_rev16(r, dl[i].b16, sl[i].b16);
        LOOP_MASK_VH(s, p, i) i_rev16(r, dh[i].b16, sh[i].b16);
        break;
    case sizeof(uint32_t):
        LOOP_MASK      (p, i) i_rev32(r, dl[i].b16, sl[i].b16);
        LOOP_MASK_VH(s, p, i) i_rev32(r, dh[i].b16, sh[i].b16);
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

static RasmOp swizzle_a64op(SwsAArch64OpRegs *regs, int8_t n, uint8_t vh, bool dst)
{
    if (n == -1)
        return regs->vt[vh];
    if (vh)
        return dst ? regs->dh[n] : regs->sh[n];
    return dst ? regs->dl[n] : regs->sl[n];
}

static void swizzle_emit(SwsAArch64Context *s, SwsAArch64OpRegs *regs,
                         int8_t dst, int8_t src)
{
    RasmContext *r = s->rctx;
    RasmOp src_op[2] = { swizzle_a64op(regs, src, 0, false), swizzle_a64op(regs, src, 1, false) };
    RasmOp dst_op[2] = { swizzle_a64op(regs, dst, 0, true),  swizzle_a64op(regs, dst, 1, true) };

    i_mov    (r, dst_op[0], src_op[0]); CMTF("%s = %s;", PRINT_SWIZZLE_V(dst, 0), PRINT_SWIZZLE_V(src, 0));
    if (s->use_vh) {
        i_mov(r, dst_op[1], src_op[1]); CMTF("%s = %s;", PRINT_SWIZZLE_V(dst, 1), PRINT_SWIZZLE_V(src, 1));
    }
}

static void asmgen_op_move(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                           SwsAArch64OpRegs *regs)
{
    for (int i = 0; i < p->par.move.num_moves; i++)
        swizzle_emit(s, regs, p->par.move.dst[i], p->par.move.src[i]);
}

/*********************************************************************/
/* split tightly packed data into components */
/* SWS_UOP_UNPACK */

static void asmgen_setup_unpack(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp *vmask = regs->vk;
    RasmOp mask_gpr = a64op_w(s->tmp0);
    uint32_t mask_val[4] = { 0 };

    /* Generate masks. */
    rasm_add_comment(r, "generate masks");
    LOOP_MASK(p, i) {
        uint32_t val = (1u << p->par.pack.pattern[i]) - 1;
        for (int j = 0; j < 4; j++) {
            if (mask_val[j] == val) {
                mask_val[i] = mask_val[j];
                vmask[i] = vmask[j];
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
                i_movi(r, vmask[i], IMM(val));
            } else {
                i_mov (r, mask_gpr, IMM(val));
                i_dup (r, vmask[i], mask_gpr);
            }
            mask_val[i] = val;
            vmask[i] = v_16b(vmask[i]);
        }
    }
}

static void asmgen_op_unpack(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                             SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp *sl    = regs->sl;
    RasmOp *sh    = regs->sh;
    RasmOp *dl    = regs->dl;
    RasmOp *dh    = regs->dh;
    RasmOp *vmask = regs->vk;

    const int offsets[4] = {
        p->par.pack.pattern[3] + p->par.pack.pattern[2] + p->par.pack.pattern[1],
        p->par.pack.pattern[3] + p->par.pack.pattern[2],
        p->par.pack.pattern[3],
        0
    };

    /* Loop backwards to avoid clobbering component 0. */
    LOOP_MASK_BWD      (p, i) {
        if (offsets[i]) {
            i_ushr  (r, dl[i], sl[0], IMM(offsets[i])); CMTF("vl[%u] >>= %u;", i, offsets[i]);
        } else if (i) {
            i_mov16b(r, dl[i], sl[0]);                  CMTF("vl[%u] = vl[0];", i);
        }
    }
    LOOP_MASK_BWD_VH(s, p, i) {
        if (offsets[i]) {
            i_ushr  (r, dh[i], sh[0], IMM(offsets[i])); CMTF("vh[%u] >>= %u;", i, offsets[i]);
        } else if (i) {
            i_mov16b(r, dh[i], sh[0]);                  CMTF("vh[%u] = vh[0];", i);
        }
    }

    /* Apply masks. */
    LOOP_MASK_BWD      (p, i) { i_and16b(r, dl[i], dl[i], vmask[i]); CMTF("vl[%u] &= 0x%x;", i, (1u << p->par.pack.pattern[i]) - 1); }
    LOOP_MASK_BWD_VH(s, p, i) { i_and16b(r, dh[i], dh[i], vmask[i]); CMTF("vh[%u] &= 0x%x;", i, (1u << p->par.pack.pattern[i]) - 1); }
}

/*********************************************************************/
/* compress components into tightly packed data */
/* SWS_UOP_PACK */

static void asmgen_op_pack(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                           SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp *sl = regs->sl;
    RasmOp *sh = regs->sh;
    RasmOp *dl = regs->dl;
    RasmOp *dh = regs->dh;

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
    LOOP      (offset_mask, i) { i_shl(r, dl[i], sl[i], IMM(offsets[i])); CMTF("vl[%u] <<= %u;", i, offsets[i]); }
    LOOP_VH(s, offset_mask, i) { i_shl(r, dh[i], sh[i], IMM(offsets[i])); CMTF("vh[%u] <<= %u;", i, offsets[i]); }
    LOOP      (offset_mask, i) { sl[i] = dl[i]; }
    LOOP_VH(s, offset_mask, i) { sh[i] = dh[i]; }

    /* Combine components. */
    for (int i = 0; i < 4; i++) {
        sl[i] = v_16b(sl[i]);
        sh[i] = v_16b(sh[i]);
        dl[i] = v_16b(dl[i]);
        dh[i] = v_16b(dh[i]);
    }
    LOOP_MASK      (p, i) {
        if (i != 0) {
            i_orr16b    (r, dl[0], sl[0], sl[i]); CMTF("vl[0] |= vl[%u];", i);
            if (s->use_vh) {
                i_orr16b(r, dh[0], sh[0], sh[i]); CMTF("vh[0] |= vh[%u];", i);
            }
        }
    }
}

/*********************************************************************/
/* logical left shift of raw pixel values */
/* SWS_UOP_LSHIFT */

static void asmgen_op_lshift(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                             SwsAArch64OpRegs *regs)
{
    uint8_t shift = p->par.shift.amount;
    RasmContext *r = s->rctx;
    RasmOp *sl = regs->sl;
    RasmOp *sh = regs->sh;
    RasmOp *dl = regs->dl;
    RasmOp *dh = regs->dh;

    LOOP_MASK      (p, i) { i_shl(r, dl[i], sl[i], IMM(shift)); CMTF("vl[%u] <<= %u;", i, shift); }
    LOOP_MASK_VH(s, p, i) { i_shl(r, dh[i], sh[i], IMM(shift)); CMTF("vh[%u] <<= %u;", i, shift); }
}

/*********************************************************************/
/* right shift of raw pixel values */
/* SWS_UOP_RSHIFT */

static void asmgen_op_rshift(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                             SwsAArch64OpRegs *regs)
{
    uint8_t shift = p->par.shift.amount;
    RasmContext *r = s->rctx;
    RasmOp *sl = regs->sl;
    RasmOp *sh = regs->sh;
    RasmOp *dl = regs->dl;
    RasmOp *dh = regs->dh;

    LOOP_MASK      (p, i) { i_ushr(r, dl[i], sl[i], IMM(shift)); CMTF("vl[%u] >>= %u;", i, shift); }
    LOOP_MASK_VH(s, p, i) { i_ushr(r, dh[i], sh[i], IMM(shift)); CMTF("vh[%u] >>= %u;", i, shift); }
}

/*********************************************************************/
/* clear pixel values */
/* SWS_UOP_CLEAR */

static void asmgen_setup_clear(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                               SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp *vk = regs->vk;

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
        i_ldr(r, v_q(vk[0]), IMPL_PRIV(s));         CMT("v128 clear_vec = impl->priv.v128;");
        asmgen_set_load_cont_node(s);
    }
}

static void emit_clear(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                       RasmOp *vx, RasmOp *vk, int i, const char *vx_str)
{
    RasmContext *r = s->rctx;
    RasmOp clear_vec = vk[0];
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

static void asmgen_op_clear(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                            SwsAArch64OpRegs *regs)
{
    RasmOp *dl = regs->dl;
    RasmOp *dh = regs->dh;
    RasmOp *vk = regs->vk;

    LOOP_MASK      (p, i) { emit_clear(s, p, dl, vk, i, "vl"); }
    LOOP_MASK_VH(s, p, i) { emit_clear(s, p, dh, vk, i, "vh"); }
}

/*********************************************************************/
/* convert (cast) between formats */
/* SWS_UOP_TO_U8 */
/* SWS_UOP_TO_U16 */
/* SWS_UOP_TO_U32 */
/* SWS_UOP_TO_F32 */

static void asmgen_op_convert(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                              SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    AArch64VecViews sl[4] = A64OP_VEC_VIEWS4(regs->sl);
    AArch64VecViews sh[4] = A64OP_VEC_VIEWS4(regs->sh);
    AArch64VecViews dl[4] = A64OP_VEC_VIEWS4(regs->dl);
    AArch64VecViews dh[4] = A64OP_VEC_VIEWS4(regs->dh);

    /**
     * Since each instruction in the convert operation needs specific
     * element types, it is simpler to use arrangement specifiers for
     * each operand instead of reshaping all vectors.
     */

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
        LOOP_MASK(p, i) i_fcvtzu(r, dl[i].s4, sl[i].s4);
        LOOP_MASK(p, i) i_fcvtzu(r, dh[i].s4, sh[i].s4);
        memcpy(sl, dl, sizeof(sl));
        memcpy(sh, dh, sizeof(sh));
    }

    if (p->block_size == 8) {
        if (src_el_size == 1 && dst_el_size > src_el_size) {
            rasm_add_comment(r, "u8 -> u16");
            LOOP_MASK(p, i) i_uxtl (r, dl[i].h8,    sl[i].b8);
            memcpy(sl, dl, sizeof(sl));
            src_el_size = 2;
        } else if (src_el_size == 4 && dst_el_size < src_el_size) {
            rasm_add_comment(r, "u32 -> u16");
            LOOP_MASK(p, i) i_xtn  (r, dl[i].h4,    sl[i].s4);
            LOOP_MASK(p, i) i_xtn  (r, dh[i].h4,    sh[i].s4);
            LOOP_MASK(p, i) i_ins  (r, dl[i].de[1], sh[i].de[0]);
            memcpy(sl, dl, sizeof(sl));
            memcpy(sh, dh, sizeof(sh));
            src_el_size = 2;
        }
        if (src_el_size == 2 && dst_el_size == 4) {
            rasm_add_comment(r, "u16 -> u32");
            LOOP_MASK(p, i) i_uxtl2(r, dh[i].s4,    sl[i].h8);
            LOOP_MASK(p, i) i_uxtl (r, dl[i].s4,    sl[i].h4);
            memcpy(sl, dl, sizeof(sl));
            memcpy(sh, dh, sizeof(sh));
            src_el_size = 4;
        } else if (src_el_size == 2 && dst_el_size == 1) {
            rasm_add_comment(r, "u16 -> u8");
            LOOP_MASK(p, i) i_xtn  (r, dl[i].b8,    sl[i].h8);
            memcpy(sl, dl, sizeof(sl));
            src_el_size = 1;
        }
    } else /* if (p->block_size == 16) */ {
        if (src_el_size == 1 && dst_el_size == 2) {
            rasm_add_comment(r, "u8 -> u16");
            LOOP_MASK(p, i) i_uxtl2(r, dh[i].h8,    sl[i].b16);
            LOOP_MASK(p, i) i_uxtl (r, dl[i].h8,    sl[i].b8);
            memcpy(sl, dl, sizeof(sl));
            memcpy(sh, dh, sizeof(sh));
        } else if (src_el_size == 2 && dst_el_size == 1) {
            rasm_add_comment(r, "u16 -> u8");
            LOOP_MASK(p, i) i_xtn  (r, dl[i].b8,    sl[i].h8);
            LOOP_MASK(p, i) i_xtn  (r, dh[i].b8,    sh[i].h8);
            LOOP_MASK(p, i) i_ins  (r, dl[i].de[1], sh[i].de[0]);
            memcpy(sl, dl, sizeof(sl));
        }
    }

    /* See comment above for high vector bank usage for u32. */
    if (to_type == SWS_PIXEL_F32) {
        rasm_add_comment(r, "u32 -> f32");
        LOOP_MASK(p, i) i_ucvtf(r, dl[i].s4, sl[i].s4);
        LOOP_MASK(p, i) i_ucvtf(r, dh[i].s4, sh[i].s4);
    }
}

/*********************************************************************/
/* expand integers to the full range */
/* SWS_UOP_EXPAND_PAIR */
/* SWS_UOP_EXPAND_QUAD */

static void asmgen_op_expand(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                             SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp *sl = regs->sl;
    RasmOp *dl = regs->dl;
    RasmOp *dh = regs->dh;

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
        reshape_io_vectors(regs, 16, 1);
        LOOP_MASK_VH(s, p, i) i_zip2(r, dh[i], sl[i], sl[i]);
        LOOP_MASK      (p, i) i_zip1(r, dl[i], sl[i], sl[i]);
        sl = dl;
    }
    if (dst_el_size == 4) {
        rasm_add_comment(r, "u16 -> u32");
        reshape_io_vectors(regs, 8, 2);
        LOOP_MASK_VH(s, p, i) i_zip2(r, dh[i], sl[i], sl[i]);
        LOOP_MASK      (p, i) i_zip1(r, dl[i], sl[i], sl[i]);
    }
}

/*********************************************************************/
/* numeric minimum */
/* SWS_UOP_MIN */

static void asmgen_setup_min(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                             SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp *vk = regs->vk;

    RasmOp min_vec = regs->vt[0];
    i_ldr(r, v_q(min_vec), IMPL_PRIV(s));                           CMT("v128 min_vec = impl->priv.v128;");
    asmgen_set_load_cont_node(s);
    LOOP_MASK(p, i) { i_dup(r, vk[i], a64op_elem(min_vec, i));      CMTF("v128 vmin%u = min_vec[%u];", i, i); }
}

static void asmgen_op_min(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                          SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp *sl = regs->sl;
    RasmOp *sh = regs->sh;
    RasmOp *dl = regs->dl;
    RasmOp *dh = regs->dh;
    RasmOp *vk = regs->vk;

    if (p->type == SWS_PIXEL_F32) {
        LOOP_MASK      (p, i) { i_fmin(r, dl[i], sl[i], vk[i]);     CMTF("vl[%u] = min(vl[%u], vmin%u);", i, i, i); }
        LOOP_MASK_VH(s, p, i) { i_fmin(r, dh[i], sh[i], vk[i]);     CMTF("vh[%u] = min(vh[%u], vmin%u);", i, i, i); }
    } else {
        LOOP_MASK      (p, i) { i_umin(r, dl[i], sl[i], vk[i]);     CMTF("vl[%u] = min(vl[%u], vmin%u);", i, i, i); }
        LOOP_MASK_VH(s, p, i) { i_umin(r, dh[i], sh[i], vk[i]);     CMTF("vh[%u] = min(vh[%u], vmin%u);", i, i, i); }
    }
}

/*********************************************************************/
/* numeric maximum */
/* SWS_UOP_MAX */

static void asmgen_setup_max(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                             SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp *vk = regs->vk;

    RasmOp max_vec = regs->vt[0];
    i_ldr(r, v_q(max_vec), IMPL_PRIV(s));                           CMT("v128 max_vec = impl->priv.v128;");
    asmgen_set_load_cont_node(s);
    LOOP_MASK(p, i) { i_dup(r, vk[i], a64op_elem(max_vec, i));      CMTF("v128 vmax%u = max_vec[%u];", i, i); }
}

static void asmgen_op_max(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                          SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp *sl = regs->sl;
    RasmOp *sh = regs->sh;
    RasmOp *dl = regs->dl;
    RasmOp *dh = regs->dh;
    RasmOp *vk = regs->vk;

    if (p->type == SWS_PIXEL_F32) {
        LOOP_MASK      (p, i) { i_fmax(r, dl[i], sl[i], vk[i]);     CMTF("vl[%u] = max(vl[%u], vmax%u);", i, i, i); }
        LOOP_MASK_VH(s, p, i) { i_fmax(r, dh[i], sh[i], vk[i]);     CMTF("vh[%u] = max(vh[%u], vmax%u);", i, i, i); }
    } else {
        LOOP_MASK      (p, i) { i_umax(r, dl[i], sl[i], vk[i]);     CMTF("vl[%u] = max(vl[%u], vmax%u);", i, i, i); }
        LOOP_MASK_VH(s, p, i) { i_umax(r, dh[i], sh[i], vk[i]);     CMTF("vh[%u] = max(vh[%u], vmax%u);", i, i, i); }
    }
}

/*********************************************************************/
/* multiplication by scalar */
/* SWS_UOP_SCALE */

static void asmgen_setup_scale(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                               SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp scale_vec = regs->vk[0];

    RasmOp priv_ptr = s->tmp0;
    i_add (r, priv_ptr, s->impl, IMM(offsetof_impl_priv));          CMT("v128 *scale_vec_ptr = &impl->priv;");
    asmgen_set_load_cont_node(s);
    i_ld1r(r, vv_1(scale_vec), a64op_base(priv_ptr));               CMT("v128 scale_vec = broadcast(*scale_vec_ptr);");
}

static void asmgen_op_scale(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                            SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp *sl       = regs->sl;
    RasmOp *sh       = regs->sh;
    RasmOp *dl       = regs->dl;
    RasmOp *dh       = regs->dh;
    RasmOp scale_vec = regs->vk[0];

    if (p->type == SWS_PIXEL_F32) {
        LOOP_MASK      (p, i) { i_fmul(r, dl[i], sl[i], scale_vec); CMTF("vl[%u] *= scale_vec;", i); }
        LOOP_MASK_VH(s, p, i) { i_fmul(r, dh[i], sh[i], scale_vec); CMTF("vh[%u] *= scale_vec;", i); }
    } else {
        LOOP_MASK      (p, i) { i_mul (r, dl[i], sl[i], scale_vec); CMTF("vl[%u] *= scale_vec;", i); }
        LOOP_MASK_VH(s, p, i) { i_mul (r, dh[i], sh[i], scale_vec); CMTF("vh[%u] *= scale_vec;", i); }
    }
}

/*********************************************************************/
/* generalized linear affine transform */
/* SWS_UOP_LINEAR */
/* SWS_UOP_LINEAR_FMA */

static void asmgen_setup_linear(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp *sl = regs->sl;
    RasmOp *sh = regs->sh;
    RasmOp *vc = regs->vk;
    RasmOp *vt = regs->vt;

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
    i_ldr(r, ptr, IMPL_PRIV(s));                            CMT("v128 *vcoeff_ptr = impl->priv.ptr;");
    asmgen_set_load_cont_node(s);
    i_ld1(r, coeff_veclist, a64op_base(ptr));               CMT("coeff_veclist = *vcoeff_ptr;");

    /**
     * Populate operands matrix from packed data into linear_vcoeff matrix
     * and compute mask for rows that must be saved before being overwritten.
     */
    SwsCompMask save_mask = 0;
    bool overwritten[4] = { false, false, false, false };
    int i_coeff = 0;
    LOOP_MASK(p, i) {
        for (int j = 0; j < 5; j++) {
            bool is_offset = (j == 0);
            int src_j = is_offset ? 4 : (j - 1);
            if (p->par.lin.zero & SWS_MASK(i, src_j))
                continue;
            uint8_t vc_i = i_coeff / 4;
            uint8_t vc_j = i_coeff & 3;
            regs->linear_vcoeff[i][j] = a64op_elem(vc[vc_i], vc_j);
            i_coeff++;
            if (!is_offset && overwritten[src_j])
                save_mask |= SWS_COMP(src_j);
            overwritten[i] = true;
        }
    }

    /**
     * Save rows that need to be used as input after they have been already
     * written to.
     */
    RasmOp *tl = &vt[0];
    RasmOp *th = &vt[4];
    LOOP      (save_mask, i) { i_mov16b(r, tl[i], sl[i]);  CMTF("vsrcl[%u] = vl[%u];", i, i); }
    LOOP_VH(s, save_mask, i) { i_mov16b(r, th[i], sh[i]);  CMTF("vsrch[%u] = vh[%u];", i, i); }
    LOOP      (save_mask, i) { sl[i] = tl[i]; }
    LOOP_VH(s, save_mask, i) { sh[i] = th[i]; }
}

/**
 * Performs one pass of the linear transform over a single vector bank
 * (low or high).
 */
static void linear_pass(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                        SwsAArch64OpRegs *regs, bool vh_pass)
{
    RasmContext *r = s->rctx;
    /**
     * The intermediate registers for fmul+fadd (for when SWS_BITEXACT
     * is set) start from temp vector 8.
     */
    RasmOp *vt = regs->vt;
    RasmOp *vtmp = &vt[8];
    RasmOp *sx = vh_pass ? regs->sh : regs->sl;
    RasmOp *dx = vh_pass ? regs->dh : regs->dl;
    char cvh = vh_pass ? 'h' : 'l';

    if (vh_pass && !s->use_vh)
        return;

    /**
     * The non-zero coefficients have been packed in aarch64_setup_linear()
     * in sequential order into the individual lanes of the coefficient
     * vector registers. We must follow the same order of execution here.
     */
    LOOP_MASK(p, i) {
        bool first = true;
        RasmNode *pre_mul = rasm_get_current_node(r);
        for (int j = 0; j < 5; j++) {
            bool is_offset = (j == 0);
            int src_j = is_offset ? 4 : (j - 1);
            if (p->par.lin.zero & SWS_MASK(i, src_j))
                continue;
            RasmOp vsrc = sx[src_j];
            RasmOp vcoeff = regs->linear_vcoeff[i][j];
            if (first && is_offset) {
                i_dup (r, dx[i], vcoeff);               CMTF("v%c[%u]  = broadcast(offset[%u]);", cvh, i, i);
            } else if (first && !is_offset) {
                if (p->par.lin.one & SWS_MASK(i, src_j)) {
                    i_mov16b(r, dx[i], vsrc);           CMTF("v%c[%u]  = vsrc%c[%u];", cvh, i, cvh, src_j);
                } else {
                    i_fmul  (r, dx[i], vsrc, vcoeff);   CMTF("v%c[%u]  = vsrc%c[%u] * coeff[%u][%u];", cvh, i, cvh, src_j, i, src_j);
                }
            } else if (p->uop == SWS_UOP_LINEAR_FMA) {
                /**
                 * Most modern aarch64 cores have a fastpath for sequences
                 * of fmla instructions. This means that even if the coefficient
                 * is 1, it is still faster to use fmla by 1 instead of fadd.
                 */
                i_fmla(r, dx[i], vsrc, vcoeff);         CMTF("v%c[%u] += vsrc%c[%u] * coeff[%u][%u];", cvh, i, cvh, src_j, i, src_j);
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
                    i_fmul(r, vtmp[src_j], vsrc, vcoeff);   CMTF("vtmp[%u] = vsrc%c[%u] * coeff[%u][%u];", src_j, cvh, src_j, i, src_j);
                    pre_mul = rasm_set_current_node(r, pre_mul);
                    i_fadd(r, dx[i], dx[i], vtmp[src_j]);   CMTF("v%c[%u] += vtmp[%u];", cvh, i, src_j);
                } else {
                    i_fadd(r, dx[i], dx[i], vsrc);          CMTF("v%c[%u] += vsrc%c[%u];", cvh, i, cvh, src_j);
                }
            }
            first = false;
        }
    }
}

static void asmgen_op_linear(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                             SwsAArch64OpRegs *regs)
{
    /* Perform linear passes for low and high vector banks. */
    linear_pass(s, p, regs, false);
    linear_pass(s, p, regs, true);
}

/*********************************************************************/
/* add dithering noise */
/* SWS_UOP_DITHER */

static void asmgen_setup_dither(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                                SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp src_ptr = s->tmp0;

    regs->dither_ptr = src_ptr;
    i_ldr(r, src_ptr, IMPL_PRIV(s));                        CMT("void *ptr = impl->priv.ptr;");
    asmgen_set_load_cont_node(s);
}

static void asmgen_op_dither(SwsAArch64Context *s, const SwsAArch64OpImplParams *p,
                             SwsAArch64OpRegs *regs)
{
    RasmContext *r = s->rctx;
    RasmOp *sl = regs->sl;
    RasmOp *sh = regs->sh;
    RasmOp *dl = regs->dl;
    RasmOp *dh = regs->dh;
    RasmOp src_ptr = regs->dither_ptr;

    RasmOp ptr = s->tmp0;
    RasmOp tmp1 = s->tmp1;
    RasmOp wtmp1 = a64op_w(tmp1);
    RasmOp dither_vl = regs->vt[0];
    RasmOp dither_vh = regs->vt[1];
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
        i_add  (r, ptr,  src_ptr,  tmp1);   CMT("ptr += tmp1;");
        src_ptr = ptr;
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
            i_add(r, ptr, src_ptr, tmp1);                   CMT("ptr += tmp1;");
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

        i_fadd    (r, dl[i], sl[i], dither_vl);             CMTF("vl[%u] += vditherl;", i);
        if (s->use_vh) {
            i_fadd(r, dh[i], sh[i], dither_vh);             CMTF("vh[%u] += vditherh;", i);
        }
        sl = dl;
        sh = dh;

        last_y_off = y_off;
        prev_i = i;
    }
}

/*********************************************************************/
/**
 * Register assignment for CPS functions.
 *
 * The entry point of the SwsOpFunc is the `process` function. The
 * first kernel function is called from `process`, and subsequent
 * kernel functions are chained by directly branching to the next
 * operation, using a continuation-passing style design. The last
 * operation must be a write operation, which returns from the call
 * to the `process` function.
 *
 * The GPRs used by the entire call-chain are listed below.
 *
 * Function arguments are passed in r0-r5. After the parameters from
 * `exec` have been read, r0 is reused to branch to the continuation
 * functions. After the original parameters from `impl` have been
 * computed, r1 is reused as the `impl` pointer for each operation.
 *
 * Loop iterators are r6 for `bx` and r3 for `y`, reused from
 * `y_start`, which doesn't need to be preserved.
 *
 * The intra-procedure-call temporary registers (r16 and r17) are used
 * as scratch registers. They may be used by call veneers and PLT code
 * inserted by the linker, so we cannot expect them to persist across
 * branches between functions.
 *
 * The Platform Register (r18) is not used.
 *
 * The read/write data pointers and padding values first use up the
 * remaining free caller-saved registers, and only then are the
 * caller-saved registers (r19-r29) used.
 *
 * The Link Register (r30) is used when calling the first kernel, so it
 * must be saved.
 */

static const int rw_gprs[] = {
     9, 10, 11, 12,
    13, 14, 15, 19,
    20, 21, 22, 23,
    24, 25, 26, 27,
};

static void asmgen_common_frame(SwsAArch64Context *s, SwsCompMask imask, SwsCompMask omask)
{
    /* Loop iterator variables. */
    s->bx        = a64op_gpw(6);
    s->y         = a64op_gpw(3);    /* Reused from SwsOpFunc.y_start argument. */

    /* Scratch registers. */
    s->tmp0      = a64op_gpx(16);   /* IP0 */
    s->tmp1      = a64op_gpx(17);   /* IP1 */

    /* Read/Write data pointers. */
    LOOP(imask, i) { s->in [i] = a64op_gpx(rw_gprs[(i * 4) + 0]); }
    LOOP(omask, i) { s->out[i] = a64op_gpx(rw_gprs[(i * 4) + 1]); }
}

static void asmgen_process_frame(SwsAArch64Context *s, SwsCompMask imask, SwsCompMask omask)
{
    asmgen_common_frame(s, imask, omask);

    /* SwsOpFunc arguments. */
    s->exec      = a64op_gpx(0);    // const SwsOpExec *exec
    s->impl      = a64op_gpx(1);    // const void *priv
    s->bx_start  = a64op_gpw(2);    // int bx_start
    s->y_start   = a64op_gpw(3);    // int y_start
    s->bx_end    = a64op_gpw(4);    // int bx_end
    s->y_end     = a64op_gpw(5);    // int y_end

    /* CPS-related variables. */
    s->op0_func  = a64op_gpx(7);
    s->op1_impl  = a64op_gpx(8);

    /* Read/Write data pointer padding. */
    LOOP(imask, i) { s->in_bump [i] = a64op_gpx(rw_gprs[(i * 4) + 2]); }
    LOOP(omask, i) { s->out_bump[i] = a64op_gpx(rw_gprs[(i * 4) + 3]); }
}

static void asmgen_op_frame(SwsAArch64Context *s, SwsCompMask imask, SwsCompMask omask)
{
    asmgen_common_frame(s, imask, omask);

    /* CPS-related variables. */
    s->cont      = a64op_gpx(0);    /* Reused from SwsOpFunc.exec argument. */
    s->impl      = a64op_gpx(1);    /* Same as SwsOpFunc.impl argument. */
}

/*********************************************************************/
/* Vector register assignment. */
static void init_vectors_cps(SwsAArch64Context *s, SwsAArch64OpRegs *regs)
{
    regs->sl[ 0] = a64op_vec( 0);
    regs->sl[ 1] = a64op_vec( 1);
    regs->sl[ 2] = a64op_vec( 2);
    regs->sl[ 3] = a64op_vec( 3);
    regs->sh[ 0] = a64op_vec( 4);
    regs->sh[ 1] = a64op_vec( 5);
    regs->sh[ 2] = a64op_vec( 6);
    regs->sh[ 3] = a64op_vec( 7);
    regs->dl[ 0] = a64op_vec( 0);
    regs->dl[ 1] = a64op_vec( 1);
    regs->dl[ 2] = a64op_vec( 2);
    regs->dl[ 3] = a64op_vec( 3);
    regs->dh[ 0] = a64op_vec( 4);
    regs->dh[ 1] = a64op_vec( 5);
    regs->dh[ 2] = a64op_vec( 6);
    regs->dh[ 3] = a64op_vec( 7);
    regs->vt[ 0] = a64op_vec(16);
    regs->vt[ 1] = a64op_vec(17);
    regs->vt[ 2] = a64op_vec(18);
    regs->vt[ 3] = a64op_vec(19);
    regs->vt[ 4] = a64op_vec(20);
    regs->vt[ 5] = a64op_vec(21);
    regs->vt[ 6] = a64op_vec(22);
    regs->vt[ 7] = a64op_vec(23);
    regs->vt[ 8] = a64op_vec(24);
    regs->vt[ 9] = a64op_vec(25);
    regs->vt[10] = a64op_vec(26);
    regs->vt[11] = a64op_vec(27);
    regs->vk[ 0] = a64op_vec(28);
    regs->vk[ 1] = a64op_vec(29);
    regs->vk[ 2] = a64op_vec(30);
    regs->vk[ 3] = a64op_vec(31);
}

/*********************************************************************/
static void asmgen_process_cps(SwsAArch64Context *s, SwsCompMask mask)
{
    RasmContext *r = s->rctx;
    char func_name[128];

    snprintf(func_name, sizeof(func_name), "ff_sws_process_%04x_neon", nibble_mask(mask));
    rasm_func_begin(r, func_name, true, false);
    asmgen_process_frame(s, mask, mask);

    asmgen_process(s, mask, mask);

    /* Load values from impl. */
    rasm_set_current_node(r, s->setup);
    RasmOp impl_cont = a64op_off(s->impl, offsetof_impl_cont);
    i_ldr(r, s->op0_func, impl_cont);                   CMT("SwsFuncPtr op0_func = impl->cont;");
    i_add(r, s->op1_impl, s->impl, IMM(sizeof_impl));   CMT("SwsOpImpl *op1_impl = impl + 1;");

    /* Reset impl and call first kernel. */
    rasm_set_current_node(r, s->loop);
    i_mov(r, s->impl, s->op1_impl);                     CMT("impl = op1_impl;");
    i_blr(r, s->op0_func);                              CMT("op0_func();");
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
    asmgen_op_frame(s, is_read ? p->mask : 0, is_write ? p->mask : 0);

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
    init_vectors_cps(s, &s->regs);
    reshape_io_vectors(&s->regs, s->el_count, el_size);
    reshape_temp_vectors(&s->regs, s->el_count, el_size);
    reshape_const_vectors(&s->regs, s->el_count, el_size);

    /* Common start for continuation-passing style (CPS) functions. */
    asmgen_set_load_cont_node(s);

    /* Set up constants. */
    switch (p->uop) {
    case SWS_UOP_READ_BIT:     asmgen_setup_read_bit(s, p, &s->regs);     break;
    case SWS_UOP_READ_NIBBLE:  asmgen_setup_read_nibble(s, p, &s->regs);  break;
    case SWS_UOP_WRITE_BIT:    asmgen_setup_write_bit(s, p, &s->regs);    break;
    case SWS_UOP_UNPACK:       asmgen_setup_unpack(s, p, &s->regs);       break;
    case SWS_UOP_CLEAR:        asmgen_setup_clear(s, p, &s->regs);        break;
    case SWS_UOP_MIN:          asmgen_setup_min(s, p, &s->regs);          break;
    case SWS_UOP_MAX:          asmgen_setup_max(s, p, &s->regs);          break;
    case SWS_UOP_SCALE:        asmgen_setup_scale(s, p, &s->regs);        break;
    case SWS_UOP_LINEAR:       asmgen_setup_linear(s, p, &s->regs);       break;
    case SWS_UOP_LINEAR_FMA:   asmgen_setup_linear(s, p, &s->regs);       break;
    case SWS_UOP_DITHER:       asmgen_setup_dither(s, p, &s->regs);       break;
    default:
        break;
    }

    /* Emit uop kernel. */
    switch (p->uop) {
    case SWS_UOP_READ_BIT:     asmgen_op_read_bit(s, p, &s->regs);     break;
    case SWS_UOP_READ_NIBBLE:  asmgen_op_read_nibble(s, p, &s->regs);  break;
    case SWS_UOP_READ_PACKED:  asmgen_op_read_packed(s, p, &s->regs);  break;
    case SWS_UOP_READ_PLANAR:  asmgen_op_read_planar(s, p, &s->regs);  break;
    case SWS_UOP_WRITE_BIT:    asmgen_op_write_bit(s, p, &s->regs);    break;
    case SWS_UOP_WRITE_NIBBLE: asmgen_op_write_nibble(s, p, &s->regs); break;
    case SWS_UOP_WRITE_PACKED: asmgen_op_write_packed(s, p, &s->regs); break;
    case SWS_UOP_WRITE_PLANAR: asmgen_op_write_planar(s, p, &s->regs); break;
    case SWS_UOP_SWAP_BYTES:   asmgen_op_swap_bytes(s, p, &s->regs);   break;
    case SWS_UOP_PERMUTE:      asmgen_op_move(s, p, &s->regs);         break;
    case SWS_UOP_COPY:         asmgen_op_move(s, p, &s->regs);         break;
    case SWS_UOP_UNPACK:       asmgen_op_unpack(s, p, &s->regs);       break;
    case SWS_UOP_PACK:         asmgen_op_pack(s, p, &s->regs);         break;
    case SWS_UOP_LSHIFT:       asmgen_op_lshift(s, p, &s->regs);       break;
    case SWS_UOP_RSHIFT:       asmgen_op_rshift(s, p, &s->regs);       break;
    case SWS_UOP_CLEAR:        asmgen_op_clear(s, p, &s->regs);        break;
    case SWS_UOP_TO_U8:        asmgen_op_convert(s, p, &s->regs);      break;
    case SWS_UOP_TO_U16:       asmgen_op_convert(s, p, &s->regs);      break;
    case SWS_UOP_TO_U32:       asmgen_op_convert(s, p, &s->regs);      break;
    case SWS_UOP_TO_F32:       asmgen_op_convert(s, p, &s->regs);      break;
    case SWS_UOP_EXPAND_PAIR:  asmgen_op_expand(s, p, &s->regs);       break;
    case SWS_UOP_EXPAND_QUAD:  asmgen_op_expand(s, p, &s->regs);       break;
    case SWS_UOP_MIN:          asmgen_op_min(s, p, &s->regs);          break;
    case SWS_UOP_MAX:          asmgen_op_max(s, p, &s->regs);          break;
    case SWS_UOP_SCALE:        asmgen_op_scale(s, p, &s->regs);        break;
    case SWS_UOP_LINEAR:       asmgen_op_linear(s, p, &s->regs);       break;
    case SWS_UOP_LINEAR_FMA:   asmgen_op_linear(s, p, &s->regs);       break;
    case SWS_UOP_DITHER:       asmgen_op_dither(s, p, &s->regs);       break;
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

    /* Generate all process functions using rasm. */
    asmgen_process_cps(&s, SWS_COMP_ELEMS(1));
    asmgen_process_cps(&s, SWS_COMP_ELEMS(2));
    asmgen_process_cps(&s, SWS_COMP_ELEMS(3));
    asmgen_process_cps(&s, SWS_COMP_ELEMS(4));

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
