/*
 * Copyright (c) 2026 Lynne <dev@lynne.ee>
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

#version 460
#pragma shader_stage(compute)
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"
#include "dct.glsl"

#define APV_MAX_NUM_COMP    4
#define APV_MAX_TILE_COLS   20
#define APV_MAX_TILE_ROWS   20
#define APV_MAX_TILE_COUNT  (APV_MAX_TILE_COLS * APV_MAX_TILE_ROWS)
#define APV_MIN_TRANS_COEFF -32768
#define APV_MAX_TRANS_COEFF 32767
#define APV_TR_SIZE         8
#define APV_BLK_COEFFS      (APV_TR_SIZE * APV_TR_SIZE)
#define APV_MB_SIZE         16

/*
 * Buffer holding per-tile, per-component coefficient blocks.
 * Layout (linear):
 *   tile_y * tile_cols * num_comp * blocks_per_tile * 64
 * + tile_x * num_comp * blocks_per_tile * 64
 * + comp * blocks_per_tile * 64
 * + block_in_tile * 64
 * + coeff_in_block
 *
 * blocks_per_tile is computed by the host as:
 *   mbs_per_tile_x * mbs_per_tile_y * blocks_per_mb[comp]
 * where blocks_per_mb is 4 (luma) or 4 (chroma in 444), etc.
 */
layout (set = 0, binding = 0, scalar) writeonly buffer coeffs_buf {
    int16_t coeffs[];
};

layout (set = 0, binding = 1) uniform readonly iimage2D src[];

layout (push_constant, scalar) uniform pushConstants {
    ivec2 frame_dim;        /* in pixels */
    ivec2 tile_count;       /* number of tile columns/rows */
    ivec2 tile_mb_dim;      /* MBs per tile (cols, rows) */
    ivec2 log2_chroma_sub;  /* 0/0 for 444, 1/0 for 422, etc. */
    int   num_comp;
    int   bit_depth;
    /* Per-component quant scale fact/(level_scale*2^qp_shift). The encoder
     * uses one QP per component, so it never varies by tile. */
    float qf[APV_MAX_NUM_COMP];
    /* The quantisation matrix (raster order), the same one signalled in the
     * frame header. Staged into shared memory at the top of main(). */
    uint8_t qmat[64];
};

/* Workgroup-local copy of qmat, filled once per workgroup (prores_raw style). */
shared uint8_t qmat_buf[64];

void main(void)
{
    /* Workgroup grid:
     *   x: total MB columns over the frame (frame_mb_x)
     *   y: total MB rows over the frame    (frame_mb_y)
     *   z: component index                  [0..num_comp)
     *
     * Local size (8, 4, 1):
     *   gl_LocalInvocationID.x in [0..7] = row index inside an 8x8 block
     *   gl_LocalInvocationID.y in [0..3] = which 8x8 block within the MB
     *
     * Luma and 4:4:4 chroma use all 4 blocks; 4:2:2 chroma uses 2 (a vertical
     * pair) and the surplus two invocations early-out before the store.
     */

    /* Stage the quantisation matrix into shared memory once, the same way
     * prores_raw does -- one workgroup-wide copy instead of a push-constant
     * read for every coefficient. */
    if (gl_LocalInvocationIndex == 0u) {
        [[unroll]]
        for (uint i = 0u; i < 64u; i++)
            qmat_buf[i] = qmat[i];
    }
    barrier();

    const uint comp     = gl_WorkGroupID.z;
    const uint mb_x_lin = gl_WorkGroupID.x;
    const uint mb_y_lin = gl_WorkGroupID.y;

    /* Map workgroup to its tile (luma coords) */
    const ivec2 sub_shift = (comp == 0u) ? ivec2(0) : log2_chroma_sub;

    /* Compute which tile this MB belongs to in MB units */
    const int tx = int(mb_x_lin) / tile_mb_dim.x;
    const int ty = int(mb_y_lin) / tile_mb_dim.y;
    const int tile_idx = ty * tile_count.x + tx;
    const int mb_x_in_tile = int(mb_x_lin) - tx * tile_mb_dim.x;
    const int mb_y_in_tile = int(mb_y_lin) - ty * tile_mb_dim.y;

    /* Remainder tiles: the rightmost tile column may be narrower than
     * tile_mb_dim.x. Pack block indices by the tile's ACTUAL width so the
     * entropy pass sees them contiguously. */
    const int frame_mb_x = (frame_dim.x + APV_MB_SIZE - 1) / APV_MB_SIZE;
    const int actual_tw  = min(tile_mb_dim.x, frame_mb_x - tx * tile_mb_dim.x);
    const int mb_in_tile = mb_y_in_tile * actual_tw + mb_x_in_tile;

    /*
     * Per-MB block grid for this component: luma and 4:4:4 chroma are 2x2
     * (4 blocks); 4:2:2 chroma is 1 wide x 2 tall (2 blocks). Derived from
     * the chroma sub-sampling shift.
     */
    const uint bw = 2u >> uint(sub_shift.x);
    const uint bh = 2u >> uint(sub_shift.y);
    const uint nb_blk = bw * bh;

    /*
     * Uniform coefficient stride: every tile-component is allocated the luma
     * block count (4 per MB). A sub-sampled chroma component leaves the rest
     * of its region unused, which keeps the buffer layout flat.
     */
    const uint blocks_per_tile = uint(tile_mb_dim.x * tile_mb_dim.y) * 4u;

    /* Block index within the workgroup -> position inside the MB */
    const uint blk = gl_LocalInvocationID.y;
    const uint row = gl_LocalInvocationID.x;

    /* Block coordinate inside the (possibly sub-sampled) macroblock */
    const ivec2 block_offset = ivec2(int(blk % bw), int(blk / bw));

    /* Compute pixel coordinate for this thread's row of samples */
    ivec2 mb_origin_luma = ivec2(int(mb_x_lin), int(mb_y_lin)) * APV_MB_SIZE;
    ivec2 mb_origin = mb_origin_luma >> sub_shift;
    ivec2 block_origin = mb_origin + block_offset * APV_TR_SIZE;
    ivec2 coord = block_origin + ivec2(0, int(row));

    /* Clamp to image bounds, in case frame dimensions are not aligned */
    ivec2 img_dim = imageSize(src[comp]);
    coord = min(coord, img_dim - ivec2(1));

    const float bias = float(1 << (bit_depth - 1));
    const float fact = bias;

    /* Load 8 horizontal samples, subtract bias, normalize to ~[-1,1] */
    [[unroll]]
    for (int i = 0; i < 8; i++) {
        int s = imageLoad(src[comp], coord + ivec2(i, 0)).x;
        blocks[blk][row * 9u + uint(i)] = (float(s) - bias) / fact;
    }

    barrier();

    /* Column DCT (offset varies on x-axis, traverses rows via stride 9) */
    fdct8(blk, row, 9);
    barrier();

    /* Row DCT (offset varies on y-axis, traverses cols via stride 1) */
    fdct8(blk, row * 9u, 1);
    barrier();

    /*
     * Quantize and store. Each thread writes its row.
     *   level = round( fdct2d((sample - bias)/fact) * qf * 1024/qmat[i] )
     * fdct8() is the exact orthonormal forward DCT, the reciprocal of the
     * spec iDCT (apv_decode_transquant_c), which reconstructs
     *   sample - bias = (qmat[i]*level_scale*2^qp_shift / 1024)
     *                   * iDCT_ortho(level).
     * qf[comp] carries fact/(level_scale*2^qp_shift); the per-coefficient
     * factor 1024/qmat[i] inverts the decoder's per-coefficient dequant.
     */
    const float scale_const = qf[comp];

    /* Compute coefficient base offset in the buffer */
    const uint tile_lin = uint(tile_idx);
    const uint coeff_base =
        (tile_lin * uint(num_comp) + comp) * blocks_per_tile * 64u +
        (uint(mb_in_tile) * nb_blk + blk) * 64u;

    /* Surplus invocations of a sub-sampled component (blk >= nb_blk) took
     * part in the barriers above but must not write any coefficients. */
    if (blk < nb_blk) {
        [[unroll]]
        for (int i = 0; i < 8; i++) {
            float v  = blocks[blk][row * 9u + uint(i)];
            float pf = 1024.0f / float(qmat_buf[row * 8u + uint(i)]);
            int lvl = int(round(v * scale_const * pf));
            lvl = clamp(lvl, APV_MIN_TRANS_COEFF, APV_MAX_TRANS_COEFF);
            coeffs[coeff_base + row * 8u + uint(i)] = int16_t(lvl);
        }
    }
}
