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

#include "libavutil/attributes.h"
#include "libavutil/cpu.h"
#include "libavutil/x86/cpu.h"
#include "libavcodec/avcodec.h"
#include "libavcodec/fdctdsp.h"
#include "fdct.h"

av_cold void ff_fdctdsp_init_x86(FDCTDSPContext *c, AVCodecContext *avctx,
                                 unsigned high_bit_depth)
{
#if HAVE_SSE2_INLINE
    int cpu_flags = av_get_cpu_flags();
    const int dct_algo = avctx->dct_algo;

    if (!high_bit_depth) {
        if ((dct_algo == FF_DCT_AUTO || dct_algo == FF_DCT_MMX)) {
            if (INLINE_SSE2(cpu_flags))
                c->fdct = ff_fdct_sse2;
        }
    }
#endif
}
