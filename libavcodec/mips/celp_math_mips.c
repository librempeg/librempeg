/*
 * Copyright (c) 2012
 *      MIPS Technologies, Inc., California.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the MIPS Technologies, Inc., nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE MIPS TECHNOLOGIES, INC. ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE MIPS TECHNOLOGIES, INC. BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author:  Nedeljko Babic (nbabic@mips.com)
 *
 * Math operations optimized for MIPS
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
 * Reference: libavcodec/celp_math.c
 */
#include "config.h"
#include "libavcodec/celp_math.h"
#include "libavutil/attributes.h"
#include "libavutil/mips/asmdefs.h"

#if HAVE_INLINE_ASM
#if !HAVE_MIPS32R6 && !HAVE_MIPS64R6
static float ff_dot_productf_mips(const float* a, const float* b,
                                              int length)
{
    float sum;
    const float* a_end = a + length;

    __asm__ volatile (
        "mtc1   $zero,      %[sum]                              \n\t"
        "blez   %[length],  ff_dot_productf_end%=               \n\t"
        "ff_dot_productf_madd%=:                                \n\t"
        "lwc1   $f2,        0(%[a])                             \n\t"
        "lwc1   $f1,        0(%[b])                             \n\t"
        PTR_ADDIU "%[a],    %[a],      4                        \n\t"
        PTR_ADDIU "%[b],    %[b],      4                        \n\t"
        "madd.s %[sum],     %[sum],    $f1, $f2                 \n\t"
        "bne   %[a],        %[a_end],  ff_dot_productf_madd%=   \n\t"
        "ff_dot_productf_end%=:                                 \n\t"

        : [sum] "=&f" (sum), [a] "+r" (a), [b] "+r" (b)
        : [a_end]"r"(a_end), [length] "r" (length)
        : "$f1", "$f2", "memory"
    );
    return sum;
}
#endif /* !HAVE_MIPS32R6 && !HAVE_MIPS64R6 */
#endif /* HAVE_INLINE_ASM */

av_cold void ff_celp_math_init_mips(CELPMContext *c)
{
#if HAVE_INLINE_ASM
#if !HAVE_MIPS32R6 && !HAVE_MIPS64R6
    c->dot_productf = ff_dot_productf_mips;
#endif
#endif
}
