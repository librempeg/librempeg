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

#include "libavutil/aarch64/asm.S"

function ff_opus_deemphasis_neon, export=1
        ld1             {v4.4s}, [x2], #16
        ld1             {v5.4s}, [x2], #16
        ld1             {v6.4s}, [x2], #16
        ld1             {v7.4s}, [x2]

        fmul            v0.4s, v4.4s, v0.s[0]

1:      ld1             {v1.4s, v2.4s}, [x1], #32

        fmla            v0.4s, v5.4s, v1.s[0]
        fmul            v3.4s, v7.4s, v2.s[2]

        fmla            v0.4s, v6.4s, v1.s[1]
        fmla            v3.4s, v6.4s, v2.s[1]

        fmla            v0.4s, v7.4s, v1.s[2]
        fmla            v3.4s, v5.4s, v2.s[0]

        fadd            v1.4s, v1.4s, v0.4s
        fadd            v2.4s, v2.4s, v3.4s

        fmla            v2.4s, v4.4s, v1.s[3]

        st1             {v1.4s, v2.4s}, [x0], #32
        fmul            v0.4s, v4.4s, v2.s[3]

        subs            w3, w3, #8
        b.gt            1b

        mov             s0, v2.s[3]

        ret
endfunc

function ff_opus_postfilter_neon, export=1
        ld1             {v0.4s}, [x2]
        sub             x5, x0, w1, sxtw #2
        sub             x1, x5, #8
        dup             v1.4s, v0.s[1]
        dup             v2.4s, v0.s[2]
        dup             v0.4s, v0.s[0]

        ld1             {v3.4s}, [x1], #16
        sub             x4, x5, #4
        add             x6, x5, #4
        fmul            v3.4s, v3.4s, v2.4s

1:      ld1             {v7.4s}, [x1], #16
        ld1             {v4.4s}, [x4], #16
        fmla            v3.4s, v7.4s, v2.4s
        ld1             {v6.4s}, [x6], #16
        ld1             {v5.4s}, [x5], #16
        fadd            v6.4s, v6.4s, v4.4s
        fmla            v3.4s, v5.4s, v0.4s

        ld1             {v4.4s}, [x0]
        fmla            v3.4s, v6.4s, v1.4s
        fadd            v4.4s, v4.4s, v3.4s
        fmul            v3.4s, v7.4s, v2.4s

        st1             {v4.4s}, [x0], #16

        subs            w3, w3, #4
        b.gt            1b

        ret
endfunc
