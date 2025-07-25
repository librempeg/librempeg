/*
 * Copyright (c) 2009 Baptiste Coudurier <baptiste.coudurier@gmail.com>
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

#define TEST 1
#include "libavutil/random_seed.c"

#undef printf
#define N 8
#define F 2
#include <stdio.h>

typedef uint32_t (*random_seed_ptr_t)(void);

int main(void)
{
    int i, j, rsf, retry;
    uint32_t seeds[N];
    random_seed_ptr_t random_seed[F] = {av_get_random_seed, get_generic_seed};

    for (rsf=0; rsf<F; ++rsf){
        for (retry=0; retry<3; retry++){
            for (i=0; i<N; i++){
                seeds[i] = random_seed[rsf]();
                for (j=0; j<i; j++)
                    if (seeds[j] == seeds[i])
                        goto retry;
            }
            printf("seeds OK\n");
            break;
            retry:;
        }
        if (retry >= 3) {
            printf("rsf %d: FAIL at %d with %"PRIX32"\n", rsf, j, seeds[j]);
            return 1;
        }
    }
    return 0;
}
