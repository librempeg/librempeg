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

#include "attributes.h"
#include "error.h"
#include "thread.h"

av_cold void ff_pthread_free(void *obj, const unsigned offsets[])
{
    unsigned cnt = *(unsigned*)((char*)obj + offsets[0]);
    const unsigned *cur_offset = offsets;

    *(unsigned*)((char*)obj + offsets[0]) = 0;

    for (; *(++cur_offset) != THREAD_SENTINEL && cnt; cnt--)
        pthread_mutex_destroy((pthread_mutex_t*)((char*)obj + *cur_offset));
    for (; *(++cur_offset) != THREAD_SENTINEL && cnt; cnt--)
        pthread_cond_destroy ((pthread_cond_t *)((char*)obj + *cur_offset));
}

av_cold int ff_pthread_init(void *obj, const unsigned offsets[])
{
    const unsigned *cur_offset = offsets;
    unsigned cnt = 0;
    int err;

#define PTHREAD_INIT_LOOP(type)                                               \
    for (; *(++cur_offset) != THREAD_SENTINEL; cnt++) {                       \
        pthread_ ## type ## _t *dst = (void*)((char*)obj + *cur_offset);      \
        err = pthread_ ## type ## _init(dst, NULL);                           \
        if (err) {                                                            \
            err = AVERROR(err);                                               \
            goto fail;                                                        \
        }                                                                     \
    }
    PTHREAD_INIT_LOOP(mutex)
    PTHREAD_INIT_LOOP(cond)

fail:
    *(unsigned*)((char*)obj + offsets[0]) = cnt;
    return err;
}
