/*
 * CyberFlix DreamFactory shared bitstream helpers
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

#ifndef AVCODEC_CFDF_AUDIO_H
#define AVCODEC_CFDF_AUDIO_H

#include <limits.h>
#include <stdint.h>

#include "libavutil/error.h"

/* A v4.0 control stream is a seed byte followed by opcodes: bit 7 clear is one
 * absolute sample; bit 7 set with bit 6 clear is a run of nibble-delta pairs
 * consuming that many further bytes; both bits set repeats the running sample.
 *
 * Walks from the first opcode and stops before the one that would exceed
 * max_samples, so a cut lands on an opcode boundary and undershoots by at most
 * 127 samples. Returns the byte count reached, including the seed;
 * *out_samples gets the exact sample count. */
static inline int ff_cfdf_v40_walk(const uint8_t *buf, int size,
                                   int64_t max_samples, int64_t *out_samples)
{
    int64_t n = 0;
    int p = 1;

    while (p < size) {
        uint8_t c = buf[p];
        int adv, smp;

        if (!(c & 0x80)) {
            adv = 1;
            smp = 1;
        } else if (!(c & 0x40)) {
            int cnt = (c & 0x3f) + 1;
            adv = 1 + cnt;
            smp = 2 * cnt;
        } else {
            adv = 1;
            smp = (c & 0x3f) + 1;
        }
        if (adv > size - p)
            return AVERROR_INVALIDDATA;
        if (max_samples < n || smp > max_samples - n)
            break;
        n += smp;
        p += adv;
    }

    *out_samples = n;

    return p;
}

static inline int ff_cfdf_v40_count(const uint8_t *buf, int size)
{
    int64_t n;
    int ret;

    ret = ff_cfdf_v40_walk(buf, size, INT64_MAX, &n);
    if (ret < 0 || n > INT_MAX)
        return AVERROR_INVALIDDATA;

    return (int)n;
}

/* Movie chunks (CFDF container, v4 .MOV) emit the seed byte as the chunk's
 * first sample; v5 SOUN blocks seed from it silently. The chunk header's
 * uncompressed field counts it, so movie sample counts are one above the walk. */
static inline int ff_cfdf_v40_movie_count(const uint8_t *buf, int size)
{
    int n;

    if (size <= 0)
        return 0;
    n = ff_cfdf_v40_count(buf, size);
    if (n < 0 || n == INT_MAX)
        return AVERROR_INVALIDDATA;

    return n + 1;
}

static inline int ff_cfdf_v40_movie_prefix(const uint8_t *buf, int size,
                                           int64_t max_samples,
                                           int64_t *out_samples)
{
    int64_t n;
    int p;

    if (size <= 0 || max_samples <= 0) {
        *out_samples = 0;
        return 0;
    }

    p = ff_cfdf_v40_walk(buf, size, max_samples - 1, &n);
    if (p < 0) {
        *out_samples = 0;
        return p;
    }
    *out_samples = n + 1;

    return p;
}

#endif /* AVCODEC_CFDF_AUDIO_H */
