/*
 * audio resampling
 * Copyright (c) 2004-2012 Michael Niedermayer <michaelni@gmx.at>
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
 * audio resampling
 * @author Michael Niedermayer <michaelni@gmx.at>
 */

#if defined(TEMPLATE_RESAMPLE_DBL)

#    define RENAME(N) N ## _double
#    define FILTER_SHIFT 0
#    define DELEM  double
#    define FELEM  double
#    define FELEM2 double
#    define FOFFSET 0
#    define OUT(d, v) d = v

#elif    defined(TEMPLATE_RESAMPLE_FLT)

#    define RENAME(N) N ## _float
#    define FILTER_SHIFT 0
#    define DELEM  float
#    define FELEM  float
#    define FELEM2 float
#    define FOFFSET 0
#    define OUT(d, v) d = v

#elif defined(TEMPLATE_RESAMPLE_S32)

#    define RENAME(N) N ## _int32
#    define FILTER_SHIFT 30
#    define DELEM  int32_t
#    define FELEM  int32_t
#    define FELEM2 int64_t
#    define FELEM_MAX INT32_MAX
#    define FELEM_MIN INT32_MIN
#    define FOFFSET (1<<(FILTER_SHIFT-1))
#    define OUT(d, v) (d) = av_clipl_int32((v)>>FILTER_SHIFT)

#elif    defined(TEMPLATE_RESAMPLE_S16)

#    define RENAME(N) N ## _int16
#    define FILTER_SHIFT 15
#    define DELEM  int16_t
#    define FELEM  int16_t
#    define FELEM2 int32_t
#    define FELEML int64_t
#    define FELEM_MAX INT16_MAX
#    define FELEM_MIN INT16_MIN
#    define FOFFSET (1<<(FILTER_SHIFT-1))
#    define OUT(d, v) (d) = av_clip_int16((v)>>FILTER_SHIFT)

#endif

static void RENAME(resample_one)(void *dest, const void *source,
                                 int dst_size, int64_t index2, int64_t incr)
{
    DELEM *dst = dest;
    const DELEM *src = source;
    int dst_index;

    for (dst_index = 0; dst_index < dst_size; dst_index++) {
        dst[dst_index] = src[index2 >> 32];
        index2 += incr;
    }
}

static int RENAME(resample_common)(ResampleContext *c,
                                   void *dest, const void *source,
                                   int n, int update_ctx)
{
    DELEM *dst = dest;
    const DELEM *src = source;
    int dst_index;
    int index= c->index;
    int frac= c->frac;
    int sample_index = 0;

    while (index >= c->phase_count) {
        sample_index++;
        index -= c->phase_count;
    }

    for (dst_index = 0; dst_index < n; dst_index++) {
        FELEM *filter = ((FELEM *) c->filter_bank) + c->filter_alloc * index;

        FELEM2 val = FOFFSET;
        FELEM2 val2= 0;
        int i;
        for (i = 0; i + 1 < c->filter_length; i+=2) {
            val  += src[sample_index + i    ] * (FELEM2)filter[i    ];
            val2 += src[sample_index + i + 1] * (FELEM2)filter[i + 1];
        }
        if (i < c->filter_length)
            val  += src[sample_index + i    ] * (FELEM2)filter[i    ];
#ifdef FELEML
        OUT(dst[dst_index], val + (FELEML)val2);
#else
        OUT(dst[dst_index], val + val2);
#endif

        frac  += c->dst_incr_mod;
        index += c->dst_incr_div;
        if (frac >= c->src_incr) {
            frac -= c->src_incr;
            index++;
        }

        while (index >= c->phase_count) {
            sample_index++;
            index -= c->phase_count;
        }
    }

    if(update_ctx){
        c->frac= frac;
        c->index= index;
    }

    return sample_index;
}

static int RENAME(resample_linear)(ResampleContext *c,
                                   void *dest, const void *source,
                                   int n, int update_ctx)
{
    DELEM *dst = dest;
    const DELEM *src = source;
    int dst_index;
    int index= c->index;
    int frac= c->frac;
    int sample_index = 0;
#if FILTER_SHIFT == 0
    double inv_src_incr = 1.0 / c->src_incr;
#endif

    while (index >= c->phase_count) {
        sample_index++;
        index -= c->phase_count;
    }

    for (dst_index = 0; dst_index < n; dst_index++) {
        FELEM *filter = ((FELEM *) c->filter_bank) + c->filter_alloc * index;
        FELEM2 val = FOFFSET, v2 = FOFFSET;

        int i;
        for (i = 0; i < c->filter_length; i++) {
            val += src[sample_index + i] * (FELEM2)filter[i];
            v2  += src[sample_index + i] * (FELEM2)filter[i + c->filter_alloc];
        }
#ifdef FELEML
        val += (v2 - val) * (FELEML) frac / c->src_incr;
#else
#    if FILTER_SHIFT == 0
        val += (v2 - val) * inv_src_incr * frac;
#    else
        val += (v2 - val) / c->src_incr * frac;
#    endif
#endif
        OUT(dst[dst_index], val);

        frac += c->dst_incr_mod;
        index += c->dst_incr_div;
        if (frac >= c->src_incr) {
            frac -= c->src_incr;
            index++;
        }

        while (index >= c->phase_count) {
            sample_index++;
            index -= c->phase_count;
        }
    }

    if(update_ctx){
        c->frac= frac;
        c->index= index;
    }

    return sample_index;
}

#undef RENAME
#undef FILTER_SHIFT
#undef DELEM
#undef FELEM
#undef FELEM2
#undef FELEML
#undef FELEM_MAX
#undef FELEM_MIN
#undef OUT
#undef FOFFSET
