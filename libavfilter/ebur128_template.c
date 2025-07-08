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

#undef FABS
#undef SCALE
#undef ftype
#undef ptype
#undef SAMPLE_FORMAT
#if DEPTH == 16
#define FABS FFABS
#define SCALE(x) ((x) * (1.0 / 32768.0))
#define ptype int
#define ftype int16_t
#define SAMPLE_FORMAT s16p
#elif DEPTH == 32
#define FABS fabsf
#define SCALE(x) (x)
#define ptype float
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define FABS fabs
#define SCALE(x) (x)
#define ptype double
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static av_always_inline void fn(process_ebur128)(const ftype **ch_samples,
                                                 const unsigned nb_channels,
                                                 const unsigned i3000_cache_size,
                                                 const unsigned i400_cache_size,
                                                 unsigned *i3000_cache_pos,
                                                 unsigned *i400_cache_pos,
                                                 unsigned *i3000_filled,
                                                 unsigned *i400_filled,
                                                 const double *ch_weighting,
                                                 const struct filter_coeffs *cf,
                                                 double *i3000_cache,
                                                 double *i400_cache,
                                                 double *i3000_sum,
                                                 double *i400_sum,
                                                 double *t0)
{
    const unsigned bin_id_3000 = i3000_cache_pos[0];
    const unsigned bin_id_400  = i400_cache_pos[0];

#define MOVE_TO_NEXT_CACHED_ENTRY(time) do { \
    i##time##_cache_pos[0]++;                \
    if (i##time##_cache_pos[0] >=            \
        i##time##_cache_size) {              \
        i##time##_filled[0]    = 1;          \
        i##time##_cache_pos[0] = 0;          \
    }                                        \
} while (0)

    MOVE_TO_NEXT_CACHED_ENTRY(400);
    MOVE_TO_NEXT_CACHED_ENTRY(3000);

    for (unsigned ch = 0; ch < nb_channels; ch++) {
        const double sample = SCALE(*ch_samples[ch]++);
        const int ch4 = ch * 4;
        double *tt0 = t0 + ch4;
        double *tt1 = tt0 + 2;
        double bin, out;

        if (ch_weighting && !ch_weighting[ch])
            continue;

#define FILTER(y, x, t, NUM, DEN) do {                       \
        y = NUM[0] * x + t[0];                               \
        t[0] = NUM[1] * x + t[1] - DEN[1] * y;               \
        t[1] = NUM[2] * x        - DEN[2] * y;               \
} while (0)

        // TODO: merge both filters in one?
        FILTER(out, sample, tt0, cf->pre_b, cf->pre_a);  // apply pre-filter
        FILTER(bin, out,    tt1, cf->rlb_b, cf->rlb_a);  // apply RLB-filter

        bin *= bin;

        /* add the new value, and limit the sum to the cache size (400ms or 3s)
         * by removing the oldest one */
        i400_sum [ch] += bin - i400_cache [bin_id_400];
        i3000_sum[ch] += bin - i3000_cache[bin_id_3000];

        /* override old cache entry with the new value */
        i400_cache [bin_id_400 ] = bin;
        i3000_cache[bin_id_3000] = bin;

        i400_cache  += i400_cache_size;
        i3000_cache += i3000_cache_size;
    }
}

static void fn(process_block)(const ftype **ch_samples,
                              const unsigned nb_channels,
                              const unsigned i3000_cache_size,
                              const unsigned i400_cache_size,
                              unsigned *i3000_cache_pos,
                              unsigned *i400_cache_pos,
                              unsigned *i3000_filled,
                              unsigned *i400_filled,
                              const double *ch_weighting,
                              const struct filter_coeffs *cf,
                              double *i3000_cache,
                              double *i400_cache,
                              double *i3000_sum,
                              double *i400_sum,
                              double *t0,
                              const int samples_to_process,
                              const int nozero_ch_weighting)
{
    if (nozero_ch_weighting && nb_channels == 2) {
        for (int n = 0; n < samples_to_process; n++)
            fn(process_ebur128)(ch_samples, 2,
                                i3000_cache_size, i400_cache_size,
                                i3000_cache_pos, i400_cache_pos,
                                i3000_filled, i400_filled,
                                NULL, cf,
                                i3000_cache, i400_cache,
                                i3000_sum, i400_sum,
                                t0);
    } else {
        for (int n = 0; n < samples_to_process; n++)
            fn(process_ebur128)(ch_samples, nb_channels,
                                i3000_cache_size, i400_cache_size,
                                i3000_cache_pos, i400_cache_pos,
                                i3000_filled, i400_filled,
                                ch_weighting, cf,
                                i3000_cache, i400_cache,
                                i3000_sum, i400_sum,
                                t0);
    }
}

static double fn(samples_peak)(const void *src_, const int nb_samples)
{
    ptype sample_peak_per_frame = 0;
    const ftype *src = src_;

    for (int idx = 0; idx < nb_samples; idx++) {
        const ptype sample = src[idx];
        const ptype asample = FABS(sample);

        sample_peak_per_frame = FFMAX(sample_peak_per_frame, asample);
    }

    return SCALE(sample_peak_per_frame);
}
