/*
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

#undef stype
#undef SCALE
#undef FIXED
#undef SAMPLE_FORMAT
#if DEPTH == 16
#define stype int16_t
#define SCALE(x) ((x) / (double)INT16_MAX)
#define FIXED(x) (x)
#define SAMPLE_FORMAT s16
#elif DEPTH == 31
#define stype int32_t
#define SCALE(x) ((x) / (double)INT32_MAX)
#define FIXED(x) (x)
#define SAMPLE_FORMAT s32
#elif DEPTH == 32
#define stype float
#define SCALE(x) (x)
#define FIXED(x) (llrint((x) * (UINT64_C(1) << 31)))
#define SAMPLE_FORMAT flt
#elif DEPTH == 63
#define stype int64_t
#define SCALE(x) ((x) / (double)INT64_MAX)
#define FIXED(x) (x)
#define SAMPLE_FORMAT s64
#else
#define stype double
#define SCALE(x) (x)
#define FIXED(x) (llrint((x) * (UINT64_C(1) << 63)))
#define SAMPLE_FORMAT dbl
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static void fn(update_peak)(AudioStatsContext *s, ChannelStats *p, stype d)
{
    double n = SCALE(d);

    if (n < p->nmin)
        p->nmin = n;
    if (n > p->nmax)
        p->nmax = n;
}

static void fn(update_minmax)(AudioStatsContext *s, ChannelStats *p, stype d)
{
    if (d < p->min)
        p->min = d;
    if (d > p->max)
        p->max = d;
}

static void fn(update_clip_count)(AudioStatsContext *s, ChannelStats *p, stype d)
{
    double scaled = SCALE(d);

    p->clip_count += fabs(scaled) > 1.0;
}

static inline void fn(update_float_stat)(AudioStatsContext *s, ChannelStats *p, stype d)
{
#if (DEPTH == 32) || (DEPTH == 64)
    const int ftype = fpclassify(d);

    p->nb_nans      += ftype == FP_NAN;
    p->nb_infs      += ftype == FP_INFINITE;
    p->nb_denormals += ftype == FP_SUBNORMAL;
#endif
}

static inline void fn(update_stat)(AudioStatsContext *s, ChannelStats *p, stype src,
                                   uint32_t mask)
{
    int64_t fixed = FIXED(src);
    double scaled = SCALE(src);
    double abs_d = FFABS(scaled);
    double drop, noise_floor;
    double d = src;

    if (p->abs_peak < abs_d) {
        p->abs_peak = abs_d;
        p->abs_peak_count = 1;
    } else if (p->abs_peak == abs_d) {
        p->abs_peak_count++;
    }
    p->clip_count += fabs(scaled) > 1.0;
    if (d < p->min) {
        p->min = d;
        p->nmin = scaled;
        p->min_run = 1;
        p->min_runs = 0;
        p->min_count = 1;
    } else if (d == p->min) {
        p->min_count++;
        p->min_run = d == p->last ? p->min_run + 1 : 1;
    } else if (p->last == p->min) {
        p->min_runs += p->min_run * p->min_run;
    }

    if (d != 0 && FFABS(d) < p->min_non_zero)
        p->min_non_zero = FFABS(d);

    if (d > p->max) {
        p->max = d;
        p->nmax = scaled;
        p->max_run = 1;
        p->max_runs = 0;
        p->max_count = 1;
    } else if (d == p->max) {
        p->max_count++;
        p->max_run = d == p->last ? p->max_run + 1 : 1;
    } else if (p->last == p->max) {
        p->max_runs += p->max_run * p->max_run;
    }

    if (d != 0) {
        p->zero_runs += FFSIGN(d) != FFSIGN(p->last_non_zero);
        p->last_non_zero = d;
    }

    p->sigma_x += scaled;
    p->sigma_ax += fabs(scaled);
    if (fabs(scaled) > 1e-16)
        p->sigma_log2_ax += log2(fabs(scaled));
    p->sigma_x2 += scaled * scaled;
    p->avg_sigma_x2 = p->avg_sigma_x2 * s->mult + (1.0 - s->mult) * scaled * scaled;
    if (!isnan(p->last)) {
        p->min_diff = FFMIN(p->min_diff, fabs(d - p->last));
        p->max_diff = FFMAX(p->max_diff, fabs(d - p->last));
        p->diff1_sum += fabs(d - p->last);
        p->diff1_sum_x2 += (d - p->last) * (d - p->last);
    }
    p->mask[0] |= (fixed < 0) ? -fixed : fixed;
    p->mask[1] |= fixed;
    p->mask[2] &= fixed;
    if (!isnan(p->last))
        p->mask[3] |= fixed ^ p->lasti;
    p->lasti = fixed;
    p->last = d;

    drop = p->win_samples[p->win_pos];
    p->win_samples[p->win_pos] = scaled;
    p->win_pos++;

    if (p->win_pos >= s->tc_samples)
        p->win_pos = 0;

    if (p->nb_samples >= s->tc_samples) {
        p->max_sigma_x2 = FFMAX(p->max_sigma_x2, p->avg_sigma_x2);
        p->min_sigma_x2 = FFMIN(p->min_sigma_x2, p->avg_sigma_x2);
    }
    p->nb_samples++;

    if (mask & MEASURE_NOISE_FLOOR) {
        noise_floor = calc_noise_floor(p->sorted_samples, scaled, drop,
                                       s->tc_samples, &p->sorted_front, &p->sorted_back);
        if (p->nb_samples >= s->tc_samples) {
            if (isnan(p->noise_floor)) {
                p->noise_floor = noise_floor;
                p->noise_floor_count = 1;
            } else {
                if (noise_floor < p->noise_floor) {
                    p->noise_floor = noise_floor;
                    p->noise_floor_count = 1;
                } else if (noise_floor == p->noise_floor) {
                    p->noise_floor_count++;
                }
            }
        }
    }

    if (mask & MEASURE_DENORMALS)
        fn(update_float_stat)(s, p, scaled);
}

static int fn(filter_channels_planar)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioStatsContext *s = ctx->priv;
    AVFrame *in = arg;
    const uint8_t * const * const data = (const uint8_t * const *)in->extended_data;
    const int channels = s->nb_channels;
    const int samples = in->nb_samples;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;
    const uint32_t mask = s->measure_overall | s->measure_perchannel;
    const int measure_denormals = mask == (mask & MEASURE_DENORMALS);
    const int measure_minmax = mask == (mask & MEASURE_MINMAX);
    const int measure_clip_count = mask == MEASURE_CLIP_COUNT;
    const int measure_nb_samples = mask == MEASURE_NB_SAMPLES;
    const int measure_peak = mask == MEASURE_PEAK;

    for (int c = start; c < end; c++) {
        ChannelStats *p = &s->chstats[c];
        const stype *src = (const stype *)data[c];

        if (measure_nb_samples) {
            p->nb_samples += samples;
            continue;
        }

        if (measure_clip_count) {
            for (int n = 0; n < samples; n++)
                fn(update_clip_count)(s, p, src[n]);
            continue;
        }

        if (measure_peak) {
            for (int n = 0; n < samples; n++)
                fn(update_peak)(s, p, src[n]);
            continue;
        }

        if (measure_minmax) {
            for (int n = 0; n < samples; n++)
                fn(update_minmax)(s, p, src[n]);
            continue;
        }

        if (measure_denormals) {
            for (int n = 0; n < samples; n++)
                fn(update_float_stat)(s, p, src[n]);
            continue;
        }

        for (int n = 0; n < samples; n++)
            fn(update_stat)(s, p, src[n], mask);
    }

    return 0;
}

static int fn(filter_channels_packed)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioStatsContext *s = ctx->priv;
    AVFrame *in = arg;
    const uint8_t * const * const data = (const uint8_t * const *)in->extended_data;
    const int channels = s->nb_channels;
    const int samples = in->nb_samples;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const uint32_t mask = s->measure_overall | s->measure_perchannel;
    const int measure_denormals = mask == (mask & MEASURE_DENORMALS);
    const int measure_minmax = mask == (mask & MEASURE_MINMAX);
    const int measure_clip_count = mask == MEASURE_CLIP_COUNT;
    const int measure_nb_samples = mask == MEASURE_NB_SAMPLES;
    const int measure_peak = mask == MEASURE_PEAK;

    for (int c = start; c < end; c++) {
        ChannelStats *p = &s->chstats[c];
        const stype *src = (const stype *)data[0];
        const stype * const srcend = src + samples * channels;

        if (measure_nb_samples) {
            p->nb_samples += samples;
            continue;
        }

        if (measure_clip_count) {
            for (src += c; src < srcend; src += channels)
                fn(update_clip_count)(s, p, src[0]);
            continue;
        }

        if (measure_peak) {
            for (src += c; src < srcend; src += channels)
                fn(update_peak)(s, p, src[0]);
            continue;
        }

        if (measure_minmax) {
            for (src += c; src < srcend; src += channels)
                fn(update_minmax)(s, p, src[0]);
            continue;
        }

        if (measure_denormals) {
            for (src += c; src < srcend; src += channels)
                fn(update_float_stat)(s, p, src[0]);
            continue;
        }

        for (src += c; src < srcend; src += channels)
            fn(update_stat)(s, p, src[0], mask);
    }

    return 0;
}
