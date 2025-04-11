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

#undef ftype
#undef stype
#undef FABS
#undef SAMPLE_FORMAT
#if DEPTH == 16
#define ftype unsigned
#define stype int16_t
#define FABS FFABS
#if PLANAR
#define SAMPLE_FORMAT s16p
#else
#define SAMPLE_FORMAT s16
#endif
#elif DEPTH == 32
#define ftype float
#define stype float
#define FABS fabsf
#if PLANAR
#define SAMPLE_FORMAT fltp
#else
#define SAMPLE_FORMAT flt
#endif
#endif

#define MAX_IDX (HISTOGRAM_SIZE-1)
#define EPSILON (1.f / (1 << 23))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static inline float fn(get_db)(unsigned x)
{
#if DEPTH == 32
    return x - 16384.f;
#else
    return 20.f * log2f(x/(float)MAX_IDX) / log2f(10.f);
#endif
}

static inline unsigned fn(get_index)(ftype x)
{
#if DEPTH == 32
    return av_clip(16384+lrintf(ceilf(20.f * log10f(x+EPSILON))), 0, MAX_IDX);
#else
    return x;
#endif
}

static void fn(print_stats)(AVFilterContext *ctx)
{
    VolDetectContext *s = ctx->priv;
    uint64_t nb_samples = 0, sum = 0, idb_sum = 0;
    int last_idb = INT_MAX;

    for (int i = 0; i < HISTOGRAM_SIZE; i++)
        nb_samples += s->histogram[i];
    if (!nb_samples)
        return;

    av_log(ctx, AV_LOG_INFO, "n_samples: %" PRId64 "\n", nb_samples);
#if DEPTH == 16
    /*
     * If nb_samples > 1<<34, there is a risk of overflow in the
     * multiplication or the sum: shift all histogram values to avoid that.
     * The total number of samples must be recomputed to avoid rounding
     * errors.
     */
    {
        uint64_t power = 0, nb_samples_shift = 0;
        int shift = av_log2(nb_samples >> 33);

        for (int i = 0; i < HISTOGRAM_SIZE; i++) {
            uint64_t count = s->histogram[i];

            if (!count)
                continue;

            nb_samples_shift += count >> shift;
            power += i * i * (count >> shift);
        }
        if (!nb_samples_shift)
            return;
        power = (power + nb_samples_shift / 2) / nb_samples_shift;
        av_log(ctx, AV_LOG_INFO, "mean_volume: %.1f dB\n", fn(get_db)(sqrt(power)));
    }
    for (int i = MAX_IDX; i >= 0; i--) {
        if (s->histogram[i]) {
            av_log(ctx, AV_LOG_INFO, "max_volume: %.1f dB\n", fn(get_db)(i));
            break;
        }
    }
#else
    av_log(ctx, AV_LOG_INFO, "mean_volume: %.1f dB\n", 20.f * log10f(sqrt(s->sum2/nb_samples)));
    av_log(ctx, AV_LOG_INFO, "max_volume: %.1f dB\n", 20.f * log10f(s->max));
#endif
    for (int i = MAX_IDX; i >= 0; i--) {
        if (s->histogram[i]) {
            const float new_db = fn(get_db)(i);
            const int new_idb = lrintf(ceilf(new_db));

            if (new_idb != last_idb) {
                if (idb_sum > 0)
                    av_log(ctx, AV_LOG_INFO, "histogram_%ddb: %" PRId64 "\n", last_idb, idb_sum);
                idb_sum = 0;
                last_idb = new_idb;
                if (sum > nb_samples / 1000)
                    break;
            }
            sum += s->histogram[i];
            idb_sum += s->histogram[i];
        }
    }
}

static void fn(update_stats)(VolDetectContext *s, stype sample)
{
    int idx;

    sample = FABS(sample);
#if DEPTH == 32
    s->max = fmaxf(s->max, sample);
    s->sum2 += sample*sample;
#endif
    idx = fn(get_index)(sample);
    s->histogram[idx]++;
}

static void fn(update_histogram)(AVFilterContext *ctx, const AVFrame *in)
{
    const int nb_channels = in->ch_layout.nb_channels;
    const int nb_samples = in->nb_samples;
    VolDetectContext *s = ctx->priv;

#if PLANAR == 1
    for (int ch = 0; ch < nb_channels; ch++) {
        const stype *src = (const stype *)in->extended_data[ch];
        for (int n = 0; n < nb_samples; n++)
            fn(update_stats)(s, src[n]);
    }
#else
    const stype *src = (const stype *)in->data[0];

    for (int n = 0; n < nb_samples * nb_channels; n++)
        fn(update_stats)(s, src[n]);
#endif
}
