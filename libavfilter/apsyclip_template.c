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

#include "libavutil/tx.h"
#include "avfilter.h"
#include "audio.h"

#undef ctype
#undef ftype
#undef MPI
#undef SAMPLE_FORMAT
#undef TX_TYPE
#undef FCOS
#undef FPOW
#undef FABS
#undef FMAX
#undef FMIN
#undef HYPOT
#undef FLOG
#undef FEXP
#if DEPTH == 32
#define MPI M_PIf
#define SAMPLE_FORMAT fltp
#define ctype AVComplexFloat
#define ftype float
#define TX_TYPE AV_TX_FLOAT_RDFT
#define FCOS cosf
#define FPOW powf
#define FABS fabsf
#define FMAX fmaxf
#define FMIN fminf
#define HYPOT hypotf
#define FLOG logf
#define FEXP expf
#else
#define MPI M_PI
#define SAMPLE_FORMAT dblp
#define ctype AVComplexDouble
#define ftype double
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define FCOS cos
#define FPOW pow
#define FABS fabs
#define FMAX fmax
#define FMIN fmin
#define HYPOT hypot
#define FLOG log
#define FEXP exp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    int channels;
    int spread_table_rows;
    int *spread_table_index;
    int (*spread_table_range)[2];
    ftype *window, *inv_window, *spread_table, *margin_curve;

    AVTXContext **tx_ctx, **itx_ctx;
    av_tx_fn tx_fn, itx_fn;
} fn(StateContext);

static void fn(generate_hann_window)(ftype *window, ftype *inv_window, const int size)
{
    for (int i = 0; i < size; i++) {
        ftype value = F(0.5) * (F(1.0) - FCOS(F(2.0) * MPI * i / size));

        window[i] = value;
        // 1/window to calculate unwindowed peak.
        inv_window[i] = value > F(0.1) ? F(1.0) / value : F(0.0);
    }
}

static void fn(set_margin_curve)(AudioPsyClipContext *s,
                                 const float *bands, const float *gains,
                                 int num_points, int sample_rate)
{
    fn(StateContext) *st = s->st;
    int j = 0;

    st->margin_curve[0] = gains[0];

    for (int i = 0; i < num_points - 1; i++) {
        while (j < s->nb_bins) {
            // linearly interpolate between points
            ftype binHz = (j * sample_rate) / (ftype)s->fft_size;
            st->margin_curve[j] = gains[i] + (binHz - bands[i]) * (gains[i + 1] - gains[i]) / (bands[i + 1] - bands[i]);
            j++;
            if (binHz >= bands[i])
                break;
        }
    }
    // handle bins after the last point
    while (j < s->nb_bins) {
        st->margin_curve[j] = gains[num_points - 1];
        j++;
    }

    // convert margin curve to linear amplitude scale
    for (int i = 0; i < s->nb_bins; i++)
        st->margin_curve[i] = FPOW(F(10.0), st->margin_curve[i] / F(20.0));
}

static void fn(generate_spread_table)(AudioPsyClipContext *s)
{
    fn(StateContext) *st = s->st;
    // Calculate tent-shape function in log-log scale.

    // As an optimization, only consider bins close to "bin"
    // This reduces the number of multiplications needed in calculate_mask_curve
    // The masking contribution at faraway bins is negligeable

    // Another optimization to save memory and speed up the calculation of the
    // spread table is to calculate and store only 2 spread functions per
    // octave, and reuse the same spread function for multiple bins.
    int table_index = 0;
    int bin = 0;
    int increment = 1;

    while (bin < s->num_psy_bins) {
        ftype sum = 0;
        int base_idx = table_index * s->num_psy_bins;
        int start_bin = bin * 3 / 4;
        int end_bin = FFMIN(s->num_psy_bins, ((bin + 1) * 4 + 2) / 3);
        int next_bin;

        for (int j = start_bin; j < end_bin; j++) {
            // add 0.5 so i=0 doesn't get log(0)
            ftype rel_idx_log = FABS(FLOG((j + F(0.5)) / (bin + F(0.5))));
            ftype value;
            if (j >= bin) {
                // mask up
                value = FEXP(-rel_idx_log * F(40.0));
            } else {
                // mask down
                value = FEXP(-rel_idx_log * F(80.0));
            }
            // the spreading function is centred in the row
            sum += value;
            st->spread_table[base_idx + s->num_psy_bins / 2 + j - bin] = value;
        }
        // now normalize it
        for (int j = start_bin; j < end_bin; j++) {
            st->spread_table[base_idx + s->num_psy_bins / 2 + j - bin] /= sum;
        }

        st->spread_table_range[table_index][0] = start_bin - bin;
        st->spread_table_range[table_index][1] = end_bin - bin;

        if (bin <= 1) {
            next_bin = bin + 1;
        } else {
            if ((bin & (bin - 1)) == 0) {
                // power of 2
                increment = bin / 2;
            }

            next_bin = bin + increment;
        }

        // set bins between "bin" and "next_bin" to use this table_index
        for (int i = bin; i < next_bin; i++)
            st->spread_table_index[i] = table_index;

        bin = next_bin;
        table_index++;
    }
}

static void fn(apply_window)(AudioPsyClipContext *s,
                             const ftype *in_frame, ftype *out_frame,
                             const int add_to_out_frame)
{
    fn(StateContext) *st = s->st;
    const ftype *window = st->window;
    const int fft_size = s->fft_size;

    if (add_to_out_frame) {
        for (int i = 0; i < fft_size; i++)
            out_frame[i] += in_frame[i] * window[i];
    } else {
        for (int i = 0; i < fft_size; i++)
            out_frame[i] = in_frame[i] * window[i];
    }
}

static void fn(calculate_mask_curve)(AudioPsyClipContext *s,
                                     const ctype *spectrum, ftype *mask_curve)
{
    fn(StateContext) *st = s->st;

    for (int i = 0; i < s->nb_bins; i++)
        mask_curve[i] = 0;

    for (int i = 0; i < s->num_psy_bins; i++) {
        int base_idx, start_bin, end_bin, table_idx;
        ftype magnitude;
        int range[2];

        if (i == 0) {
            magnitude = FABS(spectrum[0].re);
        } else if (i == s->nb_bins-1) {
            magnitude = FABS(spectrum[i].re);
        } else {
            magnitude = HYPOT(spectrum[i].re, spectrum[i].im);
        }

        table_idx = st->spread_table_index[i];
        range[0] = st->spread_table_range[table_idx][0];
        range[1] = st->spread_table_range[table_idx][1];
        base_idx = table_idx * s->num_psy_bins;
        start_bin = FFMAX(0, i + range[0]);
        end_bin = FFMIN(s->num_psy_bins, i + range[1]);

        for (int j = start_bin; j < end_bin; j++)
            mask_curve[j] += st->spread_table[base_idx + s->num_psy_bins / 2 + j - i] * magnitude;
    }

    // for ultrasonic frequencies, skip the O(n^2) spread calculation and just copy the magnitude
    for (int i = s->num_psy_bins; i < s->nb_bins; i++) {
        ftype magnitude;
        if (i == s->nb_bins-1) {
            magnitude = FABS(spectrum[i].re);
        } else {
            magnitude = HYPOT(spectrum[i].re, spectrum[i].im);
        }

        mask_curve[i] = magnitude;
    }

    for (int i = 0; i < s->nb_bins; i++)
        mask_curve[i] = mask_curve[i] / st->margin_curve[i];
}

static int fn(psy_init)(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AudioPsyClipContext *s = ctx->priv;
    fn(StateContext) *st;
    int ret, nb_points;

    s->st = av_calloc(1, sizeof(*st));
    if (!s->st)
        return AVERROR(ENOMEM);
    st = s->st;

    st->window = av_calloc(s->fft_size, sizeof(*st->window));
    st->inv_window = av_calloc(s->fft_size, sizeof(*st->inv_window));
    if (!st->window || !st->inv_window)
        return AVERROR(ENOMEM);

    fn(generate_hann_window)(st->window, st->inv_window, s->fft_size);

    st->margin_curve = av_calloc(s->nb_bins, sizeof(*st->margin_curve));
    if (!st->margin_curve)
        return AVERROR(ENOMEM);

    st->spread_table_rows = av_log2(s->num_psy_bins) * 2;
    st->spread_table = av_calloc(st->spread_table_rows * s->num_psy_bins, sizeof(*st->spread_table));
    if (!st->spread_table)
        return AVERROR(ENOMEM);

    st->spread_table_range = av_calloc(st->spread_table_rows * 2, sizeof(*st->spread_table_range));
    if (!st->spread_table_range)
        return AVERROR(ENOMEM);

    st->spread_table_index = av_calloc(s->num_psy_bins, sizeof(*st->spread_table_index));
    if (!st->spread_table_index)
        return AVERROR(ENOMEM);

    nb_points = FFMIN(s->nb_bands_opt, s->nb_gains_opt);
    fn(set_margin_curve)(s, s->bands_opt, s->gains_opt, nb_points, inlink->sample_rate);

    fn(generate_spread_table)(s);

    st->channels = inlink->ch_layout.nb_channels;

    st->tx_ctx = av_calloc(st->channels, sizeof(*st->tx_ctx));
    st->itx_ctx = av_calloc(st->channels, sizeof(*st->itx_ctx));
    if (!st->tx_ctx || !st->itx_ctx)
        return AVERROR(ENOMEM);

    for (int ch = 0; ch < st->channels; ch++) {
        ftype scale = F(1.0);

        ret = av_tx_init(&st->tx_ctx[ch], &st->tx_fn, TX_TYPE, 0, s->fft_size, &scale, 0);
        if (ret < 0)
            return ret;

        scale = F(1.0) / (F(1.5) * s->fft_size);
        ret = av_tx_init(&st->itx_ctx[ch], &st->itx_fn, TX_TYPE, 1, s->fft_size, &scale, 0);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static void fn(clip_to_window)(AudioPsyClipContext *s, const ftype *windowed_frame,
                               ftype *clipping_delta, ftype delta_boost)
{
    fn(StateContext) *st = s->st;
    const ftype clip_level = s->clip_level;
    const int fft_size = s->fft_size;
    const ftype *window = st->window;

    for (int i = 0; i < fft_size; i++) {
        const ftype limit = clip_level * window[i];
        const ftype effective_value = windowed_frame[i] + clipping_delta[i];

        if (effective_value > limit) {
            clipping_delta[i] += (limit - effective_value) * delta_boost;
        } else if (effective_value < -limit) {
            clipping_delta[i] += (-limit - effective_value) * delta_boost;
        }
    }
}

static void fn(limit_clip_spectrum)(AudioPsyClipContext *s,
                                    ctype *clip_spectrum, const ftype *mask_curve)
{
    // bin 0
    ftype relative_distortion_level = FABS(clip_spectrum[0].re) / mask_curve[0];

    if (relative_distortion_level > F(1.0))
        clip_spectrum[0].re /= relative_distortion_level;

    // bin 1..N/2-1
    for (int i = 1; i < s->nb_bins-1; i++) {
        ftype re = clip_spectrum[i].re;
        ftype im = clip_spectrum[i].im;
        relative_distortion_level = HYPOT(re, im) / mask_curve[i];
        if (relative_distortion_level > F(1.0)) {
            clip_spectrum[i].re /= relative_distortion_level;
            clip_spectrum[i].im /= relative_distortion_level;
        }
    }
    // bin N/2
    relative_distortion_level = FABS(clip_spectrum[s->nb_bins-1].re) / mask_curve[s->nb_bins-1];
    if (relative_distortion_level > F(1.0))
        clip_spectrum[s->nb_bins-1].re /= relative_distortion_level;
}

static void fn(feed)(AVFilterContext *ctx, int ch,
                     const ftype *in_samples, ftype *out_samples, int diff_only,
                     ftype *in_frame, ftype *out_dist_frame,
                     ftype *windowed_frame, ftype *clipping_delta,
                     ctype *spectrum_buf, ftype *mask_curve)
{
    AudioPsyClipContext *s = ctx->priv;
    fn(StateContext) *st = s->st;
    const ftype *inv_window = st->inv_window;
    const int overlap = s->overlap;
    const int fft_size = s->fft_size;
    const int offset = fft_size - overlap;
    const ftype clip_level_inv = F(1.0) / s->clip_level;
    const ftype level_out = s->level_out;
    ftype orig_peak = F(0.0), peak;

    // shift in/out buffers
    memmove(in_frame, &in_frame[overlap], offset * sizeof(*in_frame));
    memmove(out_dist_frame, &out_dist_frame[overlap], offset * sizeof(*out_dist_frame));

    memcpy(&in_frame[offset], in_samples, overlap * sizeof(*in_frame));
    memset(&out_dist_frame[offset], 0, overlap * sizeof(*out_dist_frame));

    fn(apply_window)(s, in_frame, windowed_frame, 0);
    st->tx_fn(st->tx_ctx[ch], spectrum_buf, windowed_frame, sizeof(*windowed_frame));
    fn(calculate_mask_curve)(s, spectrum_buf, mask_curve);

    // It would be easier to calculate the peak from the unwindowed input.
    // This is just for consistency with the clipped peak calculation
    // because the inv_window zeros out samples on the edge of the window.
    for (int i = 0; i < fft_size; i++)
        orig_peak = FMAX(orig_peak, FABS(windowed_frame[i] * inv_window[i]));
    orig_peak *= clip_level_inv;
    peak = orig_peak;

    // clear clipping_delta
    for (int i = 0; i < fft_size; i++)
        clipping_delta[i] = F(0.0);

    // repeat clipping-filtering process a few times to control both the peaks and the spectrum
    for (int i = 0; i < s->max_iterations; i++) {
        ftype mask_curve_shift = F(1.122); // 1.122 is 1dB
        // The last 1/3 of rounds have boosted delta to help reach the peak target faster
        ftype delta_boost = F(1.0);
        if (i >= s->max_iterations - s->max_iterations / 3) {
            // boosting the delta when largs peaks are still present is dangerous
            if (peak < F(2.0))
                delta_boost = F(2.0);
        }

        fn(clip_to_window)(s, windowed_frame, clipping_delta, delta_boost);

        st->tx_fn(st->tx_ctx[ch], spectrum_buf, clipping_delta, sizeof(*clipping_delta));

        fn(limit_clip_spectrum)(s, spectrum_buf, mask_curve);

        st->itx_fn(st->itx_ctx[ch], clipping_delta, spectrum_buf, sizeof(*spectrum_buf));

        peak = F(0.0);
        for (int j = 0; j < fft_size; j++)
            peak = FMAX(peak, FABS((windowed_frame[j] + clipping_delta[j]) * inv_window[j]));
        peak *= clip_level_inv;

        // Automatically adjust mask_curve as necessary to reach peak target
        if (orig_peak > F(1.0) && peak > F(1.0)) {
            ftype diff_achieved = orig_peak - peak;
            if (i + 1 < s->max_iterations - s->max_iterations / 3 && diff_achieved > 0) {
                ftype diff_needed = orig_peak - F(1.0);
                ftype diff_ratio = diff_needed / diff_achieved;
                // If a good amount of peak reduction was already achieved,
                // don't shift the mask_curve by the full peak value
                // On the other hand, if only a little peak reduction was achieved,
                // don't shift the mask_curve by the enormous diff_ratio.
                diff_ratio = FMIN(diff_ratio, peak);
                mask_curve_shift = FMAX(mask_curve_shift, diff_ratio);
            } else {
                // If the peak got higher than the input or we are in the last 1/3 rounds,
                // go back to the heavy-handed peak heuristic.
                mask_curve_shift = FMAX(mask_curve_shift, peak);
            }
        } else if (peak < F(1.0) && i >= s->min_iterations) {
            break;
        }

        mask_curve_shift = F(1.0) + (mask_curve_shift - F(1.0)) * s->adaptive;

        // Be less strict in the next iteration.
        // This helps with peak control.
        for (int j = 0; j < fft_size / 2 + 1; j++)
            mask_curve[j] *= mask_curve_shift;
    }

    // do overlap & add
    fn(apply_window)(s, clipping_delta, out_dist_frame, 1);

    if (ff_filter_disabled(ctx)) {
        memcpy(out_samples, in_frame, overlap * sizeof(*out_samples));
    } else {
        memcpy(out_samples, out_dist_frame, overlap * sizeof(*out_samples));
        if (!diff_only) {
            for (int i = 0; i < overlap; i++)
                out_samples[i] += in_frame[i];
        }

        if (s->auto_level) {
            for (int i = 0; i < overlap; i++)
                out_samples[i] *= clip_level_inv;
        }

        for (int i = 0; i < overlap; i++)
            out_samples[i] *= level_out;
    }
}

static int fn(psy_channel)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch)
{
    AudioPsyClipContext *s = ctx->priv;
    ftype *in_buffer = (ftype *)s->in_buffer->extended_data[ch];
    const ftype level_in = s->level_in;
    const int overlap = s->overlap;

    for (int offset = 0; offset < out->nb_samples; offset += overlap) {
        const ftype *src = ((const ftype *)in->extended_data[ch])+offset;
        ftype *dst = ((ftype *)out->extended_data[ch])+offset;

        for (int n = 0; n < overlap; n++)
            in_buffer[n] = src[n] * level_in;

        fn(feed)(ctx, ch, in_buffer, dst, s->diff_only,
                 (ftype *)(s->in_frame->extended_data[ch]),
                 (ftype *)(s->out_dist_frame->extended_data[ch]),
                 (ftype *)(s->windowed_frame->extended_data[ch]),
                 (ftype *)(s->clipping_delta->extended_data[ch]),
                 (ctype *)(s->spectrum_buf->extended_data[ch]),
                 (ftype *)(s->mask_curve->extended_data[ch]));
    }

    return 0;
}

static int fn(psy_flush_channel)(AVFilterContext *ctx, AVFrame *out, const int ch)
{
    AudioPsyClipContext *s = ctx->priv;
    ftype *in_buffer = (ftype *)s->in_buffer->extended_data[ch];
    const int overlap = s->overlap;

    for (int offset = 0; offset < out->nb_samples; offset += overlap) {
        ftype *dst = ((ftype *)out->extended_data[ch])+offset;

        for (int n = 0; n < overlap; n++)
            in_buffer[n] = F(0.0);

        fn(feed)(ctx, ch, in_buffer, dst, s->diff_only,
                 (ftype *)(s->in_frame->extended_data[ch]),
                 (ftype *)(s->out_dist_frame->extended_data[ch]),
                 (ftype *)(s->windowed_frame->extended_data[ch]),
                 (ftype *)(s->clipping_delta->extended_data[ch]),
                 (ctype *)(s->spectrum_buf->extended_data[ch]),
                 (ftype *)(s->mask_curve->extended_data[ch]));
    }

    return 0;
}

static void fn(psy_uninit)(AVFilterContext *ctx)
{
    AudioPsyClipContext *s = ctx->priv;
    fn(StateContext) *st = s->st;

    if (!st)
        return;

    av_freep(&st->window);
    av_freep(&st->inv_window);
    av_freep(&st->spread_table);
    av_freep(&st->spread_table_range);
    av_freep(&st->spread_table_index);
    av_freep(&st->margin_curve);

    for (int ch = 0; ch < st->channels; ch++) {
        if (st->tx_ctx)
            av_tx_uninit(&st->tx_ctx[ch]);
        if (st->itx_ctx)
            av_tx_uninit(&st->itx_ctx[ch]);
    }

    av_freep(&st->tx_ctx);
    av_freep(&st->itx_ctx);
    av_freep(&s->st);
}
