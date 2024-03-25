/*
 * Copyright (c) 2014 - 2021 Jason Jang
 * Copyright (c) 2021 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "internal.h"

typedef struct AudioPsyClipContext {
    const AVClass *class;

    float *bands_opt;
    unsigned nb_bands_opt;
    float *gains_opt;
    unsigned nb_gains_opt;

    double level_in;
    double level_out;
    double clip_level;
    double adaptive;
    int auto_level;
    int max_iterations;
    int min_iterations;
    int diff_only;
    char *protections_str;
    double *protections;

    int num_psy_bins;
    int fft_size;
    int overlap;
    int channels;

    int spread_table_rows;
    int *spread_table_index;
    int (*spread_table_range)[2];
    float *window, *inv_window, *spread_table, *margin_curve;

    AVFrame *in;
    AVFrame *in_buffer;
    AVFrame *in_frame;
    AVFrame *out_dist_frame;
    AVFrame *windowed_frame;
    AVFrame *clipping_delta;
    AVFrame *spectrum_buf;
    AVFrame *mask_curve;

    AVTXContext **tx_ctx, **itx_ctx;
    av_tx_fn tx_fn, itx_fn;
} AudioPsyClipContext;

#define OFFSET(x) offsetof(AudioPsyClipContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

#define DEFAULT_BANDS "0 125 250 500 1000 2000 4000 8000 16000 20000"
#define DEFAULT_GAINS "14 14 16 18 20 20 20 17 14 -10"

static const AVOptionArrayDef def_bands = {.def=DEFAULT_BANDS,.size_min=2,.sep=' '};
static const AVOptionArrayDef def_gains = {.def=DEFAULT_GAINS,.size_min=2,.sep=' '};

static const AVOption apsyclip_options[] = {
    { "level_in",   "set input level",         OFFSET(level_in),   AV_OPT_TYPE_DOUBLE, {.dbl=1},.015625,   64, FLAGS },
    { "level_out",  "set output level",        OFFSET(level_out),  AV_OPT_TYPE_DOUBLE, {.dbl=1},.015625,   64, FLAGS },
    { "clip",       "set clip level",          OFFSET(clip_level), AV_OPT_TYPE_DOUBLE, {.dbl=1},.015625,    1, FLAGS },
    { "diff",       "enable difference",       OFFSET(diff_only),  AV_OPT_TYPE_BOOL,   {.i64=0},      0,    1, FLAGS },
    { "adaptive",   "set adaptive distortion", OFFSET(adaptive),   AV_OPT_TYPE_DOUBLE, {.dbl=0.5},    0,    1, FLAGS },
    { "iterations", "set max iterations",      OFFSET(max_iterations), AV_OPT_TYPE_INT,{.i64=10},     1,   20, FLAGS },
    { "min_iterations", "set min iterations",  OFFSET(min_iterations), AV_OPT_TYPE_INT,{.i64=1},      1,   20, FLAGS },
    { "level",      "set auto level",          OFFSET(auto_level), AV_OPT_TYPE_BOOL,   {.i64=0},      0,    1, FLAGS },
    { "bands", "set frequency values per band",OFFSET(bands_opt),  AV_OPT_TYPE_FLOAT|AR,{.arr=&def_bands}, 0, INT_MAX, FLAGS },
    { "gains", "set gain values per band",     OFFSET(gains_opt),  AV_OPT_TYPE_FLOAT|AR,{.arr=&def_gains}, INT_MIN, INT_MAX, FLAGS },
    {NULL}
};

AVFILTER_DEFINE_CLASS(apsyclip);

static void generate_hann_window(float *window, float *inv_window, int size)
{
    for (int i = 0; i < size; i++) {
        float value = 0.5f * (1.f - cosf(2.f * M_PI * i / size));

        window[i] = value;
        // 1/window to calculate unwindowed peak.
        inv_window[i] = value > 0.1f ? 1.f / value : 0.f;
    }
}

static void set_margin_curve(AudioPsyClipContext *s,
                             const float *bands, const float *gains,
                             int num_points, int sample_rate)
{
    int j = 0;

    s->margin_curve[0] = gains[0];

    for (int i = 0; i < num_points - 1; i++) {
        while (j < s->fft_size / 2 + 1 && j * sample_rate / s->fft_size < bands[i + 1]) {
            // linearly interpolate between points
            int binHz = j * sample_rate / s->fft_size;
            s->margin_curve[j] = gains[i] + (binHz - bands[i]) * (gains[i + 1] - gains[i]) / (bands[i + 1] - bands[i]);
            j++;
        }
    }
    // handle bins after the last point
    while (j < s->fft_size / 2 + 1) {
        s->margin_curve[j] = gains[num_points - 1];
        j++;
    }

    // convert margin curve to linear amplitude scale
    for (j = 0; j < s->fft_size / 2 + 1; j++)
        s->margin_curve[j] = powf(10.f, s->margin_curve[j] / 20.f);
}

static void generate_spread_table(AudioPsyClipContext *s)
{
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
        float sum = 0;
        int base_idx = table_index * s->num_psy_bins;
        int start_bin = bin * 3 / 4;
        int end_bin = FFMIN(s->num_psy_bins, ((bin + 1) * 4 + 2) / 3);
        int next_bin;

        for (int j = start_bin; j < end_bin; j++) {
            // add 0.5 so i=0 doesn't get log(0)
            float rel_idx_log = fabsf(logf((j + 0.5f) / (bin + 0.5f)));
            float value;
            if (j >= bin) {
                // mask up
                value = expf(-rel_idx_log * 40.f);
            } else {
                // mask down
                value = expf(-rel_idx_log * 80.f);
            }
            // the spreading function is centred in the row
            sum += value;
            s->spread_table[base_idx + s->num_psy_bins / 2 + j - bin] = value;
        }
        // now normalize it
        for (int j = start_bin; j < end_bin; j++) {
            s->spread_table[base_idx + s->num_psy_bins / 2 + j - bin] /= sum;
        }

        s->spread_table_range[table_index][0] = start_bin - bin;
        s->spread_table_range[table_index][1] = end_bin - bin;

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
            s->spread_table_index[i] = table_index;

        bin = next_bin;
        table_index++;
    }
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    AudioPsyClipContext *s = ctx->priv;
    int ret, nb_points;

    s->fft_size = inlink->sample_rate > 100000 ? 1024 : inlink->sample_rate > 50000 ? 512 : 256;
    s->overlap = s->fft_size / 4;

    // The psy masking calculation is O(n^2),
    // so skip it for frequencies not covered by base sampling rantes (i.e. 44k)
    if (inlink->sample_rate <= 50000) {
        s->num_psy_bins = s->fft_size / 2;
    } else if (inlink->sample_rate <= 100000) {
        s->num_psy_bins = s->fft_size / 4;
    } else {
        s->num_psy_bins = s->fft_size / 8;
    }

    s->window = av_calloc(s->fft_size, sizeof(*s->window));
    s->inv_window = av_calloc(s->fft_size, sizeof(*s->inv_window));
    if (!s->window || !s->inv_window)
        return AVERROR(ENOMEM);

    s->in_buffer      = ff_get_audio_buffer(inlink, s->fft_size * 2);
    s->in_frame       = ff_get_audio_buffer(inlink, s->fft_size * 2);
    s->out_dist_frame = ff_get_audio_buffer(inlink, s->fft_size * 2);
    s->windowed_frame = ff_get_audio_buffer(inlink, s->fft_size * 2);
    s->clipping_delta = ff_get_audio_buffer(inlink, s->fft_size * 2);
    s->spectrum_buf   = ff_get_audio_buffer(inlink, s->fft_size * 2);
    s->mask_curve     = ff_get_audio_buffer(inlink, s->fft_size / 2 + 1);
    if (!s->in_buffer || !s->in_frame ||
        !s->out_dist_frame || !s->windowed_frame ||
        !s->clipping_delta || !s->spectrum_buf || !s->mask_curve)
        return AVERROR(ENOMEM);

    generate_hann_window(s->window, s->inv_window, s->fft_size);

    s->margin_curve = av_calloc(s->fft_size / 2 + 1, sizeof(*s->margin_curve));
    if (!s->margin_curve)
        return AVERROR(ENOMEM);

    s->spread_table_rows = av_log2(s->num_psy_bins) * 2;
    s->spread_table = av_calloc(s->spread_table_rows * s->num_psy_bins, sizeof(*s->spread_table));
    if (!s->spread_table)
        return AVERROR(ENOMEM);

    s->spread_table_range = av_calloc(s->spread_table_rows * 2, sizeof(*s->spread_table_range));
    if (!s->spread_table_range)
        return AVERROR(ENOMEM);

    s->spread_table_index = av_calloc(s->num_psy_bins, sizeof(*s->spread_table_index));
    if (!s->spread_table_index)
        return AVERROR(ENOMEM);

    nb_points = FFMIN(s->nb_bands_opt, s->nb_gains_opt);
    set_margin_curve(s, s->bands_opt, s->gains_opt, nb_points, inlink->sample_rate);

    generate_spread_table(s);

    s->channels = inlink->ch_layout.nb_channels;

    s->tx_ctx = av_calloc(s->channels, sizeof(*s->tx_ctx));
    s->itx_ctx = av_calloc(s->channels, sizeof(*s->itx_ctx));
    if (!s->tx_ctx || !s->itx_ctx)
        return AVERROR(ENOMEM);

    for (int ch = 0; ch < s->channels; ch++) {
        float scale = 1.f;

        ret = av_tx_init(&s->tx_ctx[ch], &s->tx_fn, AV_TX_FLOAT_RDFT, 0, s->fft_size, &scale, 0);
        if (ret < 0)
            return ret;

        scale = 1.f / (1.5f * s->fft_size);
        ret = av_tx_init(&s->itx_ctx[ch], &s->itx_fn, AV_TX_FLOAT_RDFT, 1, s->fft_size, &scale, 0);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static void apply_window(AudioPsyClipContext *s,
                         const float *in_frame, float *out_frame, const int add_to_out_frame)
{
    const float *window = s->window;
    const int fft_size = s->fft_size;

    if (add_to_out_frame) {
        for (int i = 0; i < fft_size; i++)
            out_frame[i] += in_frame[i] * window[i];
    } else {
        for (int i = 0; i < fft_size; i++)
            out_frame[i] = in_frame[i] * window[i];
    }
}

static void calculate_mask_curve(AudioPsyClipContext *s,
                                 const float *spectrum, float *mask_curve)
{
    for (int i = 0; i < s->fft_size / 2 + 1; i++)
        mask_curve[i] = 0;

    for (int i = 0; i < s->num_psy_bins; i++) {
        int base_idx, start_bin, end_bin, table_idx;
        float magnitude;
        int range[2];

        if (i == 0) {
            magnitude = fabsf(spectrum[0]);
        } else if (i == s->fft_size / 2) {
            magnitude = fabsf(spectrum[s->fft_size]);
        } else {
            // Because the input signal is real, the + and - frequencies are redundant.
            // Multiply the magnitude by 2 to simulate adding up the + and - frequencies.
            magnitude = hypotf(spectrum[2 * i], spectrum[2 * i + 1]) * 2;
        }

        table_idx = s->spread_table_index[i];
        range[0] = s->spread_table_range[table_idx][0];
        range[1] = s->spread_table_range[table_idx][1];
        base_idx = table_idx * s->num_psy_bins;
        start_bin = FFMAX(0, i + range[0]);
        end_bin = FFMIN(s->num_psy_bins, i + range[1]);

        for (int j = start_bin; j < end_bin; j++)
            mask_curve[j] += s->spread_table[base_idx + s->num_psy_bins / 2 + j - i] * magnitude;
    }

    // for ultrasonic frequencies, skip the O(n^2) spread calculation and just copy the magnitude
    for (int i = s->num_psy_bins; i < s->fft_size / 2 + 1; i++) {
        float magnitude;
        if (i == s->fft_size / 2) {
            magnitude = fabsf(spectrum[s->fft_size]);
        } else {
            // Because the input signal is real, the + and - frequencies are redundant.
            // Multiply the magnitude by 2 to simulate adding up the + and - frequencies.
            magnitude = hypotf(spectrum[2 * i], spectrum[2 * i + 1]) * 2;
        }

        mask_curve[i] = magnitude;
    }

    for (int i = 0; i < s->fft_size / 2 + 1; i++)
        mask_curve[i] = mask_curve[i] / s->margin_curve[i];
}

static void clip_to_window(AudioPsyClipContext *s,
                           const float *windowed_frame, float *clipping_delta, float delta_boost)
{
    const float *window = s->window;

    for (int i = 0; i < s->fft_size; i++) {
        const float limit = s->clip_level * window[i];
        const float effective_value = windowed_frame[i] + clipping_delta[i];

        if (effective_value > limit) {
            clipping_delta[i] += (limit - effective_value) * delta_boost;
        } else if (effective_value < -limit) {
            clipping_delta[i] += (-limit - effective_value) * delta_boost;
        }
    }
}

static void limit_clip_spectrum(AudioPsyClipContext *s,
                                float *clip_spectrum, const float *mask_curve)
{
    // bin 0
    float relative_distortion_level = fabsf(clip_spectrum[0]) / mask_curve[0];

    if (relative_distortion_level > 1.f)
        clip_spectrum[0] /= relative_distortion_level;

    // bin 1..N/2-1
    for (int i = 1; i < s->fft_size / 2; i++) {
        float real = clip_spectrum[i * 2];
        float imag = clip_spectrum[i * 2 + 1];
        // Because the input signal is real, the + and - frequencies are redundant.
        // Multiply the magnitude by 2 to simulate adding up the + and - frequencies.
        relative_distortion_level = hypotf(real, imag) * 2 / mask_curve[i];
        if (relative_distortion_level > 1.0) {
            clip_spectrum[i * 2] /= relative_distortion_level;
            clip_spectrum[i * 2 + 1] /= relative_distortion_level;
        }
    }
    // bin N/2
    relative_distortion_level = fabsf(clip_spectrum[s->fft_size]) / mask_curve[s->fft_size / 2];
    if (relative_distortion_level > 1.f)
        clip_spectrum[s->fft_size] /= relative_distortion_level;
}

static void feed(AVFilterContext *ctx, int ch,
                 const float *in_samples, float *out_samples, int diff_only,
                 float *in_frame, float *out_dist_frame,
                 float *windowed_frame, float *clipping_delta,
                 float *spectrum_buf, float *mask_curve)
{
    AudioPsyClipContext *s = ctx->priv;
    const float *inv_window = s->inv_window;
    const int overlap = s->overlap;
    const int fft_size = s->fft_size;
    const int offset = fft_size - overlap;
    const float clip_level_inv = 1.f / s->clip_level;
    const float level_out = s->level_out;
    float orig_peak = 0.f, peak;

    // shift in/out buffers
    memmove(in_frame, &in_frame[overlap], offset * sizeof(float));
    memmove(out_dist_frame, &out_dist_frame[overlap], offset * sizeof(float));

    memcpy(&in_frame[offset], in_samples, overlap * sizeof(float));
    memset(&out_dist_frame[offset], 0, overlap * sizeof(float));

    apply_window(s, in_frame, windowed_frame, 0);
    s->tx_fn(s->tx_ctx[ch], spectrum_buf, windowed_frame, sizeof(float));
    calculate_mask_curve(s, spectrum_buf, mask_curve);

    // It would be easier to calculate the peak from the unwindowed input.
    // This is just for consistency with the clipped peak calculation
    // because the inv_window zeros out samples on the edge of the window.
    for (int i = 0; i < fft_size; i++)
        orig_peak = fmaxf(orig_peak, fabsf(windowed_frame[i] * inv_window[i]));
    orig_peak *= clip_level_inv;
    peak = orig_peak;

    // clear clipping_delta
    for (int i = 0; i < fft_size * 2; i++)
        clipping_delta[i] = 0.f;

    // repeat clipping-filtering process a few times to control both the peaks and the spectrum
    for (int i = 0; i < s->max_iterations; i++) {
        float mask_curve_shift = 1.122f; // 1.122 is 1dB
        // The last 1/3 of rounds have boosted delta to help reach the peak target faster
        float delta_boost = 1.f;
        if (i >= s->max_iterations - s->max_iterations / 3) {
            // boosting the delta when largs peaks are still present is dangerous
            if (peak < 2.f)
                delta_boost = 2.f;
        }

        clip_to_window(s, windowed_frame, clipping_delta, delta_boost);

        s->tx_fn(s->tx_ctx[ch], spectrum_buf, clipping_delta, sizeof(float));

        limit_clip_spectrum(s, spectrum_buf, mask_curve);

        s->itx_fn(s->itx_ctx[ch], clipping_delta, spectrum_buf, sizeof(AVComplexFloat));

        peak = 0.f;
        for (int i = 0; i < fft_size; i++)
            peak = fmaxf(peak, fabsf((windowed_frame[i] + clipping_delta[i]) * inv_window[i]));
        peak *= clip_level_inv;

        // Automatically adjust mask_curve as necessary to reach peak target
        if (orig_peak > 1.f && peak > 1.f) {
            float diff_achieved = orig_peak - peak;
            if (i + 1 < s->max_iterations - s->max_iterations / 3 && diff_achieved > 0) {
                float diff_needed = orig_peak - 1.f;
                float diff_ratio = diff_needed / diff_achieved;
                // If a good amount of peak reduction was already achieved,
                // don't shift the mask_curve by the full peak value
                // On the other hand, if only a little peak reduction was achieved,
                // don't shift the mask_curve by the enormous diff_ratio.
                diff_ratio = fminf(diff_ratio, peak);
                mask_curve_shift = fmaxf(mask_curve_shift, diff_ratio);
            } else {
                // If the peak got higher than the input or we are in the last 1/3 rounds,
                // go back to the heavy-handed peak heuristic.
                mask_curve_shift = fmaxf(mask_curve_shift, peak);
            }
        } else if (peak < 1.f && i >= s->min_iterations) {
            break;
        }

        mask_curve_shift = 1.f + (mask_curve_shift - 1.f) * s->adaptive;

        // Be less strict in the next iteration.
        // This helps with peak control.
        for (int i = 0; i < fft_size / 2 + 1; i++)
            mask_curve[i] *= mask_curve_shift;
    }

    // do overlap & add
    apply_window(s, clipping_delta, out_dist_frame, 1);

    if (ctx->is_disabled) {
        memcpy(out_samples, in_frame, overlap * sizeof(float));
    } else {
        memcpy(out_samples, out_dist_frame, overlap * sizeof(float));
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

static int psy_channel(AVFilterContext *ctx, AVFrame *in, AVFrame *out, int ch)
{
    AudioPsyClipContext *s = ctx->priv;
    float *in_buffer = (float *)s->in_buffer->extended_data[ch];
    const float level_in = s->level_in;
    const int overlap = s->overlap;

    for (int offset = 0; offset < out->nb_samples; offset += overlap) {
        const float *src = ((const float *)in->extended_data[ch])+offset;
        float *dst = ((float *)out->extended_data[ch])+offset;

        for (int n = 0; n < overlap; n++)
            in_buffer[n] = src[n] * level_in;

        feed(ctx, ch, in_buffer, dst, s->diff_only,
             (float *)(s->in_frame->extended_data[ch]),
             (float *)(s->out_dist_frame->extended_data[ch]),
             (float *)(s->windowed_frame->extended_data[ch]),
             (float *)(s->clipping_delta->extended_data[ch]),
             (float *)(s->spectrum_buf->extended_data[ch]),
             (float *)(s->mask_curve->extended_data[ch]));
    }

    return 0;
}

static int psy_channels(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioPsyClipContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++)
        psy_channel(ctx, s->in, out, ch);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioPsyClipContext *s = ctx->priv;
    AVFrame *out;
    int ret;

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    s->in = in;
    av_frame_copy_props(out, in);
    ff_filter_execute(ctx, psy_channels, out, NULL,
                      FFMIN(outlink->ch_layout.nb_channels, ff_filter_get_nb_threads(ctx)));

    out->pts = in->pts;
    out->pts -= av_rescale_q(s->fft_size - s->overlap, av_make_q(1, outlink->sample_rate), outlink->time_base);
    out->nb_samples = in->nb_samples;
    ret = ff_filter_frame(outlink, out);
fail:
    av_frame_free(&in);
    s->in = NULL;
    return ret < 0 ? ret : 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AudioPsyClipContext *s = ctx->priv;
    int ret, status, available, wanted;
    AVFrame *in = NULL;
    int64_t pts;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    available = ff_inlink_queued_samples(inlink);
    wanted = FFMAX(s->overlap, (available / s->overlap) * s->overlap);
    ret = ff_inlink_consume_samples(inlink, wanted, wanted, &in);
    if (ret < 0)
        return ret;

    if (ret > 0) {
        return filter_frame(inlink, in);
    } else if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        ff_outlink_set_status(outlink, status, pts);
        return 0;
    } else {
        if (ff_inlink_queued_samples(inlink) >= s->overlap) {
            ff_filter_set_ready(ctx, 10);
        } else if (ff_outlink_frame_wanted(outlink)) {
            ff_inlink_request_frame(inlink);
        }
        return 0;
    }
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioPsyClipContext *s = ctx->priv;

    av_freep(&s->window);
    av_freep(&s->inv_window);
    av_freep(&s->spread_table);
    av_freep(&s->spread_table_range);
    av_freep(&s->spread_table_index);
    av_freep(&s->margin_curve);

    av_frame_free(&s->in_buffer);
    av_frame_free(&s->in_frame);
    av_frame_free(&s->out_dist_frame);
    av_frame_free(&s->windowed_frame);
    av_frame_free(&s->clipping_delta);
    av_frame_free(&s->spectrum_buf);
    av_frame_free(&s->mask_curve);

    for (int ch = 0; ch < s->channels; ch++) {
        if (s->tx_ctx)
            av_tx_uninit(&s->tx_ctx[ch]);
        if (s->itx_ctx)
            av_tx_uninit(&s->itx_ctx[ch]);
    }

    av_freep(&s->tx_ctx);
    av_freep(&s->itx_ctx);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

const AVFilter ff_af_apsyclip = {
    .name            = "apsyclip",
    .description     = NULL_IF_CONFIG_SMALL("Audio Psychoacoustic Clipper."),
    .priv_size       = sizeof(AudioPsyClipContext),
    .priv_class      = &apsyclip_class,
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SINGLE_SAMPLEFMT(AV_SAMPLE_FMT_FLTP),
    .flags           = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_SLICE_THREADS,
    .activate        = activate,
    .process_command = ff_filter_process_command,
};
