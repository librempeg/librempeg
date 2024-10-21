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
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ftype float
#define SAMPLE_FORMAT flt
#else
#define ftype double
#define SAMPLE_FORMAT dbl
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define DIFFSIGN(x,y) (((x)>(y)) - ((x)<-(y)))

static void fn(analyze_channel)(AVFilterContext *ctx, ChannelContext *cc,
                                     const uint8_t *srcp, int nb_samples)
{
    const ftype min_peak = F(1./32768.);
    const ftype *src = (const ftype *)srcp;
    PeriodItem *pi = (PeriodItem *)&cc->pi;
    int pi_end = cc->pi_end;
    int state = cc->state;
    int n = 0;

    if (state == -2)
        state = DIFFSIGN(src[0], min_peak);

    while (n < nb_samples) {
        int new_size, split = 0;
        ftype new_max_peak;
        ftype new_rms_sum;

        split = (!state) && pi[pi_end].size >= nb_samples;
        if (state != DIFFSIGN(src[n], min_peak) || split) {
            ftype max_peak = pi[pi_end].max_peak;
            ftype rms_sum = pi[pi_end].rms_sum;
            int old_state = state;

            state = DIFFSIGN(src[n], min_peak);
            av_assert1(pi[pi_end].size > 0);
            if (max_peak >= min_peak || split) {
                pi[pi_end].type = 1;
                cc->acc += pi[pi_end].size;
                pi_end++;
                if (pi_end >= MAX_ITEMS)
                    pi_end = 0;
                if (state != old_state) {
                    pi[pi_end].max_peak = DBL_MIN;
                    pi[pi_end].rms_sum = F(0.0);
                } else {
                    pi[pi_end].max_peak = max_peak;
                    pi[pi_end].rms_sum = rms_sum;
                }
                pi[pi_end].type = 0;
                pi[pi_end].size = 0;
                av_assert1(pi_end != cc->pi_start);
            }
        }

        new_max_peak = pi[pi_end].max_peak;
        new_rms_sum = pi[pi_end].rms_sum;
        new_size = pi[pi_end].size;
        if (state > F(0.0)) {
            while (src[n] > min_peak) {
                new_max_peak = FFMAX(new_max_peak,  src[n]);
                new_rms_sum += src[n] * src[n];
                new_size++;
                n++;
                if (n >= nb_samples)
                    break;
            }
        } else if (state < F(0.0)) {
            while (src[n] < min_peak) {
                new_max_peak = FFMAX(new_max_peak, -src[n]);
                new_rms_sum += src[n] * src[n];
                new_size++;
                n++;
                if (n >= nb_samples)
                    break;
            }
        } else {
            while (src[n] >= -min_peak && src[n] <= min_peak) {
                new_max_peak = min_peak;
                new_size++;
                n++;
                if (n >= nb_samples)
                    break;
            }
        }

        pi[pi_end].max_peak = new_max_peak;
        pi[pi_end].rms_sum = new_rms_sum;
        pi[pi_end].size = new_size;
    }
    cc->pi_end = pi_end;
    cc->state = state;
}

static ftype fn(lerp)(ftype min, ftype max, ftype mix)
{
    return min + (max - min) * mix;
}

static void fn(filter_link_channels)(AVFilterContext *ctx,
                                     AVFrame *in, AVFrame *out,
                                     int nb_samples)
{
    SpeechNormalizerContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    int n = 0;

    while (n < nb_samples) {
        int min_size = nb_samples - n;
        ftype gain = s->max_expansion;

        for (int ch = 0; ch < inlink->ch_layout.nb_channels; ch++) {
            ChannelContext *cc = &s->cc[ch];

            enum AVChannel channel = av_channel_layout_channel_from_index(&inlink->ch_layout, ch);
            cc->bypass = av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;

            next_pi(ctx, cc, cc->bypass);
            min_size = FFMIN(min_size, cc->pi_size);
        }

        av_assert1(min_size > 0);
        for (int ch = 0; ch < inlink->ch_layout.nb_channels; ch++) {
            ChannelContext *cc = &s->cc[ch];

            if (cc->bypass)
                continue;
            gain = FFMIN(gain, min_gain(ctx, cc, min_size));
        }

        for (int ch = 0; ch < inlink->ch_layout.nb_channels; ch++) {
            ChannelContext *cc = &s->cc[ch];
            const ftype *src = (const ftype *)in->extended_data[ch];
            ftype *dst = (ftype *)out->extended_data[ch];

            consume_pi(cc, min_size);
            if (cc->bypass || ff_filter_disabled(ctx)) {
                memcpy(dst + n, src + n, min_size * sizeof(*dst));
            } else {
                for (int i = n; i < n + min_size; i++) {
                    ftype g = fn(lerp)(s->prev_gain, gain, (i-n)/(ftype)min_size);
                    dst[i] = src[i] * g;
                }
            }
        }

        s->prev_gain = gain;
        n += min_size;
    }
}

static void fn(filter_channels)(AVFilterContext *ctx,
                                AVFrame *in, AVFrame *out, int nb_samples)
{
    SpeechNormalizerContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];

    for (int ch = 0; ch < inlink->ch_layout.nb_channels; ch++) {
        ChannelContext *cc = &s->cc[ch];
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        enum AVChannel channel = av_channel_layout_channel_from_index(&inlink->ch_layout, ch);
        const int bypass = av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;
        int n = 0;

        while (n < nb_samples) {
            ftype gain;
            int size;

            next_pi(ctx, cc, bypass);
            size = FFMIN(nb_samples - n, cc->pi_size);
            av_assert1(size > 0);
            gain = cc->gain_state;
            consume_pi(cc, size);
            if (ff_filter_disabled(ctx)) {
                memcpy(dst + n, src + n, size * sizeof(*dst));
            } else {
                for (int i = n; i < n + size; i++)
                    dst[i] = src[i] * gain;
            }
            n += size;
        }
    }
}
