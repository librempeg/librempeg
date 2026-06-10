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
                                const uint8_t *srcp, const int nb_samples)
{
    const ftype min_peak = F(1./32768.);
    const ftype *src = (const ftype *)srcp;
    PeriodItem *pi = &cc->pend;
    int state = cc->state;
    int n = 0;

    if (state == -2)
        state = DIFFSIGN(src[0], min_peak);

    while (n < nb_samples) {
        int new_size, split = 0;
        ftype new_max_peak;
        ftype new_rms_sum;

        split = (!state) && pi->size >= nb_samples;
        if (state != DIFFSIGN(src[n], min_peak) || split) {
            ftype max_peak = pi->max_peak;
            ftype rms_sum = pi->rms_sum;
            int old_state = state;

            state = DIFFSIGN(src[n], min_peak);
            av_assert1(pi->size > 0);
            if (max_peak >= min_peak || split) {
                pi->type = 1;
                cc->acc += pi->size;
                if (av_fifo_can_write(cc->pf) == 0)
                    av_fifo_grow2(cc->pf, 1);
                if (av_fifo_write(cc->pf, pi, 1) < 0)
                    break;

                if (state != old_state) {
                    pi->max_peak = DBL_MIN;
                    pi->rms_sum = F(0.0);
                } else {
                    pi->max_peak = max_peak;
                    pi->rms_sum = rms_sum;
                }
                pi->type = 0;
                pi->size = 0;
            }
        }

        new_max_peak = pi->max_peak;
        new_rms_sum = pi->rms_sum;
        new_size = pi->size;
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

        pi->max_peak = new_max_peak;
        pi->rms_sum = new_rms_sum;
        pi->size = new_size;
    }
    cc->state = state;
}

static ftype fn(lerp)(ftype min, ftype max, ftype mix)
{
    return min + (max - min) * mix;
}

static void fn(filter_link_channels)(AVFilterContext *ctx,
                                     AVFrame *in, AVFrame *out,
                                     const int nb_samples)
{
    const int is_disabled = ff_filter_disabled(ctx);
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

        if (min_size <= 0)
            break;

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
            if (cc->bypass || is_disabled) {
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
                                AVFrame *in, AVFrame *out, const int nb_samples)
{
    const int is_disabled = ff_filter_disabled(ctx);
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
            if (!size)
                break;

            av_assert1(size > 0);
            gain = cc->gain_state;
            consume_pi(cc, size);
            if (is_disabled) {
                memcpy(dst + n, src + n, size * sizeof(*dst));
            } else {
                for (int i = n; i < n + size; i++)
                    dst[i] = src[i] * gain;
            }
            n += size;
        }
    }
}
