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
#undef FPOW
#undef FABS
#undef FMIN
#undef FMAX
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define SAMPLE_FORMAT fltp
#define FPOW powf
#define FABS fabsf
#define FMIN fminf
#define FMAX fmaxf
#define ftype float
#else
#define SAMPLE_FORMAT dblp
#define FPOW pow
#define FABS fabs
#define FMIN fmin
#define FMAX fmax
#define ftype double
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define F(x) ((ftype)(x))

typedef struct fn(StateContext) {
    ftype *data;

    int N;
    int scan_start_low;
    int scan_start_mid;
    int scan_start_high;
    int write_pos, scan_pos, scan_stop, scan_start;
    ftype in_max, scan_max;

    ftype attack, release, hold, hold_count, current, beta;
} fn(StateContext);

static void fn(envelope_uninit)(AVFilterContext *ctx)
{
    AudioEnvelopeContext *s = ctx->priv;
    fn(StateContext) *st = s->st;

    for (int ch = 0; ch < s->nb_channels && st; ch++) {
        fn(StateContext) *stc = &st[ch];

        av_freep(&stc->data);
    }

    av_freep(&s->st);
}

static int fn(envelope_init)(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AudioEnvelopeContext *s = ctx->priv;
    const int nb_channels = outlink->ch_layout.nb_channels;
    const int sample_rate = outlink->sample_rate;
    fn(StateContext) *st;
    int look;

    look = FFMAX(lrint(s->look * 2.0 * sample_rate), 1);
    s->trim_size = s->flush_size = s->hlook = look / 2;
    s->nb_channels = nb_channels;

    if (!s->st)
        s->st = av_calloc(nb_channels, sizeof(*st));
    if (!s->st)
        return AVERROR(ENOMEM);

    st = s->st;
    for (int ch = 0; ch < nb_channels; ch++) {
        fn(StateContext) *stc = &st[ch];
        const ftype attack = s->attack[FFMIN(ch, s->nb_attack-1)];
        const ftype release = s->release[FFMIN(ch, s->nb_release-1)];
        const ftype hold = s->hold[FFMIN(ch, s->nb_hold-1)];

        if (attack > F(1.0) / sample_rate)
            stc->attack = F(1.0) / (attack * sample_rate);
        else
            stc->attack = F(1.0);

        if (release > F(1.0) / sample_rate)
            stc->release = F(1.0) / (release * sample_rate);
        else
            stc->release = F(1.0);

        if (hold > F(1.0) / sample_rate)
            stc->hold = F(1.0) / (hold * sample_rate);
        else
            stc->hold = F(1.0);

        stc->beta = F(1.0) - FPOW(F(1.0) - stc->attack, s->hlook+1);
        stc->hold_count = F(0.0);
        stc->current = NAN;
        if (!stc->data) {
            stc->N = look;
            stc->data = av_calloc(stc->N, sizeof(*stc->data));
            stc->scan_start_low = (stc->N-1)/2;
            stc->scan_start_mid = (stc->N+1)/2;
            stc->scan_start_high = stc->N-1;
            stc->scan_start = stc->scan_start_low;
            stc->write_pos = 0;
            stc->scan_stop = 0;
            stc->scan_pos = 0;

            if (stc->data) {
                for (int n = 0; n < look; n++)
                    stc->data[n] = F(-1.0);
            }
        }
        if (!stc->data)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static ftype fn(compute_peak)(fn(StateContext) *stc, const ftype x)
{
    int write_pos = stc->write_pos;
    int scan_pos = stc->scan_pos;
    ftype in_max = stc->in_max;
    ftype *data = stc->data;
    ftype p;

    scan_pos--;
    if (scan_pos >= stc->scan_stop) {
        in_max = FMAX(x, in_max);
        data[scan_pos] = FMAX(data[scan_pos], data[scan_pos+1]);
    } else {
        stc->scan_max = in_max;
        in_max = x;
        if (stc->scan_stop == 0) {
            stc->scan_start = stc->scan_start_high;
            stc->scan_stop = stc->scan_start_mid;
        } else {
            stc->scan_start = stc->scan_start_low;
            stc->scan_stop = 0;
        }
        scan_pos = stc->scan_start;
    }

    data[write_pos] = x;
    write_pos++;
    if (write_pos >= stc->N)
        write_pos = 0;

    p = FMAX(in_max, FMAX(stc->scan_max, data[write_pos]));
    stc->write_pos = write_pos;
    stc->scan_pos = scan_pos;
    stc->in_max = in_max;

    return p;
}

static int fn(do_envelope)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch)
{
    AudioEnvelopeContext *s = ctx->priv;
    const ftype *src = (const ftype *)in->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    const int disabled = ff_filter_disabled(ctx);
    const int nb_samples = in->nb_samples;
    fn(StateContext) *st = s->st;
    fn(StateContext) *stc = &st[ch];
    const ftype release = stc->release;
    ftype hold_count = stc->hold_count;
    const ftype attack = stc->attack;
    ftype current = stc->current;
    const ftype beta = stc->beta;
    const ftype hold = stc->hold;

    if (isnan(current))
        current = FABS(src[0]);

    for (int n = 0; n < nb_samples; n++) {
        const ftype r = FABS(src[n]);
        ftype p;

        p = fn(compute_peak)(stc, r);

        if (p > current) {
            current += (p/beta-current) * attack;
            hold_count = F(0.0);
        } else if (p < current) {
            if (hold_count >= F(1.0))
                current -= (current-p) * release;
            else
                hold_count += hold;
        }

        dst[n] = disabled ? src[n] : current;
    }

    stc->hold_count = hold_count;
    stc->current = current;

    return 0;
}

static int fn(do_envelope_link)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch)
{
    AudioEnvelopeContext *s = ctx->priv;
    const int nb_channels = s->nb_channels;
    const uint8_t **srce = (const uint8_t **)in->extended_data;
    const ftype *srci = (const ftype *)in->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    const int disabled = ff_filter_disabled(ctx);
    const int nb_samples = in->nb_samples;
    fn(StateContext) *st = s->st;
    fn(StateContext) *stc = &st[ch];
    const ftype release = stc->release;
    ftype hold_count = stc->hold_count;
    const ftype attack = stc->attack;
    ftype current = stc->current;
    const ftype beta = stc->beta;
    const ftype hold = stc->hold;

    for (int n = 0; n < nb_samples; n++) {
        ftype r = F(0.0), p;

        for (int chi = 0; chi < nb_channels; chi++) {
            const ftype *src = (const ftype *)srce[chi];
            const ftype cr = FABS(src[n]);

            r = FMAX(cr, r);
        }

        if (isnan(current))
            current = r;

        p = fn(compute_peak)(stc, r);

        if (p > current) {
            current += (p/beta-current) * attack;
            hold_count = F(0.0);
        } else if (p < current) {
            if (hold_count >= F(1.0))
                current -= (current-p) * release;
            else
                hold_count += hold;
        }

        dst[n] = disabled ? srci[n] : current;
    }

    stc->hold_count = hold_count;
    stc->current = current;

    return 0;
}
