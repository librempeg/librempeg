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

#undef EPS
#undef FMA
#undef FABS
#undef FEXP
#undef FLOG
#undef ATAN2
#undef HYPOT
#undef FCOS
#undef FSIN
#undef FMIN
#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define EPS FLT_EPSILON
#define FMA fmaf
#define FABS fabsf
#define FEXP expf
#define FLOG logf
#define FSIN sinf
#define FCOS cosf
#define FMIN fminf
#define HYPOT hypotf
#define ATAN2 atan2f
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define EPS DBL_EPSILON
#define FMA fma
#define FABS fabs
#define FEXP exp
#define FLOG log
#define FSIN sin
#define FCOS cos
#define FMIN fmin
#define HYPOT hypot
#define ATAN2 atan2
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(ChanParam) {
    ftype attack;
    ftype decay;
    ftype volume;

    int out_samples;
    int initial_volume;
    int delay_count;
    int delay_index;

    int front, back;
    int input_index;
} fn(ChanParam);

typedef struct fn(CompandSegment) {
    ftype x, y;
    ftype a, b;
} fn(CompandSegment);

static int fn(prepare)(AVFilterContext *ctx, AVFilterLink *outlink, const int reset)
{
    const int nb_channels = outlink->ch_layout.nb_channels;
    const int sample_rate = outlink->sample_rate;
    CompandContext *s = ctx->priv;
    const int nb_attacks = FFMIN(s->nb_attacks, nb_channels);
    const int nb_decays = FFMIN(s->nb_decays, nb_channels);
    const ftype radius = s->curve_dB * F(M_LN10/20.0);
    fn(CompandSegment) *segments;
    fn(ChanParam) *channels;
    int num;

    s->nb_segments = (FFMAX(s->nb_in_points, s->nb_out_points) + 4) * 2;

    s->channels = av_realloc_f(s->channels, nb_channels, sizeof(fn(ChanParam)));
    s->segments = av_realloc_f(s->segments, s->nb_segments, sizeof(fn(CompandSegment)));
    if (!s->channels || !s->segments)
        return AVERROR(ENOMEM);
    channels = s->channels;
    segments = s->segments;

    memset(segments, 0, s->nb_segments * sizeof(fn(CompandSegment)));

    for (int i = 0; i < nb_attacks; i++)
        channels[i].attack = s->attacks[i];

    for (int i = nb_attacks; i < nb_channels; i++)
        channels[i].attack = channels[nb_attacks-1].attack;

    for (int i = 0; i < nb_decays; i++)
        channels[i].decay = s->decays[i];

    for (int i = nb_decays; i < nb_channels; i++)
        channels[i].decay = channels[nb_decays-1].decay;

#undef S
#define S(x) segments[2*((x)+1)]
    for (int i = 0; i < FFMAX(s->nb_in_points, s->nb_out_points); i++) {
        S(i).x = s->in_points[FFMIN(i, s->nb_in_points-1)];
        S(i).y = s->out_points[FFMIN(i, s->nb_out_points-1)];

        if (i && S(i - 1).x > S(i).x) {
            av_log(ctx, AV_LOG_ERROR,
                    "Transfer function input values must be increasing.\n");
            return AVERROR(EINVAL);
        }
        S(i).y -= S(i).x;
        av_log(ctx, AV_LOG_DEBUG, "%d: x=%f y=%f\n", i, S(i).x, S(i).y);
    }
    num = FFMAX(s->nb_in_points, s->nb_out_points);

    /* Add 0,0 if necessary */
    if (num == 0 || S(num - 1).x)
        num++;

#undef S
#define S(x) segments[2 * (x)]
    /* Add a tail off segment at the start */
    S(0).x = S(1).x - F(2.0) * s->curve_dB;
    S(0).y = S(1).y;
    num++;

    /* Join adjacent colinear segments */
    for (int i = 2; i < num; i++) {
        ftype g1 = (S(i - 1).y - S(i - 2).y) * (S(i - 0).x - S(i - 1).x);
        ftype g2 = (S(i - 0).y - S(i - 1).y) * (S(i - 1).x - S(i - 2).x);

        if (FABS(g1 - g2))
            continue;
        num--;
        for (int j = --i; j < num; j++)
            S(j) = S(j + 1);
    }

    for (int i = 0; i < s->nb_segments; i += 2) {
        segments[i].y += s->gain_dB;
        segments[i].x *= F(M_LN10/20.0);
        segments[i].y *= F(M_LN10/20.0);
    }

#define line1 segments[i - 4]
#define curve segments[i - 3]
#define line2 segments[i - 2]
#define line3 segments[i - 0]
    for (int i = 4; i < s->nb_segments; i += 2) {
        ftype x, y, cx, cy, in1, in2, out1, out2, theta, len, r;

        line1.a = 0;
        line1.b = (line2.y - line1.y) / (line2.x - line1.x + EPS);

        line2.a = 0;
        line2.b = (line3.y - line2.y) / (line3.x - line2.x + EPS);

        theta = ATAN2(line2.y - line1.y, line2.x - line1.x);
        len = HYPOT(line2.x - line1.x, line2.y - line1.y);
        r = FMIN(radius, len);
        curve.x = line2.x - r * FCOS(theta);
        curve.y = line2.y - r * FSIN(theta);

        theta = ATAN2(line3.y - line2.y, line3.x - line2.x);
        len = HYPOT(line3.x - line2.x, line3.y - line2.y);
        r = FMIN(radius, len * F(0.5));
        x = line2.x + r * FCOS(theta);
        y = line2.y + r * FSIN(theta);

        cx = (curve.x + line2.x + x) / F(3.0);
        cy = (curve.y + line2.y + y) / F(3.0);

        line2.x = x;
        line2.y = y;

        in1  = cx - curve.x + EPS;
        out1 = cy - curve.y + EPS;
        in2  = line2.x - curve.x + EPS;
        out2 = line2.y - curve.y + EPS;
        curve.a = (out2 / in2 - out1 / in1) / (in2 - in1 + EPS);
        curve.b = out1 / in1 - curve.a * in1;
    }
#undef line1
#undef curve
#undef line2
#undef line3
    segments[s->nb_segments-3].x = segments[s->nb_segments-2].x;
    segments[s->nb_segments-3].y = segments[s->nb_segments-2].y;

    s->in_min_log  = segments[1].x;
    s->out_min_lin = FEXP(segments[1].y);

    for (int ch = 0; ch < nb_channels; ch++) {
        fn(ChanParam) *cp = &channels[ch];

        if (cp->attack > F(1.0) / sample_rate)
            cp->attack = F(M_LN10) / (sample_rate * cp->attack);
        else
            cp->attack = F(1.0);
        if (cp->decay > F(1.0) / sample_rate)
            cp->decay = F(M_LN10) / (sample_rate * cp->decay);
        else
            cp->decay = F(1.0);

        if (reset) {
            cp->delay_index = cp->delay_count = cp->initial_volume = 0;
            cp->front = cp->back = cp->input_index = 0;
        }

        if (s->delay_samples > 0 && reset) {
            ftype *sort = (ftype *)s->sort_frame->extended_data[ch];
            ftype *in = (ftype *)s->in_frame->extended_data[ch];

            for (int n = 0; n < s->delay_samples; n++) {
                sort[n] = F(-1.0);
                in[n] = F(-1.0);
            }
        }
    }

    return 0;
}

static ftype fn(get_volume)(const CompandContext *s, ftype in_log)
{
    const fn(CompandSegment) *css = s->segments;
    const fn(CompandSegment) *cs;
    ftype out_log;
    int i;

    if (in_log < s->in_min_log)
        return s->out_min_lin;

    for (i = 1; i < s->nb_segments; i++)
        if (in_log <= css[i].x)
            break;
    cs = &css[i - 1];
    in_log -= cs->x;
    out_log = FMA(in_log, FMA(cs->a, in_log, cs->b), cs->y);

    return FEXP(out_log);
}

static int fn(delay_count)(AVFilterContext *ctx)
{
    CompandContext *s = ctx->priv;
    fn(ChanParam) *cps = s->channels;
    fn(ChanParam) *cp = &cps[0];

    return cp->delay_count;
}

static int fn(out_samples)(AVFilterContext *ctx)
{
    CompandContext *s = ctx->priv;
    fn(ChanParam) *cps = s->channels;
    fn(ChanParam) *cp = &cps[0];

    return cp->out_samples;
}

static void fn(drain)(AVFilterContext *ctx, AVFrame *frame)
{
    AVFilterLink *outlink = ctx->outputs[0];
    const int channels = outlink->ch_layout.nb_channels;
    CompandContext *s = ctx->priv;
    const int delay_samples = s->delay_samples;
    const int nb_samples = frame->nb_samples;
    fn(ChanParam) *cps = s->channels;

    for (int ch = 0; ch < channels; ch++) {
        AVFrame *delay_frame = s->delay_frame;
        ftype *dbuf = (ftype *)delay_frame->extended_data[ch];
        ftype *dst = (ftype *)frame->extended_data[ch];
        fn(ChanParam) *cp = &cps[ch];
        int dindex = cp->delay_index;

        for (int i = 0; i < nb_samples; i++) {
            dst[i] = dbuf[dindex] * fn(get_volume)(s, cp->volume);
#define MOD(a, b) (((a) >= (b)) ? (a) - (b) : (a))
            dindex = MOD(dindex + 1, delay_samples);
        }

        cp->delay_index = dindex;
        cp->delay_count -= nb_samples;
    }
}

static void fn(update_volume)(fn(ChanParam) *cp, const ftype in)
{
    const ftype in_log = in > EPS ? FLOG(in) : FLOG(EPS);
    const ftype delta = in_log - cp->volume;

    if (delta > F(0.0))
        cp->volume = FMA(delta, cp->attack, cp->volume);
    else if (delta < F(0.0))
        cp->volume = FMA(delta, cp->decay, cp->volume);
}

static int fn(compand_nodelay_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    CompandContext *s = ctx->priv;
    AVFrame *sc = s->sc ? s->sc : s->in;
    AVFrame *in = s->in;
    AVFrame *out = arg;
    fn(ChanParam) *cps = s->channels;
    const int nb_samples = s->in->nb_samples;
    const int channels = in->ch_layout.nb_channels;
    const int is_disabled = ff_filter_disabled(ctx);
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;

    for (int ch = start; ch < end; ch++) {
        const ftype *scsrc = (const ftype *)sc->extended_data[ch];
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        fn(ChanParam) *cp = &cps[ch];

        if (!cp->initial_volume) {
            cp->volume = FLOG(FABS(scsrc[0]) + EPS);
            cp->initial_volume = 1;
        }

        if (is_disabled) {
            for (int i = 0; i < nb_samples; i++) {
                fn(update_volume)(cp, FABS(scsrc[i]));

                dst[i] = src[i];
            }
            continue;
        }
        for (int i = 0; i < nb_samples; i++) {
            fn(update_volume)(cp, FABS(scsrc[i]));

            dst[i] = src[i] * fn(get_volume)(s, cp->volume);
        }
    }

    return 0;
}

#define PEAKS(empty_value,op,sample, psample)\
    if (!empty && psample == ss[front]) {    \
        ss[front] = empty_value;             \
        if (back != front) {                 \
            front--;                         \
            if (front < 0)                   \
                front = n - 1;               \
        }                                    \
        empty = (front == back) &&           \
                (ss[front] == empty_value);  \
    }                                        \
                                             \
    while (!empty && sample op ss[front]) {  \
        ss[front] = empty_value;             \
        if (back == front) {                 \
            empty = 1;                       \
            break;                           \
        }                                    \
        front--;                             \
        if (front < 0)                       \
            front = n - 1;                   \
    }                                        \
                                             \
    while (!empty && sample op ss[back]) {   \
        ss[back] = empty_value;              \
        if (back == front) {                 \
            empty = 1;                       \
            break;                           \
        }                                    \
        back++;                              \
        if (back >= n)                       \
            back = 0;                        \
    }                                        \
                                             \
    if (!empty) {                            \
        back--;                              \
        if (back < 0)                        \
            back = n - 1;                    \
    }

static ftype fn(compute_peak)(ftype *ss, const ftype ax, const ftype px,
                              const int n, int *ffront, int *bback)
{
    const ftype empty_value = F(-1.0);
    int front = *ffront;
    int back = *bback;
    int empty = front == back && ss[front] == empty_value;
    ftype r;

    PEAKS(empty_value, >, ax, px)

    ss[back] = ax;
    r = ss[front];

    *ffront = front;
    *bback = back;

    return r;
}

static int fn(compand_delay_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    CompandContext *s = ctx->priv;
    const int channels = s->in->ch_layout.nb_channels;
    const int delay_samples = s->delay_samples;
    const int is_enabled = !ff_filter_disabled(ctx);
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = s->in->nb_samples;
    AVFrame *delay_frame = s->delay_frame;
    AVFrame *sc = s->sc ? s->sc : s->in;
    fn(ChanParam) *cps = s->channels;
    AVFrame *in = s->in;
    AVFrame *out = arg;

    for (int ch = start; ch < end; ch++) {
        const ftype *scsrc = (const ftype *)sc->extended_data[ch];
        ftype *sorted = (ftype *)s->sort_frame->extended_data[ch];
        ftype *input = (ftype *)s->in_frame->extended_data[ch];
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dbuf = (ftype *)delay_frame->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        fn(ChanParam) *cp = &cps[ch];
        int count  = cp->delay_count;
        int dindex = cp->delay_index;
        int iindex = cp->input_index;
        int front = cp->front;
        int back = cp->back;
        int oindex = 0;

        if (!cp->initial_volume) {
            cp->volume = FLOG(FABS(scsrc[0]) + EPS);
            cp->initial_volume = 1;
        }

        for (int i = 0; i < nb_samples; i++) {
            const ftype scsample = scsrc[i];
            const ftype ascsample = FABS(scsample);
            const ftype sample = src[i];
            ftype peak, prev;

            prev = input[iindex];
            input[iindex] = ascsample;

            iindex++;
            if (iindex >= delay_samples)
                iindex = 0;

            peak = fn(compute_peak)(sorted, ascsample, prev, delay_samples, &front, &back);
            fn(update_volume)(cp, peak);

            if (count >= delay_samples) {
                dst[oindex] = dbuf[dindex];
                if (is_enabled)
                    dst[oindex] *= fn(get_volume)(s, cp->volume);
                oindex++;
            } else {
                count++;
            }

            dbuf[dindex] = sample;
            dindex = MOD(dindex + 1, delay_samples);
        }

        cp->out_samples = oindex;
        cp->delay_count = count;
        cp->delay_index = dindex;
        cp->input_index = iindex;
        cp->front = front;
        cp->back = back;
    }

    return 0;
}
