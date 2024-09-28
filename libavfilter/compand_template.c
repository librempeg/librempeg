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
} fn(ChanParam);

typedef struct fn(CompandSegment) {
    ftype x, y;
    ftype a, b;
} fn(CompandSegment);

static int fn(prepare)(AVFilterContext *ctx, AVFilterLink *outlink)
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

    s->channels = av_calloc(nb_channels, sizeof(fn(ChanParam)));
    s->segments = av_calloc(s->nb_segments, sizeof(fn(CompandSegment)));
    if (!s->channels || !s->segments)
        return AVERROR(ENOMEM);
    channels = s->channels;
    segments = s->segments;

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

    for (int i = 0; i < nb_channels; i++) {
        fn(ChanParam) *cp = &channels[i];

        if (cp->attack > F(1.0) / sample_rate)
            cp->attack = F(M_LN10) / (sample_rate * cp->attack);
        else
            cp->attack = F(1.0);
        if (cp->decay > F(1.0) / sample_rate)
            cp->decay = F(M_LN10) / (sample_rate * cp->decay);
        else
            cp->decay = F(1.0);
        cp->volume = s->initial_volume * F(M_LN10/20.0);
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
    out_log = cs->y + in_log * (cs->a * in_log + cs->b);

    return FEXP(out_log);
}

static void fn(drain)(AVFilterContext *ctx, AVFrame *frame)
{
    AVFilterLink *outlink = ctx->outputs[0];
    const int channels = outlink->ch_layout.nb_channels;
    CompandContext *s = ctx->priv;
    fn(ChanParam) *cps = s->channels;
    int dindex = 0;

    for (int chan = 0; chan < channels; chan++) {
        AVFrame *delay_frame = s->delay_frame;
        ftype *dbuf = (ftype *)delay_frame->extended_data[chan];
        ftype *dst = (ftype *)frame->extended_data[chan];
        fn(ChanParam) *cp = &cps[chan];

        dindex = s->delay_index;
        for (int i = 0; i < frame->nb_samples; i++) {
            dst[i] = dbuf[dindex] * fn(get_volume)(s, cp->volume);
#define MOD(a, b) (((a) >= (b)) ? (a) - (b) : (a))
            dindex = MOD(dindex + 1, s->delay_samples);
        }
    }

    s->delay_count -= frame->nb_samples;
    s->delay_index = dindex;
}

static void fn(update_volume)(fn(ChanParam) *cp, const ftype in)
{
    const ftype in_log = in > EPS ? FLOG(in) : FLOG(EPS);
    const ftype delta = in_log - cp->volume;

    if (delta > F(0.0))
        cp->volume += delta * cp->attack;
    else if (delta < F(0.0))
        cp->volume += delta * cp->decay;
}

static int fn(compand_nodelay)(AVFilterContext *ctx)
{
    CompandContext *s    = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    const int channels   = inlink->ch_layout.nb_channels;
    const int nb_samples = s->in->nb_samples;
    const int is_disabled = ctx->is_disabled;
    AVFrame *sc = s->sc ? s->sc : s->in;
    fn(ChanParam) *cps = s->channels;
    AVFrame *out;
    int err;

    if (av_frame_is_writable(s->in)) {
        out = s->in;
    } else {
        out = ff_get_audio_buffer(ctx->outputs[0], nb_samples);
        if (!out) {
            av_frame_free(&s->in);
            av_frame_free(&s->sc);
            return AVERROR(ENOMEM);
        }
        err = av_frame_copy_props(out, s->in);
        if (err < 0) {
            av_frame_free(&out);
            av_frame_free(&s->in);
            av_frame_free(&s->sc);
            return err;
        }
    }

    for (int chan = 0; chan < channels; chan++) {
        const ftype *scsrc = (const ftype *)sc->extended_data[chan];
        const ftype *src = (const ftype *)s->in->extended_data[chan];
        ftype *dst = (ftype *)out->extended_data[chan];
        fn(ChanParam) *cp = &cps[chan];

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

    if (s->in != out)
        av_frame_free(&s->in);

    s->in = NULL;
    av_frame_free(&s->sc);
    return ff_filter_frame(ctx->outputs[0], out);
}

static int fn(compand_delay)(AVFilterContext *ctx)
{
    CompandContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    const int channels = inlink->ch_layout.nb_channels;
    const int nb_samples = s->in->nb_samples;
    const int is_enabled = !ctx->is_disabled;
    AVFrame *sc = s->sc ? s->sc : s->in;
    fn(ChanParam) *cps = s->channels;
    int dindex = 0, count = 0;
    AVFrame *out = NULL;
    int err;

    for (int chan = 0; chan < channels; chan++) {
        AVFrame *delay_frame = s->delay_frame;
        const ftype *scsrc = (const ftype *)sc->extended_data[chan];
        const ftype *src = (const ftype *)s->in->extended_data[chan];
        ftype *dbuf = (ftype *)delay_frame->extended_data[chan];
        fn(ChanParam) *cp = &cps[chan];
        ftype *dst = out ? (ftype *)out->extended_data[chan] : NULL;

        count  = s->delay_count;
        dindex = s->delay_index;
        for (int i = 0, oindex = 0; i < nb_samples; i++) {
            const ftype scsample = scsrc[i];
            const ftype sample = src[i];

            fn(update_volume)(cp, FABS(scsample));

            if (count >= s->delay_samples) {
                if (!out) {
                    out = ff_get_audio_buffer(ctx->outputs[0], nb_samples - i);
                    if (!out) {
                        av_frame_free(&s->in);
                        av_frame_free(&s->sc);
                        return AVERROR(ENOMEM);
                    }
                    err = av_frame_copy_props(out, s->in);
                    if (err < 0) {
                        av_frame_free(&out);
                        av_frame_free(&s->in);
                        av_frame_free(&s->sc);
                        return err;
                    }
                    s->pts = out->pts + out->nb_samples;
                    out->pts -= s->delay_samples - i;
                    dst = (ftype *)out->extended_data[chan];
                }

                dst[oindex] = dbuf[dindex];
                if (is_enabled)
                    dst[oindex] *= fn(get_volume)(s, cp->volume);
                oindex++;
            } else {
                count++;
            }

            dbuf[dindex] = sample;
            dindex = MOD(dindex + 1, s->delay_samples);
        }
    }

    s->delay_count = count;
    s->delay_index = dindex;

    av_frame_free(&s->in);
    av_frame_free(&s->sc);

    if (out)
        return ff_filter_frame(ctx->outputs[0], out);

    ff_inlink_request_frame(inlink);

    return 0;
}
