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
#undef FMA
#undef SQRT
#undef FTAN
#undef FMAX
#undef FMIN
#undef CLIP
#undef SAMPLE_FORMAT
#undef SAMPLE_SUFFIX
#undef FABS
#undef FLOG
#undef FEXP
#undef FLOG2
#undef FEXP2
#undef FEXP10
#undef EPSILON
#if DEPTH == 32
#define SAMPLE_FORMAT float
#define SAMPLE_SUFFIX f
#define FMA fmaf
#define SQRT sqrtf
#define FTAN tanf
#define FMIN fminf
#define FMAX fmaxf
#define CLIP av_clipf
#define FABS fabsf
#define FLOG logf
#define FEXP expf
#define FLOG2 log2f
#define FEXP2 exp2f
#define FEXP10 ff_exp10f
#define EPSILON (1.f / (1 << 23))
#define ftype float
#else
#define SAMPLE_FORMAT double
#define SAMPLE_SUFFIX
#define FMA fma
#define SQRT sqrt
#define FTAN tan
#define FMIN fmin
#define FMAX fmax
#define CLIP av_clipd
#define FABS fabs
#define FLOG log
#define FEXP exp
#define FLOG2 log2
#define FEXP2 exp2
#define FEXP10 ff_exp10
#define EPSILON (1.0 / (1LL << 53))
#define ftype double
#endif

#define Fn3(a,b)   a##b
#define Fn2(a,b)   Fn3(a,b)
#define F(a)       Fn2(a, SAMPLE_SUFFIX)

#define LIN2LOG(x) (F(6.0206) * FLOG2((x) + EPSILON))
#define LOG2LIN(x) (FEXP2((x) / F(6.0206)))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(RMSContext) {
    ftype history[10];
    ftype sum;
    int history_idx;
} fn(RMSContext);

typedef struct fn(ChannelContext) {
    ftype fa[3], fm[3];
    ftype astate[2];
    ftype lstate[2];
    ftype dstate[2];
    ftype fstate[2];
    ftype lin_gain;
    ftype detect;
    ftype threshold;
    ftype new_threshold;
    ftype threshold_log;

    fn(RMSContext) drms;
    fn(RMSContext) brms;

    int size;
    int detection;
} fn(ChannelContext);

typedef ftype (*fn(filter_svf))(ftype in, const ftype m[3], const ftype a[3], ftype b[2]);

typedef struct fn(BandContext) {
    ftype threshold;

    ftype aa[3], am[3];
    ftype da[3], dm[3];

    ftype tattack_coef;
    ftype trelease_coef;
    ftype threshold_log;

    ftype scale_tqfactor;

    fn(ChannelContext) *cc;

    fn(filter_svf) target_fn;
    fn(filter_svf) detect_fn;
    fn(filter_svf) lowpass_fn;

    void (*target_update)(ftype iQ, ftype A, ftype g, ftype a[3], ftype m[3]);
    ftype (*target_gain)(const ftype detect, const ftype threshold, const ftype threshold_log,
                         const ftype makeup, const ftype ratio, const ftype range,
                         const ftype power, const int direction);
} fn(BandContext);

static int fn(init_state)(AVFilterContext *ctx, const int nb_channels,
                          const int nb_bands)
{
    AudioDynamicEqualizerContext *s = ctx->priv;
    fn(ChannelContext) *stc;
    fn(BandContext) *bct;

    s->bc = av_calloc(nb_bands, sizeof(*bct));
    if (!s->bc)
        return AVERROR(ENOMEM);
    bct = s->bc;
    s->nb_bands = nb_bands;

    for (int n = 0; n < nb_bands; n++) {
        fn(BandContext) *b = &bct[n];
        fn(ChannelContext) *cs;

        b->cc = av_calloc(nb_channels, sizeof(*stc));
        if (!b->cc)
            return AVERROR(ENOMEM);

        cs = b->cc;
        for (int ch = 0; ch < nb_channels; ch++) {
            fn(ChannelContext) *cc = &cs[ch];
            cc->lin_gain = F(1.0);
            cc->new_threshold = EPSILON;
            cc->threshold = cc->new_threshold;
            cc->threshold_log = LIN2LOG(cc->threshold);
        }
    }

    return 0;
}

static void fn(uninit_state)(AVFilterContext *ctx)
{
    AudioDynamicEqualizerContext *s = ctx->priv;
    fn(BandContext) *bc = s->bc;

    for (int n = 0; n < s->nb_bands && bc; n++) {
        fn(BandContext) *b = &bc[n];

        av_freep(&b->cc);
    }
    av_freep(&bc);
    s->bc = NULL;
}

static void fn(update_state)(ftype *b)
{
    b[0] = isnormal(b[0]) ? b[0] : F(0.0);
    b[1] = isnormal(b[1]) ? b[1] : F(0.0);
}

static ftype fn(get_svf_bell)(ftype in, const ftype m[3],
                              const ftype a[3], ftype b[2])
{
    const ftype v0 = in;
    const ftype v3 = v0 - b[1];
    const ftype v1 = a[0] * b[0] + a[1] * v3;
    const ftype v2 = b[1] + a[1] * b[0] + a[2] * v3;

    b[0] = F(2.0) * v1 - b[0];
    b[1] = F(2.0) * v2 - b[1];

    return m[0] * v0 + m[1] * v1;
}

static ftype fn(get_svf_low)(ftype in, const ftype m[3],
                             const ftype a[3], ftype b[2])
{
    const ftype v0 = in;
    const ftype v3 = v0 - b[1];
    const ftype v1 = a[0] * b[0] + a[1] * v3;
    const ftype v2 = b[1] + a[1] * b[0] + a[2] * v3;

    b[0] = F(2.0) * v1 - b[0];
    b[1] = F(2.0) * v2 - b[1];

    return m[2] * v2;
}

static ftype fn(get_svf_band)(ftype in, const ftype m[3],
                              const ftype a[3], ftype b[2])
{
    const ftype v0 = in;
    const ftype v3 = v0 - b[1];
    const ftype v1 = a[0] * b[0] + a[1] * v3;
    const ftype v2 = b[1] + a[1] * b[0] + a[2] * v3;

    b[0] = F(2.0) * v1 - b[0];
    b[1] = F(2.0) * v2 - b[1];

    return m[1] * v1;
}

static ftype fn(get_svf)(ftype in, const ftype m[3],
                         const ftype a[3], ftype b[2])
{
    const ftype v0 = in;
    const ftype v3 = v0 - b[1];
    const ftype v1 = a[0] * b[0] + a[1] * v3;
    const ftype v2 = b[1] + a[1] * b[0] + a[2] * v3;

    b[0] = F(2.0) * v1 - b[0];
    b[1] = F(2.0) * v2 - b[1];

    return m[0] * v0 + m[1] * v1 + m[2] * v2;
}

static ftype fn(get_coef)(ftype x, ftype sr)
{
    return F(1.0) - FEXP(F(-1.0) / (F(0.001) * x * sr));
}

static void fn(update_bell)(ftype iQ, ftype A, ftype g,
                            ftype a[3], ftype m[3])
{
    ftype k = iQ / A;

    a[0] = F(1.0) / (F(1.0) + g * (g + k));
    a[1] = g * a[0];
    a[2] = g * a[1];

    m[0] = F(1.0);
    m[1] = k * (A * A - F(1.0));
    m[2] = F(0.0);
}

static void fn(update_low)(ftype iQ, ftype A, ftype g,
                           ftype a[3], ftype m[3])
{
    ftype k = iQ;

    g = g / SQRT(A);

    a[0] = F(1.0) / (F(1.0) + g * (g + k));
    a[1] = g * a[0];
    a[2] = g * a[1];

    m[0] = F(1.0);
    m[1] = k * (A - F(1.0));
    m[2] = A * A - F(1.0);
}

static void fn(update_high)(ftype iQ, ftype A, ftype g,
                            ftype a[3], ftype m[3])
{
    ftype k = iQ;
    g = g * SQRT(A);

    a[0] = F(1.0) / (F(1.0) + g * (g + k));
    a[1] = g * a[0];
    a[2] = g * a[1];

    m[0] = A * A;
    m[1] = k * (F(1.0) - A) * A;
    m[2] = F(1.0) - A * A;
}

static ftype fn(target_gain)(const ftype detect, const ftype threshold,
                             const ftype threshold_log,
                             const ftype makeup, const ftype ratio,
                             const ftype range,  const ftype power,
                             const int direction)
{
    switch (direction) {
    case BELOW:
        if (detect < threshold) {
            ftype ld = LIN2LOG(detect);
            ftype new_log_gain = CLIP(makeup + (threshold_log - ld) * ratio, F(0.0), range) * power;
            return LOG2LIN(new_log_gain);
        }
        break;
    case ABOVE:
        if (detect > threshold) {
            ftype ld = LIN2LOG(detect);
            ftype new_log_gain = CLIP(makeup + (ld - threshold_log) * ratio, F(0.0), range) * power;
            return LOG2LIN(new_log_gain);
        }
        break;
    }

    return F(1.0);
}

static ftype fn(target_gain_noratio)(const ftype detect, const ftype threshold,
                                     const ftype unused2,
                                     const ftype makeup, const ftype unused3,
                                     const ftype range,  const ftype power,
                                     const int direction)
{
    switch (direction) {
    case BELOW:
        if (detect < threshold) {
            ftype new_log_gain = FMIN(makeup, range) * power;
            return LOG2LIN(new_log_gain);
        }
        break;
    case ABOVE:
        if (detect > threshold) {
            ftype new_log_gain = FMIN(makeup, range) * power;
            return LOG2LIN(new_log_gain);
        }
        break;
    }

    return F(1.0);
}

static int fn(filter_prepare)(AVFilterContext *ctx)
{
    AudioDynamicEqualizerContext *s = ctx->priv;
    const ftype sample_rate = ctx->inputs[0]->sample_rate;
    fn(BandContext) *bc = s->bc;

    for (int band = 0; band < s->nb_bands; band++) {
        fn(BandContext) *b = &bc[band];
        const ftype dfrequency = FMIN(s->dfrequency[FFMIN(band, s->nb_dfrequency-1)], sample_rate * F(0.5));
        const ftype ratio = s->ratio[FFMIN(band, s->nb_ratio-1)];
        const ftype dg = FTAN(F(M_PI) * dfrequency / sample_rate);
        const int tftype = s->tftype[FFMIN(band, s->nb_tftype-1)];
        const int dftype = s->dftype[FFMIN(band, s->nb_dftype-1)];
        const ftype tfrequency = FMIN(s->tfrequency[FFMIN(band, s->nb_tfrequency-1)], sample_rate * F(0.5));
        const ftype tqfactor = s->tqfactor[FFMIN(band, s->nb_tqfactor-1)];
        const ftype fg = FTAN(F(M_PI) * tfrequency / sample_rate);
        const ftype itqfactor = F(1.0) / tqfactor;
        fn(ChannelContext) *cs = b->cc;
        ftype scale_dqfactor;
        ftype scale_tqfactor;
        ftype *aa = b->aa;
        ftype *am = b->am;
        ftype *da = b->da;
        ftype *dm = b->dm;
        ftype k, dqfactor;

        b->threshold = s->threshold[FFMIN(band, s->nb_threshold-1)];
        b->threshold_log = LIN2LOG(b->threshold);
        b->tattack_coef = fn(get_coef)(s->tattack[FFMIN(band, s->nb_tattack-1)], sample_rate);
        b->trelease_coef = fn(get_coef)(s->trelease[FFMIN(band, s->nb_trelease-1)], sample_rate);

        switch (dftype) {
        case DBANDPASS:
            scale_dqfactor = F(1.0)/(dg+F(1.0));
            dqfactor = s->dqfactor[FFMIN(band, s->nb_dqfactor-1)] * scale_dqfactor;
            k = F(1.0) / dqfactor;
            da[0] = F(1.0) / (F(1.0) + dg * (dg + k));
            da[1] = dg * da[0];
            da[2] = dg * da[1];

            dm[0] = F(0.0);
            dm[1] = F(1.0);
            dm[2] = F(0.0);

            b->detect_fn = fn(get_svf_band);
            break;
        case DLOWPASS:
            dqfactor = s->dqfactor[FFMIN(band, s->nb_dqfactor-1)];
            k = F(1.0) / dqfactor;
            da[0] = F(1.0) / (F(1.0) + dg * (dg + k));
            da[1] = dg * da[0];
            da[2] = dg * da[1];

            dm[0] = F(0.0);
            dm[1] = F(0.0);
            dm[2] = F(1.0);

            b->detect_fn = fn(get_svf_low);
            break;
        case DHIGHPASS:
            dqfactor = s->dqfactor[FFMIN(band, s->nb_dqfactor-1)];
            k = F(1.0) / dqfactor;
            da[0] = F(1.0) / (F(1.0) + dg * (dg + k));
            da[1] = dg * da[0];
            da[2] = dg * da[1];

            dm[0] = F(0.0);
            dm[1] = -k;
            dm[2] = -F(1.0);

            b->detect_fn = fn(get_svf);
            break;
        case DPEAK:
            scale_dqfactor = F(1.0)/(dg+F(1.0));
            dqfactor = s->dqfactor[FFMIN(band, s->nb_dqfactor-1)] * scale_dqfactor;
            k = F(1.0) / dqfactor;
            da[0] = F(1.0) / (F(1.0) + dg * (dg + k));
            da[1] = dg * da[0];
            da[2] = dg * da[1];

            dm[0] = F(1.0);
            dm[1] = -k;
            dm[2] = F(-2.0);

            b->detect_fn = fn(get_svf);
            break;
        }

        switch (tftype) {
        case TBELL:
            scale_tqfactor = fg + F(1.0);
            b->target_update = fn(update_bell);
            b->target_fn = fn(get_svf_bell);
            break;
        case TLOWSHELF:
            scale_tqfactor = F(1.0);
            b->target_update = fn(update_low);
            b->target_fn = fn(get_svf);
            break;
        case THIGHSHELF:
            scale_tqfactor = F(1.0);
            b->target_update = fn(update_high);
            b->target_fn = fn(get_svf);
            break;
        }

        {
            const ftype ag = FTAN(F(M_PI) * F(1.0) / sample_rate);
            const ftype aqfactor = F(M_SQRT1_2);

            ftype ka = F(1.0) / aqfactor;

            aa[0] = F(1.0) / (F(1.0) + ag * (ag + ka));
            aa[1] = ag * aa[0];
            aa[2] = ag * aa[1];

            am[0] = F(0.0);
            am[1] = F(0.0);
            am[2] = F(1.0);
        }

        b->scale_tqfactor = scale_tqfactor;
        b->lowpass_fn = fn(get_svf_low);

        if (ratio > F(0.0))
            b->target_gain = fn(target_gain);
        else
            b->target_gain = fn(target_gain_noratio);

        for (int ch = 0; ch < s->nb_channels; ch++) {
            fn(ChannelContext) *cc = &cs[ch];

            b->target_update(scale_tqfactor * itqfactor, cc->lin_gain, fg, cc->fa, cc->fm);
        }
    }

    return 0;
}

static ftype fn(get_rms)(fn(RMSContext) *rms, const ftype in)
{
    const int history_size = FF_ARRAY_ELEMS(rms->history);
    int idx = rms->history_idx;
    const ftype out = rms->history[idx];
    ftype ret;

    rms->sum += in*in;
    ret = SQRT(rms->sum/history_size);
    rms->sum -= out*out;
    rms->sum  = FMAX(F(0.0), rms->sum);

    rms->history[idx] = in;
    idx++;
    if (idx >= history_size)
        idx = 0;
    rms->history_idx = idx;

    return ret;
}

static ftype fn(get_detect)(const int dttype, const ftype band, const ftype broad,
                            fn(RMSContext) *drms, fn(RMSContext) *brms)
{
    ftype ret, detect, ref;

    switch (dttype) {
    case DTDISABLED:
        ret = FABS(band);
        break;
    case DTABSOLUTE:
        ret = fn(get_rms)(drms, band);
        break;
    case DTRELATIVE:
        detect = fn(get_rms)(drms, band);
        ref = fn(get_rms)(brms, broad);
        ret = detect / (F(0.1) + ref);
        break;
    }

    return ret;
}

static int fn(filter_channels_band)(AVFilterContext *ctx, void *arg,
                                    const int start, const int end, const int band)
{
    AudioDynamicEqualizerContext *s = ctx->priv;
    fn(BandContext) *bc = s->bc;
    fn(BandContext) *b = &bc[band];
    ThreadData *td = arg;
    AVFrame *in = td->in;
    AVFrame *sc = td->sc ? td->sc : in;
    AVFrame *out = td->out;
    const ftype sample_rate = in->sample_rate;
    const ftype makeup = s->makeup[FFMIN(band, s->nb_makeup-1)];
    const ftype ratio = s->ratio[FFMIN(band, s->nb_ratio-1)];
    const ftype range = s->range[FFMIN(band, s->nb_range-1)];
    const ftype tfrequency = FMIN(s->tfrequency[FFMIN(band, s->nb_tfrequency-1)], sample_rate * F(0.5));
    const AVChannelLayout ch_layout = s->channel[FFMIN(band, s->nb_channel-1)];
    const int direction = s->direction[FFMIN(band, s->nb_direction-1)];
    const int dttype = s->dttype[FFMIN(band, s->nb_dttype-1)];
    const int mode = s->mode[FFMIN(band, s->nb_mode-1)];
    const ftype power = (mode == CUT) ? F(-1.0) : F(1.0);
    const ftype band_threshold_log = b->threshold_log;
    const ftype band_threshold = b->threshold;
    const ftype trelease = b->trelease_coef;
    const ftype tattack = b->tattack_coef;
    const ftype tqfactor = s->tqfactor[FFMIN(band, s->nb_tqfactor-1)];
    const ftype itqfactor = b->scale_tqfactor / tqfactor;
    const ftype fg = FTAN(F(M_PI) * tfrequency / sample_rate);
    const int is_disabled = ff_filter_disabled(ctx);
    const int nb_samples = out->nb_samples;
    const int band_detection = s->detection[FFMIN(band, s->nb_detection-1)];
    fn(ChannelContext) *cs = b->cc;
    fn(filter_svf) lowpass_fn = b->lowpass_fn;
    fn(filter_svf) detect_fn = b->detect_fn;
    fn(filter_svf) target_fn = b->target_fn;
    const ftype *aa = b->aa;
    const ftype *am = b->am;
    const ftype *da = b->da;
    const ftype *dm = b->dm;

    for (int ch = start; ch < end; ch++) {
        enum AVChannel channel = av_channel_layout_channel_from_index(&out->ch_layout, ch);
        const int bypass = av_channel_layout_index_from_channel(&ch_layout, channel) < 0;
        ftype *dst = (ftype *)out->extended_data[ch];
        const ftype *scsrc = band ? dst : (const ftype *)sc->extended_data[ch];
        const ftype *src = band ? dst : (const ftype *)in->extended_data[ch];
        fn(ChannelContext) *cc = &cs[ch];
        ftype threshold_log = cc->threshold_log;
        ftype new_threshold = cc->new_threshold;
        ftype *fa = cc->fa, *fm = cc->fm;
        fn(RMSContext) *drms = &cc->drms;
        fn(RMSContext) *brms = &cc->brms;
        ftype threshold = cc->threshold;
        ftype lin_gain = cc->lin_gain;
        int detection = cc->detection;
        ftype *astate = cc->astate;
        ftype *dstate = cc->dstate;
        ftype *fstate = cc->fstate;
        ftype *lstate = cc->lstate;
        ftype detect = cc->detect;
        ftype score = F(-1.0);
        ftype peak = F(0.0);
        int size = cc->size;

        for (int n = 0; n < nb_samples; n++) {
            ftype new_lin_gain = F(1.0);
            ftype v, listen;

            listen = detect_fn(scsrc[n], dm, da, dstate);
            detect = fn(get_detect)(dttype, listen, scsrc[n], drms, brms);

            if (band_detection == DET_ON) {
                new_threshold = FMAX(new_threshold, detect);
                detection = band_detection;
            } else if (band_detection == DET_ADAPTIVE) {
                ftype avg = lowpass_fn(detect, am, aa, astate);
                ftype log_avg = lowpass_fn(FLOG2(detect + EPSILON), am, aa, lstate);

                if (avg > EPSILON) {
                    ftype new_score = LIN2LOG(FEXP2(log_avg) / avg);
                    if (new_score >= EPSILON) {
                        score = new_score;
                        peak = FMAX(peak, FMAX(FEXP2(log_avg), avg));
                    }
                }

                size = FFMIN(size + 1, sample_rate);

                fn(update_state)(astate);
                fn(update_state)(lstate);

                if (score >= F(0.0) && size >= sample_rate) {
                    threshold     = peak * F(10.0);
                    threshold_log = LIN2LOG(threshold);
                    av_log(ctx, AV_LOG_DEBUG, "[%d]: %g %g|%g\n", band, score, threshold, threshold_log);
                } else if (detection == DET_UNSET) {
                    threshold     = band_threshold;
                    threshold_log = band_threshold_log;
                    new_threshold = EPSILON;
                }
                detection = band_detection;
            } else if (band_detection == DET_DISABLED) {
                if (detection != band_detection) {
                    threshold     = band_threshold;
                    threshold_log = band_threshold_log;
                    new_threshold = EPSILON;
                    detection     = band_detection;
                }
            } else if (band_detection == DET_OFF) {
                if (detection == DET_ON) {
                    threshold     = new_threshold;
                    threshold_log = LIN2LOG(new_threshold);
                    av_log(ctx, AV_LOG_DEBUG, "[%d]: %g|%g\n", band, threshold, threshold_log);
                } else if (detection == DET_UNSET) {
                    threshold     = band_threshold;
                    threshold_log = band_threshold_log;
                    new_threshold = EPSILON;
                }
                detection = band_detection;
            }

            new_lin_gain = b->target_gain(detect, threshold, threshold_log,
                                          makeup, ratio, range, power, direction);

            if (FABS(lin_gain - new_lin_gain) > EPSILON) {
                ftype f = (new_lin_gain > lin_gain) * tattack + (new_lin_gain < lin_gain) * trelease;
                new_lin_gain = FMA(new_lin_gain - lin_gain, f, lin_gain);

                if (FABS(lin_gain - new_lin_gain) > EPSILON) {
                    lin_gain = new_lin_gain;

                    b->target_update(itqfactor, lin_gain, fg, fa, fm);
                }
            }

            v = target_fn(src[n], fm, fa, fstate);
            v = mode == LISTEN ? (band ? dst[n]+listen : listen) : v;
            dst[n] = (is_disabled|bypass) ? src[n] : v;
        }

        fn(update_state)(dstate);
        fn(update_state)(fstate);

        cc->size = size;
        cc->detect = detect;
        cc->lin_gain = lin_gain;
        cc->detection = detection;
        cc->threshold = threshold;
        cc->threshold_log = threshold_log;
        cc->new_threshold = new_threshold;
    }

    return 0;
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioDynamicEqualizerContext *s = ctx->priv;
    const int nb_channels = s->nb_channels;
    const int start = (nb_channels * jobnr) / nb_jobs;
    const int end = (nb_channels * (jobnr+1)) / nb_jobs;
    const int max_band_idx = s->nb_active-1;
    const int nb_bands = s->nb_bands;
    const int *active = s->active;

    for (int band = 0; band < nb_bands; band++) {
        const int bidx = FFMIN(band, max_band_idx);

        if (!active[bidx])
            continue;

        fn(filter_channels_band)(ctx, arg, start, end, band);
    }

    return 0;
}
