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
#undef FABS
#undef SQRT
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define FABS fabsf
#define SQRT sqrtf
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define FABS fabs
#define SQRT sqrt
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(ChannelParams) {
    ftype blend_old, drive_old;
    ftype kpa, kpb, kna, knb, ap,
          an, srct, pwrq;
    ftype prev_med, prev_out;

    ftype hp[5], lp[5];
    ftype hw[4][2], lw[2][2];
} fn(ChannelParams);

static inline ftype fn(D)(ftype x)
{
    x = FABS(x);

    return (x > F(0.00000001)) ? SQRT(x) : F(0.0);
}

static void fn(set_params)(fn(ChannelParams) *p,
                           double blend, double drive,
                           double srate, double freq,
                           double ceil)
{
    double rdrive, rbdr, a0, a1, a2, b0, b1, b2, w0, alpha, imr, sq, kc;

    rdrive = 12.0 / drive;
    rbdr = rdrive / (10.5 - blend) * 780.0 / 33.0;
    p->kpa = fn(D)(2.0 * (rdrive*rdrive) - 1.0) + 1.0;
    p->kpb = (2.0 - p->kpa) / 2.0;
    p->ap = ((rdrive*rdrive) - p->kpa + 1.0) / 2.0;
    kc = p->kpa / fn(D)(2.0 * fn(D)(2.0 * (rdrive*rdrive) - 1.0) - 2.0 * rdrive*rdrive);

    p->srct = (0.1 * srate) / (0.1 * srate + 1.0);
    sq = kc*kc + 1.0;
    p->knb = -1.0 * rbdr / fn(D)(sq);
    p->kna = 2.0 * kc * rbdr / fn(D)(sq);
    p->an = rbdr*rbdr / sq;
    imr = 2.0 * p->knb + fn(D)(2.0 * p->kna + 4.0 * p->an - 1.0);
    p->pwrq = 2.0 / (imr + 1.0);

    w0 = 2 * M_PI * freq / srate;
    alpha = sin(w0) / (2. * 0.707);
    a0 =   1 + alpha;
    a1 =  -2 * cos(w0);
    a2 =   1 - alpha;
    b0 =  (1 + cos(w0)) / 2;
    b1 = -(1 + cos(w0));
    b2 =  (1 + cos(w0)) / 2;

    p->hp[0] =-a1 / a0;
    p->hp[1] =-a2 / a0;
    p->hp[2] = b0 / a0;
    p->hp[3] = b1 / a0;
    p->hp[4] = b2 / a0;

    w0 = 2 * M_PI * ceil / srate;
    alpha = sin(w0) / (2. * 0.707);
    a0 =  1 + alpha;
    a1 = -2 * cos(w0);
    a2 =  1 - alpha;
    b0 = (1 - cos(w0)) / 2;
    b1 =  1 - cos(w0);
    b2 = (1 - cos(w0)) / 2;

    p->lp[0] =-a1 / a0;
    p->lp[1] =-a2 / a0;
    p->lp[2] = b0 / a0;
    p->lp[3] = b1 / a0;
    p->lp[4] = b2 / a0;
}

static int fn(init_filter)(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AExciterContext *s = ctx->priv;
    fn(ChannelParams) *cps;

    if (!s->cp)
        s->cp = av_calloc(inlink->ch_layout.nb_channels, sizeof(fn(ChannelParams)));
    if (!s->cp)
        return AVERROR(ENOMEM);

    cps = s->cp;
    for (int ch = 0; ch < inlink->ch_layout.nb_channels; ch++) {
        fn(set_params)(&cps[ch], s->blend, s->drive, inlink->sample_rate,
                       s->freq, s->ceil);
    }
    return 0;
}

static ftype fn(bprocess)(ftype in, const ftype *const c,
                          ftype *w1, ftype *w2)
{
    ftype out = c[2] * in + *w1;

    *w1 = c[3] * in + *w2 + c[0] * out;
    *w2 = c[4] * in + c[1] * out;

    return out;
}

static ftype fn(distortion_process)(fn(ChannelParams) *p, ftype in,
                                    const ftype ceil)
{
    ftype proc = in, med;

    proc = fn(bprocess)(proc, p->hp, &p->hw[0][0], &p->hw[0][1]);
    proc = fn(bprocess)(proc, p->hp, &p->hw[1][0], &p->hw[1][1]);

    if (proc >= F(0.0)) {
        med = (fn(D)(p->ap + proc * (p->kpa - proc)) + p->kpb) * p->pwrq;
    } else {
        med = (fn(D)(p->an - proc * (p->kna + proc)) + p->knb) * p->pwrq * F(-1.0);
    }

    proc = p->srct * (med - p->prev_med + p->prev_out);
    p->prev_med = isnormal(med) ? med : F(0.0);
    p->prev_out = isnormal(proc) ? proc : F(0.0);

    proc = fn(bprocess)(proc, p->hp, &p->hw[2][0], &p->hw[2][1]);
    proc = fn(bprocess)(proc, p->hp, &p->hw[3][0], &p->hw[3][1]);

    if (ceil >= F(10000.)) {
        proc = fn(bprocess)(proc, p->lp, &p->lw[0][0], &p->lw[0][1]);
        proc = fn(bprocess)(proc, p->lp, &p->lw[1][0], &p->lw[1][1]);
    }

    return proc;
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AExciterContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const ftype level_in = s->level_in;
    const ftype level_out = s->level_out;
    const ftype amount = s->amount;
    const ftype listen = F(1.0) - s->listen;
    const int is_disabled = ff_filter_disabled(ctx);
    const int nb_samples = in->nb_samples;
    const int nb_channels = in->ch_layout.nb_channels;
    const int start = (nb_channels * jobnr) / nb_jobs;
    const int end = (nb_channels * (jobnr+1)) / nb_jobs;
    fn(ChannelParams) *cps = s->cp;
    const ftype ceil = s->ceil;

    for (int ch = start; ch < end; ch++) {
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        fn(ChannelParams) *cp = &cps[ch];

        for (int n = 0; n < nb_samples; n++) {
            ftype sample = src[n] * level_in;

            sample = fn(distortion_process)(cp, sample, ceil);
            sample = sample * amount + listen * src[n];

            sample *= level_out;
            if (is_disabled)
                dst[n] = src[n];
            else
                dst[n] = sample;
        }
    }

    return 0;
}
