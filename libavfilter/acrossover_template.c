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
#undef FTAN
#if DEPTH == 32
#define ftype float
#define FTAN tanf
#define SAMPLE_FORMAT fltp
#else
#define ftype double
#define FTAN tan
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(SVFCoeffs) {
    ftype g, r, k, f, f2;
    ftype gk, g2, g2k;
    int valid;
} fn(SVFCoeffs);

typedef struct fn(SVFCache) {
    ftype sc[MAX_BANDS][2][2];
} fn(SVFCache);

static int fn(init_xover)(AVFilterContext *ctx, void **st, void **stc,
                          const double *splits, const double *resonance,
                          const int nb_splits, const int nb_channels,
                          const int sample_rate)
{
    fn(SVFCoeffs) *svf_cf;

    if (!*st) {
        *st = av_calloc(nb_channels, sizeof(fn(SVFCache)));
        if (!*st)
            return AVERROR(ENOMEM);
    }

    if (!*stc) {
        *stc = av_calloc(nb_splits+1, sizeof(fn(SVFCoeffs)));
        if (!*stc)
            return AVERROR(ENOMEM);
    }

    svf_cf = *stc;

    for (int band = 0; band < nb_splits; band++) {
        fn(SVFCoeffs) *sf = &svf_cf[band];

        sf->valid = splits[band] < sample_rate * F(0.5);
        sf->g = FTAN(F(M_PI)*splits[band]/sample_rate);
        sf->k = F(2.0) - F(2.0) * resonance[band];
        sf->g2 = sf->g*sf->g;
        sf->g2k = sf->g2*sf->k;
        sf->gk = sf->g*sf->k;
        sf->r = F(1.0) - resonance[band];
        sf->f = F(-1.0) / (F(1.0)+sf->g2+sf->gk);
        sf->f2 = sf->f * F(2.0);
    }

    return 0;
}

static void fn(update_state)(ftype *lo, ftype *hi)
{
    lo[0] = isnormal(lo[0]) ? lo[0] : F(0.0);
    lo[1] = isnormal(lo[1]) ? lo[1] : F(0.0);
    hi[0] = isnormal(hi[0]) ? hi[0] : F(0.0);
    hi[1] = isnormal(hi[1]) ? hi[1] : F(0.0);
}

static ftype fn(svf_xover_lo)(const fn(SVFCoeffs) *svf, ftype *sc,
                              const ftype in)
{
    const ftype g = svf->g;
    const ftype f = svf->f;
    const ftype f2 = svf->f2;
    const ftype gk = svf->gk;
    const ftype g2 = svf->g2;
    const ftype g2k = svf->g2k;
    ftype s0 = sc[0];
    ftype s1 = sc[1];
    ftype vband = in * svf->r;
    ftype vlow = in;
    ftype v2, v3;

    v2 = f2 * (-s0 + g*s1 - gk*s0 + g2*vband - g*vlow - g2k*vlow);
    v3 = f * (-g*s0 - s1 - g*vband - g2*vlow);
    sc[0] = v2 - s0;
    sc[1] = v3 * F(2.0) - s1;

    return v3;
}

static ftype fn(svf_xover_hi)(const fn(SVFCoeffs) *svf, ftype *sc,
                              const ftype in)
{
    const ftype g = svf->g;
    const ftype f = svf->f;
    const ftype f2 = svf->f2;
    const ftype gk = svf->gk;
    const ftype g2 = svf->g2;
    ftype s0 = sc[0];
    ftype s1 = sc[1];
    ftype vhigh = in;
    ftype vband = in * svf->r;
    ftype v2, v3;

    v2 = f2 * (-s0 + g*s1 - gk*s0 + g2*vband + g*vhigh);
    v3 = f * (-g*s0 - s1 - g*vband + g2*vhigh + gk*vhigh);
    sc[0] = v2 - s0;
    sc[1] = v3 * F(2.0) - s1;

    return vhigh + v3;
}

static void fn(run_lh)(const fn(SVFCoeffs) *svf,
                       ftype *scl, ftype *sch,
                       const ftype in, ftype *outlo, ftype *outhi)
{
    *outlo = fn(svf_xover_lo)(svf, scl, in);
    *outhi = fn(svf_xover_hi)(svf, sch, in);
}

static void fn(xover_channel)(AVFilterContext *ctx, void *st, void *stc, const int ch,
                              const int nb_outs, const int nb_samples,
                              const double in_level,
                              const uint8_t *srcp, uint8_t **dstp,
                              const double *gains, const int *active)
{
    const fn(SVFCoeffs) *svfcf = stc;
    ftype in_gain = in_level;
    fn(SVFCache) *svf = st;
    const ftype *src = (const ftype *)srcp;
    int last_next_band = 0;
    int last_band = -1;

    for (int band = 0; band < nb_outs-1; band++) {
        const fn(SVFCoeffs) *svf_cf = &svfcf[band];
        const int next_band = next_active_band(active, band, nb_outs);

        last_band = active[band] ? band : last_band;
        if (svf_cf->valid && active[band] && next_band) {
            ftype *svf_lo = svf[ch].sc[band][0];
            ftype *svf_hi = svf[ch].sc[band][1];
            ftype *dst = (ftype *)dstp[band];
            ftype *next_dst = (ftype *)dstp[next_band];
            const ftype out_gain = gains[band];

            for (int n = 0; n < nb_samples; n++) {
                ftype high = F(0.0), low = F(0.0);

                fn(run_lh)(svf_cf, svf_lo, svf_hi,
                           src[n] * in_gain, &low, &high);
                dst[n] = low * out_gain;
                next_dst[n] = high;
            }

            fn(update_state)(svf_lo, svf_hi);

            src = next_dst;
            in_gain = F(1.0);
            last_next_band = next_band;
        }
    }

    last_band = active[nb_outs-1] ? (nb_outs-1) : last_band;
    if (last_next_band) {
        ftype *dst = (ftype *)dstp[last_next_band];
        const ftype out_gain = gains[last_next_band];

        for (int n = 0; n < nb_samples; n++)
            dst[n] *= out_gain;
    } else if (last_band >= 0) {
        const int band = (last_band < (nb_outs-1)) ? last_band : last_band-1;
        const fn(SVFCoeffs) *svf_cf = &svfcf[band];
        ftype *svf_lo = svf[ch].sc[band][0];
        ftype *svf_hi = svf[ch].sc[band][1];
        ftype *dst = (ftype *)dstp[band];
        const ftype out_gain = gains[band];

        if (!svf_cf->valid)
            return;

        for (int n = 0; n < nb_samples; n++) {
            ftype high = F(0.0), low = F(0.0);

            fn(run_lh)(svf_cf, svf_lo, svf_hi,
                       src[n] * in_gain, &low, &high);
            if (last_band == (nb_outs-1))
                dst[n] = high * out_gain;
            else
                dst[n] = low * out_gain;
        }

        fn(update_state)(svf_lo, svf_hi);
    }
}
