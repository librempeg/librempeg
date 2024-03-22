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
#undef SAMPLE_SUFFIX
#if DEPTH == 32
#define ftype float
#define SAMPLE_FORMAT fltp
#define SAMPLE_SUFFIX f
#else
#define ftype double
#define SAMPLE_FORMAT dblp
#define SAMPLE_SUFFIX d
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define ft3(a,b)   a##b
#define ft2(a,b)   ft3(a,b)
#define ft(a)      ft2(a, SAMPLE_SUFFIX)

#define fN3(a,b,c) a##_##b##c
#define fN2(a,b,c) fN3(a,b,c)
#define fN(a,b)    fN2(a, SAMPLE_SUFFIX, b)

static ftype fn(svf_xover_lo)(const SVFCoeffs *svf, ftype *sc,
                              const ftype in)
{
    const ftype g = ft(svf->g);
    const ftype gk = ft(svf->gk);
    const ftype g2 = ft(svf->g2);
    const ftype g2k = ft(svf->g2k);
    ftype s0 = sc[0];
    ftype s1 = sc[1];
    ftype vband = in * ft(svf->r);
    ftype vlow = in;
    ftype v2, v3;

    v2 = F(-1.) / (F(1.) + g2 + gk) * (-s0 + g*s1 - gk*s0 + g2*vband - g*vlow - g2k*vlow);
    v3 = F(-1.) / (F(1.) + g2 + gk) * (-g*s0 - s1 - g*vband - g2*vlow);
    sc[0] = F(2.) * v2 - s0;
    sc[1] = F(2.) * v3 - s1;

    return v3;
}

static ftype fn(svf_xover_hi)(const SVFCoeffs *svf, ftype *sc,
                              const ftype in)
{
    const ftype g = ft(svf->g);
    const ftype gk = ft(svf->gk);
    const ftype g2 = ft(svf->g2);
    ftype s0 = sc[0];
    ftype s1 = sc[1];
    ftype vhigh = in;
    ftype vband = in * ft(svf->r);
    ftype v2, v3;

    v2 = F(-1.) / (F(1.) + g2 + gk) * (-s0 + g*s1 - gk*s0 + g2*vband + g*vhigh);
    v3 = F(-1.) / (F(1.) + g2 + gk) * (-g*s0 - s1 - g*vband + g2*vhigh + gk*vhigh);
    sc[0] = F(2.) * v2 - s0;
    sc[1] = F(2.) * v3 - s1;

    return vhigh + v3;
}

static void fn(run_lh)(const SVFCoeffs *svf,
                       ftype *scl, ftype *sch,
                       const ftype in, ftype *outlo, ftype *outhi)
{
    *outlo = fn(svf_xover_lo)(svf, scl, in);
    *outhi = fn(svf_xover_hi)(svf, sch, in);
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs) \
{
    AudioCrossoverContext *s = ctx->priv;
    AVFrame *in = arg;
    AVFrame **frames = s->frames;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = in->nb_samples;
    const int nb_outs = ctx->nb_outputs;
    const ftype level_in = s->level_in;
    const float *gains = s->gains;
    const int *active = s->active;

    for (int ch = start; ch < end; ch++) {
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype in_gain = level_in;
        int last_next_band = 0;
        int last_band = -1;

        for (int band = 0; band < nb_outs-1; band++) {
            const int next_band = next_active_band(active, band, nb_outs);

            last_band = active[band] ? band : last_band;
            if (active[band] && next_band) {
                const SVFCoeffs *svf_cf = &s->svf_cf[band];
                ftype *svf_lo = ft(s->svf[ch].sc)[band][0];
                ftype *svf_hi = ft(s->svf[ch].sc)[band][1];
                ftype *dst = (ftype *)frames[band]->extended_data[ch];
                ftype *next_dst = (ftype *)frames[next_band]->extended_data[ch];
                const ftype out_gain = gains[band];
                ftype high = F(0.0);
                ftype low = F(0.0);

                for (int n = 0; n < nb_samples; n++) {
                    fn(run_lh)(svf_cf, svf_lo, svf_hi,
                               src[n] * in_gain, &low, &high);
                    dst[n] = low * out_gain;
                    next_dst[n] = high;
                }

                src = next_dst;
                in_gain = F(1.0);
                last_next_band = next_band;
            }
        }

        last_band = active[nb_outs-1] ? (nb_outs-1) : last_band;
        if (last_next_band) {
            ftype *dst = (ftype *)frames[last_next_band]->extended_data[ch];
            const ftype out_gain = gains[last_next_band];

            for (int n = 0; n < nb_samples; n++)
                dst[n] *= out_gain;
        } else if (last_band >= 0) {
            const int band = (last_band < (nb_outs-1)) ? last_band : last_band-1;
            const SVFCoeffs *svf_cf = &s->svf_cf[band];
            ftype *svf_lo = ft(s->svf[ch].sc)[band][0];
            ftype *svf_hi = ft(s->svf[ch].sc)[band][1];
            ftype *dst = (ftype *)frames[band]->extended_data[ch];
            const ftype out_gain = gains[band];
            ftype high = F(0.0);
            ftype low = F(0.0);

            for (int n = 0; n < nb_samples; n++) {
                fn(run_lh)(svf_cf, svf_lo, svf_hi,
                           src[n] * in_gain, &low, &high);
                if (last_band == (nb_outs-1))
                    dst[n] = high * out_gain;
                else
                    dst[n] = low * out_gain;
            }
        }
    }

    return 0;
}
