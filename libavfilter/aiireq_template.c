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

typedef struct fn(Section) {
    ftype c1, c2, d0, d1;
} fn(Section);

typedef struct fn(Equalizer) {
    int nb_sections;

    fn(Section) sections[6];
} fn(Equalizer);

typedef struct fn(ChState) {
    ftype z1[6];
    ftype z2[6];
} fn(ChState);

static int fn(get_svf)(double fs, double fc, int filter_order, double gain, fn(Equalizer) *eq)
{
    double Dw, GB, G, gR, reci1Ord, rat_ord, nt_D, nt_D2, stD, ct_D, rat_ro, gP1, gP2;
    int L;

    if (fabs(gain) <= FLT_EPSILON)
        return 0;

    L = filter_order >> 1;
    Dw = M_PI * (fc / fs - 0.5);
    GB = pow(10.0, ((1.0 / sqrt(2.0)) * gain / 20.0));
    G = pow(10.0, gain / 20.0);
    gR = (G * G - GB * GB) / (GB * GB - 1.0);
    reci1Ord = 1.0 / filter_order;
    rat_ord = pow(gR, reci1Ord);
    nt_D = tan(Dw);
    nt_D2 = nt_D * nt_D;
    stD = sin(Dw);
    ct_D = cos(Dw);
    rat_ro = pow(gR, 1.0 / (2.0 * filter_order));
    gP1 = pow(G, 1.0 / filter_order);
    gP2 = pow(G, 2.0 / filter_order);

    for (int n = 0; n < L; n++) {
        fn(Section) *sec = &eq->sections[n];
        double si = sin((2.0 * (n + 1.0) - 1.0) * M_PI / (2.0 * filter_order));
        double den1 = nt_D2 + rat_ord - 2.0 * rat_ro * nt_D * si;
        double den2 = rat_ro * ct_D - si * stD;

        sec->c1 = 2.0 - 2.0 * (nt_D2 - rat_ord) / den1;
        sec->c2 = (rat_ro * ct_D) / den2;
        sec->d0 = (rat_ord + gP2 * nt_D2 - 2.0 * gP1 * rat_ro * nt_D * si) / den1;
        sec->d1 = (rat_ro * ct_D - gP1 * si * stD) / den2;
    }

    return L;
}

static void fn(filter_channel)(AVFilterContext *ctx, AVFrame *out, AVFrame *in, int ch)
{
    AudioIIREQContext *s = ctx->priv;
    const ftype overall_gain = s->overall_gain;
    const ftype *src = (const ftype *)in->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    const int is_disabled = ff_filter_disabled(ctx);
    const int nb_samples = in->nb_samples;
    const int start = ch * s->nb_bands;
    fn(ChState) *state = s->chs;
    fn(Equalizer) *eq = s->eqs;

    if (dst != src)
        memcpy(dst, src, nb_samples * sizeof(*dst));

    for (int n = 0; n < s->nb_bands; n++) {
        const fn(Equalizer) *eqs = &eq[n];
        fn(ChState) *chs = &state[start + n];

        for (int j = 0; j < eqs->nb_sections; j++) {
            const fn(Section) *sec = &eqs->sections[j];
            const ftype c1 = sec->c1;
            const ftype c2 = sec->c2;
            const ftype d0 = sec->d0;
            const ftype d1 = sec->d1;
            ftype z1 = chs->z1[j];
            ftype z2 = chs->z2[j];

            for (int i = 0; i < nb_samples; i++) {
                ftype x = dst[i];
                const ftype y = x - z1 - z2;

                x = d0 * y + d1 * z1 + z2;
                z2 += c2 * z1;
                z1 += c1 * y;

                dst[i] = x;
            }

            chs->z1[j] = isnormal(z1) ? z1 : F(0.0);
            chs->z2[j] = isnormal(z2) ? z2 : F(0.0);
        }
    }

    if (is_disabled) {
        memcpy(dst, src, nb_samples * sizeof(*dst));
    } else {
        for (int n = 0; n < nb_samples; n++)
            dst[n] *= overall_gain;
    }
}

static void fn(update_filter)(AVFilterContext *ctx)
{
    AudioIIREQContext *s = ctx->priv;
    const double fs = ctx->inputs[0]->sample_rate * 1.0;
    fn(Equalizer) *eq = s->eqs;

    for (int n = 0; n < s->nb_bands+1; n++) {
        fn(Equalizer) *eqs = &eq[n];
        const int gn = FFMIN(n, s->nb_gains-1);
        const int sn = FFMIN(n, s->nb_sections-1);
        double design_freq, dB;

        if (n == 0) {
            eqs->nb_sections = s->section_opt[0];
            design_freq = s->band_opt[0];
            dB = s->gain_opt[0];
        } else if (n == s->nb_bands) {
            eqs->nb_sections = s->section_opt[sn];
            design_freq = s->band_opt[n-1];
            dB = 0.0;
        } else {
            eqs->nb_sections = s->section_opt[sn];
            design_freq = s->band_opt[n-1];
            dB = s->gain_opt[gn] - s->gain_opt[gn-1];
        }

        if (design_freq >= fs * 0.5) {
            eqs->nb_sections = 0;
            continue;
        }

        eqs->nb_sections = fn(get_svf)(fs, design_freq, eqs->nb_sections * 2, dB, eqs);
    }
}

static int fn(init_filter)(AVFilterContext *ctx)
{
    AudioIIREQContext *s = ctx->priv;
    const int nb_channels = ctx->inputs[0]->ch_layout.nb_channels;

    s->eqs = av_calloc(s->nb_bands+1, sizeof(fn(Equalizer)));
    if (!s->eqs)
        return AVERROR(ENOMEM);

    s->chs = av_calloc((s->nb_bands+1) * nb_channels, sizeof(fn(ChState)));
    if (!s->chs)
        return AVERROR(ENOMEM);

    fn(update_filter)(ctx);

    s->update_filter = fn(update_filter);
    s->filter_channel = fn(filter_channel);

    return 0;
}
