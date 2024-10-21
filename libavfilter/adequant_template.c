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

#undef MSQRT1_2
#undef FTAN
#undef CLIP
#undef SQRT
#undef ftype
#undef MPI
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define MSQRT1_2 M_SQRT1_2f
#define MPI M_PIf
#define FTAN tanf
#define CLIP av_clipf
#define SQRT sqrtf
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define MSQRT1_2 M_SQRT1_2
#define MPI M_PI
#define FTAN tan
#define CLIP av_clipd
#define SQRT sqrt
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define MIN_FREQ F(0.00001)
#define MAX_FREQ F(0.49999)

typedef struct fn(StateContext) {
    ftype la[3], lm[3], lc[2];
    ftype ha[3], hm[3], hc[2];
    ftype ip, op;
    ftype pf, f;
    unsigned flat;
    unsigned pos;
    unsigned size;
    ftype *queue;
    ftype *freqs;
    ftype *block[3];
    ftype rla[3], rlm[3], rlc[2];
} fn(StateContext);

static void fn(uninit_state)(AVFilterContext *ctx)
{
    ADequantContext *s = ctx->priv;
    fn(StateContext) *stc = s->state;

    if (stc) {
        for (int ch = 0; ch < s->nb_channels; ch++) {
            fn(StateContext) *sc = &stc[ch];

            av_freep(&sc->queue);
            av_freep(&sc->freqs);
            for (int n = 0; n < 3; n++)
                av_freep(&sc->block[n]);
        }

        av_freep(&s->state);
    }
}

static void fn(hfilter_prepare)(const ftype f, ftype a[3], ftype m[3])
{
    const ftype Q = MSQRT1_2;
    const ftype g = FTAN(MPI * f);
    const ftype k = F(1.0) / Q;

    a[0] = F(1.0) / (F(1.0) + g * (g + k));
    a[1] = g * a[0];
    a[2] = g * a[1];

    m[0] = F(1.0);
    m[1] = -k;
    m[2] = F(-1.0);
}

static int fn(init_state)(AVFilterContext *ctx)
{
    ADequantContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    fn(StateContext) *stc;

    s->state = av_calloc(inlink->ch_layout.nb_channels, sizeof(*stc));
    if (!s->state)
        return AVERROR(ENOMEM);
    stc = s->state;
    s->nb_channels = inlink->ch_layout.nb_channels;

    for (int ch = 0; ch < s->nb_channels; ch++) {
        fn(StateContext) *sc = &stc[ch];

        fn(hfilter_prepare)(F(1.0) / inlink->sample_rate, sc->ha, sc->hm);

        sc->pf = NAN;
        sc->ip = NAN;
        sc->op = NAN;
        sc->f = MAX_FREQ;
        if (s->blocksize > 0) {
            for (int n = 0; n < 3; n++) {
                sc->block[n] = av_calloc(s->blocksize*2, sizeof(*sc->block[n]));
                if (!sc->block[n])
                    return AVERROR(ENOMEM);
            }
            sc->freqs = av_calloc(s->blocksize*2, sizeof(*sc->freqs));
            if (!sc->freqs)
                return AVERROR(ENOMEM);
        }
        sc->queue = av_malloc_array(s->max_size, sizeof(*sc->queue));
        if (!sc->queue)
            return AVERROR(ENOMEM);
        for (int n = 0; n < s->max_size; n++)
            sc->queue[n] = n+1;
    }

    return 0;
}

static void fn(lfilter_prepare)(const ftype f, const ftype A, ftype a[3], ftype m[3])
{
    const ftype Q = MSQRT1_2;
    const ftype g = FTAN(MPI * f) * SQRT(A);
    const ftype k = F(1.0) / Q;

    a[0] = F(1.0) / (F(1.0) + g * (g + k));
    a[1] = g * a[0];
    a[2] = g * a[1];

    m[0] = A * A;
    m[1] = k * (F(1.0) - A) * A;
    m[2] = F(1.0) - A * A;
}

static ftype fn(filter_sample)(const ftype sample,
                                const ftype a[3],
                                const ftype m[3],
                                ftype c[2])
{
    ftype v0 = sample;
    ftype v3 = v0 - c[1];
    ftype v1 = a[0] * c[0] + a[1] * v3;
    ftype v2 = c[1] + a[1] * c[0] + a[2] * v3;

    c[0] = F(2.0) * v1 - c[0];
    c[1] = F(2.0) * v2 - c[1];

    return m[0] * v0 + m[1] * v1 + m[2] * v2;
}

static void fn(update_state)(ftype *lc, ftype *hc)
{
    lc[0] = isnormal(lc[0]) ? lc[0] : F(0.0);
    lc[1] = isnormal(lc[1]) ? lc[1] : F(0.0);

    hc[0] = isnormal(hc[0]) ? hc[0] : F(0.0);
    hc[1] = isnormal(hc[1]) ? hc[1] : F(0.0);
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ADequantContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const ftype gain = s->gain;
    const unsigned max_size = s->max_size;
    const unsigned cur_size = s->cur_size;
    const int start = (s->nb_channels * jobnr) / nb_jobs;
    const int end = (s->nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = in->nb_samples;
    const int is_disabled = ff_filter_disabled(ctx);
    fn(StateContext) *st = s->state;

    for (int chan = start; chan < end; chan++) {
        const ftype *src = (const ftype *)in->extended_data[chan];
        ftype *dst = (ftype *)out->extended_data[chan];
        fn(StateContext) *stc = &st[chan];
        unsigned flat = stc->flat;
        unsigned pos = stc->pos;
        unsigned size = stc->size;
        ftype *q = stc->queue;
        ftype ip = stc->ip;
        ftype op = stc->op;
        ftype pf = stc->pf;
        ftype *la = stc->la;
        ftype *lm = stc->lm;
        ftype *lc = stc->lc;
        ftype *ha = stc->ha;
        ftype *hm = stc->hm;
        ftype *hc = stc->hc;
        ftype f = stc->f;

        for (int n = 0; n < nb_samples; n++) {
            const unsigned idx = (pos >= cur_size)? (pos-cur_size) : (pos+max_size-cur_size);
            const unsigned pidx = (idx < max_size-1) ? (idx+1) : 0;
            const ftype isample = src[n];
            ftype osample;

            ip = isnan(ip) ? isample : ip;
            op = isnan(op) ? isample : op;

            flat += isample == ip;
            flat -= q[idx] == q[pidx];
            q[pos] = isample;
            pos++;
            if (size < cur_size)
                size++;
            if (pos >= max_size)
                pos = 0;

            f = CLIP(MAX_FREQ * F(size-flat) / (F(size)), MIN_FREQ, MAX_FREQ);
            if (f != pf)
                fn(lfilter_prepare)(f, gain, la, lm);
            pf = f;

            osample = fn(filter_sample)(isample-ip, la, lm, lc);
            osample = op + fn(filter_sample)(osample, ha, hm, hc);

            dst[n] = is_disabled ? isample : osample;

            ip = isample;
            op = osample;
        }

        fn(update_state)(lc, hc);

        stc->f = f;
        stc->pf = pf;
        stc->ip = ip;
        stc->op = op;
        stc->pos = pos;
        stc->size = size;
        stc->flat = flat;
    }

    return 0;
}

static void fn(reverse_samples)(ftype *_dst, const ftype *_src,
                                int oo, int io, const int nb_samples)
{
    const ftype *src = _src + io;
    ftype *dst = _dst + oo;

    for (int i = 0, j = nb_samples - 1; i < nb_samples; i++, j--)
        dst[i] = src[j];
}

static int fn(filter_block_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ADequantContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const ftype gain = SQRT(s->gain);
    const unsigned max_size = s->max_size;
    const unsigned cur_size = s->cur_size;
    const int start = (s->nb_channels * jobnr) / nb_jobs;
    const int end = (s->nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = in->nb_samples;
    const int is_disabled = ff_filter_disabled(ctx);
    const int blocksize = s->blocksize;
    fn(StateContext) *st = s->state;

    for (int chan = start; chan < end; chan++) {
        const ftype *src = (const ftype *)in->extended_data[chan];
        ftype *dst = (ftype *)out->extended_data[chan];
        fn(StateContext) *stc = &st[chan];
        unsigned flat = stc->flat;
        unsigned pos = stc->pos;
        unsigned size = stc->size;
        ftype *freqs = stc->freqs;
        ftype *b0 = stc->block[0];
        ftype *b1 = stc->block[1];
        ftype *b2 = stc->block[2];
        ftype *q = stc->queue;
        ftype *rla = stc->rla;
        ftype *rlm = stc->rlm;
        ftype *rlc = stc->rlc;
        ftype *la = stc->la;
        ftype *lm = stc->lm;
        ftype *lc = stc->lc;
        ftype *ha = stc->ha;
        ftype *hm = stc->hm;
        ftype *hc = stc->hc;
        ftype ip = stc->ip;
        ftype op = stc->op;
        ftype pf = stc->pf;
        ftype f = stc->f;

        for (int n = 0; n < nb_samples; n++) {
            const unsigned idx = (pos >= cur_size)? (pos-cur_size) : (pos+max_size-cur_size);
            const unsigned pidx = (idx < max_size-1) ? (idx+1) : 0;
            const ftype isample = src[n];
            ftype osample;

            ip = isnan(ip) ? isample : ip;
            op = isnan(op) ? isample : op;

            flat += isample == ip;
            flat -= q[idx] == q[pidx];
            q[pos] = isample;
            pos++;
            if (size < cur_size)
                size++;
            if (pos >= max_size)
                pos = 0;

            f = CLIP(MAX_FREQ * F(size-flat) / (F(size)), MIN_FREQ, MAX_FREQ);
            freqs[n+blocksize] = f;
            if (f != pf)
                fn(lfilter_prepare)(f, gain, la, lm);
            pf = f;
            osample = fn(filter_sample)(isample-ip, la, lm, lc);

            b0[n+blocksize] = osample;

            ip = isample;
        }

        for (int n = nb_samples; n < blocksize; n++) {
            b0[n+blocksize] = F(0.0);
            freqs[n+blocksize] = f;
        }

        fn(update_state)(lc, hc);

        fn(reverse_samples)(b1, b0, 0, 0, 2 * blocksize);

        for (int n = 0; n < 2 * blocksize; n++) {
            fn(lfilter_prepare)(freqs[n], gain, rla, rlm);
            b1[n] = fn(filter_sample)(b1[n], rla, rlm, rlc);
        }

        fn(reverse_samples)(b2, b1, 0, 0, 2 * blocksize);

        for (int n = 0; n < blocksize; n++) {
            ftype osample = b2[n];

            osample = op + fn(filter_sample)(osample, ha, hm, hc);
            dst[n] = is_disabled ? src[n] : osample;

            op = osample;
        }

        fn(update_state)(rlc, hc);

        memcpy(b0, b0 + blocksize, blocksize * sizeof(*b0));
        memcpy(freqs, freqs + blocksize, blocksize * sizeof(*freqs));

        stc->f = f;
        stc->pf = pf;
        stc->ip = ip;
        stc->op = op;
        stc->pos = pos;
        stc->size = size;
        stc->flat = flat;
    }

    return 0;
}
