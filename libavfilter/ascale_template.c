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
#undef SQRT
#undef ctype
#undef ftype
#undef CLIP
#undef FSIN
#undef TX_TYPE
#undef MAXF
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define EPS FLT_EPSILON
#define SQRT sqrtf
#define FSIN sinf
#define ctype AVComplexFloat
#define ftype float
#define MAXF FLT_MAX
#define CLIP av_clipf
#define TX_TYPE AV_TX_FLOAT_RDFT
#define SAMPLE_FORMAT fltp
#else
#define EPS DBL_EPSILON
#define SQRT sqrt
#define FSIN sin
#define ctype AVComplexDouble
#define ftype double
#define MAXF DBL_MAX
#define CLIP av_clipd
#define TX_TYPE AV_TX_DOUBLE_RDFT
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(copy_samples)(AVFilterContext *ctx, const int ch,
                            const int req_period)
{
    AScaleContext *s = ctx->priv;
    const ftype fs = F(1.0)/ctx->inputs[0]->sample_rate;
    const int period = req_period ? FFMIN(req_period, s->max_period) : 1;
    ChannelContext *c = &s->c[ch];
    void *datax[1] = { (void *)c->data[0] };
    int size;

    size = av_audio_fifo_peek(c->in_fifo, datax, period);
    if (size > 0) {
        av_audio_fifo_write(c->out_fifo, datax, size);
        c->state[OUT] += size * fs;
        av_audio_fifo_drain(c->in_fifo, size);
    }

    return av_audio_fifo_size(c->in_fifo) >= s->max_period;
}

static ftype fn(get_gain)(const ftype w, const ftype c)
{
    const ftype x = w*F(2.0)-F(1.0);
    const ftype a = F(9.0/16.0)*FSIN(x*F(M_PI_2))+F(1.0/16.0)*FSIN(F(3.0)*x*F(M_PI_2));
    const ftype b = F(1.0)+c;

    return SQRT(F(0.5)/b-(F(1.0)-c)*a*a/b)+a;
}

static ftype fn(l2norm)(const ftype *x, const int N)
{
    ftype y = F(0.0);

    for (int n = 0; n < N; n++)
        y += x[n]*x[n];

    return SQRT(y);
}

static int fn(expand_write)(AVFilterContext *ctx, const int ch)
{
    AScaleContext *s = ctx->priv;
    const ftype fs = F(1.0)/ctx->inputs[0]->sample_rate;
    ChannelContext *c = &s->c[ch];
    const int best_period = c->best_period+1;
    const ftype best_score = c->best_score;
    const int max_period = s->max_period;
    const int n = max_period-best_period;
    ftype *dptrx = c->data[0];
    ftype *dptry = c->data[1];
    const ftype *rptrx = c->r_data[0];
    void *datax[1] = { (void *)c->data[0] };
    void *datay[1] = { (void *)c->data[1] };
    const ftype xx = fn(l2norm)(dptrx+n, best_period);
    const ftype yy = fn(l2norm)(dptry, best_period);
    const ftype xy = rptrx[best_period];
    ftype best_xcorr = F(-1.0), scale;
    const ftype num = xy;
    const ftype den = xx * yy + EPS;

    best_xcorr = num/den;
    best_xcorr = CLIP(best_xcorr, F(-0.75), F(1.0));

    av_log(ctx, AV_LOG_DEBUG, "E: %g/%g %d/%d\n", best_xcorr, best_score, best_period, max_period);

    dptrx += n;
    datax[0] = dptrx;

    scale = F(1.0) / best_period;
    for (int n = 0; n < best_period; n++) {
        const ftype xf = n*scale;
        const ftype yf = F(1.0)-xf;
        const ftype axf = fn(get_gain)(xf, best_xcorr);
        const ftype ayf = fn(get_gain)(yf, best_xcorr);
        const ftype x = dptrx[n];
        const ftype y = dptry[n];

        dptrx[n] = x * axf + y * ayf;
    }

    av_audio_fifo_write(c->out_fifo, datax, best_period);
    av_audio_fifo_write(c->out_fifo, datay, best_period);
    c->state[OUT] += best_period*2*fs;
    av_audio_fifo_drain(c->in_fifo, best_period);
    return av_audio_fifo_size(c->in_fifo) >= max_period;
}

static int fn(expand_samples)(AVFilterContext *ctx, const int ch)
{
    AScaleContext *s = ctx->priv;
    const ftype fs = F(1.0)/ctx->inputs[0]->sample_rate;
    const int max_period = s->max_period;
    const int max_size = s->max_size;
    ChannelContext *c = &s->c[ch];
    void *datax[1] = { (void *)c->data[0] };
    void *datay[1] = { (void *)c->data[1] };
    ctype *cptrx = c->c_data[0];
    ctype *cptry = c->c_data[1];
    ftype *rptrx = c->r_data[0];
    ftype *rptry = c->r_data[1];
    ftype *dptrx = c->data[0];
    ftype *dptry = c->data[1];
    ftype best_score = -MAXF;
    int best_period = -1, ns;
    int size;

    if (av_audio_fifo_size(c->in_fifo) <= 0)
        return 0;

    if (!s->eof && av_audio_fifo_size(c->in_fifo) < max_period)
        return 0;

    if (c->keep[IN] < max_period) {
        size = max_period-c->keep[IN];
        size = av_audio_fifo_read(c->in_fifo, datax, size);
        if (size > 0) {
            av_audio_fifo_write(c->out_fifo, datax, size);
            c->state[OUT] += size * fs;
            c->keep[IN] += size;
        }

        if (!s->eof && c->keep[IN] < max_period)
            return 0;

        if (av_audio_fifo_size(c->in_fifo) < max_period)
            return 0;
    }

    size = av_audio_fifo_peek_at(c->out_fifo, datax, max_period, FFMAX(av_audio_fifo_size(c->out_fifo)-max_period, 0));
    if (size < 0)
        size = 0;
    if (size < max_period) {
        const int offset = max_period - size;

        memmove(dptrx + offset, dptrx, size * sizeof(*dptrx));
        memset(dptrx, 0, offset * sizeof(*dptrx));
    }

    size = av_audio_fifo_peek(c->in_fifo, datay, max_period);
    if (size < 0)
        size = 0;
    if (size < max_period)
        memset(dptry+size, 0, (max_period-size)*sizeof(*dptry));

    memset(rptrx+max_period, 0, (max_size+2-max_period) * sizeof(*rptrx));
    for (int n = 0; n < max_period; n++)
        rptrx[n] = dptrx[max_period-n-1];

    memset(rptry+max_period, 0, (max_size+2-max_period) * sizeof(*rptry));
    memcpy(rptry, dptry, max_period * sizeof(*rptry));

    c->r2c_fn(c->r2c, cptrx, rptrx, sizeof(*rptrx));
    c->r2c_fn(c->r2c, cptry, rptry, sizeof(*rptry));

    cptrx[0].re = cptrx[0].im = cptry[0].re = cptry[0].im = F(0.0);
    for (int n = 0; n < max_size/2+1; n++) {
        const ftype re0 = cptrx[n].re;
        const ftype im0 = cptrx[n].im;
        const ftype re1 = cptry[n].re;
        const ftype im1 = cptry[n].im;

        cptrx[n].re = re0*re1 - im1*im0;
        cptrx[n].im = im0*re1 + im1*re0;
    }

    c->c2r_fn(c->c2r, rptrx, cptrx, sizeof(*cptrx));

    for (int n = 1; n < max_period-1; n++) {
        ns = n;
        if (rptrx[n] < rptrx[n-1] &&
            rptrx[n] < rptrx[n+1])
            break;
    }

    for (int n = ns; n < max_period-1; n++) {
        if (rptrx[n] > rptrx[n-1] &&
            rptrx[n] > rptrx[n+1]) {
            const ftype score = rptrx[n];

            if (score > best_score) {
                best_score = score;
                best_period = n;
            }
        }
    }

    if (best_period <= 0)
        best_period = max_period/2;

    c->best_period = best_period;
    c->best_score = best_score;
    c->mode = EXPAND;

    return fn(expand_write)(ctx, ch);;
}

static int fn(compress_write)(AVFilterContext *ctx, const int ch)
{
    AScaleContext *s = ctx->priv;
    const ftype fs = F(1.0)/ctx->inputs[0]->sample_rate;
    ChannelContext *c = &s->c[ch];
    const int best_period = c->best_period+1;
    const ftype best_score = c->best_score;
    const int max_period = s->max_period;
    const int n = best_period;
    ftype *dptrx = c->data[0];
    ftype *dptry = c->data[0];
    void *datax[1] = { (void *)c->data[0] };
    const ftype xx = fn(l2norm)(dptrx, best_period);
    const ftype yy = fn(l2norm)(dptry+n, best_period);
    const ftype xy = best_score;
    ftype best_xcorr = F(-1.0), scale;
    const ftype num = xy;
    const ftype den = xx * yy + EPS;

    best_xcorr = num/den;
    best_xcorr = CLIP(best_xcorr, F(-0.75), F(1.0));

    av_log(ctx, AV_LOG_DEBUG, "C: %g/%g %d/%d\n", best_xcorr, best_score, best_period, max_period);

    scale = F(1.0) / best_period;
    for (int n = 0; n < best_period; n++) {
        const ftype yf = n*scale;
        const ftype xf = F(1.0)-yf;
        const ftype axf = fn(get_gain)(xf, best_xcorr);
        const ftype ayf = fn(get_gain)(yf, best_xcorr);
        const ftype x = dptrx[n];
        const ftype y = dptry[n+best_period];

        dptrx[n] = x * axf + y * ayf;
    }

    av_audio_fifo_write(c->out_fifo, datax, best_period);
    c->state[OUT] += best_period*fs;
    av_audio_fifo_drain(c->in_fifo, best_period*2);
    return av_audio_fifo_size(c->in_fifo) >= max_period*2;
}

static int fn(compress_samples)(AVFilterContext *ctx, const int ch)
{
    AScaleContext *s = ctx->priv;
    const int max_period = s->max_period;
    const int max_size = s->max_size;
    ChannelContext *c = &s->c[ch];
    void *datax[1] = { (void *)c->data[0] };
    ctype *cptrx = c->c_data[0];
    ctype *cptry = c->c_data[1];
    ftype *rptrx = c->r_data[0];
    ftype *rptry = c->r_data[1];
    ftype *dptrx = c->data[0];
    ftype *dptry = c->data[0];
    ftype best_score = -MAXF;
    int best_period = -1, ns;
    int size;

    if (av_audio_fifo_size(c->in_fifo) <= 0)
        return 0;

    if (!s->eof && av_audio_fifo_size(c->in_fifo) < max_period*2)
        return 0;

    size = av_audio_fifo_peek(c->in_fifo, datax, max_period*2);
    if (size < 0)
        size = 0;
    if (size < max_period)
        memset(dptrx+size, 0, (max_period-size)*sizeof(*dptrx));

    memset(rptrx+max_period, 0, (max_size+2-max_period) * sizeof(*rptrx));
    for (int n = 0; n < max_period; n++)
        rptrx[n] = dptrx[max_period-n-1];

    memset(rptry+max_period, 0, (max_size+2-max_period) * sizeof(*rptry));
    memcpy(rptry, dptry + max_period, max_period * sizeof(*rptry));

    c->r2c_fn(c->r2c, cptrx, rptrx, sizeof(*rptrx));
    c->r2c_fn(c->r2c, cptry, rptry, sizeof(*rptry));

    cptrx[0].re = cptrx[0].im = cptry[0].re = cptry[0].im = F(0.0);
    for (int n = 0; n < max_size/2+1; n++) {
        const ftype re0 = cptrx[n].re;
        const ftype im0 = cptrx[n].im;
        const ftype re1 = cptry[n].re;
        const ftype im1 = cptry[n].im;

        cptrx[n].re = re0*re1 - im1*im0;
        cptrx[n].im = im0*re1 + im1*re0;
    }

    c->c2r_fn(c->c2r, rptrx, cptrx, sizeof(*cptrx));

    for (int n = 1; n < max_period-1; n++) {
        ns = n;
        if (rptrx[n] < rptrx[n-1] &&
            rptrx[n] < rptrx[n+1])
            break;
    }

    for (int n = ns; n < max_period-1; n++) {
        if (rptrx[n] > rptrx[n-1] &&
            rptrx[n] > rptrx[n+1]) {
            const ftype score = rptrx[n];

            if (score > best_score) {
                best_score = score;
                best_period = n;
            }
        }
    }

    if (best_period <= 0)
        best_period = max_period/2;

    c->best_period = best_period;
    c->best_score = best_score;
    c->mode = COMPRESS;

    return fn(compress_write)(ctx, ch);;
}

static int fn(filter_samples)(AVFilterContext *ctx, const int ch)
{
    const ftype fs = ctx->inputs[0]->sample_rate;
    AScaleContext *s = ctx->priv;
    ChannelContext *c = &s->c[ch];
    double state = c->state[OUT] * s->tempo - c->state[IN];

    c->mode = COPY;

    if (s->tempo == 1.0 || ff_filter_disabled(ctx))
        return fn(copy_samples)(ctx, ch, s->max_period);
    else if (state < 0.0 && s->tempo < 1.0)
        return fn(expand_samples)(ctx, ch);
    else if (state > 0.0 && s->tempo > 1.0)
        return fn(compress_samples)(ctx, ch);
    return fn(copy_samples)(ctx, ch, lrint(fabs(state*fs)));
}

static void fn(filter_channel)(AVFilterContext *ctx, const int ch)
{
    int ret;

    do {
        ret = fn(filter_samples)(ctx, ch);
    } while (ret > 0);
    av_assert0(ret >= 0);
}

static void fn(correlate_stereo)(AVFilterContext *ctx, AVFrame *out)
{
    const int nb_samples = out->nb_samples;
    ftype *src0 = (ftype *)out->extended_data[0];
    ftype *src1 = (ftype *)out->extended_data[1];

    for (int n = 0; n < nb_samples; n++) {
        ftype l = src0[n], r = src1[n];

        src0[n] = (l + r) * F(0.5);
        src1[n] = (l - r) * F(0.5);
    }
}

static void fn(decorrelate_stereo)(AVFilterContext *ctx, AVFrame *out)
{
    const int nb_samples = out->nb_samples;
    ftype *src0 = (ftype *)out->extended_data[0];
    ftype *src1 = (ftype *)out->extended_data[1];

    for (int n = 0; n < nb_samples; n++) {
        ftype l = src0[n], r = src1[n];

        src0[n] = l + r;
        src1[n] = l - r;
    }
}

static int fn(init_state)(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AScaleContext *s = ctx->priv;

    s->c = av_calloc(s->nb_channels, sizeof(*s->c));
    if (!s->c)
        return AVERROR(ENOMEM);

    for (int ch = 0; ch < s->nb_channels; ch++) {
        ChannelContext *c = &s->c[ch];
        const ftype scale = F(1.0);
        const ftype iscale = F(1.0) / s->max_size;
        int ret;

        c->keep[OUT] = s->max_period;

        c->r_data[0] = av_calloc(s->max_size+2, sizeof(ftype));
        if (!c->r_data[0])
            return AVERROR(ENOMEM);

        c->r_data[1] = av_calloc(s->max_size+2, sizeof(ftype));
        if (!c->r_data[1])
            return AVERROR(ENOMEM);

        c->c_data[0] = av_calloc(s->max_size/2+1, sizeof(ctype));
        if (!c->c_data[0])
            return AVERROR(ENOMEM);

        c->c_data[1] = av_calloc(s->max_size/2+1, sizeof(ctype));
        if (!c->c_data[1])
            return AVERROR(ENOMEM);

        c->data[0] = av_calloc(s->max_period*2, sizeof(ftype));
        if (!c->data[0])
            return AVERROR(ENOMEM);

        c->data[1] = av_calloc(s->max_period, sizeof(ftype));
        if (!c->data[1])
            return AVERROR(ENOMEM);

        c->in_fifo = av_audio_fifo_alloc(inlink->format, 1, s->max_period);
        if (!c->in_fifo)
            return AVERROR(ENOMEM);

        c->out_fifo = av_audio_fifo_alloc(inlink->format, 1, s->max_size);
        if (!c->out_fifo)
            return AVERROR(ENOMEM);

        ret = av_tx_init(&c->r2c, &c->r2c_fn,
                         TX_TYPE, 0, s->max_size, &scale, 0);
        if (ret < 0)
            return ret;

        ret = av_tx_init(&c->c2r, &c->c2r_fn,
                         TX_TYPE, 1, s->max_size, &iscale, 0);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static void fn(uninit_state)(AVFilterContext *ctx)
{
    AScaleContext *s = ctx->priv;

    for (int ch = 0; ch < s->nb_channels && s->c; ch++) {
        ChannelContext *c = &s->c[ch];

        av_freep(&c->data[0]);
        av_freep(&c->data[1]);
        av_freep(&c->c_data[0]);
        av_freep(&c->c_data[1]);
        av_freep(&c->r_data[0]);
        av_freep(&c->r_data[1]);

        av_log(ctx, AV_LOG_DEBUG, "[%d]: out: %d | in: %d\n", ch,
               av_audio_fifo_size(c->out_fifo),
               av_audio_fifo_size(c->in_fifo));

        av_audio_fifo_free(c->out_fifo);
        c->out_fifo = NULL;
        av_audio_fifo_free(c->in_fifo);
        c->in_fifo = NULL;

        av_tx_uninit(&c->r2c);
        av_tx_uninit(&c->c2r);
    }

    av_freep(&s->c);
}
