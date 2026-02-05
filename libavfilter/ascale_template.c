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

#undef EPS
#undef SQRT
#undef ctype
#undef ftype
#undef CLIP
#undef FSIN
#undef TX_TYPE
#undef MAXF
#undef FABS
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define EPS FLT_EPSILON
#define SQRT sqrtf
#define FSIN sinf
#define ctype AVComplexFloat
#define ftype float
#define MAXF FLT_MAX
#define FABS fabsf
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
#define FABS fabs
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

    av_log(ctx, AV_LOG_DEBUG, "P: [%d] %d/%d/%d\n", ch, size, period, s->max_period);

    return av_audio_fifo_size(c->in_fifo) >= s->max_period;
}

static ftype fn(get_wgain)(const ftype w)
{
    const ftype x = w*F(2.0)-F(1.0);
    return F(9.0/16.0)*FSIN(x*F(M_PI_2))+F(1.0/16.0)*FSIN(F(3.0)*x*F(M_PI_2));
}

static ftype fn(get_gain)(const ftype w, const ftype c)
{
    const ftype a = fn(get_wgain)(w);
    const ftype b = F(1.0)+c;

    return SQRT(F(0.5)/b-(F(1.0)-c)*a*a/b)+a;
}

static ctype fn(lmean)(const ftype *x, const ftype *y, const int N)
{
    ftype xm = F(0.0), ym = F(0.0);
    ctype m;

    for (int n = 0; n < N; n++) {
        xm += x[n];
        ym += y[n];
    }

    xm /= N;
    ym /= N;

    m.re = xm;
    m.im = ym;

    return m;
}

static ftype fn(l2norm)(const ftype *x, const ftype xm, const int N)
{
    ftype y = F(0.0);

    for (int n = 0; n < N; n++) {
        const ftype xn = x[n] - xm;

        y += xn*xn;
    }

    return SQRT(y);
}

static ftype fn(l2norm2)(const ftype *x, const ftype *y,
                         const ftype xm, const ftype ym,
                         const int N)
{
    ftype r = F(0.0);

    for (int n = 0; n < N; n++) {
        const ftype xn = x[n] - xm;
        const ftype yn = y[n] - ym;

        r += xn*yn;
    }

    return r;
}

static int fn(expand_write)(AVFilterContext *ctx, const int ch)
{
    AScaleContext *s = ctx->priv;
    const ftype fs = F(1.0)/ctx->inputs[0]->sample_rate;
    ChannelContext *c = &s->c[ch];
    const int best_period = c->best_period+1;
    const ftype best_score = c->best_score;
    const int max_period = s->max_period;
    const int best_max_period = c->best_max_period;
    const int n = max_period-best_period;
    ftype *dptrx = c->data[0];
    ftype *dptry = c->data[1];
    void *datax[1] = { (void *)c->data[0] };
    void *datay[1] = { (void *)c->data[1] };
    const ctype mean = fn(lmean)(dptrx+n, dptry, best_period);
    const ftype xx = fn(l2norm)(dptrx+n, mean.re, best_period);
    const ftype yy = fn(l2norm)(dptry, mean.im, best_period);
    const ftype xy = fn(l2norm2)(dptrx+n, dptry, mean.re, mean.im, best_period);
    ftype mean_xcorr, best_xcorr, scale;
    const ftype num = xy;
    const ftype den = xx * yy + EPS;

    best_xcorr = num/den;
    mean_xcorr = (mean.re * mean.im) / SQRT(mean.re * mean.re + mean.im * mean.im + EPS);
    best_xcorr = CLIP(FABS(best_xcorr), F(0.0), F(1.0));
    mean_xcorr = CLIP(FABS(mean_xcorr), F(0.0), F(1.0));

    av_log(ctx, AV_LOG_DEBUG, "E: [%d] %g/%g %d/%d\n", ch, best_xcorr, best_score, best_period, best_max_period);

    dptrx += n;
    datax[0] = dptrx;

    scale = F(1.0) / best_period;
    for (int n = 0; n < best_period; n++) {
        const ftype xf = n*scale;
        const ftype yf = F(1.0)-xf;
        const ftype axf = fn(get_gain)(xf, best_xcorr);
        const ftype ayf = fn(get_gain)(yf, best_xcorr);
        const ftype mxf = fn(get_gain)(xf, mean_xcorr);
        const ftype myf = fn(get_gain)(yf, mean_xcorr);
        const ftype x = dptrx[n] - mean.re;
        const ftype y = dptry[n] - mean.im;

        dptrx[n] = x * axf + y * ayf + mxf * mean.re + myf * mean.im;
    }

    av_audio_fifo_write(c->out_fifo, datax, best_period);
    av_audio_fifo_write(c->out_fifo, datay, best_period);
    c->state[OUT] += best_period*2*fs;
    av_audio_fifo_drain(c->in_fifo, best_period);
    return av_audio_fifo_size(c->in_fifo) >= max_period;
}

static int fn(drain_samples)(AVFilterContext *ctx, ChannelContext *c, const ftype fs)
{
    void *datax[1] = { (void *)c->data[0] };
    int size;

    size = av_audio_fifo_size(c->in_fifo);
    size = av_audio_fifo_read(c->in_fifo, datax, size);
    size = av_audio_fifo_write(c->out_fifo, datax, size);
    c->state[OUT] += size*fs;
    av_audio_fifo_drain(c->in_fifo, size);

    return 0;
}

static int fn(expand_samples)(AVFilterContext *ctx, const int ch)
{
    const ftype fs = F(1.0)/ctx->inputs[0]->sample_rate;
    AScaleContext *s = ctx->priv;
    const int max_period = s->max_period;
    int best_max_period = -1, best_period = -1;
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

        if (!s->eof && av_audio_fifo_size(c->in_fifo) < max_period)
            return 0;
    }

    if (s->eof && av_audio_fifo_size(c->in_fifo) < max_period)
        return fn(drain_samples)(ctx, c, fs);

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

    for (int i = MAX_STATES-1; i >= 0; i--) {
        const int cur_max_size = max_size >> i;
        int cur_max_period = max_period >> i;
        int ns = cur_max_period;

        memset(rptrx+cur_max_period, 0, (max_size+2-cur_max_period) * sizeof(*rptrx));
        for (int n = 0; n < cur_max_period; n++)
            rptrx[n] = dptrx[max_period-n-1];

        memset(rptry+cur_max_period, 0, (max_size+2-cur_max_period) * sizeof(*rptry));
        memcpy(rptry, dptry, cur_max_period * sizeof(*rptry));

        c->r2c_fn[i](c->r2c[i], cptrx, rptrx, sizeof(*rptrx));
        c->r2c_fn[i](c->r2c[i], cptry, rptry, sizeof(*rptry));

        cptrx[0].re = cptrx[0].im = cptry[0].re = cptry[0].im = F(0.0);
        for (int n = 0; n < cur_max_size/2+1; n++) {
            const ftype re0 = cptrx[n].re;
            const ftype im0 = cptrx[n].im;
            const ftype re1 = cptry[n].re;
            const ftype im1 = cptry[n].im;

            cptrx[n].re = re0*re1 - im1*im0;
            cptrx[n].im = im0*re1 + im1*re0;
        }

        c->c2r_fn[i](c->c2r[i], rptrx, cptrx, sizeof(*cptrx));

        for (int n = 1; n < cur_max_period-1; n++) {
            if (rptrx[n] < rptrx[n-1] &&
                rptrx[n] < rptrx[n+1]) {
                ns = n;
                break;
            }
        }

        for (int n = ns; n < cur_max_period-1; n++) {
            if (rptrx[n] > rptrx[n-1] &&
                rptrx[n] > rptrx[n+1]) {
                const ftype score = rptrx[n];

                if (score > best_score) {
                    best_score = score;
                    best_period = n;
                    best_max_period = cur_max_period;
                }
            }
        }

        if (best_period > 0)
            break;
    }

    if (best_period <= 0) {
        best_period = max_period/2;
        best_max_period = max_period;
    }

    c->best_max_period = best_max_period;
    c->best_period = best_period;
    c->best_score = best_score;
    c->mode = EXPAND;

    return fn(expand_write)(ctx, ch);
}

static int fn(compress_write)(AVFilterContext *ctx, const int ch)
{
    const ftype fs = F(1.0)/ctx->inputs[0]->sample_rate;
    AScaleContext *s = ctx->priv;
    ChannelContext *c = &s->c[ch];
    const int best_period = c->best_period+1;
    const ftype best_score = c->best_score;
    const int max_period = s->max_period;
    const int best_max_period = c->best_max_period;
    const int n = best_period;
    ftype *dptrx = c->data[0];
    ftype *dptry = c->data[0];
    void *datax[1] = { (void *)c->data[0] };
    const ctype mean = fn(lmean)(dptrx, dptry+n, best_period);
    const ftype xx = fn(l2norm)(dptrx, mean.re, best_period);
    const ftype yy = fn(l2norm)(dptry+n, mean.im, best_period);
    const ftype xy = fn(l2norm2)(dptrx, dptry+n, mean.re, mean.im, best_period);
    ftype mean_xcorr, best_xcorr, scale;
    const ftype num = xy;
    const ftype den = xx * yy + EPS;

    best_xcorr = num/den;
    mean_xcorr = (mean.re * mean.im) / SQRT(mean.re * mean.re + mean.im * mean.im + EPS);
    best_xcorr = CLIP(FABS(best_xcorr), F(0.0), F(1.0));
    mean_xcorr = CLIP(FABS(mean_xcorr), F(0.0), F(1.0));

    av_log(ctx, AV_LOG_DEBUG, "C: [%d] %g/%g %d/%d\n", ch, best_xcorr, best_score, best_period, best_max_period);

    scale = F(1.0) / best_period;
    for (int n = 0; n < best_period; n++) {
        const ftype yf = n*scale;
        const ftype xf = F(1.0)-yf;
        const ftype axf = fn(get_gain)(xf, best_xcorr);
        const ftype ayf = fn(get_gain)(yf, best_xcorr);
        const ftype mxf = fn(get_gain)(xf, mean_xcorr);
        const ftype myf = fn(get_gain)(yf, mean_xcorr);
        const ftype x = dptrx[n] - mean.re;
        const ftype y = dptry[n+best_period] - mean.im;

        dptrx[n] = x * axf + y * ayf + mxf * mean.re + myf * mean.im;
    }

    av_audio_fifo_write(c->out_fifo, datax, best_period);

    c->state[OUT] += best_period*fs;
    av_audio_fifo_drain(c->in_fifo, best_period*2);
    return av_audio_fifo_size(c->in_fifo) >= max_period*2;
}

static int fn(compress_samples)(AVFilterContext *ctx, const int ch)
{
    const ftype fs = F(1.0)/ctx->inputs[0]->sample_rate;
    AScaleContext *s = ctx->priv;
    const int max_period = s->max_period;
    int best_max_period = -1, best_period = -1;
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
    int size;

    if (av_audio_fifo_size(c->in_fifo) <= 0)
        return 0;

    if (!s->eof && av_audio_fifo_size(c->in_fifo) < max_period*2)
        return 0;

    if (s->eof && av_audio_fifo_size(c->in_fifo) < max_period*2)
        return fn(drain_samples)(ctx, c, fs);

    size = av_audio_fifo_peek(c->in_fifo, datax, max_period*2);
    if (size < 0)
        size = 0;
    if (size < max_period)
        memset(dptrx+size, 0, (max_period-size)*sizeof(*dptrx));

    for (int i = MAX_STATES-1; i >= 0; i--) {
        const int cur_max_size = max_size >> i;
        int cur_max_period = max_period >> i;
        int ns = cur_max_period;

        memset(rptrx+cur_max_period, 0, (max_size+2-cur_max_period) * sizeof(*rptrx));
        for (int n = 0; n < cur_max_period; n++)
            rptrx[n] = dptrx[cur_max_period-n-1];

        memset(rptry+cur_max_period, 0, (max_size+2-cur_max_period) * sizeof(*rptry));
        memcpy(rptry, dptry + cur_max_period, cur_max_period * sizeof(*rptry));

        c->r2c_fn[i](c->r2c[i], cptrx, rptrx, sizeof(*rptrx));
        c->r2c_fn[i](c->r2c[i], cptry, rptry, sizeof(*rptry));

        cptrx[0].re = cptrx[0].im = cptry[0].re = cptry[0].im = F(0.0);
        for (int n = 0; n < cur_max_size/2+1; n++) {
            const ftype re0 = cptrx[n].re;
            const ftype im0 = cptrx[n].im;
            const ftype re1 = cptry[n].re;
            const ftype im1 = cptry[n].im;

            cptrx[n].re = re0*re1 - im1*im0;
            cptrx[n].im = im0*re1 + im1*re0;
        }

        c->c2r_fn[i](c->c2r[i], rptrx, cptrx, sizeof(*cptrx));

        for (int n = 1; n < cur_max_period-1; n++) {
            if (rptrx[n] < rptrx[n-1] &&
                rptrx[n] < rptrx[n+1]) {
                ns = n;
                break;
            }
        }

        for (int n = ns; n < cur_max_period-1; n++) {
            if (rptrx[n] > rptrx[n-1] &&
                rptrx[n] > rptrx[n+1]) {
                const ftype score = rptrx[n] / FABS(rptrx[0] + EPS);

                if (score > best_score) {
                    best_score = score;
                    best_period = n;
                    best_max_period = cur_max_period;
                }
            }
        }

        if (best_period > 0)
            break;
    }

    if (best_period <= 0) {
        best_period = max_period/2;
        best_max_period = max_period;
    }

    c->best_max_period = best_max_period;
    c->best_period = best_period;
    c->best_score = best_score;
    c->mode = COMPRESS;

    return fn(compress_write)(ctx, ch);
}

static int fn(filter_samples)(AVFilterContext *ctx, const int ch)
{
    const ftype fs = ctx->inputs[0]->sample_rate;
    AScaleContext *s = ctx->priv;
    ChannelContext *c = &s->c[ch];
    const double state = c->state[OUT] * s->tempo - c->state[IN];

    c->mode = COPY;

    if (s->tempo == 1.0 || ff_filter_disabled(ctx))
        return fn(copy_samples)(ctx, ch, s->max_period);
    else if (s->tempo <= 0.5 || (state < 0.0 && s->tempo < 1.0))
        return fn(expand_samples)(ctx, ch);
    else if (s->tempo >= 2.0 || (state > 0.0 && s->tempo > 1.0))
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

        for (int i = 0; i < MAX_STATES; i++) {
            ftype iscale = F(1.0) / (s->max_size >> i);
            ftype scale = F(1.0);

            ret = av_tx_init(&c->r2c[i], &c->r2c_fn[i],
                             TX_TYPE, 0, s->max_size >> i, &scale, 0);
            if (ret < 0)
                return ret;

            ret = av_tx_init(&c->c2r[i], &c->c2r_fn[i],
                             TX_TYPE, 1, s->max_size >> i, &iscale, 0);
            if (ret < 0)
                return ret;
        }
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

        av_log(ctx, AV_LOG_DEBUG, "U: [%d] out: %d | in: %d\n", ch,
               av_audio_fifo_size(c->out_fifo),
               av_audio_fifo_size(c->in_fifo));

        av_audio_fifo_free(c->out_fifo);
        c->out_fifo = NULL;
        av_audio_fifo_free(c->in_fifo);
        c->in_fifo = NULL;

        for (int i = 0; i < MAX_STATES; i++) {
            av_tx_uninit(&c->r2c[i]);
            av_tx_uninit(&c->c2r[i]);
        }
    }

    av_freep(&s->c);
}
