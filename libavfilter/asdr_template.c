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
#undef FABS
#undef FEPS
#if DEPTH == 32
#define ftype float
#define SAMPLE_FORMAT fltp
#define FABS fabsf
#define FEPS FLT_EPSILON
#elif DEPTH == 64
#define ftype double
#define SAMPLE_FORMAT dblp
#define FABS fabs
#define FEPS DBL_EPSILON
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(sdr)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSDRContext *s = ctx->priv;
    AVFrame *u = s->cache[0];
    AVFrame *v = s->cache[1];
    const int channels = u->ch_layout.nb_channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = FFMIN(u->nb_samples, v->nb_samples);

    for (int ch = start; ch < end; ch++) {
        ChanStats *chs = &s->chs[ch];
        const ftype *const us = (ftype *)u->extended_data[ch];
        const ftype *const vs = (ftype *)v->extended_data[ch];
        double sum_uv = 0.;
        double sum_u = 0.;

        for (int n = 0; n < nb_samples; n++) {
            sum_u  += us[n] * us[n];
            sum_uv += (us[n] - vs[n]) * (us[n] - vs[n]);
        }

        chs->uv += sum_uv;
        chs->u  += sum_u;
    }

    return 0;
}

static int fn(sisdr)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSDRContext *s = ctx->priv;
    AVFrame *u = s->cache[0];
    AVFrame *v = s->cache[1];
    const int channels = u->ch_layout.nb_channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = FFMIN(u->nb_samples, v->nb_samples);

    for (int ch = start; ch < end; ch++) {
        ChanStats *chs = &s->chs[ch];
        const ftype *const us = (ftype *)u->extended_data[ch];
        const ftype *const vs = (ftype *)v->extended_data[ch];
        double sum_uv = 0.;
        double sum_u = 0.;
        double sum_v = 0.;

        for (int n = 0; n < nb_samples; n++) {
            sum_u  += us[n] * us[n];
            sum_v  += vs[n] * vs[n];
            sum_uv += us[n] * vs[n];
        }

        chs->uv += sum_uv;
        chs->u  += sum_u;
        chs->v  += sum_v;
    }

    return 0;
}

static int fn(psnr)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSDRContext *s = ctx->priv;
    AVFrame *u = s->cache[0];
    AVFrame *v = s->cache[1];
    const int channels = u->ch_layout.nb_channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = FFMIN(u->nb_samples, v->nb_samples);

    for (int ch = start; ch < end; ch++) {
        ChanStats *chs = &s->chs[ch];
        const ftype *const us = (ftype *)u->extended_data[ch];
        const ftype *const vs = (ftype *)v->extended_data[ch];
        double sum_uv = 0.;
        double max = 0;

        for (int n = 0; n < nb_samples; n++) {
            sum_uv += (us[n] - vs[n]) * (us[n] - vs[n]);
            max = fmax(max, us[n] * us[n]);
        }

        chs->uv += sum_uv;
        chs->u = fmax(chs->u, max);
    }

    return 0;
}

static int fn(nrmse)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSDRContext *s = ctx->priv;
    AVFrame *u = s->cache[0];
    AVFrame *v = s->cache[1];
    const int channels = u->ch_layout.nb_channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = FFMIN(u->nb_samples, v->nb_samples);
    const double current_sample = s->nb_samples + 1;

    for (int ch = start; ch < end; ch++) {
        ChanStats *chs = &s->chs[ch];
        const ftype *const us = (ftype *)u->extended_data[ch];
        const ftype *const vs = (ftype *)v->extended_data[ch];
        double uv = 0.;
        double v = chs->v;
        double u = chs->u;

        for (int n = 0; n < nb_samples; n++) {
            double ro = us[n] - v;
            uv += (us[n] - vs[n]) * (us[n] - vs[n]);
            v += us[n] / (current_sample + n);
            u += (us[n] - v) * ro;
        }

        chs->uv += uv;
        chs->u = u;
        chs->v = v;
    }

    return 0;
}

static int fn(mae)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSDRContext *s = ctx->priv;
    AVFrame *u = s->cache[0];
    AVFrame *v = s->cache[1];
    const int channels = u->ch_layout.nb_channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = FFMIN(u->nb_samples, v->nb_samples);

    for (int ch = start; ch < end; ch++) {
        ChanStats *chs = &s->chs[ch];
        const ftype *const us = (ftype *)u->extended_data[ch];
        const ftype *const vs = (ftype *)v->extended_data[ch];
        double uv = 0.;

        for (int n = 0; n < nb_samples; n++)
            uv += FABS(us[n] - vs[n]);

        chs->uv += uv;
    }

    return 0;
}

static int fn(mape)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSDRContext *s = ctx->priv;
    AVFrame *u = s->cache[0];
    AVFrame *v = s->cache[1];
    const int channels = u->ch_layout.nb_channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = FFMIN(u->nb_samples, v->nb_samples);

    for (int ch = start; ch < end; ch++) {
        ChanStats *chs = &s->chs[ch];
        const ftype *const us = (ftype *)u->extended_data[ch];
        const ftype *const vs = (ftype *)v->extended_data[ch];
        double uv = 0.;

        for (int n = 0; n < nb_samples; n++)
            uv += FABS((us[n] - vs[n]) / (us[n] + FEPS));

        chs->uv += uv;
    }

    return 0;
}

static int fn(mda)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSDRContext *s = ctx->priv;
    AVFrame *u = s->cache[0];
    AVFrame *v = s->cache[1];
    const int channels = u->ch_layout.nb_channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = FFMIN(u->nb_samples, v->nb_samples);

    for (int ch = start; ch < end; ch++) {
        ChanStats *chs = &s->chs[ch];
        const ftype *const us = (ftype *)u->extended_data[ch];
        const ftype *const vs = (ftype *)v->extended_data[ch];
        ftype u = chs->u;
        ftype v = chs->u;
        uint64_t cnt = 0;

        for (int n = 0; n < nb_samples; n++) {
            cnt += FFDIFFSIGN(us[n] - u, F(0.0)) == FFDIFFSIGN(vs[n] - v, F(0.0));

            u = us[n];
            v = vs[n];
        }

        chs->cnt += cnt;
        chs->u = u;
        chs->v = v;
    }

    return 0;
}

static int fn(identity)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioSDRContext *s = ctx->priv;
    AVFrame *u = s->cache[0];
    AVFrame *v = s->cache[1];
    const int channels = u->ch_layout.nb_channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = FFMIN(u->nb_samples, v->nb_samples);

    for (int ch = start; ch < end; ch++) {
        ChanStats *chs = &s->chs[ch];
        const ftype *const us = (ftype *)u->extended_data[ch];
        const ftype *const vs = (ftype *)v->extended_data[ch];
        unsigned cnt = 0;

        for (int n = 0; n < nb_samples; n++)
            cnt += FABS(us[n] - vs[n]) < FEPS;

        chs->cnt += cnt;
    }

    return 0;
}
