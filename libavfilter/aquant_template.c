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
#undef ROUND
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ROUND roundf
#define SAMPLE_FORMAT float
#define ftype float
#else
#define ROUND round
#define SAMPLE_FORMAT double
#define ftype double
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(shaper_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    const int is_disabled = ff_filter_disabled(ctx);
    AudioQuantContext *s = ctx->priv;
    AVFrame **frames = s->frame;
    AVFrame **outs = arg;
    AVFrame *out0 = outs[0];
    AVFrame *out1 = outs[1];
    const int start = (out0->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out0->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = out0->nb_samples;
    const ftype factor = 1U << (s->bits-1);
    const ftype scale = F(1.0) / factor;

    for (int c = start; c < end; c++) {
        const ftype *input = (const ftype *)frames[0]->extended_data[c];
        const ftype *filter = frames[1] ? (const ftype *)frames[1]->extended_data[c] : NULL;
        ftype *output = (ftype *)out0->extended_data[c];
        ftype *error = (ftype *)out1->extended_data[c];

        if (is_disabled) {
            for (int n = 0; n < nb_samples; n++)
                output[n] = input[n];
        } else if (filter) {
            for (int n = 0; n < nb_samples; n++) {
                const ftype wanted = input[n] - filter[n];

                output[n] = ROUND(wanted * factor) * scale;
                error[n] = output[n] - wanted;
            }
        } else {
            for (int n = 0; n < nb_samples; n++) {
                const ftype wanted = input[n];

                output[n] = ROUND(wanted * factor) * scale;
                error[n] = output[n] - wanted;
            }
        }
    }

    return 0;
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    const int is_disabled = ff_filter_disabled(ctx);
    AudioQuantContext *s = ctx->priv;
    AVFrame **frames = s->frame;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = out->nb_samples;
    const ftype factor = 1U << (s->bits-1);
    const ftype scale = F(1.0) / factor;

    for (int c = start; c < end; c++) {
        const ftype *input = (const ftype *)frames[0]->extended_data[c];
        ftype *output = (ftype *)out->extended_data[c];

        if (is_disabled) {
            for (int n = 0; n < nb_samples; n++)
                output[n] = input[n];

            continue;
        }

        for (int n = 0; n < nb_samples; n++)
            output[n] = ROUND(input[n] * factor) * scale;
    }

    return 0;
}
