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
#undef EPS
#undef CLIP
#if DEPTH == 32
#define SAMPLE_FORMAT fltp
#define ftype float
#define EPS FLT_EPSILON
#define CLIP av_clipf
#else
#define SAMPLE_FORMAT dblp
#define ftype double
#define EPS FLT_EPSILON
#define CLIP av_clipd
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const int nb_channels = out->ch_layout.nb_channels;
    const int start = (nb_channels * jobnr) / nb_jobs;
    const int end = (nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = in->nb_samples;
    AudioInversionContext *s = ctx->priv;
    const ftype unity = s->unity;
    const ftype maxf = s->maxf;

    for (int ch = start; ch < end; ch++) {
        enum AVChannel channel = av_channel_layout_channel_from_index(&in->ch_layout, ch);
        const int bypass = av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];

        if (bypass) {
            if (in != out)
                memcpy(dst, src, nb_samples * sizeof(*dst));
            continue;
        }

        for (int n = 0; n < nb_samples; n++)
            dst[n] = CLIP(unity / (src[n] + EPS), -maxf, maxf);
    }

    return 0;
}
