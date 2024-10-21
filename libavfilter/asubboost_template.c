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

#undef FABS
#undef CLIP
#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define FABS fabsf
#define CLIP av_clipf
#define ftype float
#define SAMPLE_FORMAT fltp
#else
#define FABS fabs
#define CLIP av_clipd
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    ASubBoostContext *s = ctx->priv;
    const int disabled = ff_filter_disabled(ctx);
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const ftype wet = disabled ? F(1.0) : s->wet_gain;
    const ftype dry = disabled ? F(1.0) : s->dry_gain;
    const ftype feedback = s->feedback;
    const ftype decay = s->decay;
    const ftype max_boost = s->max_boost;
    const ftype b0 = s->b0;
    const ftype b1 = s->b1;
    const ftype b2 = s->b2;
    const ftype a1 = -s->a1;
    const ftype a2 = -s->a2;
    const int start = (in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int buffer_samples = s->buffer_samples;
    const int nb_samples = in->nb_samples;
    const ftype a = s->attack;
    const ftype b = F(1.0) - a;
    const ftype c = s->release;
    const ftype d = F(1.0) - c;
    const ftype fade = disabled ? b : -b;

    for (int ch = start; ch < end; ch++) {
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        ftype *buffer = (ftype *)s->buffer->extended_data[ch];
        ftype *w = (ftype *)s->w->extended_data[ch];
        int write_pos = s->write_pos[ch];
        enum AVChannel channel = av_channel_layout_channel_from_index(&in->ch_layout, ch);
        const int bypass = av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;

        if (bypass) {
            if (in != out)
                memcpy(dst, in->extended_data[ch], nb_samples * sizeof(*dst));
            continue;
        }

        for (int n = 0; n < nb_samples; n++) {
            ftype out_sample, boost;

            out_sample = src[n] * b0 + w[0];
            w[0] = b1 * src[n] + w[1] + a1 * out_sample;
            w[1] = b2 * src[n] + a2 * out_sample;

            buffer[write_pos] = buffer[write_pos] * decay + out_sample * feedback;
            boost = CLIP((F(0.9) - FABS(src[n] * dry)) / FABS(buffer[write_pos]), F(0.0), max_boost);
            w[2] = (boost > w[2]) ? w[2] * a + b * boost : w[2] * c + d * boost;
            w[2] *= (F(1.0) - w[3]);
            w[2] = CLIP(w[2], F(0.0), max_boost);
            dst[n] = (src[n] * dry + w[2] * buffer[write_pos]) * wet;

            if (++write_pos >= buffer_samples)
                write_pos = 0;
            w[3] += fade;
            w[3] = CLIP(w[3], F(0.0), F(1.0));
        }

        w[0] = isnormal(w[0]) ? w[0] : F(0.0);
        w[1] = isnormal(w[1]) ? w[1] : F(0.0);
        w[2] = isnormal(w[2]) ? w[2] : F(0.0);

        s->write_pos[ch] = write_pos;
    }

    return 0;
}
