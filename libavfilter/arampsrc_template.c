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
#define SAMPLE_FORMAT fltp
#else
#define ftype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static void fn(output_samples)(AVFilterContext *ctx, AVFrame *frame)
{
    AudioRampSourceContext *s = ctx->priv;
    const unsigned period = FFMAX3(s->nb_start, s->nb_stop, s->nb_interval);
    ftype *dst = (ftype *)frame->extended_data[0];
    const int nb_samples = frame->nb_samples;
    const int sample_rate = frame->sample_rate;
    unsigned index = s->index;
    ftype stop = s->stop[FFMIN(index, s->nb_stop-1)];
    ftype value = s->value;
    ftype step = s->step;
    ftype left = s->left;

    for (int n = 0; n < nb_samples; n++) {
        dst[n] = value;
        value += step;
        left--;

        if (left <= F(0.0)) {
            index++;
            if (index >= period)
                index = 0;

            stop = s->stop[FFMIN(index, s->nb_stop-1)];
            value = s->start[FFMIN(index, s->nb_start-1)];
            left = s->interval[FFMIN(index, s->nb_interval-1)] * sample_rate;
            step = (stop - value) / left;
        }
    }

    for (int ch = 1; ch < s->nb_channels; ch++)
        memcpy(frame->extended_data[ch], dst, nb_samples * sizeof(ftype));

    s->value = value;
    s->index = index;
    s->left = left;
    s->step = step;
}
