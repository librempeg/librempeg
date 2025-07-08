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
#undef FMOD
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define ftype float
#define FMOD modff
#define SAMPLE_FORMAT fltp
#else
#define ftype double
#define FMOD modf
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    VibratoContext *s = ctx->priv;
    AVFrame *in = s->in;
    AVFrame *out = arg;
    const int channels = s->channels;
    const int start = (channels * jobnr) / nb_jobs;
    const int end = (channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = in->nb_samples;
    const int buf_size = s->buf_size;
    const ftype depth = s->depth;
    const int wave_table_size = s->wave_table_size;
    const ftype *wave_table = s->wave_table;

    for (int ch = start; ch < end; ch++) {
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *buf = (ftype *)s->buf->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        int wave_table_index = s->wave_table_index[ch];
        int buf_index = s->buf_index[ch];
        int samp1_index, samp2_index;
        ftype integer, decimal, this_samp;

        for (int n = 0; n < nb_samples; n++) {
            decimal = FMOD(depth * wave_table[wave_table_index], &integer);

            wave_table_index++;
            if (wave_table_index >= wave_table_size)
                wave_table_index -= wave_table_size;

            samp1_index = buf_index + integer;
            if (samp1_index >= buf_size)
                samp1_index -= buf_size;
            samp2_index = samp1_index + 1;
            if (samp2_index >= buf_size)
                samp2_index -= buf_size;

            this_samp = src[n];
            dst[n] = buf[samp1_index] + (decimal * (buf[samp2_index] - buf[samp1_index]));
            buf[buf_index] = this_samp;

            buf_index++;
            if (buf_index >= buf_size)
                buf_index -= buf_size;
        }

        s->wave_table_index[ch] = wave_table_index;
        s->buf_index[ch] = buf_index;
    }

    return 0;
}
