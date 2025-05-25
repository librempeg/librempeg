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

#undef stype
#undef ftype
#undef CLIP
#undef SAMPLE_FORMAT
#if DEPTH == 16
#define stype int16_t
#define ftype float
#define CLIP(x) av_clip_int16(x)
#define SAMPLE_FORMAT s16p
#elif DEPTH == 31
#define stype int32_t
#define ftype double
#define CLIP(x) av_clipl_int32(x)
#define SAMPLE_FORMAT s32p
#elif DEPTH == 32
#define stype float
#define ftype float
#define SAMPLE_FORMAT fltp
#define CLIP(x) (x)
#else
#define stype double
#define ftype double
#define SAMPLE_FORMAT dblp
#define CLIP(x) (x)
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define MOD(a, b) (((a) >= (b)) ? (a) - (b) : (a))

static int fn(echo_samples)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioEchoContext *s = ctx->priv;
    ThreadData *td = arg;
    const ftype out_gain = s->out_gain;
    const ftype in_gain = s->in_gain;
    const unsigned nb_decays = s->nb_decays;
    const unsigned nb_echoes = s->nb_echoes;
    const int start = (td->in->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (td->in->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = td->out->nb_samples;
    const int is_disabled = ff_filter_disabled(ctx);
    const int max_samples = s->max_samples;
    uint8_t **delayptrs = s->delayptrs;
    const float *decays = s->decays;
    const int *samples = s->samples;

    for (int ch = start; ch < end; ch++) {
        const stype *sample = (stype *)td->in->extended_data[ch];
        stype *d = (stype *)td->out->extended_data[ch];
        stype *dbuf = (stype *)delayptrs[ch];
        int index = s->delay_index[ch];

        for (int n = 0; n < nb_samples; n++) {
            ftype out, in;

            in = sample[n];
            out = in * in_gain;
            for (unsigned j = 0; j < nb_echoes; j++) {
                const int jidx = FFMIN(j, nb_decays-1);
                int ix = index + max_samples - samples[j];

                ix = MOD(ix, max_samples);
                out += dbuf[ix] * decays[jidx];
            }
            out *= out_gain;

            d[n] = is_disabled ? in : CLIP(out);
            dbuf[index] = in;

            index = MOD(index + 1, max_samples);
        }

        s->delay_index[ch] = index;
    }

    return 0;
}
