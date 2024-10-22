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

static void fn(echo_samples)(AudioEchoContext *ctx, uint8_t **delayptrs,
                             uint8_t * const *src, uint8_t **dst,
                             int nb_samples, int channels)
{
    const ftype out_gain = ctx->out_gain;
    const ftype in_gain = ctx->in_gain;
    const unsigned nb_decays = ctx->nb_decays;
    const unsigned nb_echoes = ctx->nb_echoes;
    const int max_samples = ctx->max_samples;
    const float *decays = ctx->decays;
    int av_uninit(index);

    av_assert1(channels > 0); /* would corrupt delay_index */

    for (int chan = 0; chan < channels; chan++) {
        const stype *s = (stype *)src[chan];
        stype *d = (stype *)dst[chan];
        stype *dbuf = (stype *)delayptrs[chan];

        index = ctx->delay_index;
        for (int i = 0; i < nb_samples; i++, s++, d++) {
            ftype out, in;

            in = *s;
            out = in * in_gain;
            for (unsigned j = 0; j < nb_echoes; j++) {
                const int jidx = FFMIN(j, nb_decays-1);
                int ix = index + max_samples - ctx->samples[j];

                ix = MOD(ix, max_samples);
                out += dbuf[ix] * decays[jidx];
            }
            out *= out_gain;

            *d = CLIP(out);
            dbuf[index] = in;

            index = MOD(index + 1, max_samples);
        }
    }

    ctx->delay_index = index;
}
