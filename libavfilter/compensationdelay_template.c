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

// The maximum distance for options
#define COMP_DELAY_MAX_DISTANCE            F(100.0 * 100.0 + 100.0 * 1.0 + 1.0)
// The actual speed of sound in normal conditions
#define COMP_DELAY_SOUND_SPEED_KM_H(temp)  F(1.85325 * (643.95 * sqrt(((temp + 273.15) / 273.15))))
#define COMP_DELAY_SOUND_SPEED_CM_S(temp)  F(COMP_DELAY_SOUND_SPEED_KM_H(temp) * (1000.0 * 100.0) /* cm/km */ / (60.0 * 60.0) /* s/h */)
#define COMP_DELAY_SOUND_FRONT_DELAY(temp) F(1.0 / COMP_DELAY_SOUND_SPEED_CM_S(temp))
// The maximum delay may be reached by this filter
#define COMP_DELAY_MAX_DELAY               F(COMP_DELAY_MAX_DISTANCE * COMP_DELAY_SOUND_FRONT_DELAY(50))

static int fn(delay_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    CompensationDelayContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *in = td->in;
    const unsigned b_mask = s->buf_size - 1;
    const unsigned buf_size = s->buf_size;
    const int nb_channels = in->ch_layout.nb_channels;
    const int start = (nb_channels * jobnr) / nb_jobs;
    const int end = (nb_channels * (jobnr+1)) / nb_jobs;
    const int nb_samples = in->nb_samples;
    const ftype dry = s->dry;
    const ftype wet = s->wet;

    for (int ch = start; ch < end; ch++) {
        const ftype *src = (const ftype *)in->extended_data[ch];
        ftype *dst = (ftype *)out->extended_data[ch];
        ftype *buffer = (ftype *)s->delay_frame->extended_data[ch];
        const int cm_idx = FFMIN(ch, s->distance_cm_size-1);
        const int mm_idx = FFMIN(ch, s->distance_mm_size-1);
        const int m_idx  = FFMIN(ch, s->distance_m_size-1);
        const int t_idx  = FFMIN(ch, s->temp_size-1);
        unsigned delay = (s->distance_m[m_idx] * F(100.0) +
                          s->distance_cm[cm_idx] * F(1.0) +
                          s->distance_mm[mm_idx] * F(0.1)) *
            COMP_DELAY_SOUND_FRONT_DELAY(s->temp[t_idx]) * in->sample_rate;
        unsigned w_ptr, r_ptr;

        w_ptr =  s->w_ptr[ch];
        r_ptr = (w_ptr + buf_size - delay) & b_mask;

        for (int n = 0; n < nb_samples; n++) {
            const ftype sample = src[n];

            buffer[w_ptr] = sample;
            if (out != in)
                dst[n] = dry * sample + wet * buffer[r_ptr];
            w_ptr = (w_ptr + 1) & b_mask;
            r_ptr = (r_ptr + 1) & b_mask;
        }

        s->w_ptr[ch] = w_ptr;
    }

    return 0;
}
