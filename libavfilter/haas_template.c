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

static int fn(do_update)(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    HaasContext *s = ctx->priv;

    s->delay[0] = lrint(s->par_delay[0] * F(0.001) * inlink->sample_rate);
    s->delay[1] = lrint(s->par_delay[1] * F(0.001) * inlink->sample_rate);

    s->phase[0] = s->par_phase[0] ? F(1.0) : F(-1.0);
    s->phase[1] = s->par_phase[1] ? F(1.0) : F(-1.0);

    s->balance_l[0] = (s->par_balance[0] + 1) / 2 * s->par_gain[0] * s->phase[0];
    s->balance_r[0] = (F(1.0) - (s->par_balance[0] + 1) / 2) * (s->par_gain[0]) * s->phase[0];
    s->balance_l[1] = (s->par_balance[1] + 1) / 2 * s->par_gain[1] * s->phase[1];
    s->balance_r[1] = (F(1.0) - (s->par_balance[1] + 1) / 2) * (s->par_gain[1]) * s->phase[1];

    return 0;
}

static void fn(do_haas)(AVFilterContext *ctx, AVFrame *out, const AVFrame *in)
{
    HaasContext *s = ctx->priv;
    const ftype *srcl = (const ftype *)in->extended_data[0];
    const ftype *srcr = (const ftype *)in->extended_data[1];
    const ftype balance_l[2] = { s->balance_l[0], s->balance_l[1] };
    const ftype balance_r[2] = { s->balance_r[0], s->balance_r[1] };
    const int par_middle_phase = s->par_middle_phase;
    ftype *dstl = (ftype *)out->extended_data[0];
    ftype *dstr = (ftype *)out->extended_data[1];
    const ftype par_side_gain = s->par_side_gain;
    const size_t buffer_size = s->buffer_size;
    const int is_disabled = ff_filter_disabled(ctx);
    const int par_m_source = s->par_m_source;
    const int nb_samples = in->nb_samples;
    const ftype level_out = s->level_out;
    const size_t mask = buffer_size - 1;
    const ftype level_in = s->level_in;
    const size_t *delay = s->delay;
    int write_ptr = s->write_ptr;
    ftype *buffer = s->buffer;

    for (int n = 0; n < nb_samples; n++) {
        ftype mid, side[2], sidep[2];
        size_t s_ptr[2];

        switch (par_m_source) {
        case 0: mid = srcl[n]; break;
        case 1: mid = srcr[n]; break;
        case 2: mid = (srcl[n] + srcr[n]) * F(0.5); break;
        case 3: mid = (srcl[n] - srcr[n]) * F(0.5); break;
        }

        mid *= level_in;

        buffer[write_ptr] = mid;

        s_ptr[0] = (write_ptr + buffer_size - delay[0]) & mask;
        s_ptr[1] = (write_ptr + buffer_size - delay[1]) & mask;

        if (par_middle_phase)
            mid = -mid;

        side[0] = buffer[s_ptr[0]] * par_side_gain;
        side[1] = buffer[s_ptr[1]] * par_side_gain;
        sidep[0] = side[0] * balance_l[0] - side[1] * balance_l[1];
        sidep[1] = side[1] * balance_r[1] - side[0] * balance_r[0];

        if (is_disabled) {
            dstl[n] = srcl[n];
            dstr[n] = srcr[n];
        } else {
            dstl[n] = (mid + sidep[0]) * level_out;
            dstr[n] = (mid + sidep[1]) * level_out;
        }

        write_ptr = (write_ptr + 1) & mask;
    }

    s->write_ptr = write_ptr;
}
