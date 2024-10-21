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
#define SAMPLE_FORMAT float
#define ftype float
#else
#define SAMPLE_FORMAT double
#define ftype double
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static ftype fn(fir_sample)(AudioKalmanContext *s, ftype sample, ftype *delay,
                            ftype *coeffs, ftype *tmp, int *offset)
{
    const int order = s->order;
    ftype output;

    delay[*offset] = sample;

    memcpy(tmp, coeffs + order - *offset, order * sizeof(ftype));

#if DEPTH == 32
    output = s->fdsp->scalarproduct_float(delay, tmp, s->kernel_size);
#else
    output = s->fdsp->scalarproduct_double(delay, tmp, s->kernel_size);
#endif

    if (--(*offset) < 0)
        *offset = order - 1;

    return output;
}

static ftype fn(process_sample)(AudioKalmanContext *s, ftype input, ftype desired, int ch)
{
    ftype *coeffs = (ftype *)s->coeffs->extended_data[ch];
    ftype *delay = (ftype *)s->delay->extended_data[ch];
    ftype *tmp = (ftype *)s->tmp->extended_data[ch];
    ftype *Pn = (ftype *)s->Pn->extended_data[ch];
    ftype *Ku = (ftype *)s->Ku->extended_data[ch];
    ftype *IKu = (ftype *)s->IKu->extended_data[ch];
    ftype *P = (ftype *)s->P->extended_data[ch];
    ftype *r = (ftype *)s->r->extended_data[ch];
    int *offsetp = (int *)s->offset->extended_data[ch];
    const int kernel_size = s->kernel_size;
    const ftype delta = s->delta;
    const int order = s->order;
    int offset = *offsetp;
    ftype output, e, R, ur = F(0.0);

    delay[offset + order] = input;

    output = fn(fir_sample)(s, input, delay, coeffs, tmp, offsetp);
    e = desired - output;
    R = e * e + 2e-10;

    for (int m = 0; m < order; m++) {
        const int moffset = m * kernel_size;
        const ftype *Pm = P + moffset;
        ftype *Pnm = Pn + moffset;

        for (int n = 0; n < order; n++)
            Pnm[n] = Pm[n];
        Pnm[m] += delta;
    }

    for (int m = 0; m < order; m++) {
        const int moffset = m * kernel_size;
        const ftype *delayo = delay + offset;
        const ftype *Pnm = Pn + moffset;
        ftype sum = F(0.0);

        for (int n = 0; n < order; n++)
            sum += Pnm[n] * delayo[n];
        r[m] = sum;
    }

    for (int n = 0; n < order; n++)
        ur += delay[offset + n] * r[n];

    ur += R;
    ur = F(1.0) / ur;
    for (int n = 0; n < order; n++)
        r[n] *= ur;

    for (int n = 0; n < order; n++)
        coeffs[n] = coeffs[order + n] = coeffs[n] + r[n] * e;

    for (int m = 0; m < order; m++) {
        const ftype *delayo = delay + offset;
        const int moffset = m * kernel_size;
        ftype *Kum = Ku + moffset;
        const ftype rm = r[m];

        for (int n = 0; n < order; n++)
            Kum[n] = rm * delayo[n];
    }

    for (int m = 0; m < order; m++) {
        const int moffset = m * kernel_size;
        const ftype *Kum = Ku + moffset;
        ftype *IKum = IKu + moffset;

        for (int n = 0; n < order; n++)
            IKum[n] = F(1.0) * (m == n) - Kum[n];
    }

    for (int m = 0; m < order; m++) {
        const int moffset = m * kernel_size;
        const ftype *IKum = IKu + moffset;
        ftype *Pm = P + moffset;

        for (int n = 0; n < order; n++) {
            const ftype *Pnn = Pn + n;
            ftype sum = F(0.0);

            for (int k = 0; k < order; k++, Pnn += kernel_size)
                sum += IKum[k] * Pnn[0];
            Pm[n] = sum;
        }
    }

    switch (s->output_mode) {
    case IN_MODE:       output = input;         break;
    case DESIRED_MODE:  output = desired;       break;
    case OUT_MODE:   output = desired - output; break;
    case NOISE_MODE: output = input - output;   break;
    case ERROR_MODE:                            break;
    }
    return output;
}

static int fn(filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    AudioKalmanContext *s = ctx->priv;
    AVFrame *out = arg;
    const int start = (out->ch_layout.nb_channels * jobnr) / nb_jobs;
    const int end = (out->ch_layout.nb_channels * (jobnr+1)) / nb_jobs;

    for (int c = start; c < end; c++) {
        const ftype *input = (const ftype *)s->frame[0]->extended_data[c];
        const ftype *desired = (const ftype *)s->frame[1]->extended_data[c];
        ftype *output = (ftype *)out->extended_data[c];

        for (int n = 0; n < out->nb_samples; n++) {
            output[n] = fn(process_sample)(s, input[n], desired[n], c);
            if (ff_filter_disabled(ctx))
                output[n] = input[n];
        }
    }

    return 0;
}
