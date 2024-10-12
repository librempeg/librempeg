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

#undef ctype
#undef ftype
#undef FABS
#undef FEXP
#undef SAMPLE_FORMAT
#undef TX_TYPE
#if DEPTH == 32
#define ctype AVComplexFloat
#define ftype float
#define FABS fabsf
#define FEXP expf
#define SAMPLE_FORMAT flt
#define TX_TYPE AV_TX_FLOAT_FFT
#else
#define ctype AVComplexDouble
#define ftype double
#define FABS fabs
#define FEXP exp
#define SAMPLE_FORMAT dbl
#define TX_TYPE AV_TX_DOUBLE_FFT
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

static int fn(headphone_convolute)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    HeadphoneContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *in = td->in, *out = td->out;
    int offset = jobnr;
    int *write = &s->write[jobnr];
    const ftype *const ir = s->data_ir[jobnr];
    int *n_clippings = &td->n_clippings[jobnr];
    ftype *ringbuffer = s->ringbuffer[jobnr];
    ftype *temp_src = s->temp_src[jobnr];
    const int ir_len = s->ir_len;
    const int air_len = s->air_len;
    const ftype *src = (const ftype *)in->data[0];
    ftype *dst = (ftype *)out->data[0];
    const int in_channels = in->ch_layout.nb_channels;
    const int buffer_length = s->buffer_length;
    const uint32_t modulo = (uint32_t)buffer_length - 1;
    const int nb_samples = in->nb_samples;
    const ftype gain_lfe = s->gain_lfe;
    ftype *buffer[64];
    int wr = *write;
    int read;
    int i, l;

    dst += offset;
    for (l = 0; l < in_channels; l++) {
        buffer[l] = ringbuffer + l * buffer_length;
    }

    for (i = 0; i < nb_samples; i++) {
        const ftype *cur_ir = ir;

        *dst = 0;
        for (l = 0; l < in_channels; l++) {
            *(buffer[l] + wr) = src[l];
        }

        for (l = 0; l < in_channels; cur_ir += air_len, l++) {
            const ftype *const bptr = buffer[l];

            if (l == s->lfe_channel) {
                *dst += *(buffer[s->lfe_channel] + wr) * gain_lfe;
                continue;
            }

            read = (wr - (ir_len - 1)) & modulo;

            if (read + ir_len < buffer_length) {
                memcpy(temp_src, bptr + read, ir_len * sizeof(*temp_src));
            } else {
                int len = FFMIN(air_len - (read % ir_len), buffer_length - read);

                memcpy(temp_src, bptr + read, len * sizeof(*temp_src));
                memcpy(temp_src + len, bptr, (air_len - len) * sizeof(*temp_src));
            }

#if DEPTH == 32
            dst[0] += s->scalarproduct_flt(cur_ir, temp_src, FFALIGN(ir_len, 32));
#else
            dst[0] += s->scalarproduct_dbl(cur_ir, temp_src, FFALIGN(ir_len, 32));
#endif
        }

        if (FABS(dst[0]) > 1)
            n_clippings[0]++;

        dst += 2;
        src += in_channels;
        wr   = (wr + 1) & modulo;
    }

    *write = wr;

    return 0;
}

static int fn(headphone_fast_convolute)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    HeadphoneContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *in = td->in, *out = td->out;
    int offset = jobnr;
    int *write = &s->write[jobnr];
    ctype *hrtf = s->data_hrtf[jobnr];
    int *n_clippings = &td->n_clippings[jobnr];
    const int nb_samples = in->nb_samples;
    ftype *ringbuffer = s->ringbuffer[jobnr];
    const int ir_len = s->ir_len;
    const ftype *src = (const ftype *)in->data[0];
    ftype *dst = (ftype *)out->data[0];
    const int in_channels = in->ch_layout.nb_channels;
    const int buffer_length = s->buffer_length;
    const uint32_t modulo = (uint32_t)buffer_length - 1;
    ctype *fft_out = s->out_fft[jobnr];
    ctype *fft_in = s->in_fft[jobnr];
    ctype *fft_acc = s->temp_afft[jobnr];
    AVTXContext *ifft = s->ifft[jobnr];
    AVTXContext *fft = s->fft[jobnr];
    av_tx_fn tx_fn = s->tx_fn[jobnr];
    av_tx_fn itx_fn = s->itx_fn[jobnr];
    const int n_fft = s->n_fft;
    const ftype fft_scale = F(1.0) / s->n_fft;
    const ftype gain_lfe = s->gain_lfe;
    ctype *hrtf_offset;
    int wr = *write;
    int n_read;
    int i, j;

    dst += offset;

    n_read = FFMIN(ir_len, nb_samples);
    for (j = 0; j < n_read; j++) {
        dst[2 * j]     = ringbuffer[wr];
        ringbuffer[wr] = F(0.0);
        wr  = (wr + 1) & modulo;
    }

    for (j = n_read; j < nb_samples; j++)
        dst[2 * j] = F(0.0);

    memset(fft_acc, 0, sizeof(ctype) * n_fft);

    for (i = 0; i < in_channels; i++) {
        if (i == s->lfe_channel) {
            for (j = 0; j < nb_samples; j++)
                dst[2 * j] += src[i + j * in_channels] * gain_lfe;
            continue;
        }

        offset = i * n_fft;
        hrtf_offset = hrtf + s->hrir_map[i] * n_fft;

        memset(fft_in, 0, sizeof(ctype) * n_fft);

        for (j = 0; j < nb_samples; j++)
            fft_in[j].re = src[j * in_channels + i];

        tx_fn(fft, fft_out, fft_in, sizeof(*fft_in));

        for (j = 0; j < n_fft; j++) {
            const ctype *hcomplex = hrtf_offset + j;
            const ftype re = fft_out[j].re;
            const ftype im = fft_out[j].im;

            fft_acc[j].re += re * hcomplex->re - im * hcomplex->im;
            fft_acc[j].im += re * hcomplex->im + im * hcomplex->re;
        }
    }

    itx_fn(ifft, fft_out, fft_acc, sizeof(*fft_acc));

    for (j = 0; j < nb_samples; j++) {
        dst[2 * j] += fft_out[j].re * fft_scale;
        if (FABS(dst[2 * j]) > 1)
            n_clippings[0]++;
    }

    for (j = 0; j < ir_len - 1; j++) {
        int write_pos = (wr + j) & modulo;

        *(ringbuffer + write_pos) += fft_out[nb_samples + j].re * fft_scale;
    }

    *write = wr;

    return 0;
}

static int fn(convert_coeffs)(AVFilterContext *ctx, AVFilterLink *inlink)
{
    struct HeadphoneContext *s = ctx->priv;
    const int ir_len = s->ir_len;
    int nb_input_channels = ctx->inputs[0]->ch_layout.nb_channels;
    const int nb_hrir_channels = s->nb_hrir_inputs == 1 ? ctx->inputs[1]->ch_layout.nb_channels : s->nb_hrir_inputs * 2;
    ftype gain_lin = FEXP((s->gain - 3 * nb_input_channels) / 20 * M_LN10);
    AVFrame *frame;
    int ret = 0;
    int n_fft;
    int i, j, k;

    s->air_len = 1 << (32 - ff_clz(ir_len));
    if (s->type == TIME_DOMAIN) {
        s->air_len = FFALIGN(s->air_len, 32);
    }
    s->buffer_length = 1 << (32 - ff_clz(s->air_len));
    s->n_fft = n_fft = 1 << (32 - ff_clz(ir_len + s->size));

    if (s->type == FREQUENCY_DOMAIN) {
        ftype scale = F(1.0);

        ret = av_tx_init(&s->fft[0], &s->tx_fn[0], TX_TYPE, 0, s->n_fft, &scale, 0);
        if (ret < 0)
            goto fail;
        ret = av_tx_init(&s->fft[1], &s->tx_fn[1], TX_TYPE, 0, s->n_fft, &scale, 0);
        if (ret < 0)
            goto fail;
        ret = av_tx_init(&s->ifft[0], &s->itx_fn[0], TX_TYPE, 1, s->n_fft, &scale, 0);
        if (ret < 0)
            goto fail;
        ret = av_tx_init(&s->ifft[1], &s->itx_fn[1], TX_TYPE, 1, s->n_fft, &scale, 0);
        if (ret < 0)
            goto fail;

        if (!s->fft[0] || !s->fft[1] || !s->ifft[0] || !s->ifft[1]) {
            av_log(ctx, AV_LOG_ERROR, "Unable to create FFT contexts of size %d.\n", s->n_fft);
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }

    if (s->type == TIME_DOMAIN) {
        s->ringbuffer[0] = av_calloc(s->buffer_length, sizeof(ftype) * nb_input_channels);
        s->ringbuffer[1] = av_calloc(s->buffer_length, sizeof(ftype) * nb_input_channels);
    } else {
        s->ringbuffer[0] = av_calloc(s->buffer_length, sizeof(ftype));
        s->ringbuffer[1] = av_calloc(s->buffer_length, sizeof(ftype));
        s->out_fft[0] = av_calloc(s->n_fft, sizeof(ctype));
        s->out_fft[1] = av_calloc(s->n_fft, sizeof(ctype));
        s->in_fft[0] = av_calloc(s->n_fft, sizeof(ctype));
        s->in_fft[1] = av_calloc(s->n_fft, sizeof(ctype));
        s->temp_afft[0] = av_calloc(s->n_fft, sizeof(ctype));
        s->temp_afft[1] = av_calloc(s->n_fft, sizeof(ctype));
        if (!s->in_fft[0] || !s->in_fft[1] ||
            !s->out_fft[0] || !s->out_fft[1] ||
            !s->temp_afft[0] || !s->temp_afft[1]) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }

    if (!s->ringbuffer[0] || !s->ringbuffer[1]) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    if (s->type == TIME_DOMAIN) {
        s->temp_src[0] = av_calloc(s->air_len, sizeof(ftype));
        s->temp_src[1] = av_calloc(s->air_len, sizeof(ftype));

        s->data_ir[0] = av_calloc(nb_hrir_channels * s->air_len, sizeof(ftype));
        s->data_ir[1] = av_calloc(nb_hrir_channels * s->air_len, sizeof(ftype));
        if (!s->data_ir[0] || !s->data_ir[1] || !s->temp_src[0] || !s->temp_src[1]) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    } else {
        s->data_hrtf[0] = av_calloc(n_fft, sizeof(ctype) * nb_hrir_channels);
        s->data_hrtf[1] = av_calloc(n_fft, sizeof(ctype) * nb_hrir_channels);
        if (!s->data_hrtf[0] || !s->data_hrtf[1]) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }

    for (i = 0; i < s->nb_hrir_inputs; av_frame_free(&frame), i++) {
        ctype *data_hrtf[2] = { s->data_hrtf[0], s->data_hrtf[1] };
        ftype *data_ir[2] = { s->data_ir[0], s->data_ir[1] };
        int len = s->hrir_in[i].ir_len;
        ftype *ptr;

        ret = ff_inlink_consume_samples(ctx->inputs[i + 1], len, len, &frame);
        if (ret < 0)
            goto fail;
        ptr = (ftype *)frame->extended_data[0];

        if (s->hrir_fmt == HRIR_STEREO) {
            int idx = av_channel_layout_index_from_channel(&s->map_channel_layout,
                                                          s->mapping[i]);
            if (idx < 0)
                continue;

            s->hrir_map[i] = idx;
            if (s->type == TIME_DOMAIN) {
                ftype *data_ir_l = data_ir[0] + idx * s->air_len;
                ftype *data_ir_r = data_ir[1] + idx * s->air_len;

                for (j = 0; j < len; j++) {
                    data_ir_l[j] = ptr[len * 2 - j * 2 - 2] * gain_lin;
                    data_ir_r[j] = ptr[len * 2 - j * 2 - 1] * gain_lin;
                }
            } else {
                ctype *fft_out_l = data_hrtf[0] + idx * n_fft;
                ctype *fft_out_r = data_hrtf[1] + idx * n_fft;
                ctype *fft_in_l = s->in_fft[0];
                ctype *fft_in_r = s->in_fft[1];

                for (j = 0; j < len; j++) {
                    fft_in_l[j].re = ptr[j * 2    ] * gain_lin;
                    fft_in_r[j].re = ptr[j * 2 + 1] * gain_lin;
                }

                s->tx_fn[0](s->fft[0], fft_out_l, fft_in_l, sizeof(*fft_in_l));
                s->tx_fn[0](s->fft[0], fft_out_r, fft_in_r, sizeof(*fft_in_r));
            }
        } else {
            int I, N = ctx->inputs[1]->ch_layout.nb_channels;

            for (k = 0; k < N / 2; k++) {
                int idx = av_channel_layout_index_from_channel(&inlink->ch_layout,
                                                              s->mapping[k]);
                if (idx < 0)
                    continue;

                s->hrir_map[k] = idx;
                I = k * 2;
                if (s->type == TIME_DOMAIN) {
                    ftype *data_ir_l = data_ir[0] + idx * s->air_len;
                    ftype *data_ir_r = data_ir[1] + idx * s->air_len;

                    for (j = 0; j < len; j++) {
                        data_ir_l[j] = ptr[len * N - j * N - N + I    ] * gain_lin;
                        data_ir_r[j] = ptr[len * N - j * N - N + I + 1] * gain_lin;
                    }
                } else {
                    ctype *fft_out_l = data_hrtf[0] + idx * n_fft;
                    ctype *fft_out_r = data_hrtf[1] + idx * n_fft;
                    ctype *fft_in_l = s->in_fft[0];
                    ctype *fft_in_r = s->in_fft[1];

                    for (j = 0; j < len; j++) {
                        fft_in_l[j].re = ptr[j * N + I    ] * gain_lin;
                        fft_in_r[j].re = ptr[j * N + I + 1] * gain_lin;
                    }

                    s->tx_fn[0](s->fft[0], fft_out_l, fft_in_l, sizeof(*fft_in_l));
                    s->tx_fn[0](s->fft[0], fft_out_r, fft_in_r, sizeof(*fft_in_r));
                }
            }
        }
    }

    s->have_hrirs = 1;

fail:
    return ret;
}
