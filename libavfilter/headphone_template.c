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
#define TX_TYPE AV_TX_FLOAT_RDFT
#else
#define ctype AVComplexDouble
#define ftype double
#define FABS fabs
#define FEXP exp
#define SAMPLE_FORMAT dbl
#define TX_TYPE AV_TX_DOUBLE_RDFT
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
    const int planar = av_sample_fmt_is_planar(in->format);
    const int dst_step = planar ? 1 : 2;
    int offset = planar ? 0 : jobnr;
    int *write = &s->write[jobnr];
    const ftype *const ir = s->data_ir[jobnr];
    int *n_clippings = &td->n_clippings[jobnr];
    ftype *ringbuffer = s->ringbuffer[jobnr];
    ftype *temp_src = s->temp_src[jobnr];
    const int ir_len = s->ir_len;
    const int air_len = s->air_len;
    const ftype *src = (const ftype *)in->data[0];
    ftype *dst = planar ? (ftype *)out->extended_data[jobnr] : (ftype *)out->data[0];
    const int in_channels = in->ch_layout.nb_channels;
    const int src_step = planar ? 1 : in_channels;
    const int buffer_length = s->buffer_length;
    const uint32_t modulo = (uint32_t)buffer_length - 1;
    const int nb_samples = in->nb_samples;
    const ftype gain_lfe = s->gain_lfe;
    ftype *buffer[64];
    int read, sidx = 0;
    int wr = *write;

    dst += offset;
    for (int l = 0; l < in_channels; l++)
        buffer[l] = ringbuffer + l * buffer_length;

    for (int i = 0; i < nb_samples; i++) {
        const ftype *cur_ir = ir;

        *dst = 0;
        if (planar) {
            for (int l = 0; l < in_channels; l++) {
                const ftype *src = (const ftype *)in->extended_data[l];

                *(buffer[l] + wr) = src[sidx];
            }
        } else {
            for (int l = 0; l < in_channels; l++)
                *(buffer[l] + wr) = src[sidx + l];
        }

        for (int l = 0; l < in_channels; cur_ir += air_len, l++) {
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

        dst += dst_step;
        sidx += src_step;
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
    const int planar = av_sample_fmt_is_planar(in->format);
    const int offset = planar ? 0 : jobnr;
    int *write = &s->write[jobnr];
    ctype *hrtf = s->data_hrtf[jobnr];
    int *n_clippings = &td->n_clippings[jobnr];
    const int nb_samples = in->nb_samples;
    ftype *ringbuffer = s->ringbuffer[jobnr];
    const int ir_len = s->ir_len;
    ftype *dst = planar ? (ftype *)out->extended_data[jobnr] : ((ftype *)out->data[0]) + offset;
    const int in_channels = in->ch_layout.nb_channels;
    const int buffer_length = s->buffer_length;
    const uint32_t modulo = (uint32_t)buffer_length - 1;
    ctype *tx_out = s->out_tx[jobnr];
    ftype *tx_in = s->in_tx[jobnr];
    ctype *fft_acc = s->temp_afft[jobnr];
    AVTXContext *itx_ctx = s->itx_ctx[jobnr];
    AVTXContext *tx_ctx = s->tx_ctx[jobnr];
    av_tx_fn tx_fn = s->tx_fn[jobnr];
    av_tx_fn itx_fn = s->itx_fn[jobnr];
    const int mult = planar ? 1 : 2;
    const int n_tx = s->n_tx;
    const ftype gain_lfe = s->gain_lfe;
    ctype *hrtf_offset;
    int wr = *write;
    int n_read;

    n_read = FFMIN(ir_len, nb_samples);
    for (int j = 0; j < n_read; j++) {
        dst[mult * j]  = ringbuffer[wr];
        ringbuffer[wr] = F(0.0);
        wr  = (wr + 1) & modulo;
    }

    if (planar) {
        memset(dst+n_read, 0, (nb_samples-n_read) * sizeof(*dst));
    } else {
        for (int j = n_read; j < nb_samples; j++)
            dst[2 * j] = F(0.0);
    }

    memset(fft_acc, 0, sizeof(ctype) * (n_tx/2+1));

    for (int i = 0; i < in_channels; i++) {
        const ftype *src = planar ? (const ftype *)in->extended_data[i] : ((const ftype *)in->data[0]) + i;

        if (i == s->lfe_channel) {
            if (planar) {
                for (int j = 0; j < nb_samples; j++)
                    dst[j] += src[j] * gain_lfe;
            } else {
                for (int j = 0; j < nb_samples; j++)
                    dst[2 * j] += src[j * in_channels] * gain_lfe;
            }
            continue;
        }

        hrtf_offset = hrtf + s->hrir_map[i] * s->atx_len;

        if (planar) {
            memcpy(tx_in, src, nb_samples * sizeof(*tx_in));
        } else {
            for (int j = 0; j < nb_samples; j++)
                tx_in[j] = src[j * in_channels];
        }
        memset(tx_in + nb_samples, 0, sizeof(ftype) * (n_tx - nb_samples));

        tx_fn(tx_ctx, tx_out, tx_in, sizeof(*tx_in));

        for (int j = 0; j < (n_tx/2+1); j++) {
            const ctype *hcomplex = hrtf_offset + j;
            const ftype re = tx_out[j].re;
            const ftype im = tx_out[j].im;

            fft_acc[j].re += re * hcomplex->re - im * hcomplex->im;
            fft_acc[j].im += re * hcomplex->im + im * hcomplex->re;
        }
    }

    itx_fn(itx_ctx, tx_in, fft_acc, sizeof(*fft_acc));

    for (int j = 0; j < nb_samples; j++) {
        dst[mult * j] += tx_in[j];
        if (FABS(dst[mult * j]) > 1)
            n_clippings[0]++;
    }

    for (int j = 0; j < ir_len - 1; j++) {
        int write_pos = (wr + j) & modulo;

        *(ringbuffer + write_pos) += tx_in[nb_samples + j];
    }

    *write = wr;

    return 0;
}

static int fn(convert_coeffs)(AVFilterContext *ctx, AVFilterLink *inlink)
{
    struct HeadphoneContext *s = ctx->priv;
    const int planar = av_sample_fmt_is_planar(inlink->format);
    const int ir_len = s->ir_len;
    int nb_input_channels = ctx->inputs[0]->ch_layout.nb_channels;
    const int nb_hrir_channels = s->nb_hrir_inputs == 1 ? ctx->inputs[1]->ch_layout.nb_channels : s->nb_hrir_inputs * 2;
    ftype gain_lin = FEXP((s->gain - 3 * nb_input_channels) / 20 * M_LN10);
    AVFrame *frame;
    int ret = 0;
    int n_tx;

    s->air_len = 1 << (32 - ff_clz(ir_len));
    if (s->type == TIME_DOMAIN) {
        s->air_len = FFALIGN(s->air_len, 32);
    }
    s->buffer_length = 1 << (32 - ff_clz(s->air_len));
    s->n_tx = n_tx = 1 << (32 - ff_clz(ir_len + s->size));

    if (s->type == FREQUENCY_DOMAIN) {
        const size_t cpu_align = av_cpu_max_align();
        ftype scale = F(1.0);
        ftype iscale = F(1.0) / s->n_tx;

        s->atx_len = FFALIGN(n_tx/2+1, cpu_align);

        ret = av_tx_init(&s->tx_ctx[0], &s->tx_fn[0], TX_TYPE, 0, s->n_tx, &scale, 0);
        if (ret < 0)
            goto fail;
        ret = av_tx_init(&s->tx_ctx[1], &s->tx_fn[1], TX_TYPE, 0, s->n_tx, &scale, 0);
        if (ret < 0)
            goto fail;
        ret = av_tx_init(&s->itx_ctx[0], &s->itx_fn[0], TX_TYPE, 1, s->n_tx, &iscale, 0);
        if (ret < 0)
            goto fail;
        ret = av_tx_init(&s->itx_ctx[1], &s->itx_fn[1], TX_TYPE, 1, s->n_tx, &iscale, 0);
        if (ret < 0)
            goto fail;

        if (!s->tx_ctx[0] || !s->tx_ctx[1] || !s->itx_ctx[0] || !s->itx_ctx[1]) {
            av_log(ctx, AV_LOG_ERROR, "Unable to create TX contexts of size %d.\n", s->n_tx);
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
        s->out_tx[0] = av_calloc(s->n_tx/2+1, sizeof(ctype));
        s->out_tx[1] = av_calloc(s->n_tx/2+1, sizeof(ctype));
        s->in_tx[0] = av_calloc(s->n_tx, sizeof(ftype));
        s->in_tx[1] = av_calloc(s->n_tx, sizeof(ftype));
        s->temp_afft[0] = av_calloc(s->n_tx/2+1, sizeof(ctype));
        s->temp_afft[1] = av_calloc(s->n_tx/2+1, sizeof(ctype));
        if (!s->in_tx[0] || !s->in_tx[1] ||
            !s->out_tx[0] || !s->out_tx[1] ||
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
        s->data_hrtf[0] = av_calloc(s->atx_len, sizeof(ctype) * nb_hrir_channels);
        s->data_hrtf[1] = av_calloc(s->atx_len, sizeof(ctype) * nb_hrir_channels);
        if (!s->data_hrtf[0] || !s->data_hrtf[1]) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }

    for (int i = 0; i < s->nb_hrir_inputs; av_frame_free(&frame), i++) {
        ctype *data_hrtf[2] = { s->data_hrtf[0], s->data_hrtf[1] };
        ftype *data_ir[2] = { s->data_ir[0], s->data_ir[1] };
        int len = s->hrir_in[i].ir_len;
        ftype *ptr_l, *ptr_r;

        ret = ff_inlink_consume_samples(ctx->inputs[i + 1], len, len, &frame);
        if (ret < 0)
            goto fail;

        if (s->hrir_fmt == HRIR_STEREO) {
            const int step = planar ? 1 : 2;
            int idx = av_channel_layout_index_from_channel(&inlink->ch_layout,
                                                           s->mapping[i]);
            if (idx < 0)
                continue;

            if (planar) {
                ptr_l = (ftype *)frame->extended_data[0];
                ptr_r = (ftype *)frame->extended_data[1];
            } else {
                ptr_l = (ftype *)frame->data[0];
                ptr_r = ((ftype *)frame->data[0])+1;
            }

            s->hrir_map[i] = idx;
            if (s->type == TIME_DOMAIN) {
                ftype *data_ir_l = data_ir[0] + idx * s->air_len;
                ftype *data_ir_r = data_ir[1] + idx * s->air_len;

                for (int j = 0; j < len; j++) {
                    data_ir_l[j] = ptr_l[len * step - j * step - step] * gain_lin;
                    data_ir_r[j] = ptr_r[len * step - j * step - step] * gain_lin;
                }
            } else {
                ctype *tx_out_l = data_hrtf[0] + idx * s->atx_len;
                ctype *tx_out_r = data_hrtf[1] + idx * s->atx_len;
                ftype *tx_in_l = s->in_tx[0];
                ftype *tx_in_r = s->in_tx[1];

                for (int j = 0; j < len; j++) {
                    tx_in_l[j] = ptr_l[j * step] * gain_lin;
                    tx_in_r[j] = ptr_r[j * step] * gain_lin;
                }

                s->tx_fn[0](s->tx_ctx[0], tx_out_l, tx_in_l, sizeof(*tx_in_l));
                s->tx_fn[0](s->tx_ctx[0], tx_out_r, tx_in_r, sizeof(*tx_in_r));
            }
        } else {
            const int N = ctx->inputs[1]->ch_layout.nb_channels;
            const int M = planar ? 1 : N;

            for (int k = 0; k < N / 2; k++) {
                int idx = av_channel_layout_index_from_channel(&inlink->ch_layout,
                                                              s->mapping[k]);
                if (idx < 0)
                    continue;

                if (planar) {
                    ptr_l = (ftype *)frame->extended_data[k*2];
                    ptr_r = (ftype *)frame->extended_data[k*2+1];
                } else {
                    ptr_l = ((ftype *)frame->data[0])+k*2;
                    ptr_r = ((ftype *)frame->data[0])+k*2+1;
                }

                s->hrir_map[k] = idx;
                if (s->type == TIME_DOMAIN) {
                    ftype *data_ir_l = data_ir[0] + idx * s->air_len;
                    ftype *data_ir_r = data_ir[1] + idx * s->air_len;

                    for (int j = 0; j < len; j++) {
                        data_ir_l[j] = ptr_l[len * M - j * M - M] * gain_lin;
                        data_ir_r[j] = ptr_r[len * M - j * M - M] * gain_lin;
                    }
                } else {
                    ctype *tx_out_l = data_hrtf[0] + idx * s->atx_len;
                    ctype *tx_out_r = data_hrtf[1] + idx * s->atx_len;
                    ftype *tx_in_l = s->in_tx[0];
                    ftype *tx_in_r = s->in_tx[1];

                    for (int j = 0; j < len; j++) {
                        tx_in_l[j] = ptr_l[j * M] * gain_lin;
                        tx_in_r[j] = ptr_r[j * M] * gain_lin;
                    }

                    s->tx_fn[0](s->tx_ctx[0], tx_out_l, tx_in_l, sizeof(*tx_in_l));
                    s->tx_fn[0](s->tx_ctx[0], tx_out_r, tx_in_r, sizeof(*tx_in_r));
                }
            }
        }
    }

    s->have_hrirs = 1;

fail:
    return ret;
}
