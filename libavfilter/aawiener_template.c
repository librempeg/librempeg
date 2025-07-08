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
#undef FMAX
#undef SAMPLE_FORMAT
#if DEPTH == 32
#define SAMPLE_FORMAT fltp
#define FMAX fmaxf
#define ftype float
#define EPS FLT_EPSILON
#else
#define SAMPLE_FORMAT dblp
#define FMAX fmax
#define ftype double
#define EPS FLT_EPSILON
#endif

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

#define F(x) ((ftype)(x))

typedef struct fn(StateContext) {
    ftype *cache;

    unsigned filled, idx, size, hidx;

    ftype mean_sum, var_sum;
} fn(StateContext);

static void fn(awiener_uninit)(AVFilterContext *ctx)
{
    AudioAWienerContext *s = ctx->priv;
    fn(StateContext) *st = s->st;

    for (int ch = 0; ch < s->nb_channels && st; ch++) {
        fn(StateContext) *stc = &st[ch];

        av_freep(&stc->cache);
    }

    av_freep(&s->st);
}

static int fn(awiener_init)(AVFilterContext *ctx)
{
    AVFilterLink *outlink = ctx->outputs[0];
    AudioAWienerContext *s = ctx->priv;
    const int nb_channels = outlink->ch_layout.nb_channels;
    fn(StateContext) *st;
    int look;

    look = s->look;
    s->trim_size = s->flush_size = s->hlook = look / 2;
    s->nb_channels = nb_channels;

    if (!s->st)
        s->st = av_calloc(nb_channels, sizeof(*st));
    if (!s->st)
        return AVERROR(ENOMEM);

    st = s->st;
    for (int ch = 0; ch < nb_channels; ch++) {
        fn(StateContext) *stc = &st[ch];

        stc->size = look;

        if (!stc->cache)
            stc->cache = av_calloc(look, sizeof(*stc->cache));
        if (!stc->cache)
            return AVERROR(ENOMEM);
    }

    return 0;
}

static int fn(do_awiener)(AVFilterContext *ctx, AVFrame *in, AVFrame *out, const int ch)
{
    AudioAWienerContext *s = ctx->priv;
    const ftype *src = (const ftype *)in->extended_data[ch];
    ftype *dst = (ftype *)out->extended_data[ch];
    enum AVChannel channel = av_channel_layout_channel_from_index(&ctx->inputs[0]->ch_layout, ch);
    const int bypass = av_channel_layout_index_from_channel(&s->ch_layout, channel) < 0;
    const int disabled = ff_filter_disabled(ctx) | bypass;
    const int nb_samples = in->nb_samples;
    const ftype noise_var = s->noise_var;
    fn(StateContext) *st = s->st;
    fn(StateContext) *stc = &st[ch];
    const unsigned size = stc->size;
    ftype *cache = stc->cache;
    unsigned filled = stc->filled;
    unsigned hidx = stc->hidx;
    unsigned idx = stc->idx;
    ftype mean_sum = stc->mean_sum;
    ftype var_sum = stc->var_sum;
    ftype prev;

    for (int n = 0; n < nb_samples; n++) {
        ftype mean, new_mean, mid;
        const ftype r = src[n];

        mean = mean_sum / size;

        mean_sum += r;
        if (filled < size) {
            prev = F(0.0);
            cache[idx] = r;
            filled++;
            idx++;
            if (idx >= size/2)
                hidx++;
            if (idx >= size)
                idx = 0;
        } else {
            prev = cache[idx];
            mean_sum -= prev;
            cache[idx] = r;
            idx++;
            hidx++;
            if (idx >= size)
                idx = 0;
            if (hidx >= size)
                hidx = 0;
        }

        new_mean = mean_sum / filled;
        var_sum += (r - mean) * (r - new_mean) - (prev - mean) * (prev - new_mean);
        mid = cache[hidx];

        dst[n] = disabled ? mid : (new_mean + (mid - new_mean) * FMAX(var_sum - noise_var * size, F(0.0)) / (FMAX(var_sum + noise_var*size, F(0.0)) + EPS));
    }

    stc->mean_sum = mean_sum;
    stc->var_sum = var_sum;
    stc->filled = filled;
    stc->hidx = hidx;
    stc->idx = idx;

    return 0;
}
