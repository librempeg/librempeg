/*
 * Copyright (c) 2025 Paul B Mahol
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <float.h>

#include "libavutil/avstring.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "formats.h"
#include "video.h"

typedef struct MinVarContext {
    const AVClass *class;

    double *variance;
    unsigned nb_variance;

    double *percent;
    unsigned nb_percent;

    int *sizew;
    unsigned nb_sizew;

    int *sizeh;
    unsigned nb_sizeh;

    int *filter_plane;
    unsigned nb_filter_plane;

    int depth;
    int max;
    int planewidth[4];
    int planeheight[4];

    uint64_t *ii[4];
    uint64_t *i2[4];
    ptrdiff_t i_linesize[4];
    size_t i_size[4];

    int nb_planes;

    void (*compute_sat)(const uint8_t *ssrc,
                        ptrdiff_t linesize,
                        int w, int h,
                        uint64_t *ii,
                        uint64_t *i2,
                        ptrdiff_t i_linesize);
    int (*filter)(AVFilterContext *ctx, void *arg,
                  int jobnr, int nb_jobs);
} MinVarContext;

#define OFFSET(x) offsetof(MinVarContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_var     = {.def="0",.size_min=1,.size_max=4,.sep=' '};
static const AVOptionArrayDef def_percent = {.def="85",.size_min=1,.size_max=4,.sep=' '};
static const AVOptionArrayDef def_sizew   = {.def="5",.size_min=1,.size_max=4,.sep=' '};
static const AVOptionArrayDef def_sizeh   = {.def="5",.size_min=1,.size_max=4,.sep=' '};
static const AVOptionArrayDef def_filter  = {.def="1",.size_min=1,.size_max=4,.sep=' '};

static const AVOption minvar_options[] = {
    { "var",     "set global noise variance", OFFSET(variance), AV_OPT_TYPE_DOUBLE|AR,{.arr=&def_var},    0, 100, FLAGS },
    { "percent", "set percent of denoising",  OFFSET(percent),  AV_OPT_TYPE_DOUBLE|AR,{.arr=&def_percent},0, 100, FLAGS },
    { "rw",      "set horizontal patch size", OFFSET(sizew),    AV_OPT_TYPE_INT|AR,   {.arr=&def_sizew},  1,  33, FLAGS },
    { "rh",      "set vertical patch size",   OFFSET(sizeh),    AV_OPT_TYPE_INT|AR,   {.arr=&def_sizeh},  1,  33, FLAGS },
    { "planes",  "set planes to filter",      OFFSET(filter_plane),AV_OPT_TYPE_BOOL|AR,{.arr=&def_filter},0,   1, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(minvar);

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

static av_cold void uninit(AVFilterContext *ctx)
{
    MinVarContext *s = ctx->priv;

    for (int p = 0; p < 4; p++) {
        av_freep(&s->ii[p]);
        av_freep(&s->i2[p]);
    }
}

#define COMPUTE_SAT(type, depth)                     \
static void compute_sat##depth(const uint8_t *ssrc,  \
                               ptrdiff_t linesize,   \
                               int w, int h,         \
                               uint64_t *ii,         \
                               uint64_t *i2,         \
                               ptrdiff_t i_linesize) \
{                                                    \
    const type *src = (const type *)ssrc;            \
                                                     \
    linesize /= sizeof(type);                        \
    ii += i_linesize;                                \
    i2 += i_linesize;                                \
                                                     \
    for (int y = 0; y < h; y++) {                    \
        uint64_t sum2 = 0;                           \
        uint64_t sum = 0;                            \
                                                     \
        for (int x = 1; x <= w; x++) {               \
            const int v = src[x-1];                  \
            sum += v;                                \
            sum2 += v*v;                             \
            ii[x] = sum + ii[x - i_linesize];        \
            i2[x] = sum2 + i2[x - i_linesize];       \
        }                                            \
                                                     \
        src += linesize;                             \
        ii += i_linesize;                            \
        i2 += i_linesize;                            \
    }                                                \
}

COMPUTE_SAT(uint8_t, 8)
COMPUTE_SAT(uint16_t, 16)

#define SQR(x) ((x) * (x))

#define FILTER(type, ftype, ddepth, LRINT, MAX)      \
static int filter##ddepth(AVFilterContext *ctx,      \
                          void *arg,                 \
                          int jobnr, int nb_jobs)    \
{                                                    \
    MinVarContext *s = ctx->priv;                    \
    const int depth = s->depth;                      \
    ThreadData *td = arg;                            \
    AVFrame *out = td->out;                          \
    AVFrame *in = td->in;                            \
                                                               \
    for (int plane = 0; plane < s->nb_planes; plane++) {       \
        const ptrdiff_t slinesize = in->linesize[plane] / sizeof(type); \
        const ptrdiff_t linesize = out->linesize[plane] / sizeof(type); \
        const int height = s->planeheight[plane];              \
        const int width = s->planewidth[plane];                \
        const int slice_start = (height * jobnr) / nb_jobs;    \
        const int slice_end = (height * (jobnr+1)) / nb_jobs;  \
        const ptrdiff_t i_linesize = s->i_linesize[plane];     \
        const int sizeh = s->sizeh[FFMIN(s->nb_sizeh-1, plane)]; \
        const int sizew = s->sizew[FFMIN(s->nb_sizew-1, plane)]; \
        const uint64_t *ii = s->ii[plane] + i_linesize + 1;    \
        const uint64_t *i2 = s->i2[plane] + i_linesize + 1;    \
        const type *src = ((const type *)in->data[plane]) + slice_start * slinesize; \
        type *dst = ((type *)out->data[plane]) + slice_start * linesize;  \
        const ftype percent = s->percent[FFMIN(s->nb_percent-1, plane)] * 0.01; \
        const ftype gvar = ((1<<depth)-1) * s->variance[FFMIN(s->nb_variance-1, plane)]; \
                                                                    \
        if (!(s->filter_plane[FFMIN(s->nb_filter_plane-1, plane)])) \
            continue;                                               \
                                                                    \
        for (int y = slice_start; y < slice_end; y++) {             \
            for (int x = 0; x < width; x++) {                       \
                int v = src[x];                                     \
                ftype lvar, org, new;                               \
                const int t = FFMIN(sizeh, y);                      \
                const int b = FFMIN(sizeh, height - y - 1);         \
                const ftype tb = t+b+1;                             \
                const ptrdiff_t yb = (y+b) * i_linesize;            \
                const ptrdiff_t yt = (y-t-1) * i_linesize;          \
                const ftype l = FFMIN(sizew, x);                    \
                const ftype r = FFMIN(sizew, width - x - 1);        \
                const int xl = x-l-1;                               \
                const int xr = x+r;                                 \
                ftype tl2 = i2[yt + xl];                            \
                ftype tr2 = i2[yt + xr];                            \
                ftype bl2 = i2[yb + xl];                            \
                ftype br2 = i2[yb + xr];                            \
                ftype tl  = ii[yt + xl];                            \
                ftype tr  = ii[yt + xr];                            \
                ftype bl  = ii[yb + xl];                            \
                ftype br  = ii[yb + xr];                            \
                const ftype scale = 1.f / ((l+r+1) * tb);           \
                const ftype I2 = br2 - bl2 - tr2 + tl2;             \
                const ftype I1 = br - bl - tr + tl;                 \
                const ftype mean = I1 * scale;                      \
                lvar = MAX(I2 * scale - mean * mean, 0);            \
                                                                    \
                v = mean + (v - mean) * MAX(lvar - gvar, 0) / (MAX(lvar, gvar)+FLT_EPSILON); \
                                                                    \
                new = percent;                                      \
                org = 1.f - percent;                                \
                dst[x] = av_clip_uintp2_c(LRINT(org * src[x] + new * v), depth);\
            }                                                       \
                                                                    \
            src += slinesize;                                       \
            dst += linesize;                                        \
        }                                                           \
    }                                                               \
                                                                    \
    return 0;                                                       \
}

FILTER(uint8_t, float, 8, lrintf, fmaxf)
FILTER(uint16_t, double, 16, lrint, fmax)

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    MinVarContext *s = ctx->priv;

    uninit(ctx);

    s->depth = desc->comp[0].depth;
    s->max = (1 << s->depth) - 1;
    s->planewidth[1] = s->planewidth[2] = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
    s->planewidth[0] = s->planewidth[3] = inlink->w;
    s->planeheight[1] = s->planeheight[2] = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
    s->planeheight[0] = s->planeheight[3] = inlink->h;

    s->compute_sat = s->depth <= 8 ? compute_sat8 : compute_sat16;
    s->filter = s->depth <= 8 ? filter8 : filter16;
    s->nb_planes = av_pix_fmt_count_planes(inlink->format);

    for (int p = 0; p < s->nb_planes; p++) {
        s->i_linesize[p] = (s->planewidth[p] + 1);
        s->i_size[p] = s->i_linesize[p] * (s->planeheight[p] + 1);
        s->ii[p] = av_calloc(s->i_size[p], sizeof(*s->ii[0]));
        s->i2[p] = av_calloc(s->i_size[p], sizeof(*s->i2[0]));
        if (!s->ii[p] || !s->i2[p])
            return AVERROR(ENOMEM);
    }

    return 0;
}

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_YUVA444P, AV_PIX_FMT_YUV444P, AV_PIX_FMT_YUV440P,
    AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ440P,
    AV_PIX_FMT_YUVA422P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUVA420P, AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ420P,
    AV_PIX_FMT_YUVJ411P, AV_PIX_FMT_YUV411P, AV_PIX_FMT_YUV410P,
    AV_PIX_FMT_YUV420P9, AV_PIX_FMT_YUV422P9, AV_PIX_FMT_YUV444P9,
    AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV444P10,
    AV_PIX_FMT_YUV420P12, AV_PIX_FMT_YUV422P12, AV_PIX_FMT_YUV444P12, AV_PIX_FMT_YUV440P12,
    AV_PIX_FMT_YUV420P14, AV_PIX_FMT_YUV422P14, AV_PIX_FMT_YUV444P14,
    AV_PIX_FMT_YUV420P16, AV_PIX_FMT_YUV422P16, AV_PIX_FMT_YUV444P16,
    AV_PIX_FMT_YUVA420P9, AV_PIX_FMT_YUVA422P9, AV_PIX_FMT_YUVA444P9,
    AV_PIX_FMT_YUVA420P10, AV_PIX_FMT_YUVA422P10, AV_PIX_FMT_YUVA444P10,
    AV_PIX_FMT_YUVA422P12, AV_PIX_FMT_YUVA444P12,
    AV_PIX_FMT_YUVA420P16, AV_PIX_FMT_YUVA422P16, AV_PIX_FMT_YUVA444P16,
    AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRP9, AV_PIX_FMT_GBRP10,
    AV_PIX_FMT_GBRP12, AV_PIX_FMT_GBRP14, AV_PIX_FMT_GBRP16,
    AV_PIX_FMT_GBRAP, AV_PIX_FMT_GBRAP10, AV_PIX_FMT_GBRAP12, AV_PIX_FMT_GBRAP16,
    AV_PIX_FMT_GRAY8, AV_PIX_FMT_GRAY9, AV_PIX_FMT_GRAY10, AV_PIX_FMT_GRAY12, AV_PIX_FMT_GRAY14, AV_PIX_FMT_GRAY16,
    AV_PIX_FMT_NONE
};

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    MinVarContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    ThreadData td;
    AVFrame *out;

    if (av_frame_is_writable(in)) {
        out = in;
    } else {
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    for (int plane = 0; plane < s->nb_planes; plane++) {
        const int height = s->planeheight[plane];
        const ptrdiff_t i_linesize = s->i_linesize[plane];
        const int width = s->planewidth[plane];
        uint64_t *ii = s->ii[plane];
        uint64_t *i2 = s->i2[plane];

        if (!(s->filter_plane[FFMIN(s->nb_filter_plane-1, plane)])) {
            if (out != in)
                av_image_copy_plane(out->data[plane], out->linesize[plane],
                                    in->data[plane], in->linesize[plane],
                                    width * ((s->depth + 7) / 8), height);
            continue;
        }

        s->compute_sat(in->data[plane], in->linesize[plane],
                       width, height, ii, i2, i_linesize);
    }

    td.in = in;
    td.out = out;

    ff_filter_execute(ctx, s->filter, &td, NULL,
                      FFMIN(s->planeheight[1], ff_filter_get_nb_threads(ctx)));

    if (out != in)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static const AVFilterPad minvar_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad minvar_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
    },
};

const FFFilter ff_vf_minvar = {
    .p.name        = "minvar",
    .p.description = NULL_IF_CONFIG_SMALL("Apply Adaptive Wiener filter."),
    .p.priv_class  = &minvar_class,
    .priv_size     = sizeof(MinVarContext),
    .uninit        = uninit,
    FILTER_INPUTS(minvar_inputs),
    FILTER_OUTPUTS(minvar_outputs),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC |
                     AVFILTER_FLAG_SLICE_THREADS,
    .process_command = ff_filter_process_command,
};
