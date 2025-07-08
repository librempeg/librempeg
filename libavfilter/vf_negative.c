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

#include <float.h>

#include "libavutil/attributes.h"
#include "libavutil/common.h"
#include "libavutil/ffmath.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "drawutils.h"
#include "formats.h"
#include "video.h"

#define AA_DMIN     (1 << 0)
#define AA_DMAX     (1 << 1)
#define AA_OFFSET   (1 << 2)
#define AA_EXPOSURE (1 << 3)
#define AA_BLACK    (1 << 4)
#define AA_WBH      (1 << 5)
#define AA_WBL      (1 << 6)

typedef struct ThreadData {
    AVFrame *in, *out;
} ThreadData;

typedef struct NegativeContext {
    const AVClass *class;

    float *sampler;
    unsigned nb_sampler;

    float picker_avg[3];
    float picker_min[3];
    float picker_max[3];
    int   autoadjust;

    float *dmin;
    unsigned nb_dmin;

    float dmax;

    float *owbh;
    unsigned nb_owbh;

    float *owbl;
    unsigned nb_owbl;

    float ooffset;
    float oblack;
    float wbh[3];
    float offset[3];
    float black;
    float gamma;
    float softclip;
    float exposure;
} NegativeContext;

#define OFFSET(x) offsetof(NegativeContext, x)
#define VF AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_dmin  = {.def="1.0 1.0 1.13", .size_min=3, .size_max=3, .sep=' '};
static const AVOptionArrayDef def_wbh   = {.def="1.0 1.0 1.00", .size_min=3, .size_max=3, .sep=' '};
static const AVOptionArrayDef def_wbl   = {.def="1.0 1.0 1.00", .size_min=3, .size_max=3, .sep=' '};
static const AVOptionArrayDef def_sampler = {.def="0.0 0.0 0.0 0.0", .size_min=4, .size_max=4, .sep=' '};

static const AVOption negative_options[] = {
    { "color",    "set the values of G/B/R components of film color substrate",      OFFSET(dmin),      AV_OPT_TYPE_FLOAT|AR, {.arr=&def_dmin},  0, 1.5, VF },
    { "wbh",      "set the values of G/B/R components highlights offset for whites", OFFSET(owbh),      AV_OPT_TYPE_FLOAT|AR, {.arr=&def_wbh}, .25, 2.0, VF },
    { "wbl",      "set the values of G/B/R components shadows offset for whites",    OFFSET(owbl),      AV_OPT_TYPE_FLOAT|AR, {.arr=&def_wbl}, .25, 2.0, VF },
    { "density",  "set the max film density",                                        OFFSET(dmax),      AV_OPT_TYPE_FLOAT,    {.dbl=1.06}, .1, 6.0, VF },
    { "offset",   "set the inversion offset",                                        OFFSET(ooffset),   AV_OPT_TYPE_FLOAT,    {.dbl=-.05},-1., 1.0, VF },
    { "black",    "set the display black",                                           OFFSET(oblack),    AV_OPT_TYPE_FLOAT,    {.dbl=-.07},-.5,  .5, VF },
    { "gamma",    "set the display gamma",                                           OFFSET(gamma),     AV_OPT_TYPE_FLOAT,    {.dbl=4.00}, 1., 8.0, VF },
    { "softclip", "set the highlights roll-off",                                     OFFSET(softclip),  AV_OPT_TYPE_FLOAT,    {.dbl= .75},.0001,1., VF },
    { "exposure", "set the extra exposure",                                          OFFSET(exposure),  AV_OPT_TYPE_FLOAT,    {.dbl=0.92}, .5, 2.0, VF },
    { "sampler",  "set the rectangular block to pick color",                         OFFSET(sampler),   AV_OPT_TYPE_FLOAT|AR, {.arr=&def_sampler}, 0.0, 1.0, VF },
    { "autoadjust","set the auto adjust of options from sampler",                    OFFSET(autoadjust),AV_OPT_TYPE_FLAGS,    {.i64=0},     0,  0xFF,VF, "auto" },
    {  "dmin",    "set auto-adjust for dmin",                                        0,                 AV_OPT_TYPE_CONST,    {.i64=AA_DMIN},   0, 0,VF, "auto" },
    {  "dmax",    "set auto-adjust for dmax",                                        0,                 AV_OPT_TYPE_CONST,    {.i64=AA_DMAX},   0, 0,VF, "auto" },
    {  "offset",  "set auto-adjust for offset",                                      0,                 AV_OPT_TYPE_CONST,    {.i64=AA_OFFSET}, 0, 0,VF, "auto" },
    {  "exposure","set auto-adjust for exposure",                                    0,                 AV_OPT_TYPE_CONST,    {.i64=AA_EXPOSURE},0,0,VF, "auto" },
    {  "black",   "set auto-adjust for black",                                       0,                 AV_OPT_TYPE_CONST,    {.i64=AA_BLACK},  0, 0,VF, "auto" },
    {  "wbh",     "set auto-adjust for white-balance highlights",                    0,                 AV_OPT_TYPE_CONST,    {.i64=AA_WBH},    0, 0,VF, "auto" },
    {  "wbl",     "set auto-adjust for white-balance shadows",                       0,                 AV_OPT_TYPE_CONST,    {.i64=AA_WBL},    0, 0,VF, "auto" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(negative);

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    NegativeContext *s = ctx->priv;

    for (int c = 0; c < 3; c++)
        s->wbh[c] = s->owbh[c] / s->dmax;
    for (int c = 0; c < 3; c++)
        s->offset[c] = s->owbh[c] * s->owbl[c] * s->ooffset;

    s->black = -s->exposure * (1.f + s->oblack);

    return 0;
}

#define THRESHOLD 2.3283064365386963e-10f // -32 EV

static int filter_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    NegativeContext *s = ctx->priv;
    const float exposure = s->exposure;
    const float black = s->black;
    const float gamma = s->gamma;
    const float softclip = s->softclip;
    const float softclipcomp = 1.f - softclip;
    const float isoftclipcomp = 1.f / softclipcomp;
    ThreadData *td = arg;
    AVFrame *in = td->in;
    AVFrame *out = td->out;

    for (int p = 0; p < 3; p++) {
        const int w = in->width;
        const int h = in->height;
        const int slice_start = (h * jobnr) / nb_jobs;
        const int slice_end = (h * (jobnr+1)) / nb_jobs;
        const float Dmin = s->dmin[p];
        const float wb_high = s->wbh[p];
        const float offset = s->offset[p];

        for (int y = slice_start; y < slice_end; y++) {
            float *const restrict dst = (float *)(out->data[p] + y * out->linesize[p]);
            const float *const restrict src = (const float *const )(in->data[p] + y * in->linesize[p]);

            for (int x = 0; x < w; x++) {
                const float density = -log10f(Dmin / fmaxf(src[x], THRESHOLD));
                const float corrected_de = wb_high * density + offset;
                const float print_linear = -(exposure * ff_exp10f(corrected_de) + black);
                const float print_gamma = powf(fmaxf(print_linear, 0.f), gamma);
                dst[x] = (print_gamma > softclip) ? softclip + (1.f - expf(-(print_gamma - softclip) * isoftclipcomp)) * softclipcomp : print_gamma;
            }
        }
    }

    return 0;
}

static void adjust_dmin(NegativeContext *s)
{
    memcpy(s->dmin, s->picker_max, s->nb_dmin * sizeof(*s->dmin));
}

static void adjust_dmax(NegativeContext *s)
{
    float rgb[3];

    for (int c = 0; c < 3; c++)
        rgb[c] = log10f(s->dmin[c] / fmaxf(s->picker_min[c], THRESHOLD));
    s->dmax = FFMAX3(rgb[0], rgb[1], rgb[2]);
    s->dmax = av_clipf(s->dmax, 0.1f, 6.f);
}

static void adjust_offset(NegativeContext *s)
{
    float rgb[3];

    for (int c = 0; c < 3; c++)
        rgb[c] = log10f(s->dmin[c] / fmaxf(s->picker_max[c], THRESHOLD)) / s->dmax;
    s->ooffset = FFMIN3(rgb[0], rgb[1], rgb[2]);
    s->ooffset = av_clipf(s->ooffset, -1.f, 1.f);
}

static void adjust_exposure(NegativeContext *s)
{
    float rgb[3];

    for (int c = 0; c < 3; c++) {
        rgb[c] = -log10f(s->dmin[c] / fmaxf(s->picker_min[c], THRESHOLD));
        rgb[c] *= s->owbh[c] / s->dmax;
        rgb[c] += s->owbl[c] * s->ooffset;
        rgb[c] = 0.96f / (1.0f - ff_exp10f(rgb[c]) + s->oblack);
    }

    s->exposure = FFMIN3(rgb[0], rgb[1], rgb[2]);
    s->exposure = av_clipf(s->exposure, 0.5f, 2.f);
}

static void adjust_black(NegativeContext *s)
{
    float rgb[3];

    for (int c = 0; c < 3; c++) {
        rgb[c] = -log10f(s->dmin[c] / fmaxf(s->picker_max[c], THRESHOLD));
        rgb[c] *= s->owbh[c] / s->dmax;
        rgb[c] += s->owbl[c] * s->ooffset * s->owbh[c];
        rgb[c] = 0.1f - (1.0f - ff_exp10f(rgb[c]));
    }

    s->oblack = FFMAX3(rgb[0], rgb[1], rgb[2]);
    s->oblack = av_clipf(s->oblack, -.5f, .5f);
}

static void adjust_wbh(NegativeContext *s)
{
    float rgb_min[3], min;

    for (int c = 0; c < 3; c++)
        rgb_min[c] = fabsf(-1.f / (s->ooffset * s->owbl[c] - log10f(s->dmin[c] / fmaxf(s->picker_avg[c], THRESHOLD)) / s->dmax));

    min = FFMIN3(rgb_min[0], rgb_min[1], rgb_min[2]);
    for (int c = 0; c < 3; c++) {
        s->owbh[c] = rgb_min[c] / min;
        s->owbh[c] = av_clipf(s->owbh[c], 0.25f, 2.f);
    }
}

static void adjust_wbl(NegativeContext *s)
{
    float rgb_min[3], min;

    for (int c = 0; c < 3; c++)
        rgb_min[c] = log10f(s->dmin[c] / fmaxf(s->picker_avg[c], THRESHOLD)) / s->dmax;

    min = FFMIN3(rgb_min[0], rgb_min[1], rgb_min[2]);
    for (int c = 0; c < 3; c++) {
        s->owbl[c] = min / fmaxf(rgb_min[c], THRESHOLD);
        s->owbl[c] = av_clipf(s->owbl[c], 0.25f, 2.f);
    }
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    NegativeContext *s = ctx->priv;
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

    if (s->sampler[0] < s->sampler[2] &&
        s->sampler[1] < s->sampler[3]) {
        const int x = lrintf(s->sampler[0] * (in->width - 1));
        const int y = lrintf(s->sampler[1] * (in->height - 1));
        const int X = lrintf(s->sampler[2] * (in->width - 1));
        const int Y = lrintf(s->sampler[3] * (in->height - 1));

        for (int p = 0; p < 3; p++) {
            s->picker_min[p] =  FLT_MAX;
            s->picker_max[p] = -FLT_MAX;
            s->picker_avg[p] = 0.f;

            for (int j = y; j < Y; j++) {
                const float *src = (const float *)(in->data[p] + j * in->linesize[p]);
                for (int i = x; i < X; i++) {
                    s->picker_avg[p] += src[i];
                    s->picker_min[p]  = fminf(src[i], s->picker_min[p]);
                    s->picker_max[p]  = fmaxf(src[i], s->picker_max[p]);
                }
            }

            s->picker_avg[p] /= (X - x) * (Y - y);
        }
    }

    if (s->autoadjust & AA_DMIN) adjust_dmin(s);
    if (s->autoadjust & AA_DMAX) adjust_dmax(s);
    if (s->autoadjust & AA_OFFSET) adjust_offset(s);
    if (s->autoadjust & AA_EXPOSURE) adjust_exposure(s);
    if (s->autoadjust & AA_BLACK) adjust_black(s);
    if (s->autoadjust & AA_WBH) adjust_wbh(s);
    if (s->autoadjust & AA_WBL) adjust_wbl(s);

    config_input(inlink);

    td.out = out;
    td.in = in;
    ff_filter_execute(ctx, filter_slice, &td, NULL,
                      FFMIN(in->height, ff_filter_get_nb_threads(ctx)));
    if (out != in)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
    },
};

const FFFilter ff_vf_negative = {
    .p.name        = "negative",
    .p.description = NULL_IF_CONFIG_SMALL("Invert scanned film negatives."),
    .priv_size     = sizeof(NegativeContext),
    .p.priv_class  = &negative_class,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_PIXFMTS(AV_PIX_FMT_GBRPF32, AV_PIX_FMT_GBRAPF32),
    .p.flags       = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC | AVFILTER_FLAG_SLICE_THREADS,
    .process_command = ff_filter_process_command,
};
