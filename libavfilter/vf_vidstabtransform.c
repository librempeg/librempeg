/*
 * Copyright (c) 2013 Georg Martius <georg dot martius at web dot de>
 *
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

#define DEFAULT_INPUT_NAME     "transforms.trf"

#include <vid.stab/libvidstab.h>

#include "libavutil/common.h"
#include "libavutil/file_open.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "filters.h"
#include "video.h"

#include "vidstabutils.h"

typedef struct TransformContext {
    const AVClass *class;

    VSTransformData td;
    VSTransformConfig conf;

    VSTransformations trans;    // transformations
    char *input;                // name of transform file
    int tripod;
    int debug;
} TransformContext;

#define OFFSET(x) offsetof(TransformContext, x)
#define OFFSETC(x) (offsetof(TransformContext, conf)+offsetof(VSTransformConfig, x))
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption vidstabtransform_options[] = {
    {"input",     "set path to the file storing the transforms", OFFSET(input),
                   AV_OPT_TYPE_STRING, {.str = DEFAULT_INPUT_NAME}, .flags = FLAGS },
    {"smoothing", "set number of frames*2 + 1 used for lowpass filtering", OFFSETC(smoothing),
                   AV_OPT_TYPE_INT,    {.i64 = 15},       0, 1000, FLAGS},

    {"optalgo",   "set camera path optimization algo", OFFSETC(camPathAlgo),
                   AV_OPT_TYPE_INT,    {.i64 = VSOptimalL1}, VSOptimalL1, VSAvg, FLAGS, .unit = "optalgo"},
    {  "opt",     "global optimization",                                            0, // from version 1.0 on
                   AV_OPT_TYPE_CONST,  {.i64 = VSOptimalL1 }, 0, 0, FLAGS, .unit = "optalgo"},
    {  "gauss",   "gaussian kernel",                                                0,
                   AV_OPT_TYPE_CONST,  {.i64 = VSGaussian }, 0, 0, FLAGS,  .unit = "optalgo"},
    {  "avg",     "simple averaging on motion",                                     0,
                   AV_OPT_TYPE_CONST,  {.i64 = VSAvg },      0, 0, FLAGS,  .unit = "optalgo"},

    {"maxshift",  "set maximal number of pixels to translate image", OFFSETC(maxShift),
                   AV_OPT_TYPE_INT,    {.i64 = -1},      -1, 500,  FLAGS},
    {"maxangle",  "set maximal angle in rad to rotate image", OFFSETC(maxAngle),
                   AV_OPT_TYPE_DOUBLE, {.dbl = -1.0},  -1.0, 3.14, FLAGS},

    {"crop",      "set cropping mode", OFFSETC(crop),
                   AV_OPT_TYPE_INT,    {.i64 = 0},        0, 1,    FLAGS, .unit = "crop"},
    {  "keep",    "keep border",                                                    0,
                   AV_OPT_TYPE_CONST,  {.i64 = VSKeepBorder }, 0, 0, FLAGS, .unit = "crop"},
    {  "black",   "black border",                                                   0,
                   AV_OPT_TYPE_CONST,  {.i64 = VSCropBorder }, 0, 0, FLAGS, .unit = "crop"},

    {"invert",    "invert transforms", OFFSETC(invert),
                   AV_OPT_TYPE_INT,    {.i64 = 0},        0, 1,    FLAGS},
    {"relative",  "consider transforms as relative", OFFSETC(relative),
                   AV_OPT_TYPE_INT,    {.i64 = 1},        0, 1,    FLAGS},
    {"zoom",      "set percentage to zoom (>0: zoom in, <0: zoom out", OFFSETC(zoom),
                   AV_OPT_TYPE_DOUBLE, {.dbl = 0},     -100, 100,  FLAGS},
    {"optzoom",   "set optimal zoom (0: nothing, 1: optimal static zoom, 2: optimal dynamic zoom)", OFFSETC(optZoom),
                   AV_OPT_TYPE_INT,    {.i64 = 1},        0, 2,    FLAGS},
    {"zoomspeed", "for adative zoom: percent to zoom maximally each frame",         OFFSETC(zoomSpeed),
                   AV_OPT_TYPE_DOUBLE, {.dbl = 0.25},     0, 5,    FLAGS},

    {"interpol",  "set type of interpolation", OFFSETC(interpolType),
                   AV_OPT_TYPE_INT,    {.i64 = 2},        0, 3,    FLAGS, .unit = "interpol"},
    {  "no",      "no interpolation",                                               0,
                   AV_OPT_TYPE_CONST,  {.i64 = VS_Zero  },  0, 0,  FLAGS, .unit = "interpol"},
    {  "linear",  "linear (horizontal)",                                            0,
                   AV_OPT_TYPE_CONST,  {.i64 = VS_Linear }, 0, 0,  FLAGS, .unit = "interpol"},
    {  "bilinear","bi-linear",                                                      0,
                   AV_OPT_TYPE_CONST,  {.i64 = VS_BiLinear},0, 0,  FLAGS, .unit = "interpol"},
    {  "bicubic", "bi-cubic",                                                       0,
                   AV_OPT_TYPE_CONST,  {.i64 = VS_BiCubic },0, 0,  FLAGS, .unit = "interpol"},

    {"tripod",    "enable virtual tripod mode (same as relative=0:smoothing=0)", OFFSET(tripod),
                   AV_OPT_TYPE_BOOL,   {.i64 = 0},        0, 1,    FLAGS},
    {"debug",     "enable debug mode and writer global motions information to file", OFFSET(debug),
                   AV_OPT_TYPE_BOOL,   {.i64 = 0},        0, 1,    FLAGS},
    {NULL}
};

AVFILTER_DEFINE_CLASS(vidstabtransform);

static av_cold int init(AVFilterContext *ctx)
{
    TransformContext *tc = ctx->priv;
    ff_vs_init();
    tc->class = &vidstabtransform_class;
    av_log(ctx, AV_LOG_VERBOSE, "vidstabtransform filter: init %s\n", LIBVIDSTAB_VERSION);
    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    TransformContext *tc = ctx->priv;

    vsTransformDataCleanup(&tc->td);
    vsTransformationsCleanup(&tc->trans);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    TransformContext *tc = ctx->priv;
    FILE *f;

    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int is_planar = desc->flags & AV_PIX_FMT_FLAG_PLANAR;

    VSTransformData *td = &(tc->td);

    VSFrameInfo fi_src;
    VSFrameInfo fi_dest;

    if (!vsFrameInfoInit(&fi_src, inlink->w, inlink->h,
                         ff_av2vs_pixfmt(ctx, inlink->format)) ||
        !vsFrameInfoInit(&fi_dest, inlink->w, inlink->h,
                         ff_av2vs_pixfmt(ctx, inlink->format))) {
        av_log(ctx, AV_LOG_ERROR, "unknown pixel format: %i (%s)",
               inlink->format, desc->name);
        return AVERROR(EINVAL);
    }

    if ((!is_planar && fi_src.bytesPerPixel != av_get_bits_per_pixel(desc)/8) ||
        fi_src.log2ChromaW != desc->log2_chroma_w ||
        fi_src.log2ChromaH != desc->log2_chroma_h) {
        av_log(ctx, AV_LOG_ERROR, "pixel-format error: bpp %i<>%i  ",
               fi_src.bytesPerPixel, av_get_bits_per_pixel(desc)/8);
        av_log(ctx, AV_LOG_ERROR, "chroma_subsampl: w: %i<>%i  h: %i<>%i\n",
               fi_src.log2ChromaW, desc->log2_chroma_w,
               fi_src.log2ChromaH, desc->log2_chroma_h);
        return AVERROR(EINVAL);
    }

    // set values that are not initializes by the options
    tc->conf.modName = "vidstabtransform";
    tc->conf.verbose = 1 + tc->debug;
    if (tc->tripod) {
        av_log(ctx, AV_LOG_INFO, "Virtual tripod mode: relative=0, smoothing=0\n");
        tc->conf.relative  = 0;
        tc->conf.smoothing = 0;
    }
    tc->conf.simpleMotionCalculation = 0;
    tc->conf.storeTransforms         = tc->debug;
    tc->conf.smoothZoom              = 0;

    if (vsTransformDataInit(td, &tc->conf, &fi_src, &fi_dest) != VS_OK) {
        av_log(ctx, AV_LOG_ERROR, "initialization of vid.stab transform failed, please report a BUG\n");
        return AVERROR(EINVAL);
    }

    vsTransformGetConfig(&tc->conf, td);
    av_log(ctx, AV_LOG_INFO, "Video transformation/stabilization settings (pass 2/2):\n");
    av_log(ctx, AV_LOG_INFO, "    input     = %s\n", tc->input);
    av_log(ctx, AV_LOG_INFO, "    smoothing = %d\n", tc->conf.smoothing);
    av_log(ctx, AV_LOG_INFO, "    optalgo   = %s\n",
           tc->conf.camPathAlgo == VSOptimalL1 ? "opt" :
           (tc->conf.camPathAlgo == VSGaussian ? "gauss" : "avg"));
    av_log(ctx, AV_LOG_INFO, "    maxshift  = %d\n", tc->conf.maxShift);
    av_log(ctx, AV_LOG_INFO, "    maxangle  = %f\n", tc->conf.maxAngle);
    av_log(ctx, AV_LOG_INFO, "    crop      = %s\n", tc->conf.crop ? "Black" : "Keep");
    av_log(ctx, AV_LOG_INFO, "    relative  = %s\n", tc->conf.relative ? "True": "False");
    av_log(ctx, AV_LOG_INFO, "    invert    = %s\n", tc->conf.invert ? "True" : "False");
    av_log(ctx, AV_LOG_INFO, "    zoom      = %f\n", tc->conf.zoom);
    av_log(ctx, AV_LOG_INFO, "    optzoom   = %s\n",
           tc->conf.optZoom == 1 ? "Static (1)" : (tc->conf.optZoom == 2 ? "Dynamic (2)" : "Off (0)"));
    if (tc->conf.optZoom == 2)
        av_log(ctx, AV_LOG_INFO, "    zoomspeed = %g\n", tc->conf.zoomSpeed);
    av_log(ctx, AV_LOG_INFO, "    interpol  = %s\n", getInterpolationTypeName(tc->conf.interpolType));

    f = avpriv_fopen_utf8(tc->input, "rb");
    if (!f) {
        int ret = AVERROR(errno);
        av_log(ctx, AV_LOG_ERROR, "cannot open input file %s\n", tc->input);
        return ret;
    } else {
        VSManyLocalMotions mlms;
        if (vsReadLocalMotionsFile(f, &mlms) == VS_OK) {
            // calculate the actual transforms from the local motions
            if (vsLocalmotions2Transforms(td, &mlms, &tc->trans) != VS_OK) {
                av_log(ctx, AV_LOG_ERROR, "calculating transformations failed\n");
                return AVERROR(EINVAL);
            }
        } else { // try to read old format
            if (!vsReadOldTransforms(td, f, &tc->trans)) { /* read input file */
                av_log(ctx, AV_LOG_ERROR, "error parsing input file %s\n", tc->input);
                return AVERROR(EINVAL);
            }
        }
    }
    fclose(f);

    if (vsPreprocessTransforms(td, &tc->trans) != VS_OK) {
        av_log(ctx, AV_LOG_ERROR, "error while preprocessing transforms\n");
        return AVERROR(EINVAL);
    }

    // TODO: add sharpening, so far the user needs to call the unsharp filter manually
    return 0;
}


static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    TransformContext *tc = ctx->priv;
    VSTransformData* td = &(tc->td);

    AVFilterLink *outlink = inlink->dst->outputs[0];
    int direct = 0;
    AVFrame *out;
    VSFrame inframe;
    int plane;

    if (av_frame_is_writable(in)) {
        direct = 1;
        out = in;
    } else {
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    for (plane = 0; plane < vsTransformGetSrcFrameInfo(td)->planes; plane++) {
        inframe.data[plane] = in->data[plane];
        inframe.linesize[plane] = in->linesize[plane];
    }
    if (direct) {
        vsTransformPrepare(td, &inframe, &inframe);
    } else { // separate frames
        VSFrame outframe;
        for (plane = 0; plane < vsTransformGetDestFrameInfo(td)->planes; plane++) {
            outframe.data[plane] = out->data[plane];
            outframe.linesize[plane] = out->linesize[plane];
        }
        vsTransformPrepare(td, &inframe, &outframe);
    }

    vsDoTransform(td, vsGetNextTransform(td, &tc->trans));

    vsTransformFinish(td);

    if (!direct)
        av_frame_free(&in);

    return ff_filter_frame(outlink, out);
}

static const AVFilterPad vidstabtransform_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
};

const FFFilter ff_vf_vidstabtransform = {
    .p.name        = "vidstabtransform",
    .p.description = NULL_IF_CONFIG_SMALL("Transform the frames, "
                                          "pass 2 of 2 for stabilization "
                                          "(see vidstabdetect for pass 1)."),
    .p.priv_class  = &vidstabtransform_class,
    .priv_size     = sizeof(TransformContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(vidstabtransform_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_PIXFMTS_ARRAY(ff_vidstab_pix_fmts),
};
