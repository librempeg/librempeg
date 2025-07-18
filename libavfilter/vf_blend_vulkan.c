/*
 * copyright (c) 2021-2022 Wu Jianhua <jianhua.wu@intel.com>
 * Copyright (c) Lynne
 *
 * The blend modes are based on the blend.c.
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

#include "libavutil/random_seed.h"
#include "libavutil/vulkan_spirv.h"
#include "libavutil/opt.h"
#include "vulkan_filter.h"

#include "filters.h"
#include "framesync.h"
#include "blend.h"
#include "video.h"

#define IN_TOP    0
#define IN_BOTTOM 1

typedef struct FilterParamsVulkan {
    const char *blend;
    const char *blend_func;
    double opacity;
    enum BlendMode mode;
} FilterParamsVulkan;

typedef struct BlendVulkanContext {
    FFVulkanContext vkctx;
    FFFrameSync fs;

    int initialized;
    FFVkExecPool e;
    AVVulkanDeviceQueueFamily *qf;
    FFVulkanShader shd;

    FilterParamsVulkan params[4];
    double all_opacity;
    enum BlendMode all_mode;
} BlendVulkanContext;

#define DEFINE_BLEND_MODE(MODE, EXPR) \
static const char blend_##MODE[] = "blend_"#MODE; \
static const char blend_##MODE##_func[] = { \
    C(0, vec4 blend_##MODE(vec4 top, vec4 bottom, float opacity) {   ) \
    C(1,     vec4 dst = EXPR;                                        ) \
    C(1,     return dst;                                             ) \
    C(0, }                                                           ) \
};

#define A top
#define B bottom

#define FN(EXPR) A + ((EXPR) - A) * opacity

DEFINE_BLEND_MODE(NORMAL, A * opacity + B * (1.0f - opacity))
DEFINE_BLEND_MODE(MULTIPLY, FN(1.0f * A * B / 1.0f))

static inline void init_blend_func(FilterParamsVulkan *param)
{
#define CASE(MODE) case BLEND_##MODE: \
            param->blend = blend_##MODE;\
            param->blend_func =  blend_##MODE##_func; \
            break;

    switch (param->mode) {
    CASE(NORMAL)
    CASE(MULTIPLY)
    default: param->blend = NULL; break;
    }

#undef CASE
}

static int config_params(AVFilterContext *avctx)
{
    BlendVulkanContext *s = avctx->priv;

    for (int plane = 0; plane < FF_ARRAY_ELEMS(s->params); plane++) {
        FilterParamsVulkan *param = &s->params[plane];

        if (s->all_mode >= 0)
            param->mode = s->all_mode;
        if (s->all_opacity < 1)
            param->opacity = s->all_opacity;

        init_blend_func(param);
        if (!param->blend) {
            av_log(avctx, AV_LOG_ERROR,
                   "Currently the blend mode specified is not supported yet.\n");
            return AVERROR(EINVAL);
        }
    }

    return 0;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    int ret = ff_filter_process_command(ctx, cmd, arg);

    if (ret < 0)
        return ret;

    return config_params(ctx);
}

static av_cold int init_filter(AVFilterContext *avctx)
{
    int err = 0;
    uint8_t *spv_data;
    size_t spv_len;
    void *spv_opaque = NULL;
    BlendVulkanContext *s = avctx->priv;
    FFVulkanContext *vkctx = &s->vkctx;
    const int planes = av_pix_fmt_count_planes(s->vkctx.output_format);
    FFVulkanShader *shd = &s->shd;
    FFVkSPIRVCompiler *spv;
    FFVulkanDescriptorSetBinding *desc;

    spv = ff_vk_spirv_init();
    if (!spv) {
        av_log(avctx, AV_LOG_ERROR, "Unable to initialize SPIR-V compiler!\n");
        return AVERROR_EXTERNAL;
    }

    s->qf = ff_vk_qf_find(vkctx, VK_QUEUE_COMPUTE_BIT, 0);
    if (!s->qf) {
        av_log(avctx, AV_LOG_ERROR, "Device has no compute queues\n");
        err = AVERROR(ENOTSUP);
        goto fail;
    }

    RET(ff_vk_exec_pool_init(vkctx, s->qf, &s->e, s->qf->num*4, 0, 0, 0, NULL));
    RET(ff_vk_shader_init(vkctx, &s->shd, "blend",
                          VK_SHADER_STAGE_COMPUTE_BIT,
                          NULL, 0,
                          32, 32, 1,
                          0));

    desc = (FFVulkanDescriptorSetBinding []) {
        {
            .name       = "top_images",
            .type       = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
            .mem_layout = ff_vk_shader_rep_fmt(s->vkctx.input_format, FF_VK_REP_FLOAT),
            .mem_quali  = "readonly",
            .dimensions = 2,
            .elems      = planes,
            .stages     = VK_SHADER_STAGE_COMPUTE_BIT,
        },
        {
            .name       = "bottom_images",
            .type       = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
            .mem_layout = ff_vk_shader_rep_fmt(s->vkctx.input_format, FF_VK_REP_FLOAT),
            .mem_quali  = "readonly",
            .dimensions = 2,
            .elems      = planes,
            .stages     = VK_SHADER_STAGE_COMPUTE_BIT,
        },
        {
            .name       = "output_images",
            .type       = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
            .mem_layout = ff_vk_shader_rep_fmt(s->vkctx.output_format, FF_VK_REP_FLOAT),
            .mem_quali  = "writeonly",
            .dimensions = 2,
            .elems      = planes,
            .stages     = VK_SHADER_STAGE_COMPUTE_BIT,
        },
    };

    RET(ff_vk_shader_add_descriptor_set(vkctx, &s->shd, desc, 3, 0, 0));

    for (int i = 0, j = 0; i < planes; i++) {
        for (j = 0; j < i; j++)
            if (s->params[i].blend_func == s->params[j].blend_func)
                break;
        /* note: the bracket is needed, for GLSLD is a macro with multiple statements. */
        if (j == i) {
            GLSLD(s->params[i].blend_func);
        }
    }

    GLSLC(0, void main()                                                    );
    GLSLC(0, {                                                              );
    GLSLC(1,     ivec2 size;                                                );
    GLSLC(1,     const ivec2 pos = ivec2(gl_GlobalInvocationID.xy);         );
    for (int i = 0; i < planes; i++) {
        GLSLC(0,                                                            );
        GLSLF(1, size = imageSize(output_images[%i]);                     ,i);
        GLSLC(1, if (IS_WITHIN(pos, size)) {                                );
        GLSLF(2,     const vec4 top = imageLoad(top_images[%i], pos);       ,i);
        GLSLF(2,     const vec4 bottom = imageLoad(bottom_images[%i], pos); ,i);
        GLSLF(2,     const float opacity = %f;                            ,s->params[i].opacity);
        GLSLF(2,     vec4 dst = %s(top, bottom, opacity);                 ,s->params[i].blend);
        GLSLC(0,                                                            );
        GLSLF(2,     imageStore(output_images[%i], pos, dst);             ,i);
        GLSLC(1, }                                                          );
    }
    GLSLC(0, }                                                              );

    RET(spv->compile_shader(vkctx, spv, shd, &spv_data, &spv_len, "main",
                            &spv_opaque));
    RET(ff_vk_shader_link(vkctx, shd, spv_data, spv_len, "main"));

    RET(ff_vk_shader_register_exec(vkctx, &s->e, &s->shd));

    s->initialized = 1;

fail:
    if (spv_opaque)
        spv->free_shader(spv, &spv_opaque);
    if (spv)
        spv->uninit(&spv);

    return err;
}

static int blend_frame(FFFrameSync *fs)
{
    int err;
    AVFilterContext *avctx = fs->parent;
    BlendVulkanContext *s = avctx->priv;
    AVFilterLink *outlink = avctx->outputs[0];
    AVFrame *top, *bottom, *out;

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    RET(ff_framesync_get_frame(fs, IN_TOP,    &top,    0));
    RET(ff_framesync_get_frame(fs, IN_BOTTOM, &bottom, 0));

    RET(av_frame_copy_props(out, top));

    if (!s->initialized) {
        AVHWFramesContext *top_fc = (AVHWFramesContext*)top->hw_frames_ctx->data;
        AVHWFramesContext *bottom_fc = (AVHWFramesContext*)bottom->hw_frames_ctx->data;
        if (top_fc->sw_format != bottom_fc->sw_format) {
            av_log(avctx, AV_LOG_ERROR,
                   "Currently the sw format of the bottom video need to match the top!\n");
            err = AVERROR(EINVAL);
            goto fail;
        }
        RET(init_filter(avctx));
    }

    RET(ff_vk_filter_process_Nin(&s->vkctx, &s->e, &s->shd,
                                 out, (AVFrame *[]){ top, bottom }, 2,
                                 VK_NULL_HANDLE, NULL, 0));

    return ff_filter_frame(outlink, out);

fail:
    av_frame_free(&out);
    return err;
}

static av_cold int init(AVFilterContext *avctx)
{
    BlendVulkanContext *s = avctx->priv;

    s->fs.on_event = blend_frame;

    return ff_vk_filter_init(avctx);
}

static av_cold void uninit(AVFilterContext *avctx)
{
    BlendVulkanContext *s = avctx->priv;
    FFVulkanContext *vkctx = &s->vkctx;

    ff_vk_exec_pool_free(vkctx, &s->e);
    ff_vk_shader_free(vkctx, &s->shd);

    ff_vk_uninit(&s->vkctx);
    ff_framesync_uninit(&s->fs);

    s->initialized = 0;
}

static int config_props_output(AVFilterLink *outlink)
{
    int err;
    FilterLink *outl       = ff_filter_link(outlink);
    AVFilterContext *avctx = outlink->src;
    BlendVulkanContext *s = avctx->priv;
    AVFilterLink *toplink = avctx->inputs[IN_TOP];
    FilterLink   *tl      = ff_filter_link(toplink);
    AVFilterLink *bottomlink = avctx->inputs[IN_BOTTOM];

    if (toplink->w != bottomlink->w || toplink->h != bottomlink->h) {
        av_log(avctx, AV_LOG_ERROR, "First input link %s parameters "
                "(size %dx%d) do not match the corresponding "
                "second input link %s parameters (size %dx%d)\n",
                avctx->input_pads[IN_TOP].name, toplink->w, toplink->h,
                avctx->input_pads[IN_BOTTOM].name, bottomlink->w, bottomlink->h);
        return AVERROR(EINVAL);
    }

    outlink->sample_aspect_ratio = toplink->sample_aspect_ratio;
    outl->frame_rate = tl->frame_rate;

    RET(ff_vk_filter_config_output(outlink));

    RET(ff_framesync_init_dualinput(&s->fs, avctx));

    RET(ff_framesync_configure(&s->fs));
    outlink->time_base = s->fs.time_base;

    RET(config_params(avctx));

fail:
    return err;
}

static int activate(AVFilterContext *avctx)
{
    BlendVulkanContext *s = avctx->priv;
    return ff_framesync_activate(&s->fs);
}

#define OFFSET(x) offsetof(BlendVulkanContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)

static const AVOption blend_vulkan_options[] = {
    { "c0_mode", "set component #0 blend mode", OFFSET(params[0].mode), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, BLEND_NB - 1, FLAGS, .unit = "mode" },
    { "c1_mode", "set component #1 blend mode", OFFSET(params[1].mode), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, BLEND_NB - 1, FLAGS, .unit = "mode" },
    { "c2_mode", "set component #2 blend mode", OFFSET(params[2].mode), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, BLEND_NB - 1, FLAGS, .unit = "mode" },
    { "c3_mode", "set component #3 blend mode", OFFSET(params[3].mode), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, BLEND_NB - 1, FLAGS, .unit = "mode" },
    { "all_mode", "set blend mode for all components", OFFSET(all_mode), AV_OPT_TYPE_INT, { .i64 = -1 }, -1, BLEND_NB - 1, FLAGS, .unit = "mode" },
        { "normal",   "", 0, AV_OPT_TYPE_CONST, { .i64 = BLEND_NORMAL   }, 0, 0, FLAGS, .unit = "mode" },
        { "multiply", "", 0, AV_OPT_TYPE_CONST, { .i64 = BLEND_MULTIPLY }, 0, 0, FLAGS, .unit = "mode" },

    { "c0_opacity",  "set color component #0 opacity", OFFSET(params[0].opacity), AV_OPT_TYPE_DOUBLE, { .dbl = 1 }, 0, 1, FLAGS },
    { "c1_opacity",  "set color component #1 opacity", OFFSET(params[1].opacity), AV_OPT_TYPE_DOUBLE, { .dbl = 1 }, 0, 1, FLAGS },
    { "c2_opacity",  "set color component #2 opacity", OFFSET(params[2].opacity), AV_OPT_TYPE_DOUBLE, { .dbl = 1 }, 0, 1, FLAGS },
    { "c3_opacity",  "set color component #3 opacity", OFFSET(params[3].opacity), AV_OPT_TYPE_DOUBLE, { .dbl = 1 }, 0, 1, FLAGS },
    { "all_opacity", "set opacity for all color components", OFFSET(all_opacity), AV_OPT_TYPE_DOUBLE, { .dbl = 1 }, 0, 1, FLAGS },

    { NULL }
};

AVFILTER_DEFINE_CLASS(blend_vulkan);

static const AVFilterPad blend_vulkan_inputs[] = {
    {
        .name         = "top",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = &ff_vk_filter_config_input,
    },
    {
        .name         = "bottom",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = &ff_vk_filter_config_input,
    },
};


static const AVFilterPad blend_vulkan_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = &config_props_output,
    }
};

const FFFilter ff_vf_blend_vulkan = {
    .p.name          = "blend_vulkan",
    .p.description   = NULL_IF_CONFIG_SMALL("Blend two video frames in Vulkan"),
    .p.priv_class    = &blend_vulkan_class,
    .p.flags         = AVFILTER_FLAG_HWDEVICE,
    .priv_size       = sizeof(BlendVulkanContext),
    .init            = &init,
    .uninit          = &uninit,
    .activate        = &activate,
    FILTER_INPUTS(blend_vulkan_inputs),
    FILTER_OUTPUTS(blend_vulkan_outputs),
    FILTER_SINGLE_PIXFMT(AV_PIX_FMT_VULKAN),
    .flags_internal  = FF_FILTER_FLAG_HWFRAME_AWARE,
    .process_command = &process_command,
};
