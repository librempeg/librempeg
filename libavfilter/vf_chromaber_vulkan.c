/*
 * Copyright (c) Lynne
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
#include "libavutil/opt.h"
#include "libavutil/vulkan_spirv.h"
#include "vulkan_filter.h"

#include "filters.h"
#include "video.h"

typedef struct ChromaticAberrationVulkanContext {
    FFVulkanContext vkctx;

    int initialized;
    FFVkExecPool e;
    AVVulkanDeviceQueueFamily *qf;
    FFVulkanShader shd;
    VkSampler sampler;

    /* Push constants / options */
    struct {
        float dist[2];
    } opts;
} ChromaticAberrationVulkanContext;

static const char distort_chroma_kernel[] = {
    C(0, void distort_rgb(ivec2 size, ivec2 pos)                               )
    C(0, {                                                                     )
    C(1,     const vec2 p = ((vec2(pos)/vec2(size)) - 0.5f)*2.0f;              )
    C(1,     const vec2 o = p * (dist - 1.0f);                                 )
    C(0,                                                                       )
    C(1,     vec4 res;                                                         )
    C(1,     res.r = texture(input_img[0], ((p - o)/2.0f) + 0.5f).r;           )
    C(1,     res.g = texture(input_img[0], ((p    )/2.0f) + 0.5f).g;           )
    C(1,     res.b = texture(input_img[0], ((p + o)/2.0f) + 0.5f).b;           )
    C(1,     res.a = texture(input_img[0], ((p    )/2.0f) + 0.5f).a;           )
    C(1,     imageStore(output_img[0], pos, res);                              )
    C(0, }                                                                     )
    C(0,                                                                       )
    C(0, void distort_chroma(int idx, ivec2 size, ivec2 pos)                   )
    C(0, {                                                                     )
    C(1,     vec2 p = ((vec2(pos)/vec2(size)) - 0.5f)*2.0f;                    )
    C(1,     float d = sqrt(p.x*p.x + p.y*p.y);                                )
    C(1,     p *= d / (d*dist);                                                )
    C(1,     vec4 res = texture(input_img[idx], (p/2.0f) + 0.5f);              )
    C(1,     imageStore(output_img[idx], pos, res);                            )
    C(0, }                                                                     )
};

static av_cold int init_filter(AVFilterContext *ctx, AVFrame *in)
{
    int err;
    uint8_t *spv_data;
    size_t spv_len;
    void *spv_opaque = NULL;
    ChromaticAberrationVulkanContext *s = ctx->priv;
    FFVulkanContext *vkctx = &s->vkctx;
    const int planes = av_pix_fmt_count_planes(s->vkctx.output_format);
    FFVulkanShader *shd = &s->shd;
    FFVkSPIRVCompiler *spv;
    FFVulkanDescriptorSetBinding *desc;

    /* Normalize options */
    s->opts.dist[0] = (s->opts.dist[0] / 100.0f) + 1.0f;
    s->opts.dist[1] = (s->opts.dist[1] / 100.0f) + 1.0f;

    spv = ff_vk_spirv_init();
    if (!spv) {
        av_log(ctx, AV_LOG_ERROR, "Unable to initialize SPIR-V compiler!\n");
        return AVERROR_EXTERNAL;
    }

    s->qf = ff_vk_qf_find(vkctx, VK_QUEUE_COMPUTE_BIT, 0);
    if (!s->qf) {
        av_log(ctx, AV_LOG_ERROR, "Device has no compute queues\n");
        err = AVERROR(ENOTSUP);
        goto fail;
    }

    RET(ff_vk_exec_pool_init(vkctx, s->qf, &s->e, s->qf->num*4, 0, 0, 0, NULL));
    RET(ff_vk_init_sampler(vkctx, &s->sampler, 0, VK_FILTER_LINEAR));
    RET(ff_vk_shader_init(vkctx, &s->shd, "chromatic_abberation",
                          VK_SHADER_STAGE_COMPUTE_BIT,
                          NULL, 0,
                          32, 32, 1,
                          0));

    GLSLC(0, layout(push_constant, std430) uniform pushConstants {        );
    GLSLC(1,    vec2 dist;                                                );
    GLSLC(0, };                                                           );
    GLSLC(0,                                                              );

    ff_vk_shader_add_push_const(&s->shd, 0, sizeof(s->opts),
                                VK_SHADER_STAGE_COMPUTE_BIT);

    desc = (FFVulkanDescriptorSetBinding []) {
        {
            .name       = "input_img",
            .type       = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            .dimensions = 2,
            .elems      = planes,
            .stages     = VK_SHADER_STAGE_COMPUTE_BIT,
            .samplers   = DUP_SAMPLER(s->sampler),
        },
        {
            .name       = "output_img",
            .type       = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
            .mem_layout = ff_vk_shader_rep_fmt(s->vkctx.output_format, FF_VK_REP_FLOAT),
            .mem_quali  = "writeonly",
            .dimensions = 2,
            .elems      = planes,
            .stages     = VK_SHADER_STAGE_COMPUTE_BIT,
        },
    };

    RET(ff_vk_shader_add_descriptor_set(vkctx, &s->shd, desc, 2, 0, 0));

    GLSLD(   distort_chroma_kernel                                        );
    GLSLC(0, void main()                                                  );
    GLSLC(0, {                                                            );
    GLSLC(1,     ivec2 pos = ivec2(gl_GlobalInvocationID.xy);             );
    if (planes == 1) {
        GLSLC(1, distort_rgb(imageSize(output_img[0]), pos);              );
    } else {
        GLSLC(1, ivec2 size = imageSize(output_img[0]);                   );
        GLSLC(1, vec2 npos = vec2(pos)/vec2(size);                        );
        GLSLC(1, vec4 res = texture(input_img[0], npos);                  );
        GLSLC(1, imageStore(output_img[0], pos, res);                     );
        for (int i = 1; i < planes; i++) {
            GLSLC(0,                                                      );
            GLSLF(1,  size = imageSize(output_img[%i]);                 ,i);
            GLSLC(1,  if (!IS_WITHIN(pos, size))                          );
            GLSLC(2,      return;                                         );
            GLSLF(1,  distort_chroma(%i, size, pos);                    ,i);
        }
    }
    GLSLC(0, }                                                            );

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

static int chromaber_vulkan_filter_frame(AVFilterLink *link, AVFrame *in)
{
    int err;
    AVFilterContext *ctx = link->dst;
    ChromaticAberrationVulkanContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];

    AVFrame *out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        err = AVERROR(ENOMEM);
        goto fail;
    }

    if (!s->initialized)
        RET(init_filter(ctx, in));

    RET(ff_vk_filter_process_simple(&s->vkctx, &s->e, &s->shd, out, in,
                                    s->sampler, &s->opts, sizeof(s->opts)));

    err = av_frame_copy_props(out, in);
    if (err < 0)
        goto fail;

    av_frame_free(&in);

    return ff_filter_frame(outlink, out);

fail:
    av_frame_free(&in);
    av_frame_free(&out);
    return err;
}

static void chromaber_vulkan_uninit(AVFilterContext *avctx)
{
    ChromaticAberrationVulkanContext *s = avctx->priv;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;

    ff_vk_exec_pool_free(vkctx, &s->e);
    ff_vk_shader_free(vkctx, &s->shd);

    if (s->sampler)
        vk->DestroySampler(vkctx->hwctx->act_dev, s->sampler,
                           vkctx->hwctx->alloc);

    ff_vk_uninit(&s->vkctx);

    s->initialized = 0;
}

#define OFFSET(x) offsetof(ChromaticAberrationVulkanContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption chromaber_vulkan_options[] = {
    { "dist_x", "Set horizontal distortion amount", OFFSET(opts.dist[0]), AV_OPT_TYPE_FLOAT, {.dbl = 0.0f}, -10.0f, 10.0f, .flags = FLAGS },
    { "dist_y", "Set vertical distortion amount",   OFFSET(opts.dist[1]), AV_OPT_TYPE_FLOAT, {.dbl = 0.0f}, -10.0f, 10.0f, .flags = FLAGS },
    { NULL },
};

AVFILTER_DEFINE_CLASS(chromaber_vulkan);

static const AVFilterPad chromaber_vulkan_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = &chromaber_vulkan_filter_frame,
        .config_props = &ff_vk_filter_config_input,
    },
};

static const AVFilterPad chromaber_vulkan_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
        .config_props = &ff_vk_filter_config_output,
    },
};

const FFFilter ff_vf_chromaber_vulkan = {
    .p.name         = "chromaber_vulkan",
    .p.description  = NULL_IF_CONFIG_SMALL("Offset chroma of input video (chromatic aberration)"),
    .p.priv_class   = &chromaber_vulkan_class,
    .p.flags        = AVFILTER_FLAG_HWDEVICE,
    .priv_size      = sizeof(ChromaticAberrationVulkanContext),
    .init           = &ff_vk_filter_init,
    .uninit         = &chromaber_vulkan_uninit,
    FILTER_INPUTS(chromaber_vulkan_inputs),
    FILTER_OUTPUTS(chromaber_vulkan_outputs),
    FILTER_SINGLE_PIXFMT(AV_PIX_FMT_VULKAN),
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
};
