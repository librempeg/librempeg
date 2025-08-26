/*
 * Copyright (c) 2022 Paul B Mahol
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

#include "libavutil/avassert.h"
#include "libavutil/channel_layout.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "audio.h"
#include "video.h"

typedef struct Audio3dScopeContext {
    const AVClass *class;
    int w, h;
    int size;
    float fov;
    float roll;
    float pitch;
    float yaw;
    float zoom[3];
    float eye[3];

    float *ch_x0;
    unsigned nb_ch_x0;
    float *ch_y0;
    unsigned nb_ch_y0;
    float *ch_z0;
    unsigned nb_ch_z0;

    float *ch_x1;
    unsigned nb_ch_x1;
    float *ch_y1;
    unsigned nb_ch_y1;
    float *ch_z1;
    unsigned nb_ch_z1;

    AVRational frame_rate;
    int nb_samples;

    float view_matrix[4][4];
    float projection_matrix[4][4];

    AVFrame *frames[60];
} Audio3dScopeContext;

#define OFFSET(x) offsetof(Audio3dScopeContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
#define TFLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_ch_x0 = {.def="-1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_ch_y0 = {.def="-1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_ch_z0 = {.def="-8",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_ch_x1 = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_ch_y1 = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_ch_z1 = {.def="-0.25",.size_min=1,.sep=' '};

static const AVOption a3dscope_options[] = {
    { "rate", "set video rate", OFFSET(frame_rate), AV_OPT_TYPE_VIDEO_RATE, {.str="25"}, 0, INT_MAX, FLAGS },
    { "r",    "set video rate", OFFSET(frame_rate), AV_OPT_TYPE_VIDEO_RATE, {.str="25"}, 0, INT_MAX, FLAGS },
    { "size", "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str="hd720"}, 0, 0, FLAGS },
    { "s",    "set video size", OFFSET(w), AV_OPT_TYPE_IMAGE_SIZE, {.str="hd720"}, 0, 0, FLAGS },
    { "fov",  "set camera FoV", OFFSET(fov),    AV_OPT_TYPE_FLOAT, {.dbl=90.f},  40, 150, TFLAGS },
    { "roll", "set camera roll",OFFSET(roll),   AV_OPT_TYPE_FLOAT, {.dbl=0.f}, -180, 180, TFLAGS },
    { "pitch","set camera pitch",OFFSET(pitch), AV_OPT_TYPE_FLOAT, {.dbl=0.f}, -180, 180, TFLAGS },
    { "yaw",  "set camera yaw",  OFFSET(yaw),   AV_OPT_TYPE_FLOAT, {.dbl=0.f}, -180, 180, TFLAGS },
    { "xzoom","set camera zoom", OFFSET(zoom[0]),AV_OPT_TYPE_FLOAT, {.dbl=1.f}, 0.01,  10, TFLAGS },
    { "yzoom","set camera zoom", OFFSET(zoom[1]),AV_OPT_TYPE_FLOAT, {.dbl=1.f}, 0.01,  10, TFLAGS },
    { "zzoom","set camera zoom", OFFSET(zoom[2]),AV_OPT_TYPE_FLOAT, {.dbl=1.f}, 0.01,  10, TFLAGS },
    { "xpos", "set camera position", OFFSET(eye[0]), AV_OPT_TYPE_FLOAT, {.dbl=0.f},-60.f, 60.f, TFLAGS },
    { "ypos", "set camera position", OFFSET(eye[1]), AV_OPT_TYPE_FLOAT, {.dbl=0.f},-60.f, 60.f, TFLAGS },
    { "zpos", "set camera position", OFFSET(eye[2]), AV_OPT_TYPE_FLOAT, {.dbl=0.f},-60.f, 60.f, TFLAGS },
    { "length","set length",    OFFSET(size),   AV_OPT_TYPE_INT,   {.i64=15},      1,  60,  FLAGS },
    { "ch_x0", "set channel's X-axis start position", OFFSET(ch_x0), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_ch_x0}, -60, 60, FLAGS },
    { "ch_y0", "set channel's Y-axis start position", OFFSET(ch_y0), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_ch_y0}, -60, 60, FLAGS },
    { "ch_z0", "set channel's Z-axis start position", OFFSET(ch_z0), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_ch_z0}, -60, 60, FLAGS },
    { "ch_x1", "set channel's X-axis stop position",  OFFSET(ch_x1), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_ch_x1}, -60, 60, FLAGS },
    { "ch_y1", "set channel's Y-axis stop position",  OFFSET(ch_y1), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_ch_y1}, -60, 60, FLAGS },
    { "ch_z1", "set channel's Z-axis stop position",  OFFSET(ch_z1), AV_OPT_TYPE_FLOAT|AR, {.arr=&def_ch_z1}, -60, 60, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(a3dscope);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    AVFilterFormats *formats = NULL;
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_NONE };
    static const enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_RGBA, AV_PIX_FMT_NONE };
    int ret;

    formats = ff_make_format_list(sample_fmts);
    if ((ret = ff_formats_ref(formats, &cfg_in[0]->formats)) < 0)
        return ret;

    formats = ff_make_format_list(pix_fmts);
    if ((ret = ff_formats_ref(formats, &cfg_out[0]->formats)) < 0)
        return ret;

    return 0;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    Audio3dScopeContext *s = ctx->priv;

    s->nb_samples = FFMAX(1, av_rescale(inlink->sample_rate, s->frame_rate.den, s->frame_rate.num));

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    FilterLink *l = ff_filter_link(outlink);
    Audio3dScopeContext *s = outlink->src->priv;

    outlink->w = s->w;
    outlink->h = s->h;
    outlink->sample_aspect_ratio = (AVRational){1,1};
    l->frame_rate = s->frame_rate;
    outlink->time_base = av_inv_q(l->frame_rate);

    return 0;
}

static void projection_matrix(float fov, float a, float near, float far,
                              float matrix[4][4])
{
    float f;

    memset(matrix, 0, sizeof(*matrix));

    f = 1.0f / tanf(fov * 0.5f * M_PI / 180.f);
    matrix[0][0] = f * a;
    matrix[1][1] = f;
    matrix[2][2] = -(far + near) / (far - near);
    matrix[2][3] = -1.f;
    matrix[3][2] = -(near * far) / (far - near);
}

static inline void vmultiply(const float v[4], const float m[4][4], float d[4])
{
    d[0] = v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0] + v[3] * m[3][0];
    d[1] = v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1] + v[3] * m[3][1];
    d[2] = v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2] + v[3] * m[3][2];
    d[3] = v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3] + v[3] * m[3][3];
}

static void mmultiply(const float m2[4][4], const float m1[4][4], float m[4][4])
{
    vmultiply(m2[0], m1, m[0]);
    vmultiply(m2[1], m1, m[1]);
    vmultiply(m2[2], m1, m[2]);
    vmultiply(m2[3], m1, m[3]);
}

static float vdot(const float x[3], const float y[3])
{
    return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
}

static void view_matrix(const float eye[3],
                        const float z[3],
                        const float roll,
                        const float pitch,
                        const float yaw, float m[4][4])
{
    float cr = cosf(roll * M_PI / 180.f);
    float sr = sinf(roll * M_PI / 180.f);
    float cp = cosf(pitch * M_PI / 180.f);
    float sp = sinf(pitch * M_PI / 180.f);
    float cy = cosf(yaw * M_PI / 180.f);
    float sy = sinf(yaw * M_PI / 180.f);
    float t[4][4];
    float rx[4][4] = {
        {z[0], 0.f, 0.f, 0.f },
        { 0.f, cy,  -sy, 0.f },
        { 0.f, sy,   cy, 0.f },
        { 0.f, 0.f, 0.f, 1.f },
    };

    float ry[4][4] = {
        { cp,  0.f, sp,  0.f },
        { 0.f,z[1], 0.f, 0.f },
        {-sp,  0.f, cp,  0.f },
        { 0.f, 0.f, 0.f, 1.f },
    };

    float rz[4][4] = {
        { cr, -sr,  0.f, 0.f },
        { sr,  cr,  0.f, 0.f },
        { 0.f, 0.f,z[2], 0.f },
        { 0.f, 0.f, 0.f, 1.f },
    };

    memset(m, 0, sizeof(*m));

    mmultiply(rx, ry, t);
    mmultiply(rz,  t, m);

    m[3][0] = -vdot(m[0], eye);
    m[3][1] = -vdot(m[1], eye);
    m[3][2] = -vdot(m[2], eye);
}

static void draw_dot(uint8_t *out, unsigned x, unsigned y, float z,
                     int r, int g, int b, const ptrdiff_t linesize)
{
    uint8_t *dst;

    dst = out + y * linesize + x * 4;
    if (dst[3] == 0) {
        dst[0] = r * z;
        dst[1] = g * z;
        dst[2] = b * z;
        dst[3] = 255 * z;
    }
}

static float lerpf(float a, float b, float x)
{
    const float y = 1.f - x;

    return a * y + b * x;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    Audio3dScopeContext *s = ctx->priv;
    const float half_height = (s->h - 1) * 0.5f;
    const float half_width = (s->w - 1) * 0.5f;
    float matrix[4][4];
    const int w = s->w;
    const int h = s->h;
    AVFrame *out;

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }

    s->frames[0] = in;

    out->sample_aspect_ratio = (AVRational){1,1};
    for (int y = 0; y < outlink->h; y++)
        memset(out->data[0] + y * out->linesize[0], 0, outlink->w * 4);
    out->pts = av_rescale_q(in->pts, inlink->time_base, outlink->time_base);
    out->duration = 1;

    projection_matrix(s->fov, half_width / half_height, 0.1f, 1000000.f, s->projection_matrix);
    view_matrix(s->eye, s->zoom, s->roll, s->pitch, s->yaw, s->view_matrix);
    mmultiply(s->projection_matrix, s->view_matrix, matrix);

    for (int nb_frame = s->size - 1; nb_frame >= 0; nb_frame--) {
        const int nb_samples = s->nb_samples;
        const float scale = 1.f / (nb_samples * s->size);
        AVFrame *frame = s->frames[nb_frame];
        int frame_nb_samples;
        float channels;

        if (!frame)
            continue;

        channels = frame->ch_layout.nb_channels;
        frame_nb_samples = frame->nb_samples;
        for (int ch = 0; ch < channels; ch++) {
            const float *src = (float *)frame->extended_data[ch];
            const int r = 128.f + 127.f * sinf(ch / (channels - 1) * M_PI);
            const int g = 128.f + 127.f * ch / (channels - 1);
            const int b = 128.f + 127.f * cosf(ch / (channels - 1) * M_PI);
            const ptrdiff_t linesize = out->linesize[0];
            const float ch_x0 = s->ch_x0[FFMIN(ch, s->nb_ch_x0-1)];
            const float ch_x1 = s->ch_x1[FFMIN(ch, s->nb_ch_x1-1)];
            const float ch_y0 = s->ch_y0[FFMIN(ch, s->nb_ch_y0-1)];
            const float ch_y1 = s->ch_y1[FFMIN(ch, s->nb_ch_y1-1)];
            const float ch_z0 = s->ch_z0[FFMIN(ch, s->nb_ch_z0-1)];
            const float ch_z1 = s->ch_z1[FFMIN(ch, s->nb_ch_z1-1)];
            const float chan = ch / (float)(channels-1);
            uint8_t *dst = out->data[0];

            for (int n = frame_nb_samples - 1, nn = nb_samples * nb_frame; n >= 0; n--, nn++) {
                const float ch_x = lerpf(ch_x0, ch_x1, 0.5f*(1.f + src[n]));
                const float ch_y = lerpf(ch_y0, ch_y1, chan);
                const float ch_z = lerpf(ch_z0, ch_z1, nn * scale);
                float v[4] = { ch_x, ch_y, ch_z,  1.f };
                float d[4];
                int x, y;

                vmultiply(v, matrix, d);

                d[0] /= d[3];
                x = d[0] * half_width  + half_width;
                if (x >= w || x < 0)
                    continue;

                d[1] /= d[3];
                y = d[1] * half_height + half_height;
                if (y >= h || y < 0)
                    continue;

                draw_dot(dst, x, y, av_clipf(1.f / d[3], 0.f, 1.f),
                         r, g, b, linesize);
            }
        }
    }

    ff_graph_frame_free(ctx, &s->frames[59]);
    memmove(&s->frames[1], &s->frames[0], 59 * sizeof(AVFrame *));
    s->frames[0] = NULL;

    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    Audio3dScopeContext *s = ctx->priv;
    AVFrame *in;
    int ret;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_samples(inlink, s->nb_samples, s->nb_samples, &in);
    if (ret < 0)
        return ret;
    if (ret > 0)
        return filter_frame(inlink, in);

    if (ff_inlink_queued_samples(inlink) >= s->nb_samples) {
        ff_filter_set_ready(ctx, 10);
        return 0;
    }

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    Audio3dScopeContext *s = ctx->priv;

    for (int n = 0; n < 60; n++)
        av_frame_free(&s->frames[n]);
}

static const AVFilterPad audio3dscope_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
    },
};

static const AVFilterPad audio3dscope_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_output,
    },
};

const FFFilter ff_avf_a3dscope = {
    .p.name        = "a3dscope",
    .p.description = NULL_IF_CONFIG_SMALL("Convert input audio to 3d scope video output."),
    .p.priv_class  = &a3dscope_class,
    .uninit        = uninit,
    .priv_size     = sizeof(Audio3dScopeContext),
    .activate      = activate,
    FILTER_INPUTS(audio3dscope_inputs),
    FILTER_OUTPUTS(audio3dscope_outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .process_command = ff_filter_process_command,
};
