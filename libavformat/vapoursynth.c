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

/**
* @file
* VapourSynth demuxer
*
* Synthesizes vapour (?)
*/

#include <limits.h>

#include <vapoursynth/VSScript4.h>

#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/eval.h"
#include "libavutil/frame.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"

/* Platform-specific directives. */
#ifdef _WIN32
  #include <windows.h>
  #include "compat/w32dlfcn.h"
  #include "libavutil/wchar_filename.h"
  #undef EXTERN_C
  #define VSSCRIPT_LIB "VSScript.dll"
#else
  #include <dlfcn.h>
  #define VSSCRIPT_NAME "libvapoursynth-script"
  #define VSSCRIPT_LIB VSSCRIPT_NAME SLIBSUF
#endif

struct VSState {
    const VSSCRIPTAPI *vssapi;
    VSScript *vss;
};

typedef const VSSCRIPTAPI *(*VSScriptGetAPIFunc)(int version);

typedef struct VSContext {
    const AVClass *class;

    AVBufferRef *vss_state;

    const VSSCRIPTAPI *vssapi;
    const VSAPI *vsapi;
    void *vslibrary;

    VSNode *outnode;
    int is_cfr;
    int current_frame;

    int c_order[4];

    /* options */
    int64_t max_script_size;
} VSContext;

#define OFFSET(x) offsetof(VSContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
#define D AV_OPT_FLAG_DECODING_PARAM
static const AVOption options[] = {
    {"max_script_size",    "set max file size supported (in bytes)", OFFSET(max_script_size),    AV_OPT_TYPE_INT64, {.i64 = 1 * 1024 * 1024}, 0,    SIZE_MAX - 1, A|D},
    {NULL}
};

static av_cold void* vs_load_library(VSScriptGetAPIFunc *get_vssapi)
{
    void *vslibrary = NULL;
#ifdef _WIN32
    const HKEY hkeys[] = {HKEY_CURRENT_USER, HKEY_LOCAL_MACHINE};
    LONG r;
    WCHAR vss_path[512];
    DWORD buf_size = sizeof(vss_path) - 2;
    char *vss_path_utf8;
    int i;

    for (i = 0; i < FF_ARRAY_ELEMS(hkeys); i++) {
        if ((r = RegGetValueW(hkeys[i], L"SOFTWARE\\VapourSynth",
                      L"VSScriptDLL", RRF_RT_REG_SZ, NULL,
                      &vss_path, &buf_size)) == ERROR_SUCCESS)
            break;
    }
    if (r == ERROR_SUCCESS && wchartoutf8(vss_path, &vss_path_utf8) == 0) {
        vslibrary = dlopen(vss_path_utf8, RTLD_NOW | RTLD_GLOBAL);
        av_free(vss_path_utf8);
    }
    else
#endif
    vslibrary = dlopen(VSSCRIPT_LIB, RTLD_NOW | RTLD_GLOBAL);

    if (vslibrary != NULL) {
        if (!(*get_vssapi = (VSScriptGetAPIFunc)dlsym(vslibrary, "getVSScriptAPI"))) {
            dlclose(vslibrary);
            return NULL;
        }
    }
    return vslibrary;
}

static void free_vss_state(void *opaque, uint8_t *data)
{
    struct VSState *vss = opaque;

    if (vss->vss) {
        vss->vssapi->freeScript(vss->vss);
    }
}

static av_cold int read_close_vs(AVFormatContext *s)
{
    VSContext *vs = s->priv_data;

    if (vs->outnode)
        vs->vsapi->freeNode(vs->outnode);

    av_buffer_unref(&vs->vss_state);

    vs->vsapi = NULL;
    vs->outnode = NULL;

    if (vs->vslibrary)
        dlclose(vs->vslibrary);

    return 0;
}

static av_cold int is_native_endian(enum AVPixelFormat pixfmt)
{
    enum AVPixelFormat other = av_pix_fmt_swap_endianness(pixfmt);
    const AVPixFmtDescriptor *pd;
    if (other == AV_PIX_FMT_NONE || other == pixfmt)
        return 1; // not affected by byte order
    pd = av_pix_fmt_desc_get(pixfmt);
    return pd && (!!HAVE_BIGENDIAN == !!(pd->flags & AV_PIX_FMT_FLAG_BE));
}

static av_cold enum AVPixelFormat match_pixfmt(const VSVideoFormat *vsf, int c_order[4])
{
    static const int yuv_order[4] = {0, 1, 2, 0};
    static const int rgb_order[4] = {1, 2, 0, 0};
    const AVPixFmtDescriptor *pd;

    for (pd = av_pix_fmt_desc_next(NULL); pd; pd = av_pix_fmt_desc_next(pd)) {
        int is_rgb, is_yuv, i;
        const int *order;
        enum AVPixelFormat pixfmt;

        pixfmt = av_pix_fmt_desc_get_id(pd);

        if (pd->flags & (AV_PIX_FMT_FLAG_BAYER | AV_PIX_FMT_FLAG_ALPHA |
                         AV_PIX_FMT_FLAG_HWACCEL | AV_PIX_FMT_FLAG_BITSTREAM |
                         AV_PIX_FMT_FLAG_XYZ))
            continue;

        if (pd->log2_chroma_w != vsf->subSamplingW ||
            pd->log2_chroma_h != vsf->subSamplingH)
            continue;

        is_rgb = vsf->colorFamily == cfRGB;
        if (is_rgb != !!(pd->flags & AV_PIX_FMT_FLAG_RGB))
            continue;

        is_yuv = vsf->colorFamily == cfYUV ||
                 vsf->colorFamily == cfGray;
        if (!is_rgb && !is_yuv)
            continue;

        if (vsf->sampleType != ((pd->flags & AV_PIX_FMT_FLAG_FLOAT) ? stFloat : stInteger))
            continue;

        if (av_pix_fmt_count_planes(pixfmt) != vsf->numPlanes)
            continue;

        if (!is_native_endian(pixfmt))
            continue;

        order = is_yuv ? yuv_order : rgb_order;

        for (i = 0; i < pd->nb_components; i++) {
            const AVComponentDescriptor *c = &pd->comp[i];
            if (order[c->plane] != i ||
                c->offset != 0 || c->shift != 0 ||
                c->step != vsf->bytesPerSample ||
                c->depth != vsf->bitsPerSample)
                goto cont;
        }

        // Use it.
        memcpy(c_order, order, sizeof(int[4]));
        return pixfmt;

    cont: ;
    }

    return AV_PIX_FMT_NONE;
}

static av_cold int read_header_vs(AVFormatContext *s)
{
    AVStream *st;
    AVIOContext *pb = s->pb;
    VSContext *vs = s->priv_data;
    VSScriptGetAPIFunc get_vssapi;
    int64_t sz = avio_size(pb);
    char *buf = NULL;
    char dummy;
    char vsfmt[32];
    const VSVideoInfo *info;
    struct VSState *vss_state;
    int err = 0;

    if (!(vs->vslibrary = vs_load_library(&get_vssapi))) {
        av_log(s, AV_LOG_ERROR, "Could not open " VSSCRIPT_LIB ". "
                                "Check VapourSynth installation.\n");
        err = AVERROR_EXTERNAL;
        goto done;
    }

    if (!(vs->vssapi = get_vssapi(VSSCRIPT_API_VERSION))) {
        av_log(s, AV_LOG_ERROR, "Failed to initialize VSScript (possibly PYTHONPATH not set).\n");
        err = AVERROR_EXTERNAL;
        goto done;
    }

    if (!(vs->vsapi = vs->vssapi->getVSAPI(VAPOURSYNTH_API_VERSION))) {
        av_log(s, AV_LOG_ERROR, "Could not get VSAPI. "
                                "Check VapourSynth installation.\n");
        err = AVERROR_EXTERNAL;
        goto done;
    }

    vss_state = av_mallocz(sizeof(*vss_state));
    if (!vss_state) {
        err = AVERROR(ENOMEM);
        goto done;
    }
    vss_state->vssapi = vs->vssapi;

    vs->vss_state = av_buffer_create(NULL, 0, free_vss_state, vss_state, 0);
    if (!vs->vss_state) {
        err = AVERROR(ENOMEM);
        av_free(vss_state);
        goto done;
    }

    if (!(vss_state->vss = vs->vssapi->createScript(NULL))) {
        av_log(s, AV_LOG_ERROR, "Failed to create script instance.\n");
        err = AVERROR_EXTERNAL;
        goto done;
    }

    if (sz < 0 || sz > vs->max_script_size) {
        if (sz < 0)
            av_log(s, AV_LOG_WARNING, "Could not determine file size\n");
        sz = vs->max_script_size;
    }

    buf = av_malloc(sz + 1);
    if (!buf) {
        err = AVERROR(ENOMEM);
        goto done;
    }
    sz = avio_read(pb, buf, sz);

    if (sz < 0) {
        av_log(s, AV_LOG_ERROR, "Could not read script.\n");
        err = sz;
        goto done;
    }

    // Data left means our buffer (the max_script_size option) is too small
    if (avio_read(pb, &dummy, 1) == 1) {
        av_log(s, AV_LOG_ERROR, "File size is larger than max_script_size option "
               "value %"PRIi64", consider increasing the max_script_size option\n",
               vs->max_script_size);
        err = AVERROR_BUFFER_TOO_SMALL;
        goto done;
    }

    buf[sz] = '\0';
    if (vs->vssapi->evaluateBuffer(vss_state->vss, buf, s->url)) {
        const char *msg = vs->vssapi->getError(vss_state->vss);
        av_log(s, AV_LOG_ERROR, "Failed to parse script: %s\n", msg ? msg : "(unknown)");
        err = AVERROR_EXTERNAL;
        goto done;
    }

    vs->outnode = vs->vssapi->getOutputNode(vss_state->vss, 0);
    if (!vs->outnode) {
        av_log(s, AV_LOG_ERROR, "Could not get script output node.\n");
        err = AVERROR_EXTERNAL;
        goto done;
    }

    st = avformat_new_stream(s, NULL);
    if (!st) {
        err = AVERROR(ENOMEM);
        goto done;
    }

    info = vs->vsapi->getVideoInfo(vs->outnode);

    if (!info->format.colorFamily || !info->width || !info->height) {
        av_log(s, AV_LOG_ERROR, "Non-constant input format not supported.\n");
        err = AVERROR_PATCHWELCOME;
        goto done;
    }

    if (info->fpsDen) {
        vs->is_cfr = 1;
        avpriv_set_pts_info(st, 64, info->fpsDen, info->fpsNum);
        st->duration = info->numFrames;
    } else {
        // VFR. Just set "something".
        avpriv_set_pts_info(st, 64, 1, AV_TIME_BASE);
        s->ctx_flags |= AVFMTCTX_UNSEEKABLE;
    }

    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id = AV_CODEC_ID_WRAPPED_AVFRAME;
    st->codecpar->width = info->width;
    st->codecpar->height = info->height;
    st->codecpar->format = match_pixfmt(&info->format, vs->c_order);

    if (st->codecpar->format == AV_PIX_FMT_NONE) {
        av_log(s, AV_LOG_ERROR, "Unsupported VS pixel format %s\n",
               vs->vsapi->getVideoFormatName(&info->format, vsfmt) ? vsfmt : "(unknown)");
        err = AVERROR_EXTERNAL;
        goto done;
    }
    av_log(s, AV_LOG_VERBOSE, "VS format %s -> pixfmt %s\n",
           vs->vsapi->getVideoFormatName(&info->format, vsfmt) ? vsfmt : "(unknown)",
           av_get_pix_fmt_name(st->codecpar->format));

done:
    av_free(buf);
    return err;
}

static void free_frame(void *opaque, uint8_t *data)
{
    AVFrame *frame = (AVFrame *)data;

    av_frame_free(&frame);
}

static int get_vs_prop_int(AVFormatContext *s, const VSMap *map, const char *name, int def)
{
    VSContext *vs = s->priv_data;
    int64_t res;
    int err = 1;

    res = vs->vsapi->mapGetInt(map, name, 0, &err);
    return err || res < INT_MIN || res > INT_MAX ? def : res;
}

struct vsframe_ref_data {
    const VSAPI *vsapi;
    const VSFrame *frame;
    AVBufferRef *vss_state;
};

static void free_vsframe_ref(void *opaque, uint8_t *data)
{
    struct vsframe_ref_data *d = opaque;

    if (d->frame)
        d->vsapi->freeFrame(d->frame);

    av_buffer_unref(&d->vss_state);

    av_free(d);
}

static int read_packet_vs(AVFormatContext *s, AVPacket *pkt)
{
    VSContext *vs = s->priv_data;
    AVStream *st = s->streams[0];
    AVFrame *frame = NULL;
    char vserr[80];
    const VSFrame *vsframe;
    const VSVideoInfo *info = vs->vsapi->getVideoInfo(vs->outnode);
    const VSMap *props;
    const AVPixFmtDescriptor *desc;
    AVBufferRef *vsframe_ref = NULL;
    struct vsframe_ref_data *ref_data;
    int err = 0;
    int i;

    if (vs->current_frame >= info->numFrames)
        return AVERROR_EOF;

    ref_data = av_mallocz(sizeof(*ref_data));
    if (!ref_data) {
        err = AVERROR(ENOMEM);
        goto end;
    }

    // (the READONLY flag is important because the ref is reused for plane data)
    vsframe_ref = av_buffer_create(NULL, 0, free_vsframe_ref, ref_data, AV_BUFFER_FLAG_READONLY);
    if (!vsframe_ref) {
        err = AVERROR(ENOMEM);
        av_free(ref_data);
        goto end;
    }

    vsframe = vs->vsapi->getFrame(vs->current_frame, vs->outnode, vserr, sizeof(vserr));
    if (!vsframe) {
        av_log(s, AV_LOG_ERROR, "Error getting frame: %s\n", vserr);
        err = AVERROR_EXTERNAL;
        goto end;
    }

    ref_data->vsapi = vs->vsapi;
    ref_data->frame = vsframe;

    ref_data->vss_state = av_buffer_ref(vs->vss_state);
    if (!ref_data->vss_state) {
        err = AVERROR(ENOMEM);
        goto end;
    }

    props = vs->vsapi->getFramePropertiesRO(vsframe);

    frame = av_frame_alloc();
    if (!frame) {
        err = AVERROR(ENOMEM);
        goto end;
    }

    frame->format       = st->codecpar->format;
    frame->width        = st->codecpar->width;
    frame->height       = st->codecpar->height;
    frame->colorspace   = st->codecpar->color_space;

    // Values according to ISO/IEC 14496-10.
    frame->colorspace       = get_vs_prop_int(s, props, "_Matrix",      frame->colorspace);
    frame->color_primaries  = get_vs_prop_int(s, props, "_Primaries",   frame->color_primaries);
    frame->color_trc        = get_vs_prop_int(s, props, "_Transfer",    frame->color_trc);

    if (get_vs_prop_int(s, props, "_ColorRange", 1) == 0)
        frame->color_range = AVCOL_RANGE_JPEG;

    frame->sample_aspect_ratio.num = get_vs_prop_int(s, props, "_SARNum", 0);
    frame->sample_aspect_ratio.den = get_vs_prop_int(s, props, "_SARDen", 1);

    av_assert0(vs->vsapi->getFrameWidth(vsframe, 0) == frame->width);
    av_assert0(vs->vsapi->getFrameHeight(vsframe, 0) == frame->height);

    desc = av_pix_fmt_desc_get(frame->format);

    for (i = 0; i < info->format.numPlanes; i++) {
        int p = vs->c_order[i];
        ptrdiff_t plane_h = frame->height;

        frame->data[i] = (void *)vs->vsapi->getReadPtr(vsframe, p);
        frame->linesize[i] = vs->vsapi->getStride(vsframe, p);

        frame->buf[i] = av_buffer_ref(vsframe_ref);
        if (!frame->buf[i]) {
            err = AVERROR(ENOMEM);
            goto end;
        }

        // Each plane needs an AVBufferRef that indicates the correct plane
        // memory range. VapourSynth doesn't even give us the memory range,
        // so make up a bad guess to make FFmpeg happy (even if almost nothing
        // checks the memory range).
        if (i == 1 || i == 2)
            plane_h = AV_CEIL_RSHIFT(plane_h, desc->log2_chroma_h);
        frame->buf[i]->data = frame->data[i];
        frame->buf[i]->size = frame->linesize[i] * plane_h;
    }

    pkt->buf = av_buffer_create((uint8_t*)frame, sizeof(*frame),
                                free_frame, NULL, 0);
    if (!pkt->buf) {
        err = AVERROR(ENOMEM);
        goto end;
    }

    frame = NULL; // pkt owns it now

    pkt->data   = pkt->buf->data;
    pkt->size   = pkt->buf->size;
    pkt->flags |= AV_PKT_FLAG_TRUSTED;

    if (vs->is_cfr)
        pkt->pts = vs->current_frame;

    vs->current_frame++;

end:
    av_frame_free(&frame);
    av_buffer_unref(&vsframe_ref);
    return err;
}

static int read_seek_vs(AVFormatContext *s, int stream_idx, int64_t ts, int flags)
{
    VSContext *vs = s->priv_data;

    if (!vs->is_cfr)
        return AVERROR(ENOSYS);

    vs->current_frame = FFMIN(FFMAX(0, ts), s->streams[0]->duration);
    return 0;
}

static av_cold int probe_vs(const AVProbeData *p)
{
    // Explicitly do not support this. VS scripts are written in Python, and
    // can run arbitrary code on the user's system.
    return 0;
}

static const AVClass class_vs = {
    .class_name = "VapourSynth demuxer",
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFInputFormat ff_vapoursynth_demuxer = {
    .p.name         = "vapoursynth",
    .p.long_name    = NULL_IF_CONFIG_SMALL("VapourSynth demuxer"),
    .p.priv_class   = &class_vs,
    .priv_data_size = sizeof(VSContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = probe_vs,
    .read_header    = read_header_vs,
    .read_packet    = read_packet_vs,
    .read_close     = read_close_vs,
    .read_seek      = read_seek_vs,
};
