/*
 * Copyright (c) 2024 Paul B Mahol
 *
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

/**
 * @file
 * CLAP wrapper
 */

#include <dlfcn.h>
#include <clap/audio-buffer.h>
#include <clap/entry.h>
#include <clap/events.h>
#include <clap/ext/audio-ports.h>
#include <clap/ext/log.h>
#include <clap/ext/params.h>
#include <clap/factory/plugin-factory.h>
#include <clap/host.h>

#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"

typedef struct PluginControl {
    uint32_t id;
    double val;

    clap_event_param_value_t ev;
} PluginControl;

typedef struct CLAPContext {
    const AVClass *class;
    char *dl_name;
    char *plugin;
    char **options;
    unsigned nb_options;
    void *dl_handle;

    clap_plugin_entry_t *plugin_entry;
    clap_plugin_factory_t *plugin_factory;
    clap_version_t clap_version;

    const clap_plugin_descriptor_t *plugin_desc;
    const clap_plugin_t *clap_plugin;
    clap_plugin_audio_ports_t *clap_ports;

    clap_input_events_t in_events;
    clap_output_events_t out_events;

    clap_audio_buffer_t *in_buffers;
    clap_audio_buffer_t *out_buffers;

    PluginControl *plugin_controls;

    int update_controls;
    int activated;
    int processing;
    int sample_rate;
    unsigned min_nb_samples;
    unsigned max_nb_samples;
    uint32_t in_ports;
    uint32_t out_ports;
    uint32_t control_count;

    AVFrame **in_frames;
    AVFrame **out_frames;

    int64_t steady_time;
    int64_t pts;
    int64_t duration;
} CLAPContext;

#define OFFSET(x) offsetof(CLAPContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_FILTERING_PARAM
#define TFLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY

static const AVOptionArrayDef def_controls = {.def=NULL,.size_min=0,.sep='|'};

static const AVOption clap_options[] = {
    { "file", "set library name or full path", OFFSET(dl_name), AV_OPT_TYPE_STRING, .flags = FLAGS },
    { "f",    "set library name or full path", OFFSET(dl_name), AV_OPT_TYPE_STRING, .flags = FLAGS },
    { "plugin", "set plugin name", OFFSET(plugin), AV_OPT_TYPE_STRING, .flags = FLAGS },
    { "p",      "set plugin name", OFFSET(plugin), AV_OPT_TYPE_STRING, .flags = FLAGS },
    { "controls", "set plugin options", OFFSET(options), AV_OPT_TYPE_STRING|AR, {.arr=&def_controls}, .flags = TFLAGS },
    { "c",        "set plugin options", OFFSET(options), AV_OPT_TYPE_STRING|AR, {.arr=&def_controls}, .flags = TFLAGS },
    { "min_nb_samples", "set the min number of samples per frame", OFFSET(min_nb_samples), AV_OPT_TYPE_INT, {.i64=32},   1, INT_MAX, FLAGS },
    { "minn",           "set the min number of samples per frame", OFFSET(min_nb_samples), AV_OPT_TYPE_INT, {.i64=32},   1, INT_MAX, FLAGS },
    { "max_nb_samples", "set the max number of samples per frame", OFFSET(max_nb_samples), AV_OPT_TYPE_INT, {.i64=4096}, 1, INT_MAX, FLAGS },
    { "maxn",           "set the max number of samples per frame", OFFSET(max_nb_samples), AV_OPT_TYPE_INT, {.i64=4096}, 1, INT_MAX, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(clap);

static int filter_frame(AVFilterContext *ctx)
{
    CLAPContext *s = ctx->priv;
    clap_process_t process;
    int ret, err;

    for (int i = 0; i < s->out_ports; i++) {
        s->out_frames[i] = ff_get_audio_buffer(ctx->outputs[i],
                                               s->in_frames[0]->nb_samples);
        if (!s->out_frames[i]) {
            for (int j = 0; j < s->in_ports; j++)
                av_frame_free(&s->in_frames[j]);
            return AVERROR(ENOMEM);
        }
        if (s->in_frames[0])
            av_frame_copy_props(s->out_frames[i], s->in_frames[0]);
    }

    for (int i = 0; i < s->in_ports; i++) {
        s->in_buffers[i].data64 = NULL;

        for (int ch = 0; ch < s->in_buffers[i].channel_count; ch++)
            s->in_buffers[i].data32[ch] = (float *)s->in_frames[i]->extended_data[ch];
    }

    for (int i = 0; i < s->out_ports; i++) {
        s->out_buffers[i].data64 = NULL;

        for (int ch = 0; ch < s->out_buffers[i].channel_count; ch++)
            s->out_buffers[i].data32[ch] = (float *)s->out_frames[i]->extended_data[ch];
    }

    process.steady_time = s->steady_time;
    process.frames_count = s->in_frames[0]->nb_samples;
    process.transport = NULL;

    process.audio_inputs = s->in_buffers;
    process.audio_inputs_count = s->in_ports;

    process.audio_outputs = s->out_buffers;
    process.audio_outputs_count = s->out_ports;

    process.in_events = &s->in_events;
    process.out_events = &s->out_events;

    if (s->update_controls) {
        s->update_controls = 0;
    }

    if (!s->clap_plugin->process) {
        for (int i = 0; i < s->in_ports; i++)
            av_frame_free(&s->in_frames[i]);

        for (int i = 0; i < s->out_ports; i++)
            av_frame_free(&s->out_frames[i]);

        return AVERROR_BUG;
    }

    err = s->clap_plugin->process(s->clap_plugin, &process);

    s->steady_time += s->in_frames[0]->nb_samples;

    for (int i = 0; i < s->in_ports; i++)
        av_frame_free(&s->in_frames[i]);

    for (int i = 0; i < s->out_ports; i++) {
        if (!err) {
            av_frame_free(&s->out_frames[i]);
            ret = AVERROR(EINVAL);
            continue;
        }
        ret = ff_filter_frame(ctx->outputs[i], s->out_frames[i]);
        s->out_frames[i] = NULL;
    }

    return ret;
}

static void *try_load(const char *dir, const char *soname)
{
    char *path = av_asprintf("%s/%s.so", dir, soname);
    void *ret = NULL;

    if (path) {
        ret = dlopen(path, RTLD_LOCAL|RTLD_LAZY);
        av_free(path);
    }

    return ret;
}

static void clap_avlog(const clap_host_t *h,
                       clap_log_severity severity,
                       const char *message)
{
    int level = AV_LOG_ERROR;

    switch (severity) {
    case CLAP_LOG_DEBUG:
        level = AV_LOG_DEBUG;
        break;
    case CLAP_LOG_INFO:
        level = AV_LOG_INFO;
        break;
    case CLAP_LOG_WARNING:
        level = AV_LOG_WARNING;
        break;
    case CLAP_LOG_ERROR:
        level = AV_LOG_ERROR;
        break;
    case CLAP_LOG_FATAL:
        level = AV_LOG_FATAL;
        break;
    }

    av_log(h->host_data, level, "%s\n", message);
}

static const clap_host_log_t clap_log = {
    clap_avlog,
};

static void clap_params_rescan(const clap_host_t *h,
                               clap_param_rescan_flags flags)
{
    AVFilterContext *ctx = h->host_data;
    av_log(ctx, AV_LOG_DEBUG, "clap_params_rescan\n");
    //params_rescan(flags);
}

static void clap_params_clear(const clap_host_t *h,
                              clap_id param_id,
                              clap_param_clear_flags flags)
{
    AVFilterContext *ctx = h->host_data;
    av_log(ctx, AV_LOG_DEBUG, "clap_params_clear\n");
    //paramsClear(param_id, flags);
}

static void clap_params_request_flush(const clap_host_t *h)
{
    AVFilterContext *ctx = h->host_data;
    av_log(ctx, AV_LOG_DEBUG, "clap_params_request_flush\n");
    //paramsRequestFlush();
}

static const clap_host_params_t clap_params = {
    clap_params_rescan,
    clap_params_clear,
    clap_params_request_flush,
};

static const void *get_extension(const clap_host_t *h, const char *eid)
{
    AVFilterContext *ctx = h->host_data;

    av_log(ctx, AV_LOG_DEBUG, "get_extension %s\n", eid);

    if (!strcmp(eid, CLAP_EXT_LOG))
         return &clap_log;

    if (!strcmp(eid, CLAP_EXT_PARAMS))
        return &clap_params;

    return NULL;
}

static void request_restart(const clap_host_t *h)
{
    AVFilterContext *ctx = h->host_data;
    av_log(ctx, AV_LOG_DEBUG, "restart\n");
}

static void request_process(const clap_host_t *h)
{
    AVFilterContext *ctx = h->host_data;
    av_log(ctx, AV_LOG_DEBUG, "process\n");
}

static void request_callback(const clap_host_t *h)
{
    AVFilterContext *ctx = h->host_data;
    av_log(ctx, AV_LOG_DEBUG, "callback\n");
}

static const clap_event_header_t *get(const clap_input_events_t *e, uint32_t index)
{
    CLAPContext *s = e->ctx;

    return (clap_event_header_t *)&s->plugin_controls[index].ev;
}

static uint32_t size(const clap_input_events_t *e)
{
    CLAPContext *s = e->ctx;

    return FFMIN(s->nb_options, s->control_count);
}

static bool try_push(const clap_output_events_t *list, const clap_event_header_t *event)
{
    av_log(NULL, AV_LOG_DEBUG, "try_push\n");
    return 0;
}

static clap_host_t avfilter_host =
{
    CLAP_VERSION_INIT, NULL,          "FFmpeg",        "FFmpeg developers", "http://ffmpeg.org",
        "1.1.1",       get_extension, request_restart, request_process,     request_callback
};

static int process_options(AVFilterContext *ctx, clap_plugin_params_t *plugin_params)
{
    CLAPContext *s = ctx->priv;

    for (int i = 0; i < s->nb_options; i++) {
        const char *arg = s->options[i];
        clap_param_info_t inf;
        const char *vstr;
        double val;
        int j;

        if (i >= s->control_count)
            break;

        vstr = strstr(arg, "=");
        if (vstr == NULL) {
            av_log(ctx, AV_LOG_ERROR, "Invalid syntax.\n");
            return AVERROR(EINVAL);
        }

        for (j = 0; j < s->control_count; j++) {
            plugin_params->get_info(s->clap_plugin, j, &inf);
            if ((strlen(inf.name) == vstr-arg) &&
                !strncmp(arg, inf.name, vstr-arg))
                break;
        }

        if (j >= s->control_count) {
            av_log(ctx, AV_LOG_ERROR, "Option: %s not found.\n", arg);
            continue;
        }

        s->plugin_controls[i].id = inf.id;

        val  = atof(vstr+1);
        if (val < inf.min_value || val > inf.max_value) {
            av_log(ctx, AV_LOG_WARNING, "[%d] value %g out of range: [%g, %g]\n",
                   j, val, inf.min_value, inf.max_value);
        } else {
            clap_event_param_value_t *ev = &s->plugin_controls[i].ev;

            ev->header.time = 0;
            ev->header.type = CLAP_EVENT_PARAM_VALUE;
            ev->header.space_id = CLAP_CORE_EVENT_SPACE_ID;
            ev->header.flags = 0;
            ev->header.size = sizeof(*ev);
            ev->param_id = inf.id;
            ev->cookie = NULL;
            ev->port_index = 0;
            ev->key = -1;
            ev->channel = -1;
            ev->note_id = -1;
            ev->value = val;

            s->plugin_controls[i].val = val;
        }
    }

    s->update_controls = s->nb_options != 0;

    return 0;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    CLAPContext *s = ctx->priv;
    const int i = FF_INLINK_IDX(inlink);
    clap_audio_port_info_t inf;

    s->clap_ports->get(s->clap_plugin, i, 1, &inf);

    s->in_buffers[i].latency = 0;
    s->in_buffers[i].constant_mask = 0;
    s->in_buffers[i].channel_count = inf.channel_count;
    s->in_buffers[i].data32 = av_calloc(inf.channel_count, sizeof(*s->in_buffers[i].data32));
    if (!s->in_buffers[i].data32)
        return AVERROR(ENOMEM);

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    CLAPContext *s = ctx->priv;
    const int i = FF_OUTLINK_IDX(outlink);
    clap_audio_port_info_t inf;

    s->clap_ports->get(s->clap_plugin, i, 0, &inf);

    s->out_buffers[i].latency = 0;
    s->out_buffers[i].constant_mask = 0;
    s->out_buffers[i].channel_count = inf.channel_count;
    s->out_buffers[i].data32 = av_calloc(inf.channel_count, sizeof(*s->out_buffers[i].data32));
    if (!s->out_buffers[i].data32)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    CLAPContext *s = ctx->priv;
    AVFilterPad pad = { NULL };
    uint32_t plugin_count;
    uint32_t plugin_index;

    pad.type = AVMEDIA_TYPE_AUDIO;

    if (!s->dl_name) {
        av_log(ctx, AV_LOG_ERROR, "No plugin name provided\n");
        return AVERROR(EINVAL);
    }

    if (s->dl_name[0] == '/' || s->dl_name[0] == '.') {
        // argument is a path
        s->dl_handle = dlopen(s->dl_name, RTLD_LOCAL|RTLD_LAZY);
    } else {
        // argument is a shared object name
        char *p, *arg, *saveptr = NULL;
        char *paths = av_strdup(getenv("CLAP_PATH"));
        const char *home_path = getenv("HOME");
        const char *separator = ":";

        if (paths) {
            p = paths;
            while ((arg = av_strtok(p, separator, &saveptr)) && !s->dl_handle) {
                s->dl_handle = try_load(arg, s->dl_name);
                p = NULL;
            }
        }

        av_free(paths);
        if (!s->dl_handle && home_path && (paths = av_asprintf("%s/.clap", home_path))) {
            s->dl_handle = try_load(paths, s->dl_name);
            av_free(paths);
        }

        if (!s->dl_handle && home_path && (paths = av_asprintf("%s/.clap/lib", home_path))) {
            s->dl_handle = try_load(paths, s->dl_name);
            av_free(paths);
        }

        if (!s->dl_handle)
            s->dl_handle = try_load("/usr/local/lib/clap", s->dl_name);

        if (!s->dl_handle)
            s->dl_handle = try_load("/usr/lib/clap", s->dl_name);
    }
    if (!s->dl_handle) {
        av_log(ctx, AV_LOG_ERROR, "Failed to load '%s'\n", s->dl_name);
        return AVERROR(EINVAL);
    }

    s->plugin_entry = dlsym(s->dl_handle, "clap_entry");
    if (!s->plugin_entry) {
        av_log(ctx, AV_LOG_ERROR, "Could not find clap_entry: %s\n", dlerror());
        return AVERROR(EINVAL);
    }

    s->clap_version = s->plugin_entry->clap_version;
    av_log(ctx, AV_LOG_VERBOSE, "CLAP version: %d.%d.%d\n",
           s->clap_version.major, s->clap_version.minor, s->clap_version.revision);

    if (!s->plugin_entry->init(NULL)) {
        av_log(ctx, AV_LOG_ERROR, "Could not init plugin\n");
        return AVERROR(EINVAL);
    }

    s->plugin_factory = (clap_plugin_factory_t *)s->plugin_entry->get_factory(CLAP_PLUGIN_FACTORY_ID);
    if (!s->plugin_factory) {
        av_log(ctx, AV_LOG_ERROR, "Could not get plugin factory\n");
        return AVERROR(EINVAL);
    }

    plugin_count = s->plugin_factory->get_plugin_count(s->plugin_factory);
    if (plugin_count == 0) {
        av_log(ctx, AV_LOG_INFO, "Plugin factory has no plugins.\n");
        return AVERROR_EXIT;
    }

    plugin_index = 0;
    if (s->plugin) {
        for (int i = 0; i < plugin_count; i++) {
            const clap_plugin_descriptor_t *plugin_desci = s->plugin_factory->get_plugin_descriptor(s->plugin_factory, i);

            if (!strcmp(s->plugin, plugin_desci->id)) {
                plugin_index = i;
                break;
            }

            if (i+1 == plugin_count) {
                av_log(ctx, AV_LOG_ERROR, "Requested plugin '%s' is not found.\n", s->plugin);
                av_log(ctx, AV_LOG_ERROR, "Available plugins are:\n");

                for (int j = 0; j < plugin_count; j++) {
                    const clap_plugin_descriptor_t *plugin_descj = s->plugin_factory->get_plugin_descriptor(s->plugin_factory, j);

                    av_log(ctx, AV_LOG_ERROR, "%s\n", plugin_descj->id);
                }

                return AVERROR_EXIT;
            }
        }
    }

    s->plugin_desc = s->plugin_factory->get_plugin_descriptor(s->plugin_factory, plugin_index);
    if (!s->plugin_desc) {
        av_log(ctx, AV_LOG_ERROR, "Could not get plugin factory descriptor\n");
        return AVERROR(EINVAL);
    }

    if (!clap_version_is_compatible(s->plugin_desc->clap_version)) {
        av_log(ctx, AV_LOG_ERROR, "Plugin version not compatible with host.\n");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_VERBOSE, "Plugin description:\n");
    av_log(ctx, AV_LOG_VERBOSE, " id          : %s\n", s->plugin_desc->id);
    av_log(ctx, AV_LOG_VERBOSE, " name        : %s\n", s->plugin_desc->name);
    if (s->plugin_desc->vendor)
        av_log(ctx, AV_LOG_VERBOSE, " vendor      : %s\n", s->plugin_desc->vendor);
    if (s->plugin_desc->url)
        av_log(ctx, AV_LOG_VERBOSE, " url         : %s\n", s->plugin_desc->url);
    if (s->plugin_desc->manual_url)
        av_log(ctx, AV_LOG_VERBOSE, " manual url  : %s\n", s->plugin_desc->manual_url);
    if (s->plugin_desc->support_url)
        av_log(ctx, AV_LOG_VERBOSE, " support url : %s\n", s->plugin_desc->support_url);
    if (s->plugin_desc->version)
        av_log(ctx, AV_LOG_VERBOSE, " version     : %s\n", s->plugin_desc->version);
    if (s->plugin_desc->description)
        av_log(ctx, AV_LOG_VERBOSE, " desc        : %s\n", s->plugin_desc->description);
    if (s->plugin_desc->features) {
        av_log(ctx, AV_LOG_VERBOSE, " features    :\n");
        for (unsigned n = 0;; n++) {
            if (!s->plugin_desc->features[n])
                break;
            av_log(ctx, AV_LOG_VERBOSE, "  %s\n", s->plugin_desc->features[n]);
        }
    }

    avfilter_host.host_data = ctx;
    s->clap_plugin = s->plugin_factory->create_plugin(s->plugin_factory, &avfilter_host, s->plugin_desc->id);
    if (!s->clap_plugin) {
        av_log(ctx, AV_LOG_ERROR, "Could not create plugin: %s\n", s->plugin_desc->id);
        return AVERROR(EINVAL);
    }

    if (!s->clap_plugin->init(s->clap_plugin)) {
        av_log(ctx, AV_LOG_ERROR, "Could not init plugin\n");
        return AVERROR(EINVAL);
    }

    if (s->nb_options > 0 && !strcmp(s->options[0], "help")) {
        clap_plugin_params_t *plugin_params = (clap_plugin_params_t *)s->clap_plugin->get_extension(s->clap_plugin, CLAP_EXT_PARAMS);
        if (plugin_params) {
            uint32_t control_count = plugin_params->count(s->clap_plugin);

            av_log(ctx, AV_LOG_INFO, "Plugin has %d controls\n", control_count);
            for (unsigned i = 0; i < control_count; i++) {
                clap_param_info_t inf;

                plugin_params->get_info(s->clap_plugin, i, &inf);

                av_log(ctx, AV_LOG_INFO, "%s\t\t <double> (from %g to %g) (default: %g)\n",
                       inf.name, inf.min_value, inf.max_value, inf.default_value);
            }
        } else {
            av_log(ctx, AV_LOG_INFO, "No Controls Available\n");
        }

        return AVERROR_EXIT;
    }

    {
        clap_plugin_params_t *plugin_params = (clap_plugin_params_t *)s->clap_plugin->get_extension(s->clap_plugin, CLAP_EXT_PARAMS);

        if (plugin_params) {
            int ret;

            s->control_count = plugin_params->count(s->clap_plugin);
            s->plugin_controls = av_calloc(s->control_count, sizeof(*s->plugin_controls));
            if (!s->plugin_controls)
                return AVERROR(ENOMEM);

            ret = process_options(ctx, plugin_params);
            if (ret < 0)
                return ret;
        }
    }

    s->clap_ports = (clap_plugin_audio_ports_t *)s->clap_plugin->get_extension(s->clap_plugin, CLAP_EXT_AUDIO_PORTS);
    if (s->clap_ports) {
        int ret;

        s->in_ports = s->clap_ports->count(s->clap_plugin, 1);
        s->out_ports = s->clap_ports->count(s->clap_plugin, 0);

        av_log(ctx, AV_LOG_VERBOSE, "In: %d, Out: %d ports\n", s->in_ports, s->out_ports);

        s->in_frames = av_calloc(s->in_ports, sizeof(*s->in_frames));
        s->out_frames = av_calloc(s->out_ports, sizeof(*s->out_frames));
        s->in_buffers = av_calloc(s->in_ports, sizeof(*s->in_buffers));
        s->out_buffers = av_calloc(s->out_ports, sizeof(*s->out_buffers));
        if (!s->in_buffers || !s->out_buffers || !s->in_frames || !s->out_frames)
            return AVERROR(ENOMEM);

        for (int i = 0; i < s->in_ports; i++) {
            pad.name = av_asprintf("in:%d/%d", i, s->in_ports-1);
            if (!pad.name)
                return AVERROR(ENOMEM);

            pad.config_props = config_input;
            if ((ret = ff_append_inpad_free_name(ctx, &pad)) < 0)
                return ret;
        }

        for (int i = 0; i < s->out_ports; i++) {
            pad.name = av_asprintf("out:%d/%d", i, s->out_ports-1);
            if (!pad.name)
                return AVERROR(ENOMEM);

            pad.config_props = config_output;
            if ((ret = ff_append_outpad_free_name(ctx, &pad)) < 0)
                return ret;
        }
    } else {
        av_log(ctx, AV_LOG_INFO, "No ports extension\n");
        return AVERROR_EXIT;
    }

    s->in_events.ctx = s;
    s->in_events.get = get;
    s->in_events.size = size;
    s->out_events.ctx = s;
    s->out_events.try_push = try_push;

    return 0;
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const CLAPContext *s = ctx->priv;
    AVFilterChannelLayouts *layouts;
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_NONE };
    int ret;

    ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, sample_fmts);
    if (ret < 0)
        return ret;

    for (int i = 0; i < s->in_ports; i++) {
        clap_audio_port_info_t inf;
        AVChannelLayout inlayout;

        s->clap_ports->get(s->clap_plugin, i, 1, &inf);
        inlayout = FF_COUNT2LAYOUT(inf.channel_count);

        layouts = NULL;
        ret = ff_add_channel_layout(&layouts, &inlayout);
        if (ret < 0)
            return ret;
        ret = ff_channel_layouts_ref(layouts, &cfg_in[i]->channel_layouts);
        if (ret < 0)
            return ret;
    }

    for (int i = 0; i < s->out_ports; i++) {
        clap_audio_port_info_t inf;
        AVChannelLayout outlayout;

        s->clap_ports->get(s->clap_plugin, i, 0, &inf);
        outlayout = FF_COUNT2LAYOUT(inf.channel_count);

        layouts = NULL;
        ret = ff_add_channel_layout(&layouts, &outlayout);
        if (ret < 0)
            return ret;
        ret = ff_channel_layouts_ref(layouts, &cfg_out[i]->channel_layouts);
        if (ret < 0)
            return ret;
    }

    return 0;
}

static int activate(AVFilterContext *ctx)
{
    AVFilterLink *inlink = ctx->inputs[0];
    CLAPContext *s = ctx->priv;
    int status, ret;
    int64_t pts;

    if (!s->activated) {
        if (!s->clap_plugin->activate(s->clap_plugin, inlink->sample_rate,
                                      s->min_nb_samples, s->max_nb_samples))
            return AVERROR(EINVAL);
        s->activated = 1;
    }

    for (int i = 0; i < ctx->nb_outputs; i++) {
        FF_FILTER_FORWARD_STATUS_BACK_ALL(ctx->outputs[i], ctx);
    }

    if (!s->processing) {
        if (!s->clap_plugin->start_processing(s->clap_plugin))
            return AVERROR(EINVAL);
        s->processing = 1;
    }

    if (!s->in_frames[0]) {
        ret = ff_inlink_consume_samples(ctx->inputs[0],
                                        s->min_nb_samples,
                                        s->max_nb_samples,
                                        &s->in_frames[0]);
        if (ret < 0)
            return ret;
    } else {
        const int nb_samples = s->in_frames[0]->nb_samples;

        for (int i = 1; i < ctx->nb_inputs; i++) {
            if (s->in_frames[i])
                continue;
            ret = ff_inlink_consume_samples(ctx->inputs[i],
                                            nb_samples, nb_samples,
                                            &s->in_frames[i]);
            if (ret < 0)
                return ret;
        }
    }

    for (int i = 0; i < s->in_ports; i++) {
        if (!s->in_frames[i])
            break;
        if ((i+1) == s->in_ports)
            return filter_frame(ctx);
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        for (int i = 0; i < ctx->nb_outputs; i++) {
            if (ff_outlink_get_status(ctx->outputs[i]))
                continue;
            ff_outlink_set_status(ctx->outputs[i], status, pts);
        }

        if (s->processing) {
            s->clap_plugin->stop_processing(s->clap_plugin);
            s->processing = 0;
        }

        return 0;
    }

    for (int i = 0; i < ctx->nb_outputs; i++) {
        if (ff_outlink_get_status(ctx->outputs[i]))
            continue;

        if (ff_outlink_frame_wanted(ctx->outputs[i])) {
            for (int j = 0; j < ctx->nb_inputs; j++) {
                if (s->in_frames[j])
                    continue;
                ff_inlink_request_frame(ctx->inputs[j]);
            }

            return 0;
        }
    }

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    CLAPContext *s = ctx->priv;

    if (s->clap_plugin) {
        if (s->activated) {
            s->clap_plugin->deactivate(s->clap_plugin);
            s->activated = 0;
        }
        s->clap_plugin->destroy(s->clap_plugin);
        s->clap_plugin = NULL;
    }

    if (s->in_buffers) {
        for (int i = 0; i < s->in_ports; i++)
            av_freep(&s->in_buffers[i].data32);
        av_freep(&s->in_buffers);
    }

    av_freep(&s->in_frames);
    av_freep(&s->out_frames);
    av_freep(&s->plugin_controls);

    if (s->out_buffers) {
        for (int i = 0; i < s->out_ports; i++)
            av_freep(&s->out_buffers[i].data32);
        av_freep(&s->out_buffers);
    }

    if (s->plugin_entry) {
        s->plugin_entry->deinit();
        s->plugin_entry = NULL;
    }

    if (s->dl_handle) {
        dlclose(s->dl_handle);
        s->dl_handle = NULL;
    }
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    CLAPContext *s = ctx->priv;
    clap_plugin_params_t *plugin_params;
    int ret = ff_filter_process_command(ctx, cmd, arg);

    if (ret < 0)
        return ret;

    plugin_params = (clap_plugin_params_t *)s->clap_plugin->get_extension(s->clap_plugin, CLAP_EXT_PARAMS);
    if (plugin_params)
        ret = process_options(ctx, plugin_params);

    return ret;
}

const FFFilter ff_af_clap = {
    .p.name        = "clap",
    .p.description = NULL_IF_CONFIG_SMALL("Apply CLAP effect."),
    .p.priv_class  = &clap_class,
    .priv_size     = sizeof(CLAPContext),
    .init          = init,
    .activate      = activate,
    .uninit        = uninit,
    .process_command = process_command,
    .p.inputs      = NULL,
    .p.outputs     = NULL,
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags       = AVFILTER_FLAG_DYNAMIC_OUTPUTS |
                     AVFILTER_FLAG_DYNAMIC_INPUTS,
};
