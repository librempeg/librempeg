/*
 * filter layer
 * Copyright (c) 2007 Bobby Bingham
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

#include "config.h"

#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/bprint.h"
#include "libavutil/buffer.h"
#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/eval.h"
#include "libavutil/frame.h"
#include "libavutil/hwcontext.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/rational.h"
#include "libavutil/samplefmt.h"

#include "audio.h"
#include "avfilter.h"
#include "avfilter_internal.h"
#include "filters.h"
#include "formats.h"
#include "framequeue.h"
#include "framepool.h"
#include "video.h"

static void tlog_ref(void *ctx, AVFrame *ref, int end)
{
#ifdef TRACE
    ff_tlog(ctx,
            "ref[%p buf:%p data:%p linesize[%d, %d, %d, %d] pts:%"PRId64,
            ref, ref->buf, ref->data[0],
            ref->linesize[0], ref->linesize[1], ref->linesize[2], ref->linesize[3],
            ref->pts);

    if (ref->width) {
        ff_tlog(ctx, " a:%d/%d s:%dx%d i:%c iskey:%d type:%c",
                ref->sample_aspect_ratio.num, ref->sample_aspect_ratio.den,
                ref->width, ref->height,
                !(ref->flags & AV_FRAME_FLAG_INTERLACED) ? 'P' : /* Progressive  */
                (ref->flags & AV_FRAME_FLAG_TOP_FIELD_FIRST) ? 'T' : 'B', /* Top / Bottom */
                !!(ref->flags & AV_FRAME_FLAG_KEY),
                av_get_picture_type_char(ref->pict_type));
    }
    if (ref->nb_samples) {
        AVBPrint bprint;

        av_bprint_init(&bprint, 1, AV_BPRINT_SIZE_UNLIMITED);
        av_channel_layout_describe_bprint(&ref->ch_layout, &bprint);
        ff_tlog(ctx, " cl:%s n:%d r:%d",
                bprint.str,
                ref->nb_samples,
                ref->sample_rate);
        av_bprint_finalize(&bprint, NULL);
    }

    ff_tlog(ctx, "]%s", end ? "\n" : "");
#endif
}

static void command_queue_pop(AVFilterContext *filter)
{
    FFFilterContext *ctxi = fffilterctx(filter);
    AVFilterCommand *c    = ctxi->command_queue;
    av_freep(&c->arg);
    av_freep(&c->command);
    ctxi->command_queue = c->next;
    av_free(c);
}

/**
 * Append a new pad.
 *
 * @param count  Pointer to the number of pads in the list
 * @param pads   Pointer to the pointer to the beginning of the list of pads
 * @param links  Pointer to the pointer to the beginning of the list of links
 * @param newpad The new pad to add. A copy is made when adding.
 * @return >= 0 in case of success, a negative AVERROR code on error
 */
static int append_pad(unsigned *count, AVFilterPad **pads,
                      AVFilterLink ***links, AVFilterPad *newpad)
{
    AVFilterLink **newlinks;
    AVFilterPad *newpads;
    unsigned idx = *count;

    newpads  = av_realloc_array(*pads,  idx + 1, sizeof(*newpads));
    newlinks = av_realloc_array(*links, idx + 1, sizeof(*newlinks));
    if (newpads)
        *pads  = newpads;
    if (newlinks)
        *links = newlinks;
    if (!newpads || !newlinks) {
        if (newpad->flags & AVFILTERPAD_FLAG_FREE_NAME)
            av_freep(&newpad->name);
        return AVERROR(ENOMEM);
    }

    memcpy(*pads + idx, newpad, sizeof(AVFilterPad));
    (*links)[idx] = NULL;

    (*count)++;

    return 0;
}

int ff_append_inpad(AVFilterContext *f, AVFilterPad *p)
{
    return append_pad(&f->nb_inputs, &f->input_pads, &f->inputs, p);
}

int ff_append_inpad_free_name(AVFilterContext *f, AVFilterPad *p)
{
    p->flags |= AVFILTERPAD_FLAG_FREE_NAME;
    return ff_append_inpad(f, p);
}

int ff_append_outpad(AVFilterContext *f, AVFilterPad *p)
{
    return append_pad(&f->nb_outputs, &f->output_pads, &f->outputs, p);
}

int ff_append_outpad_free_name(AVFilterContext *f, AVFilterPad *p)
{
    p->flags |= AVFILTERPAD_FLAG_FREE_NAME;
    return ff_append_outpad(f, p);
}

FilterLinkInternal *ff_link_alloc(AVFilterGraph *graph, enum AVMediaType type)
{
    FilterLinkInternal *li;

    li = av_mallocz(sizeof(*li));
    if (!li)
        return NULL;

    li->l.pub.type = type;
    li->l.graph    = graph;

    av_assert0(AV_PIX_FMT_NONE == -1 && AV_SAMPLE_FMT_NONE == -1);
    li->l.pub.format     = -1;
    li->l.pub.colorspace = AVCOL_SPC_UNSPECIFIED;

    return li;
}

int avfilter_link(AVFilterContext *src, unsigned srcpad,
                  AVFilterContext *dst, unsigned dstpad)
{
    FilterLinkInternal *li;
    AVFilterLink *link;

    av_assert0(src->graph);
    av_assert0(dst->graph);
    av_assert0(src->graph == dst->graph);

    if (src->nb_outputs <= srcpad || dst->nb_inputs <= dstpad ||
        src->outputs[srcpad]      || dst->inputs[dstpad])
        return AVERROR(EINVAL);

    if (!(fffilterctx(src)->state_flags & AV_CLASS_STATE_INITIALIZED) ||
        !(fffilterctx(dst)->state_flags & AV_CLASS_STATE_INITIALIZED)) {
        av_log(src, AV_LOG_ERROR, "Filters must be initialized before linking.\n");
        return AVERROR(EINVAL);
    }

    if (src->output_pads[srcpad].type != dst->input_pads[dstpad].type) {
        av_log(src, AV_LOG_ERROR,
               "Media type mismatch between the '%s' filter output pad %d (%s) and the '%s' filter input pad %d (%s)\n",
               src->name, srcpad, (char *)av_x_if_null(av_get_media_type_string(src->output_pads[srcpad].type), "?"),
               dst->name, dstpad, (char *)av_x_if_null(av_get_media_type_string(dst-> input_pads[dstpad].type), "?"));
        return AVERROR(EINVAL);
    }

    li = ff_link_alloc(src->graph, src->output_pads[srcpad].type);
    if (!li)
        return AVERROR(ENOMEM);
    link = &li->l.pub;

    link->src     = src;
    link->dst     = dst;
    link->srcpad  = &src->output_pads[srcpad];
    link->dstpad  = &dst->input_pads[dstpad];

    src->outputs[srcpad] = dst->inputs[dstpad] = &li->l.pub;

    ff_framequeue_init(&li->fifo, &fffiltergraph(src->graph)->frame_queues);

    return 0;
}

static void link_free(AVFilterLink **link)
{
    FilterLinkInternal *li;

    if (!*link)
        return;
    li = ff_link_internal(*link);

    ff_framequeue_free(&li->fifo);
    ff_frame_pool_uninit(&li->frame_pool);
    av_channel_layout_uninit(&(*link)->ch_layout);
    av_frame_side_data_free(&(*link)->side_data, &(*link)->nb_side_data);

    av_buffer_unref(&li->l.hw_frames_ctx);

    av_freep(link);
}

static void update_link_current_pts(FilterLinkInternal *li, int64_t pts)
{
    AVFilterLink *const link = &li->l.pub;

    if (pts == AV_NOPTS_VALUE)
        return;
    li->l.current_pts = pts;
    li->l.current_pts_us = av_rescale_q(pts, link->time_base, AV_TIME_BASE_Q);
    /* TODO use duration */
    if (li->l.graph && li->age_index >= 0)
        ff_avfilter_graph_update_heap(li->l.graph, li);
}

void ff_filter_set_ready(AVFilterContext *filter, unsigned priority)
{
    FFFilterContext *ctxi = fffilterctx(filter);
    ctxi->ready = FFMAX(ctxi->ready, priority);
}

/**
 * Clear frame_blocked_in on all outputs.
 * This is necessary whenever something changes on input.
 */
static void filter_unblock(AVFilterContext *filter)
{
    unsigned i;

    for (i = 0; i < filter->nb_outputs; i++) {
        FilterLinkInternal * const li = ff_link_internal(filter->outputs[i]);
        li->frame_blocked_in = 0;
    }
}


void ff_avfilter_link_set_in_status(AVFilterLink *link, int status, int64_t pts)
{
    FilterLinkInternal * const li = ff_link_internal(link);

    if (li->status_in == status)
        return;
    av_assert0(!li->status_in);
    li->status_in = status;
    li->status_in_pts = pts;
    li->frame_wanted_out = 0;
    li->frame_blocked_in = 0;
    filter_unblock(link->dst);
    ff_filter_set_ready(link->dst, 200);
}

/**
 * Set the status field of a link from the destination filter.
 * The pts should probably be left unset (AV_NOPTS_VALUE).
 */
static void link_set_out_status(AVFilterLink *link, int status, int64_t pts)
{
    FilterLinkInternal * const li = ff_link_internal(link);

    av_assert0(!li->frame_wanted_out);
    av_assert0(!li->status_out);
    li->status_out = status;
    if (pts != AV_NOPTS_VALUE)
        update_link_current_pts(li, pts);
    filter_unblock(link->dst);
    ff_filter_set_ready(link->src, 200);
}

int avfilter_insert_filter(AVFilterLink *link, AVFilterContext *filt,
                           unsigned filt_srcpad_idx, unsigned filt_dstpad_idx)
{
    int ret;
    unsigned dstpad_idx = link->dstpad - link->dst->input_pads;

    av_log(link->dst, AV_LOG_VERBOSE, "auto-inserting filter '%s' "
           "between the filter '%s' and the filter '%s'\n",
           filt->name, link->src->name, link->dst->name);

    link->dst->inputs[dstpad_idx] = NULL;
    if ((ret = avfilter_link(filt, filt_dstpad_idx, link->dst, dstpad_idx)) < 0) {
        /* failed to link output filter to new filter */
        link->dst->inputs[dstpad_idx] = link;
        return ret;
    }

    /* re-hookup the link to the new destination filter we inserted */
    link->dst                     = filt;
    link->dstpad                  = &filt->input_pads[filt_srcpad_idx];
    filt->inputs[filt_srcpad_idx] = link;

    /* if any information on supported media formats already exists on the
     * link, we need to preserve that */
    if (link->outcfg.formats)
        ff_formats_changeref(&link->outcfg.formats,
                             &filt->outputs[filt_dstpad_idx]->outcfg.formats);
    if (link->outcfg.color_spaces)
        ff_formats_changeref(&link->outcfg.color_spaces,
                             &filt->outputs[filt_dstpad_idx]->outcfg.color_spaces);
    if (link->outcfg.color_ranges)
        ff_formats_changeref(&link->outcfg.color_ranges,
                             &filt->outputs[filt_dstpad_idx]->outcfg.color_ranges);
    if (link->outcfg.samplerates)
        ff_formats_changeref(&link->outcfg.samplerates,
                             &filt->outputs[filt_dstpad_idx]->outcfg.samplerates);
    if (link->outcfg.channel_layouts)
        ff_channel_layouts_changeref(&link->outcfg.channel_layouts,
                                     &filt->outputs[filt_dstpad_idx]->outcfg.channel_layouts);

    return 0;
}

int ff_filter_config_links(AVFilterContext *filter)
{
    int (*config_link)(AVFilterLink *);
    unsigned i;
    int ret;

    for (i = 0; i < filter->nb_inputs; i ++) {
        AVFilterLink *link = filter->inputs[i];
        AVFilterLink *inlink;
        FilterLinkInternal *li = ff_link_internal(link);
        FilterLinkInternal *li_in;

        if (!link) continue;
        if (!link->src || !link->dst) {
            av_log(filter, AV_LOG_ERROR,
                   "Not all input and output are properly linked (%d).\n", i);
            return AVERROR(EINVAL);
        }

        inlink = link->src->nb_inputs ? link->src->inputs[0] : NULL;
        li_in  = inlink ? ff_link_internal(inlink) : NULL;
        li->l.current_pts =
        li->l.current_pts_us = AV_NOPTS_VALUE;

        switch (li->init_state) {
        case AVLINK_INIT:
            continue;
        case AVLINK_STARTINIT:
            av_log(filter, AV_LOG_INFO, "circular filter chain detected\n");
            return 0;
        case AVLINK_UNINIT:
            li->init_state = AVLINK_STARTINIT;

            if ((ret = ff_filter_config_links(link->src)) < 0)
                return ret;

            if (!(config_link = link->srcpad->config_props)) {
                if (link->src->nb_inputs != 1) {
                    av_log(link->src, AV_LOG_ERROR, "Source filters and filters "
                                                    "with more than one input "
                                                    "must set config_props() "
                                                    "callbacks on all outputs\n");
                    return AVERROR(EINVAL);
                }
            }

            /* Copy side data before link->srcpad->config_props() is called, so the filter
             * may remove it for the next filter in the chain */
            if (inlink && inlink->nb_side_data && !link->nb_side_data) {
                for (int j = 0; j < inlink->nb_side_data; j++) {
                    ret = av_frame_side_data_clone(&link->side_data, &link->nb_side_data,
                                                   inlink->side_data[j], 0);
                    if (ret < 0) {
                        av_frame_side_data_free(&link->side_data, &link->nb_side_data);
                        return ret;
                    }
                }
            }

            if (config_link && (ret = config_link(link)) < 0) {
                av_log(link->src, AV_LOG_ERROR,
                       "Failed to configure output pad on %s\n",
                       link->src->name);
                return ret;
            }

            switch (link->type) {
            case AVMEDIA_TYPE_VIDEO:
                if (!link->time_base.num && !link->time_base.den)
                    link->time_base = inlink ? inlink->time_base : AV_TIME_BASE_Q;

                if (!link->sample_aspect_ratio.num && !link->sample_aspect_ratio.den)
                    link->sample_aspect_ratio = inlink ?
                        inlink->sample_aspect_ratio : (AVRational){1,1};

                if (inlink) {
                    if (!li->l.frame_rate.num && !li->l.frame_rate.den)
                        li->l.frame_rate = li_in->l.frame_rate;
                    if (!link->w)
                        link->w = inlink->w;
                    if (!link->h)
                        link->h = inlink->h;
                } else if (!link->w || !link->h) {
                    av_log(link->src, AV_LOG_ERROR,
                           "Video source filters must set their output link's "
                           "width and height\n");
                    return AVERROR(EINVAL);
                }
                break;

            case AVMEDIA_TYPE_AUDIO:
                if (inlink) {
                    if (!link->time_base.num && !link->time_base.den)
                        link->time_base = inlink->time_base;
                }

                if (!link->time_base.num && !link->time_base.den)
                    link->time_base = (AVRational) {1, link->sample_rate};
            }

            if (link->src->nb_inputs &&
                !(fffilter(link->src->filter)->flags_internal & FF_FILTER_FLAG_HWFRAME_AWARE)) {
                FilterLink *l0 = ff_filter_link(link->src->inputs[0]);

                av_assert0(!li->l.hw_frames_ctx &&
                           "should not be set by non-hwframe-aware filter");

                if (l0->hw_frames_ctx) {
                    li->l.hw_frames_ctx = av_buffer_ref(l0->hw_frames_ctx);
                    if (!li->l.hw_frames_ctx)
                        return AVERROR(ENOMEM);
                }
            }

            if ((config_link = link->dstpad->config_props))
                if ((ret = config_link(link)) < 0) {
                    av_log(link->dst, AV_LOG_ERROR,
                           "Failed to configure input pad on %s\n",
                           link->dst->name);
                    return ret;
                }

            li->init_state = AVLINK_INIT;
        }
    }

    return 0;
}

#ifdef TRACE
void ff_tlog_link(void *ctx, AVFilterLink *link, int end)
{
    if (link->type == AVMEDIA_TYPE_VIDEO) {
        ff_tlog(ctx,
                "link[%p s:%dx%d fmt:%s %s->%s]%s",
                link, link->w, link->h,
                av_get_pix_fmt_name(link->format),
                link->src ? link->src->filter->name : "",
                link->dst ? link->dst->filter->name : "",
                end ? "\n" : "");
    } else {
        char buf[128];
        av_channel_layout_describe(&link->ch_layout, buf, sizeof(buf));

        ff_tlog(ctx,
                "link[%p r:%d cl:%s fmt:%s %s->%s]%s",
                link, (int)link->sample_rate, buf,
                av_get_sample_fmt_name(link->format),
                link->src ? link->src->filter->name : "",
                link->dst ? link->dst->filter->name : "",
                end ? "\n" : "");
    }
}
#endif

int ff_request_frame(AVFilterLink *link)
{
    FilterLinkInternal * const li = ff_link_internal(link);

    FF_TPRINTF_START(NULL, request_frame); ff_tlog_link(NULL, link, 1);

    av_assert1(!fffilter(link->dst->filter)->activate);
    if (li->status_out)
        return li->status_out;
    if (li->status_in) {
        if (ff_framequeue_queued_frames(&li->fifo)) {
            av_assert1(!li->frame_wanted_out);
            av_assert1(fffilterctx(link->dst)->ready >= 300);
            return 0;
        } else {
            /* Acknowledge status change. Filters using ff_request_frame() will
               handle the change automatically. Filters can also check the
               status directly but none do yet. */
            link_set_out_status(link, li->status_in, li->status_in_pts);
            return li->status_out;
        }
    }
    li->frame_wanted_out = 1;
    ff_filter_set_ready(link->src, 100);
    return 0;
}

static int64_t guess_status_pts(AVFilterContext *ctx, int status, AVRational link_time_base)
{
    unsigned i;
    int64_t r = INT64_MAX;

    for (i = 0; i < ctx->nb_inputs; i++) {
        FilterLinkInternal * const li = ff_link_internal(ctx->inputs[i]);
        if (li->status_out == status)
            r = FFMIN(r, av_rescale_q(li->l.current_pts, ctx->inputs[i]->time_base, link_time_base));
    }
    if (r < INT64_MAX)
        return r;
    av_log(ctx, AV_LOG_WARNING, "EOF timestamp not reliable\n");
    for (i = 0; i < ctx->nb_inputs; i++) {
        FilterLinkInternal * const li = ff_link_internal(ctx->inputs[i]);
        r = FFMIN(r, av_rescale_q(li->status_in_pts, ctx->inputs[i]->time_base, link_time_base));
    }
    if (r < INT64_MAX)
        return r;
    return AV_NOPTS_VALUE;
}

static int request_frame_to_filter(AVFilterLink *link)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    int ret = -1;

    FF_TPRINTF_START(NULL, request_frame_to_filter); ff_tlog_link(NULL, link, 1);
    /* Assume the filter is blocked, let the method clear it if not */
    li->frame_blocked_in = 1;
    if (link->srcpad->request_frame)
        ret = link->srcpad->request_frame(link);
    else if (link->src->inputs[0])
        ret = ff_request_frame(link->src->inputs[0]);
    if (ret < 0) {
        if (ret != AVERROR(EAGAIN) && ret != li->status_in)
            ff_avfilter_link_set_in_status(link, ret, guess_status_pts(link->src, ret, link->time_base));
        if (ret == AVERROR_EOF) {
#if CONFIG_AVFILTER_THREAD_FRAME
            if (link->src->thread_type & AVFILTER_THREAD_FRAME_FILTER) {
                ret = ff_filter_frame_thread_flush(link->src);
                if (ret < 0)
                    return ret;
            }
#endif

            ret = 0;
        }
    }
    return ret;
}

static const char *const var_names[] = {
    "t",
    "n",
    "w",
    "h",
    NULL
};

enum {
    VAR_T,
    VAR_N,
    VAR_W,
    VAR_H,
    VAR_VARS_NB
};

static int set_enable_expr(FFFilterContext *ctxi, const char *expr)
{
    AVFilterContext *ctx = &ctxi->p;
    int ret;
    char *expr_dup;
    AVExpr *old = ctxi->enable;

    if (!(ctx->filter->flags & AVFILTER_FLAG_SUPPORT_TIMELINE)) {
        av_log(ctx, AV_LOG_ERROR, "Timeline ('enable' option) not supported "
               "with filter '%s'\n", ctx->filter->name);
        return AVERROR_PATCHWELCOME;
    }

    expr_dup = av_strdup(expr);
    if (!expr_dup)
        return AVERROR(ENOMEM);

    if (!ctxi->var_values) {
        ctxi->var_values = av_calloc(VAR_VARS_NB, sizeof(*ctxi->var_values));
        if (!ctxi->var_values) {
            av_free(expr_dup);
            return AVERROR(ENOMEM);
        }
    }

    ret = av_expr_parse(&ctxi->enable, expr_dup, var_names,
                        NULL, NULL, NULL, NULL, 0, ctx->priv);
    if (ret < 0) {
        av_log(ctx->priv, AV_LOG_ERROR,
               "Error when evaluating the expression '%s' for enable\n",
               expr_dup);
        av_free(expr_dup);
        return ret;
    }

    av_expr_free(old);
    av_free(ctx->enable_str);
    ctx->enable_str = expr_dup;
    return 0;
}

int avfilter_process_command(AVFilterContext *filter, const char *cmd, const char *arg, char *res, int res_len, int flags)
{
    if(!strcmp(cmd, "ping")){
        char local_res[256] = {0};

        if (!res) {
            res = local_res;
            res_len = sizeof(local_res);
        }
        av_strlcatf(res, res_len, "pong from:%s %s\n", filter->filter->name, filter->name);
        if (res == local_res)
            av_log(filter, AV_LOG_INFO, "%s", res);
        return 0;
    }else if(!strcmp(cmd, "enable")) {
        return set_enable_expr(fffilterctx(filter), arg);
    }else if (fffilter(filter->filter)->process_command) {
        return fffilter(filter->filter)->process_command(filter, cmd, arg);
    }
    return AVERROR(ENOSYS);
}

unsigned avfilter_filter_pad_count(const AVFilter *filter, int is_output)
{
    return is_output ? fffilter(filter)->nb_outputs : fffilter(filter)->nb_inputs;
}

static const char *default_filter_name(void *filter_ctx)
{
    AVFilterContext *ctx = filter_ctx;
    return ctx->name ? ctx->name : ctx->filter->name;
}

static void *filter_child_next(void *obj, void *prev)
{
    AVFilterContext *ctx = obj;
    if (!prev && ctx->filter && ctx->filter->priv_class && ctx->priv)
        return ctx->priv;
    return NULL;
}

static const AVClass *filter_child_class_iterate(void **iter)
{
    const AVFilter *f;

    while ((f = av_filter_iterate(iter)))
        if (f->priv_class)
            return f->priv_class;

    return NULL;
}

#define OFFSET(x) offsetof(AVFilterContext, x)
#define FOFFSET(x) offsetof(FFFilterContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM
#define TFLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define X AV_OPT_FLAG_EXPORT
#define R AV_OPT_FLAG_READONLY
static const AVOption avfilter_options[] = {
    { "thread_type", "Allowed thread types", OFFSET(thread_type), AV_OPT_TYPE_FLAGS,
        { .i64 = AVFILTER_THREAD_SLICE }, 0, INT_MAX, FLAGS, .unit = "thread_type" },
        { "slice", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = AVFILTER_THREAD_SLICE }, .flags = FLAGS, .unit = "thread_type" },
        { "frame", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = AVFILTER_THREAD_FRAME_FILTER }, .flags = FLAGS, .unit = "thread_type" },
    { "enable", "set enable expression", OFFSET(enable_str), AV_OPT_TYPE_STRING, {.str=NULL}, .flags = TFLAGS },
    { "disabled", "is filter timeline disabled", FOFFSET(is_disabled), AV_OPT_TYPE_INT, {.i64=0}, .flags = TFLAGS|X|R },
    { "threads", "Allowed number of threads", OFFSET(nb_threads), AV_OPT_TYPE_INT,
        { .i64 = 0 }, 0, INT_MAX, FLAGS, .unit = "threads" },
        {"auto", "autodetect a suitable number of threads to use", 0, AV_OPT_TYPE_CONST, {.i64 = 0 }, .flags = FLAGS, .unit = "threads"},
    { "extra_hw_frames", "Number of extra hardware frames to allocate for the user",
        OFFSET(extra_hw_frames), AV_OPT_TYPE_INT, { .i64 = -1 }, -1, INT_MAX, FLAGS },
    { NULL },
};

static const AVClass avfilter_class = {
    .class_name = "AVFilter",
    .item_name  = default_filter_name,
    .version    = LIBAVUTIL_VERSION_INT,
    .category   = AV_CLASS_CATEGORY_FILTER,
    .child_next = filter_child_next,
    .child_class_iterate = filter_child_class_iterate,
    .option           = avfilter_options,
    .state_flags_offset = offsetof(FFFilterContext, state_flags),
};

static int default_execute(AVFilterContext *ctx, avfilter_action_func *func, void *arg,
                           int *ret, int nb_jobs)
{
    int i;

    for (i = 0; i < nb_jobs; i++) {
        int r = func(ctx, arg, i, nb_jobs);
        if (ret)
            ret[i] = r;
    }
    return 0;
}

AVFilterContext *ff_filter_alloc(const AVFilter *filter, const char *inst_name)
{
    FFFilterContext *ctx;
    AVFilterContext *ret;
    const FFFilter *const fi = fffilter(filter);
    int preinited = 0;

    if (!filter)
        return NULL;

    ctx = av_mallocz(sizeof(*ctx));
    if (!ctx)
        return NULL;
    ret = &ctx->p;

    ret->av_class = &avfilter_class;
    ret->filter   = filter;
    ret->name     = inst_name ? av_strdup(inst_name) : NULL;
    if (fi->priv_size) {
        ret->priv     = av_mallocz(fi->priv_size);
        if (!ret->priv)
            goto err;
    }
    if (fi->preinit) {
        if (fi->preinit(ret) < 0)
            goto err;
        preinited = 1;
    }

    av_opt_set_defaults(ret);
    if (filter->priv_class) {
        *(const AVClass**)ret->priv = filter->priv_class;
        av_opt_set_defaults(ret->priv);
    }

    ctx->execute = default_execute;

    ret->nb_inputs  = fi->nb_inputs;
    if (ret->nb_inputs ) {
        ret->input_pads   = av_memdup(filter->inputs,  ret->nb_inputs  * sizeof(*filter->inputs));
        if (!ret->input_pads)
            goto err;
        ret->inputs      = av_calloc(ret->nb_inputs, sizeof(*ret->inputs));
        if (!ret->inputs)
            goto err;
    }

    ret->nb_outputs = fi->nb_outputs;
    if (ret->nb_outputs) {
        ret->output_pads  = av_memdup(filter->outputs, ret->nb_outputs * sizeof(*filter->outputs));
        if (!ret->output_pads)
            goto err;
        ret->outputs     = av_calloc(ret->nb_outputs, sizeof(*ret->outputs));
        if (!ret->outputs)
            goto err;
    }

    return ret;

err:
    if (preinited)
        fi->uninit(ret);
    av_freep(&ret->inputs);
    av_freep(&ret->input_pads);
    ret->nb_inputs = 0;
    av_freep(&ret->outputs);
    av_freep(&ret->output_pads);
    ret->nb_outputs = 0;
    av_freep(&ret->priv);
    av_free(ret);
    return NULL;
}

static void free_link(AVFilterLink *link)
{
    if (!link)
        return;

    if (link->src)
        link->src->outputs[link->srcpad - link->src->output_pads] = NULL;
    if (link->dst)
        link->dst->inputs[link->dstpad - link->dst->input_pads] = NULL;

    ff_formats_unref(&link->incfg.formats);
    ff_formats_unref(&link->outcfg.formats);
    ff_formats_unref(&link->incfg.color_spaces);
    ff_formats_unref(&link->outcfg.color_spaces);
    ff_formats_unref(&link->incfg.color_ranges);
    ff_formats_unref(&link->outcfg.color_ranges);
    ff_formats_unref(&link->incfg.samplerates);
    ff_formats_unref(&link->outcfg.samplerates);
    ff_channel_layouts_unref(&link->incfg.channel_layouts);
    ff_channel_layouts_unref(&link->outcfg.channel_layouts);
    link_free(&link);
}

void avfilter_free(AVFilterContext *filter)
{
    FFFilterContext *ctxi;
    int i;

    if (!filter)
        return;
    ctxi = fffilterctx(filter);

#if CONFIG_AVFILTER_THREAD_FRAME
    if ((filter->thread_type & AVFILTER_THREAD_FRAME_FILTER) &&
        !ctxi->is_frame_thread)
        ff_filter_frame_thread_free(ctxi);
#endif

    if (filter->graph)
        ff_filter_graph_remove_filter(filter->graph, filter);

    if (fffilter(filter->filter)->uninit)
        fffilter(filter->filter)->uninit(filter);

    for (i = 0; i < filter->nb_inputs; i++) {
        free_link(filter->inputs[i]);
        if (filter->input_pads[i].flags  & AVFILTERPAD_FLAG_FREE_NAME)
            av_freep(&filter->input_pads[i].name);
    }
    for (i = 0; i < filter->nb_outputs; i++) {
        free_link(filter->outputs[i]);
        if (filter->output_pads[i].flags & AVFILTERPAD_FLAG_FREE_NAME)
            av_freep(&filter->output_pads[i].name);
    }

    if (filter->filter->priv_class)
        av_opt_free(filter->priv);

    av_buffer_unref(&filter->hw_device_ctx);

    av_freep(&filter->name);
    av_freep(&filter->input_pads);
    av_freep(&filter->output_pads);
    av_freep(&filter->inputs);
    av_freep(&filter->outputs);
    av_freep(&filter->priv);
    while (ctxi->command_queue)
        command_queue_pop(filter);
    av_opt_free(filter);
    av_expr_free(ctxi->enable);
    ctxi->enable = NULL;
    av_freep(&ctxi->var_values);
    av_free(filter);
}

int ff_filter_get_nb_threads(AVFilterContext *ctx)
{
    if (ctx->nb_threads > 0)
        return FFMIN(ctx->nb_threads, ctx->graph->nb_threads);
    return ctx->graph->nb_threads;
}

int ff_filter_opt_parse(void *logctx, const AVClass *priv_class,
                        AVDictionary **options, const char *args)
{
    const AVOption *o = NULL;
    int ret;
    int offset= -1;

    if (!args)
        return 0;

    while (*args) {
        char *parsed_key, *value;
        const char *key;
        const char *shorthand = NULL;
        int additional_flags  = 0;

        if (priv_class && (o = av_opt_next(&priv_class, o))) {
            if (o->type == AV_OPT_TYPE_CONST || o->offset == offset)
                continue;
            offset = o->offset;
            shorthand = o->name;
        }

        ret = av_opt_get_key_value(&args, "=", ":",
                                   shorthand ? AV_OPT_FLAG_IMPLICIT_KEY : 0,
                                   &parsed_key, &value);
        if (ret < 0) {
            if (ret == AVERROR(EINVAL))
                av_log(logctx, AV_LOG_ERROR, "No option name near '%s'\n", args);
            else
                av_log(logctx, AV_LOG_ERROR, "Unable to parse '%s': %s\n", args,
                       av_err2str(ret));
            return ret;
        }
        if (*args)
            args++;
        if (parsed_key) {
            key = parsed_key;
            additional_flags = AV_DICT_DONT_STRDUP_KEY;
            priv_class = NULL; /* reject all remaining shorthand */
        } else {
            key = shorthand;
        }

        av_log(logctx, AV_LOG_DEBUG, "Setting '%s' to value '%s'\n", key, value);

        av_dict_set(options, key, value,
                    additional_flags | AV_DICT_DONT_STRDUP_VAL | AV_DICT_MULTIKEY);
    }

    return 0;
}

int ff_filter_process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    const AVOption *o;

    if (!ctx->filter->priv_class)
        return 0;
    o = av_opt_find2(ctx->priv, cmd, NULL, AV_OPT_FLAG_RUNTIME_PARAM | AV_OPT_FLAG_FILTERING_PARAM, AV_OPT_SEARCH_CHILDREN, NULL);
    if (!o)
        return AVERROR(ENOSYS);
    return av_opt_set(ctx->priv, cmd, arg, 0);
}

static int threading_init(FFFilterContext *ctxi)
{
    AVFilterContext *const     ctx = &ctxi->p;
    const int thread_types_allowed = ctx->thread_type & ctx->graph->thread_type;

#if CONFIG_AVFILTER_THREAD_FRAME
    if (ctxi->is_frame_thread)
        return 0;

    if ((ctx->filter->flags & AVFILTER_FLAG_FRAME_THREADS) &&
        (thread_types_allowed & AVFILTER_THREAD_FRAME_FILTER)) {
        int ret = ff_filter_frame_thread_init(ctxi);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "Error initializing frame threading\n");
            return ret;
        }
    } else
#endif
    if (ctx->filter->flags & AVFILTER_FLAG_SLICE_THREADS &&
               (thread_types_allowed & AVFILTER_THREAD_SLICE)   &&
               fffiltergraph(ctx->graph)->thread_execute) {
        ctxi->execute    = fffiltergraph(ctx->graph)->thread_execute;
        ctx->thread_type = AVFILTER_THREAD_SLICE;
    } else {
        ctx->thread_type = 0;
        ctx->nb_threads  = 0;
    }


    return 0;
}

int avfilter_init_dict(AVFilterContext *ctx, AVDictionary **options)
{
    FFFilterContext *ctxi = fffilterctx(ctx);
    int ret = 0;

    if (ctxi->state_flags & AV_CLASS_STATE_INITIALIZED) {
        av_log(ctx, AV_LOG_ERROR, "Filter already initialized\n");
        return AVERROR(EINVAL);
    }

    ret = av_opt_set_dict2(ctx, options, AV_OPT_SEARCH_CHILDREN);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Error applying generic filter options.\n");
        return ret;
    }

    if (fffilter(ctx->filter)->init)
        ret = fffilter(ctx->filter)->init(ctx);
    if (ret < 0)
        return ret;

    if (ctx->enable_str) {
        ret = set_enable_expr(ctxi, ctx->enable_str);
        if (ret < 0)
            return ret;
    }

    ret = threading_init(ctxi);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Error initializing threading.\n");
        return ret;
    }

    ctxi->state_flags |= AV_CLASS_STATE_INITIALIZED;

    return 0;
}

int avfilter_init_str(AVFilterContext *filter, const char *args)
{
    AVDictionary *options = NULL;
    const AVDictionaryEntry *e;
    int ret = 0;

    if (args && *args) {
        ret = ff_filter_opt_parse(filter, filter->filter->priv_class, &options, args);
        if (ret < 0)
            goto fail;
    }

    ret = avfilter_init_dict(filter, &options);
    if (ret < 0)
        goto fail;

    if ((e = av_dict_iterate(options, NULL))) {
        av_log(filter, AV_LOG_ERROR, "No such option: %s.\n", e->key);
        ret = AVERROR_OPTION_NOT_FOUND;
        goto fail;
    }

fail:
    av_dict_free(&options);

    return ret;
}

const char *avfilter_pad_get_name(const AVFilterPad *pads, int pad_idx)
{
    return pads[pad_idx].name;
}

enum AVMediaType avfilter_pad_get_type(const AVFilterPad *pads, int pad_idx)
{
    return pads[pad_idx].type;
}

static int default_filter_frame(AVFilterLink *link, AVFrame *frame)
{
    return ff_filter_frame(link->dst->outputs[0], frame);
}

/**
 * Evaluate the timeline expression of the link for the time and properties
 * of the frame.
 * @return  >0 if enabled, 0 if disabled
 * @note  It does not update link->dst->is_disabled.
 */
static int evaluate_timeline_at_frame(AVFilterLink *link, const AVFrame *frame)
{
    FilterLink *l = ff_filter_link(link);
    AVFilterContext *dstctx = link->dst;
    FFFilterContext *dsti = fffilterctx(dstctx);
    int64_t pts = frame->pts;

    if (!dstctx->enable_str)
        return 1;

    dsti->var_values[VAR_N] = l->frame_count_out;
    dsti->var_values[VAR_T] = pts == AV_NOPTS_VALUE ? NAN : pts * av_q2d(link->time_base);
    dsti->var_values[VAR_W] = link->w;
    dsti->var_values[VAR_H] = link->h;

    return fabs(av_expr_eval(dsti->enable, dsti->var_values, NULL)) >= 0.5;
}

static int filter_frame_framed(AVFilterLink *link, AVFrame *frame)
{
    FilterLink *l = ff_filter_link(link);
    int (*filter_frame)(AVFilterLink *, AVFrame *);
    AVFilterContext *dstctx = link->dst;
    FFFilterContext   *dsti = fffilterctx(dstctx);
    AVFilterPad *dst = link->dstpad;
    int ret;

    if (!(filter_frame = dst->filter_frame))
        filter_frame = default_filter_frame;

    if (dst->flags & AVFILTERPAD_FLAG_NEEDS_WRITABLE) {
        ret = ff_inlink_make_frame_writable(link, &frame);
        if (ret < 0)
            goto fail;
    }

    ff_inlink_process_commands(link, frame);
    dsti->is_disabled = !evaluate_timeline_at_frame(link, frame);

    if (dsti->is_disabled &&
        (dstctx->filter->flags & AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC))
        filter_frame = default_filter_frame;

#if CONFIG_AVFILTER_THREAD_FRAME
    if ((dstctx->thread_type & AVFILTER_THREAD_FRAME_FILTER) &&
        !dsti->is_frame_thread)
        ret = ff_filter_frame_thread_submit(link, frame);
    else
#endif
        ret = filter_frame(link, frame);

    l->frame_count_out++;
    return ret;

fail:
    av_frame_free(&frame);
    return ret;
}

int ff_filter_frame(AVFilterLink *link, AVFrame *frame)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    int ret;
    FF_TPRINTF_START(NULL, filter_frame); ff_tlog_link(NULL, link, 1); ff_tlog(NULL, " "); tlog_ref(NULL, frame, 1);

#if CONFIG_AVFILTER_THREAD_FRAME
    if (fffilterctx(link->src)->is_frame_thread)
        return ff_filter_frame_thread_frame_out(link, frame);
#endif

    /* Consistency checks */
    if (link->type == AVMEDIA_TYPE_VIDEO) {
        if (strcmp(link->dst->filter->name, "buffersink") &&
            strcmp(link->dst->filter->name, "format") &&
            strcmp(link->dst->filter->name, "idet") &&
            strcmp(link->dst->filter->name, "null") &&
            strcmp(link->dst->filter->name, "scale") &&
            strcmp(link->dst->filter->name, "libplacebo")) {
            av_assert1(frame->format        == link->format);
            av_assert1(frame->width         == link->w);
            av_assert1(frame->height        == link->h);
        }

        frame->sample_aspect_ratio = link->sample_aspect_ratio;
    } else {
        if (frame->format != link->format) {
            av_log(link->dst, AV_LOG_ERROR, "Format change is not supported\n");
            goto error;
        }
        if (av_channel_layout_compare(&frame->ch_layout, &link->ch_layout)) {
            av_log(link->dst, AV_LOG_ERROR, "Channel layout change is not supported\n");
            goto error;
        }
        if (frame->sample_rate != link->sample_rate) {
            av_log(link->dst, AV_LOG_ERROR, "Sample rate change is not supported\n");
            goto error;
        }

        frame->duration = av_rescale_q(frame->nb_samples, (AVRational){ 1, frame->sample_rate },
                                       link->time_base);
    }

    li->frame_blocked_in = li->frame_wanted_out = 0;
    li->l.frame_count_in++;
    li->l.sample_count_in += frame->nb_samples;
    filter_unblock(link->dst);
    ret = ff_framequeue_add(&li->fifo, frame);
    if (ret < 0) {
        av_frame_free(&frame);
        return ret;
    }
    ff_filter_set_ready(link->dst, 300);
    return 0;

error:
    av_frame_free(&frame);
    return AVERROR_PATCHWELCOME;
}

static int samples_ready(FilterLinkInternal *link, unsigned min)
{
    return ff_framequeue_queued_frames(&link->fifo) &&
           (ff_framequeue_queued_samples(&link->fifo) >= min ||
            link->status_in);
}

static int take_samples(FilterLinkInternal *li, unsigned min, unsigned max,
                        AVFrame **rframe)
{
    FilterLink *l = &li->l;
    AVFilterLink *link = &l->pub;
    AVFrame *frame0, *frame, *buf;
    unsigned nb_samples, nb_frames, i, p;
    int ret;

    /* Note: this function relies on no format changes and must only be
       called with enough samples. */
    av_assert1(samples_ready(li, l->min_samples));
    frame0 = frame = ff_framequeue_peek(&li->fifo, 0);
    if (!li->fifo.samples_skipped && frame->nb_samples >= min && frame->nb_samples <= max) {
        *rframe = ff_framequeue_take(&li->fifo);
        return 0;
    }
    nb_frames = 0;
    nb_samples = 0;
    while (1) {
        if (nb_samples + frame->nb_samples > max) {
            if (nb_samples < min)
                nb_samples = max;
            break;
        }
        nb_samples += frame->nb_samples;
        nb_frames++;
        if (nb_frames == ff_framequeue_queued_frames(&li->fifo))
            break;
        frame = ff_framequeue_peek(&li->fifo, nb_frames);
    }

    buf = ff_get_audio_buffer(link, nb_samples);
    if (!buf)
        return AVERROR(ENOMEM);
    ret = av_frame_copy_props(buf, frame0);
    if (ret < 0) {
        av_frame_free(&buf);
        return ret;
    }

    p = 0;
    for (i = 0; i < nb_frames; i++) {
        frame = ff_framequeue_take(&li->fifo);
        av_samples_copy(buf->extended_data, frame->extended_data, p, 0,
                        frame->nb_samples, link->ch_layout.nb_channels, link->format);
        p += frame->nb_samples;
        av_frame_free(&frame);
    }
    if (p < nb_samples) {
        unsigned n = nb_samples - p;
        frame = ff_framequeue_peek(&li->fifo, 0);
        av_samples_copy(buf->extended_data, frame->extended_data, p, 0, n,
                        link->ch_layout.nb_channels, link->format);
        ff_framequeue_skip_samples(&li->fifo, n, link->time_base);
    }

    *rframe = buf;
    return 0;
}

static int filter_frame_to_filter(AVFilterLink *link)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    AVFrame *frame = NULL;
    AVFilterContext *dst = link->dst;
    int ret;

    av_assert1(ff_framequeue_queued_frames(&li->fifo));
    ret = li->l.min_samples ?
          ff_inlink_consume_samples(link, li->l.min_samples, li->l.max_samples, &frame) :
          ff_inlink_consume_frame(link, &frame);
    av_assert1(ret);
    if (ret < 0) {
        av_assert1(!frame);
        return ret;
    }
    /* The filter will soon have received a new frame, that may allow it to
       produce one or more: unblock its outputs. */
    filter_unblock(dst);
    /* AVFilterPad.filter_frame() expect frame_count_out to have the value
       before the frame; filter_frame_framed() will re-increment it. */
    li->l.frame_count_out--;
    ret = filter_frame_framed(link, frame);
    if (ret < 0 && ret != li->status_out) {
        link_set_out_status(link, ret, AV_NOPTS_VALUE);
    } else {
        /* Run once again, to see if several frames were available, or if
           the input status has also changed, or any other reason. */
        ff_filter_set_ready(dst, 300);
    }
    return ret;
}

static int forward_status_change(AVFilterContext *filter, FilterLinkInternal *li_in)
{
    AVFilterLink *in = &li_in->l.pub;
    unsigned out = 0, progress = 0;
    int ret;

    av_assert0(!li_in->status_out);
    if (!filter->nb_outputs) {
        /* not necessary with the current API and sinks */
        return 0;
    }
    while (!li_in->status_out) {
        FilterLinkInternal *li_out = ff_link_internal(filter->outputs[out]);

        if (!li_out->status_in) {
            progress++;
            ret = request_frame_to_filter(filter->outputs[out]);
            if (ret < 0)
                return ret;
        }
        if (++out == filter->nb_outputs) {
            if (!progress) {
                /* Every output already closed: input no longer interesting
                   (example: overlay in shortest mode, other input closed). */
                link_set_out_status(in, li_in->status_in, li_in->status_in_pts);
                return 0;
            }
            progress = 0;
            out = 0;
        }
    }
    ff_filter_set_ready(filter, 200);
    return 0;
}

static int filter_activate_default(AVFilterContext *filter)
{
    unsigned i;

    for (i = 0; i < filter->nb_outputs; i++) {
        FilterLinkInternal *li = ff_link_internal(filter->outputs[i]);
        int ret = li->status_in;

        if (ret) {
            for (int j = 0; j < filter->nb_inputs; j++)
                ff_inlink_set_status(filter->inputs[j], ret);
            return 0;
        }
    }

    for (i = 0; i < filter->nb_inputs; i++) {
        FilterLinkInternal *li = ff_link_internal(filter->inputs[i]);
        if (samples_ready(li, li->l.min_samples)) {
            return filter_frame_to_filter(filter->inputs[i]);
        }
    }
    for (i = 0; i < filter->nb_inputs; i++) {
        FilterLinkInternal * const li = ff_link_internal(filter->inputs[i]);
        if (li->status_in && !li->status_out) {
            av_assert1(!ff_framequeue_queued_frames(&li->fifo));
            return forward_status_change(filter, li);
        }
    }
    for (i = 0; i < filter->nb_outputs; i++) {
        FilterLinkInternal * const li = ff_link_internal(filter->outputs[i]);
        if (li->frame_wanted_out &&
            !li->frame_blocked_in) {
            return request_frame_to_filter(filter->outputs[i]);
        }
    }
    return FFERROR_NOT_READY;
}

/*
   Filter scheduling and activation

   When a filter is activated, it must:
   - if possible, output a frame;
   - else, if relevant, forward the input status change;
   - else, check outputs for wanted frames and forward the requests.

   The following AVFilterLink fields are used for activation:

   - frame_wanted_out:

     This field indicates if a frame is needed on this input of the
     destination filter. A positive value indicates that a frame is needed
     to process queued frames or internal data or to satisfy the
     application; a zero value indicates that a frame is not especially
     needed but could be processed anyway; a negative value indicates that a
     frame would just be queued.

     It is set by filters using ff_request_frame() or ff_request_no_frame(),
     when requested by the application through a specific API or when it is
     set on one of the outputs.

     It is cleared when a frame is sent from the source using
     ff_filter_frame().

     It is also cleared when a status change is sent from the source using
     ff_avfilter_link_set_in_status().

   - frame_blocked_in:

     This field means that the source filter can not generate a frame as is.
     Its goal is to avoid repeatedly calling the request_frame() method on
     the same link.

     It is set by the framework on all outputs of a filter before activating it.

     It is automatically cleared by ff_filter_frame().

     It is also automatically cleared by ff_avfilter_link_set_in_status().

     It is also cleared on all outputs (using filter_unblock()) when
     something happens on an input: processing a frame or changing the
     status.

   - fifo:

     Contains the frames queued on a filter input. If it contains frames and
     frame_wanted_out is not set, then the filter can be activated. If that
     result in the filter not able to use these frames, the filter must set
     frame_wanted_out to ask for more frames.

   - status_in and status_in_pts:

     Status (EOF or error code) of the link and timestamp of the status
     change (in link time base, same as frames) as seen from the input of
     the link. The status change is considered happening after the frames
     queued in fifo.

     It is set by the source filter using ff_avfilter_link_set_in_status().

   - status_out:

     Status of the link as seen from the output of the link. The status
     change is considered having already happened.

     It is set by the destination filter using
     link_set_out_status().

   Filters are activated according to the ready field, set using the
   ff_filter_set_ready(). Eventually, a priority queue will be used.
   ff_filter_set_ready() is called whenever anything could cause progress to
   be possible. Marking a filter ready when it is not is not a problem,
   except for the small overhead it causes.

   Conditions that cause a filter to be marked ready are:

   - frames added on an input link;

   - changes in the input or output status of an input link;

   - requests for a frame on an output link;

   - after any actual processing using the legacy methods (filter_frame(),
     and request_frame() to acknowledge status changes), to run once more
     and check if enough input was present for several frames.

   Examples of scenarios to consider:

   - buffersrc: activate if frame_wanted_out to notify the application;
     activate when the application adds a frame to push it immediately.

   - testsrc: activate only if frame_wanted_out to produce and push a frame.

   - concat (not at stitch points): can process a frame on any output.
     Activate if frame_wanted_out on output to forward on the corresponding
     input. Activate when a frame is present on input to process it
     immediately.

   - framesync: needs at least one frame on each input; extra frames on the
     wrong input will accumulate. When a frame is first added on one input,
     set frame_wanted_out<0 on it to avoid getting more (would trigger
     testsrc) and frame_wanted_out>0 on the other to allow processing it.

   Activation of old filters:

   In order to activate a filter implementing the legacy filter_frame() and
   request_frame() methods, perform the first possible of the following
   actions:

   - If an input has frames in fifo and frame_wanted_out == 0, dequeue a
     frame and call filter_frame().

     Rationale: filter frames as soon as possible instead of leaving them
     queued; frame_wanted_out < 0 is not possible since the old API does not
     set it nor provides any similar feedback; frame_wanted_out > 0 happens
     when min_samples > 0 and there are not enough samples queued.

   - If an input has status_in set but not status_out, try to call
     request_frame() on one of the outputs in the hope that it will trigger
     request_frame() on the input with status_in and acknowledge it. This is
     awkward and fragile, filters with several inputs or outputs should be
     updated to direct activation as soon as possible.

   - If an output has frame_wanted_out > 0 and not frame_blocked_in, call
     request_frame().

     Rationale: checking frame_blocked_in is necessary to avoid requesting
     repeatedly on a blocked input if another is not blocked (example:
     [buffersrc1][testsrc1][buffersrc2][testsrc2]concat=v=2).
 */

int ff_filter_activate(AVFilterContext *filter)
{
    FFFilterContext *ctxi = fffilterctx(filter);
    const FFFilter *const fi = fffilter(filter->filter);

    int ret;

    /* Generic timeline support is not yet implemented but should be easy */
    av_assert1(!(fi->p.flags & AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC &&
                 fi->activate));
    ctxi->ready = 0;

    if (fi->filter_prepare) {
        ret = fi->filter_prepare(filter);
#if CONFIG_AVFILTER_THREAD_FRAME
        if (ret == AVERROR_EOF &&
            (filter->thread_type & AVFILTER_THREAD_FRAME_FILTER)) {
            ret = ff_filter_frame_thread_flush(filter);
            ret = (ret >= 0) ? AVERROR_EOF : ret;
        }
#endif
        if (ret < 0)
            return (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) ? 0 : ret;
    }

#if CONFIG_AVFILTER_THREAD_FRAME
    // XXX: use the same path independently of the callback being used?
    if (filter->thread_type & AVFILTER_THREAD_FRAME_FILTER && fi->activate)
        return ff_filter_frame_thread_activate(filter);
#endif

    ret = fi->activate ? fi->activate(filter) : filter_activate_default(filter);
    if (ret == FFERROR_NOT_READY)
        ret = 0;
    return ret;
}

int ff_inlink_acknowledge_status(AVFilterLink *link, int *rstatus, int64_t *rpts)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    *rpts = li->l.current_pts;
    if (ff_framequeue_queued_frames(&li->fifo))
        return *rstatus = 0;
    if (li->status_out)
        return *rstatus = li->status_out;
    if (!li->status_in)
        return *rstatus = 0;
    *rstatus = li->status_out = li->status_in;
    update_link_current_pts(li, li->status_in_pts);
    *rpts = li->l.current_pts;
    return 1;
}

size_t ff_inlink_queued_frames(AVFilterLink *link)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    return ff_framequeue_queued_frames(&li->fifo);
}

int ff_inlink_check_available_frame(AVFilterLink *link)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    return ff_framequeue_queued_frames(&li->fifo) > 0;
}

int ff_inlink_queued_samples(AVFilterLink *link)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    return ff_framequeue_queued_samples(&li->fifo);
}

int ff_inlink_check_available_samples(AVFilterLink *link, unsigned min)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    uint64_t samples = ff_framequeue_queued_samples(&li->fifo);
    av_assert1(min);
    return samples >= min || (li->status_in && samples);
}

static void consume_update(FilterLinkInternal *li, const AVFrame *frame)
{
    AVFilterLink *const link = &li->l.pub;
    update_link_current_pts(li, frame->pts);
    ff_inlink_process_commands(link, frame);
    if (link == link->dst->inputs[0])
        fffilterctx(link->dst)->is_disabled = !evaluate_timeline_at_frame(link, frame);
    li->l.frame_count_out++;
    li->l.sample_count_out += frame->nb_samples;
}

int ff_inlink_consume_frame(AVFilterLink *link, AVFrame **rframe)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    AVFrame *frame;

    *rframe = NULL;
    if (!ff_inlink_check_available_frame(link))
        return 0;

    if (li->fifo.samples_skipped) {
        frame = ff_framequeue_peek(&li->fifo, 0);
        return ff_inlink_consume_samples(link, frame->nb_samples, frame->nb_samples, rframe);
    }

    frame = ff_framequeue_take(&li->fifo);
    consume_update(li, frame);
    *rframe = frame;
    return 1;
}

int ff_inlink_consume_samples(AVFilterLink *link, unsigned min, unsigned max,
                            AVFrame **rframe)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    AVFrame *frame;
    int ret;

    av_assert1(min);
    *rframe = NULL;
    if (!ff_inlink_check_available_samples(link, min))
        return 0;
    if (li->status_in)
        min = FFMIN(min, ff_framequeue_queued_samples(&li->fifo));
    ret = take_samples(li, min, max, &frame);
    if (ret < 0)
        return ret;
    consume_update(li, frame);
    frame->duration = av_rescale_q(frame->nb_samples, (AVRational){ 1, frame->sample_rate },
                                   link->time_base);
    *rframe = frame;
    return 1;
}

AVFrame *ff_inlink_peek_frame(AVFilterLink *link, size_t idx)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    return ff_framequeue_peek(&li->fifo, idx);
}

int ff_inlink_make_frame_writable(AVFilterLink *link, AVFrame **rframe)
{
    AVFrame *frame = *rframe;
    AVFrame *out;
    int ret;

    if (av_frame_is_writable(frame))
        return 0;
    av_log(link->dst, AV_LOG_DEBUG, "Copying data in avfilter.\n");

    switch (link->type) {
    case AVMEDIA_TYPE_VIDEO:
        out = ff_get_video_buffer(link, link->w, link->h);
        break;
    case AVMEDIA_TYPE_AUDIO:
        out = ff_get_audio_buffer(link, frame->nb_samples);
        break;
    default:
        return AVERROR(EINVAL);
    }
    if (!out)
        return AVERROR(ENOMEM);

    ret = av_frame_copy_props(out, frame);
    if (ret < 0) {
        av_frame_free(&out);
        return ret;
    }

    ret = av_frame_copy(out, frame);
    if (ret < 0) {
        av_frame_free(&out);
        return ret;
    }

    av_frame_free(&frame);
    *rframe = out;
    return 0;
}

int ff_inlink_process_commands(AVFilterLink *link, const AVFrame *frame)
{
    FFFilterContext *ctxi = fffilterctx(link->dst);
    AVFilterCommand *cmd  = ctxi->command_queue;

    while(cmd && cmd->time <= frame->pts * av_q2d(link->time_base)){
        av_log(link->dst, AV_LOG_DEBUG,
               "Processing command time:%f command:%s arg:%s\n",
               cmd->time, cmd->command, cmd->arg);
        avfilter_process_command(link->dst, cmd->command, cmd->arg, 0, 0, cmd->flags);
        command_queue_pop(link->dst);
        cmd = ctxi->command_queue;
    }
    return 0;
}

void ff_inlink_request_frame(AVFilterLink *link)
{
    av_unused FilterLinkInternal *li = ff_link_internal(link);
    av_assert1(!li->status_in);
    av_assert1(!li->status_out);
    li->frame_wanted_out = 1;
    ff_filter_set_ready(link->src, 100);
}

void ff_inlink_set_status(AVFilterLink *link, int status)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    if (li->status_out)
        return;
    li->frame_wanted_out = 0;
    li->frame_blocked_in = 0;
    link_set_out_status(link, status, AV_NOPTS_VALUE);
    while (ff_framequeue_queued_frames(&li->fifo)) {
           AVFrame *frame = ff_framequeue_take(&li->fifo);
           av_frame_free(&frame);
    }
    if (!li->status_in)
        li->status_in = status;
}

int ff_outlink_get_status(AVFilterLink *link)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    return li->status_in;
}

int ff_inoutlink_check_flow(AVFilterLink *inlink, AVFilterLink *outlink)
{
    FilterLinkInternal * const li_in = ff_link_internal(inlink);
    return ff_outlink_frame_wanted(outlink) ||
           ff_inlink_check_available_frame(inlink) ||
           li_in->status_out;
}


const AVClass *avfilter_get_class(void)
{
    return &avfilter_class;
}

int ff_filter_init_hw_frames(AVFilterContext *avctx, AVFilterLink *link,
                             int default_pool_size)
{
    FilterLink *l = ff_filter_link(link);
    AVHWFramesContext *frames;

    // Must already be set by caller.
    av_assert0(l->hw_frames_ctx);

    frames = (AVHWFramesContext*)l->hw_frames_ctx->data;

    if (frames->initial_pool_size == 0) {
        // Dynamic allocation is necessarily supported.
    } else if (avctx->extra_hw_frames >= 0) {
        frames->initial_pool_size += avctx->extra_hw_frames;
    } else {
        frames->initial_pool_size = default_pool_size;
    }

    return 0;
}

int ff_outlink_frame_wanted(AVFilterLink *link)
{
    FilterLinkInternal * const li = ff_link_internal(link);
    return li->frame_wanted_out;
}

int ff_filter_execute(AVFilterContext *ctx, avfilter_action_func *func,
                      void *arg, int *ret, int nb_jobs)
{
    return fffilterctx(ctx)->execute(ctx, func, arg, ret, nb_jobs);
}

int ff_filter_disabled(const AVFilterContext *ctx)
{
    return cfffilterctx(ctx)->is_disabled;
}

int ff_filter_get_buffer_ext(AVFilterContext *ctx, AVFrame *frame,
                             int out_idx, int align, unsigned flags)
{
    AVFilterLink *outlink;
    AVFrame *tmp;

#if CONFIG_AVFILTER_THREAD_FRAME
    if (fffilterctx(ctx)->is_frame_thread)
        return ff_filter_frame_thread_get_buffer(ctx, frame, out_idx,
                                                 align, flags);
#endif

    if (align)
        return AVERROR(ENOSYS);

    if (out_idx < 0 || out_idx >= ctx->nb_outputs) {
        av_log(ctx, AV_LOG_ERROR, "Invalid output index: %d\n", out_idx);
        return AVERROR_BUG;
    }
    outlink = ctx->outputs[out_idx];

    switch (outlink->type) {
    case AVMEDIA_TYPE_VIDEO: {
        int w = frame->width  ? frame->width  : outlink->w;
        int h = frame->height ? frame->height : outlink->h;
        AVFrame *(*get_buffer)(AVFilterLink *, int, int) = outlink->dstpad->get_buffer.video;

        if (!get_buffer)
            get_buffer = ff_default_get_video_buffer;

        if (w < outlink->w || h < outlink->h) {
            av_log(ctx, AV_LOG_ERROR,
                   "Allocating video frame with invalid dimensions: %dx%d<%dx%d\n",
                   w, h, outlink->w, outlink->h);
            return AVERROR_BUG;
        }

        tmp = get_buffer(outlink, w, h);
        if (!tmp)
            return AVERROR(ENOMEM);

        break;
        }
    case AVMEDIA_TYPE_AUDIO: {
        AVFrame *(*get_buffer)(AVFilterLink *, int) = outlink->dstpad->get_buffer.audio;
        int nb_samples = frame->nb_samples;

        if (!get_buffer)
            get_buffer = ff_default_get_audio_buffer;

        if (nb_samples <= 0) {
            av_log(ctx, AV_LOG_ERROR, "Allocating frame with invalid sample count\n");
            return AVERROR_BUG;
        }

        tmp = get_buffer(outlink, nb_samples);
        if (!tmp)
            return AVERROR(ENOMEM);

        break;
        }
    default: av_assert0(0);
    }

    av_frame_unref(frame);
    av_frame_move_ref(frame, tmp);
    av_frame_free(&tmp);

    return 0;
}

int ff_filter_is_frame_thread(const AVFilterContext *ctx)
{
    return cfffilterctx(ctx)->is_frame_thread;
}
