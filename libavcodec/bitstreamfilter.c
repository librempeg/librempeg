/*
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

#include "config.h"

#include <string.h>

#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "bsf.h"
#include "bsf_internal.h"

static const char *default_bsf_name(void *filter_ctx)
{
    AVBitStreamFilterContext *ctx = filter_ctx;
    return ctx->name ? ctx->name : ctx->filter->name;
}

static void *bsf_child_next(void *obj, void *prev)
{
    AVBitStreamFilterContext *ctx = obj;
    if (!prev && ctx->filter && ctx->filter->priv_class && ctx->priv_data)
        return ctx->priv_data;
    return NULL;
}

static const AVClass bitstreamfilter_class = {
    .class_name = "AVBitStreamFilterContext",
    .item_name  = default_bsf_name,
    .version    = LIBAVUTIL_VERSION_INT,
    .category   = AV_CLASS_CATEGORY_BITSTREAM_FILTER,
    .child_next = bsf_child_next,
    .child_class_iterate = ff_bsf_child_class_iterate,
    .state_flags_offset = offsetof(FFBitStreamFilterContext, state_flags),
};

int ff_bsf_alloc(const AVBitStreamFilter *filter, const char *inst_name, AVBitStreamFilterContext **pctx)
{
    FFBitStreamFilterContext *ctxi;
    AVBitStreamFilterContext *ctx;
    const FFBitStreamFilter *const fi = ff_bsf(filter);
    int ret = AVERROR(ENOMEM), preinited = 0;

    av_assert0(filter);

    ctxi = av_mallocz(sizeof(*ctxi));
    if (!ctxi)
        return AVERROR(ENOMEM);
    *pctx = ctx = &ctxi->p;

    ctx->av_class = &bitstreamfilter_class;
    ctx->filter   = filter;
    ctx->name     = inst_name ? av_strdup(inst_name) : NULL;
    if (fi->priv_data_size) {
        ctx->priv_data     = av_mallocz(fi->priv_data_size);
        if (!ctx->priv_data)
            goto err;
    }
    if (fi->preinit) {
        ret = fi->preinit(ctx);
        if (ret < 0)
            goto err;
        preinited = 1;
    }

    av_opt_set_defaults(ctx);
    if (filter->priv_class) {
        *(const AVClass**)ctx->priv_data = filter->priv_class;
        av_opt_set_defaults(ctx->priv_data);
    }

    ctx->nb_inputs  = fi->nb_inputs;
    if (ctx->nb_inputs ) {
        ctx->input_pads = av_memdup(fi->inputs, ctx->nb_inputs * sizeof(*fi->inputs));
        if (!ctx->input_pads)
            goto err;
        ctx->inputs     = av_calloc(ctx->nb_inputs, sizeof(*ctx->inputs));
        if (!ctx->inputs)
            goto err;
    }

    ctx->nb_outputs = fi->nb_outputs;
    if (ctx->nb_outputs) {
        ctx->output_pads = av_memdup(fi->outputs, ctx->nb_outputs * sizeof(*fi->outputs));
        if (!ctx->output_pads)
            goto err;
        ctx->outputs     = av_calloc(ctx->nb_outputs, sizeof(*ctx->outputs));
        if (!ctx->outputs)
            goto err;
    }

    return 0;

err:
    if (preinited)
        fi->uninit(ctx);
    av_freep(&ctx->name);
    av_freep(&ctx->inputs);
    av_freep(&ctx->input_pads);
    ctx->nb_inputs = 0;
    av_freep(&ctx->outputs);
    av_freep(&ctx->output_pads);
    ctx->nb_outputs = 0;
    av_freep(&ctx->priv_data);
    av_freep(pctx);
    return ret;
}

int av_bsf_link(AVBitStreamFilterContext *src, unsigned srcpad,
                AVBitStreamFilterContext *dst, unsigned dstpad)
{
    BitStreamFilterLinkInternal *li;
    AVBitStreamFilterLink *link;

    av_assert0(src->graph);
    av_assert0(dst->graph);
    av_assert0(src->graph == dst->graph);

    if (src->nb_outputs <= srcpad || dst->nb_inputs <= dstpad ||
        src->outputs[srcpad]      || dst->inputs[dstpad])
        return AVERROR(EINVAL);

    if (!(ffbsfctx(src)->state_flags & AV_CLASS_STATE_INITIALIZED) ||
        !(ffbsfctx(dst)->state_flags & AV_CLASS_STATE_INITIALIZED)) {
        av_log(src, AV_LOG_ERROR, "Filters must be initialized before linking.\n");
        return AVERROR(EINVAL);
    }

    if (src->output_pads[srcpad].codec_ids && dst->input_pads[dstpad].codec_ids) {
        int found = 0;
        for (int i = 0; src->output_pads[srcpad].codec_ids[i] != AV_CODEC_ID_NONE; i++) {
            for (int j = 0; dst->input_pads[dstpad].codec_ids[j] != AV_CODEC_ID_NONE; j++)
                if (src->output_pads[srcpad].codec_ids[i] == dst->input_pads[dstpad].codec_ids[j]) {
                    found = 1;
                    break;
                }
            if (found)
                break;
        }

        if (!found) {
            av_log(src, AV_LOG_ERROR, "No common codec id between source and dest pads\n");
            av_log(src, AV_LOG_ERROR, "Supported input pad codecs are: ");
            for (int i = 0; dst->input_pads[dstpad].codec_ids[i] != AV_CODEC_ID_NONE; i++) {
                enum AVCodecID codec_id = dst->input_pads[dstpad].codec_ids[i];
                av_log(src, AV_LOG_ERROR, "%s (%d) ",
                       avcodec_get_name(codec_id), codec_id);
            }
            av_log(src, AV_LOG_ERROR, "\n");
            av_log(src, AV_LOG_ERROR, "Supported output pad codecs are: ");
            for (int i = 0; src->output_pads[srcpad].codec_ids[i] != AV_CODEC_ID_NONE; i++) {
                enum AVCodecID codec_id = src->output_pads[srcpad].codec_ids[i];
                av_log(src, AV_LOG_ERROR, "%s (%d) ",
                       avcodec_get_name(codec_id), codec_id);
            }
            av_log(src, AV_LOG_ERROR, "\n");

            return AVERROR(EINVAL);
        }
    }

    li = av_mallocz(sizeof(*li));
    if (!li)
        return AVERROR(ENOMEM);
    link = &li->l;

    src->outputs[srcpad] = dst->inputs[dstpad] = link;

    link->src     = src;
    link->dst     = dst;
    link->srcpad  = &src->output_pads[srcpad];
    link->dstpad  = &dst->input_pads[dstpad];
    link->graph   = src->graph;
    li->fifo      = av_container_fifo_alloc_avpacket(0);
    if (!li->fifo)
        return AVERROR(ENOMEM);

    return 0;
}

static void link_free(AVBitStreamFilterLink **plink)
{
    AVBitStreamFilterLink *link = *plink;
    BitStreamFilterLinkInternal *li;

    if (!link)
        return;
    li = ff_link_internal(link);

    avcodec_parameters_free(&link->par);
    av_container_fifo_free(&li->fifo);

    av_freep(plink);
}

static void free_link(AVBitStreamFilterLink *link)
{
    if (!link)
        return;

    if (link->src)
        link->src->outputs[link->srcpad - link->src->output_pads] = NULL;
    if (link->dst)
        link->dst->inputs[link->dstpad - link->dst->input_pads] = NULL;

    link_free(&link);
}

void ff_bsf_free(AVBitStreamFilterContext *ctx)
{
    int i;

    if (!ctx)
        return;

    if (ctx->graph)
        ff_bsf_graph_remove_filter(ctx->graph, ctx);

    if (ff_bsf(ctx->filter)->uninit)
        ff_bsf(ctx->filter)->uninit(ctx);

    for (i = 0; i < ctx->nb_inputs; i++) {
        free_link(ctx->inputs[i]);
        if (ctx->input_pads[i].flags  & FF_BSF_PAD_FLAG_FREE_NAME)
            av_freep(&ctx->input_pads[i].name);
    }
    for (i = 0; i < ctx->nb_outputs; i++) {
        free_link(ctx->outputs[i]);
        if (ctx->output_pads[i].flags & FF_BSF_PAD_FLAG_FREE_NAME)
            av_freep(&ctx->output_pads[i].name);
    }

    if (ctx->filter->priv_class)
        av_opt_free(ctx->priv_data);

    av_freep(&ctx->name);
    av_freep(&ctx->input_pads);
    av_freep(&ctx->output_pads);
    av_freep(&ctx->inputs);
    av_freep(&ctx->outputs);
    av_freep(&ctx->priv_data);
    av_opt_free(ctx);
    av_free(ctx);
}

/**
 * Parse filter options into a dictionary.
 *
 * @param logctx context for logging
 * @param priv_class a filter's private class for shorthand options or NULL
 * @param options dictionary to store parsed options in
 * @param args options string to parse
 *
 * @return a non-negative number on success, a negative error code on failure
 */
static int bsf_opt_parse(void *logctx, const AVClass *priv_class,
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

int av_bsf_init_dict(AVBitStreamFilterContext *ctx, AVDictionary **options)
{
    FFBitStreamFilterContext *ctxi = ffbsfctx(ctx);
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

    if (ff_bsf(ctx->filter)->init2)
        ret = ff_bsf(ctx->filter)->init2(ctx);
    if (ret < 0)
        return ret;

    ctxi->state_flags |= AV_CLASS_STATE_INITIALIZED;

    return 0;
}

int av_bsf_init_str(AVBitStreamFilterContext *ctx, const char *args)
{
    AVDictionary *options = NULL;
    const AVDictionaryEntry *e;
    int ret = 0;

    if (args && *args) {
        ret = bsf_opt_parse(ctx, ctx->filter->priv_class, &options, args);
        if (ret < 0)
            goto fail;
    }

    ret = av_bsf_init_dict(ctx, &options);
    if (ret < 0)
        goto fail;

    if ((e = av_dict_iterate(options, NULL))) {
        av_log(ctx, AV_LOG_ERROR, "No such option: %s.\n", e->key);
        ret = AVERROR_OPTION_NOT_FOUND;
        goto fail;
    }

fail:
    av_dict_free(&options);

    return ret;
}

const char *av_bsf_pad_get_name(const AVBitStreamFilterPad *pads, int pad_idx)
{
    return pads[pad_idx].name;
}

const enum AVCodecID *av_bsf_pad_get_codec_ids(const AVBitStreamFilterPad *pads, int pad_idx)
{
    return pads[pad_idx].codec_ids;
}

static void update_link_current_pts(BitStreamFilterLinkInternal *li, int64_t pts)
{
    if (pts == AV_NOPTS_VALUE)
        return;
    li->l.current_pts = pts;
    li->l.current_pts_us = av_rescale_q(pts, li->l.time_base, AV_TIME_BASE_Q);
    if (li->l.graph && li->age_index >= 0)
        ff_bsf_graph_update_heap(li->l.graph, li);
}

void ff_bsf_set_ready(AVBitStreamFilterContext *filter, unsigned priority)
{
    FFBitStreamFilterContext *ctxi = ffbsfctx(filter);
    ctxi->ready = FFMAX(ctxi->ready, priority);
}

/**
 * Clear packet_blocked_in on all outputs.
 * This is necessary whenever something changes on input.
 */
static void filter_unblock(AVBitStreamFilterContext *filter)
{
    unsigned i;

    for (i = 0; i < filter->nb_outputs; i++) {
        BitStreamFilterLinkInternal * const li = ff_link_internal(filter->outputs[i]);
        li->packet_blocked_in = 0;
    }
}

void ff_bsf_link_set_in_status(AVBitStreamFilterLink *link, int status, int64_t pts)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);

    if (li->status_in == status)
        return;
    av_assert0(!li->status_in);
    li->status_in = status;
    li->status_in_pts = pts;
    li->packet_wanted_out = 0;
    li->packet_blocked_in = 0;
    filter_unblock(link->dst);
    ff_bsf_set_ready(link->dst, 200);
}

/**
 * Set the status field of a link from the destination filter.
 * The pts should probably be left unset (AV_NOPTS_VALUE).
 */
static void link_set_out_status(AVBitStreamFilterLink *link, int status, int64_t pts)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);

    av_assert0(!li->packet_wanted_out);
    av_assert0(!li->status_out);
    li->status_out = status;
    if (pts != AV_NOPTS_VALUE)
        update_link_current_pts(li, pts);
    filter_unblock(link->dst);
    ff_bsf_set_ready(link->src, 200);
}

int ff_bsf_config_links(AVBitStreamFilterContext *filter)
{
    int (*config_link)(AVBitStreamFilterLink *);
    unsigned i;
    int ret;

    for (i = 0; i < filter->nb_inputs; i ++) {
        AVBitStreamFilterLink *link = filter->inputs[i];
        AVBitStreamFilterLink *inlink;
        BitStreamFilterLinkInternal *li = ff_link_internal(link);

        if (!link) continue;
        if (!link->src || !link->dst) {
            av_log(filter, AV_LOG_ERROR,
                   "Not all input and output are properly linked (%d).\n", i);
            return AVERROR(EINVAL);
        }

        inlink = link->src->nb_inputs ? link->src->inputs[0] : NULL;
        link->current_pts =
        link->current_pts_us = AV_NOPTS_VALUE;

        switch (li->init_state) {
        case AVLINK_INIT:
            continue;
        case AVLINK_STARTINIT:
            av_log(filter, AV_LOG_INFO, "circular filter chain detected\n");
            return 0;
        case AVLINK_UNINIT:
            li->init_state = AVLINK_STARTINIT;

            if ((ret = ff_bsf_config_links(link->src)) < 0)
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

            if (inlink) {
                av_assert0(!link->par && inlink->par);
                link->par = avcodec_parameters_alloc();
                if (!link->par)
                    return AVERROR(ENOMEM);
                ret = avcodec_parameters_copy(link->par, inlink->par);
                if (ret < 0)
                    return ret;
            } else {
                av_assert0(!link->par);
                link->par = avcodec_parameters_alloc();
                if (!link->par)
                    return AVERROR(ENOMEM);
            }

            if (config_link && (ret = config_link(link)) < 0) {
                av_log(link->src, AV_LOG_ERROR,
                       "Failed to configure output pad on %s\n",
                       link->src->name);
                return ret;
            }

            switch (link->par->codec_type) {
            case AVMEDIA_TYPE_VIDEO:
                if (!link->time_base.num && !link->time_base.den)
                    link->time_base = inlink ? inlink->time_base : AV_TIME_BASE_Q;
                break;

            case AVMEDIA_TYPE_AUDIO:
                if (inlink) {
                    if (!link->time_base.num && !link->time_base.den)
                        link->time_base = inlink->time_base;
                }

                if (!link->time_base.num && !link->time_base.den)
                    link->time_base = (AVRational) {1, link->par->sample_rate};
                break;
            }

            if (link->dstpad->codec_ids) {
                int j;
                for (j = 0; link->dstpad->codec_ids[j] != AV_CODEC_ID_NONE; j++)
                    if (link->par->codec_id == link->dstpad->codec_ids[j])
                        break;
                if (link->dstpad->codec_ids[j] == AV_CODEC_ID_NONE) {
			        av_log(link->dst, AV_LOG_ERROR, "Unsupported input codec id %s (%d). Supported input pad codecs are: ",
                           avcodec_get_name(link->par->codec_id), link->par->codec_id);
                    for (j = 0; link->dstpad->codec_ids[j] != AV_CODEC_ID_NONE; j++) {
                        enum AVCodecID codec_id = link->dstpad->codec_ids[j];
                        av_log(link->dst, AV_LOG_ERROR, "%s (%d) ",
                               avcodec_get_name(codec_id), codec_id);
                    }
                    av_log(link->dst, AV_LOG_ERROR, "\n");
                    return AVERROR(EINVAL);
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

int ff_bsf_request_packet(AVBitStreamFilterLink *link)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);

    av_assert1(!ff_bsf(link->dst->filter)->activate);
    if (li->status_out)
        return li->status_out;
    if (li->status_in) {
        if (av_container_fifo_can_read(li->fifo)) {
            av_assert1(!li->packet_wanted_out);
            av_assert1(ffbsfctx(link->dst)->ready >= 300);
            return 0;
        } else {
            /* Acknowledge status change. Filters using ff_bsf_request_packet() will
               handle the change automatically. Filters can also check the
               status directly but none do yet. */
            link_set_out_status(link, li->status_in, li->status_in_pts);
            return li->status_out;
        }
    }
    li->packet_wanted_out = 1;
    ff_bsf_set_ready(link->src, 100);
    return 0;
}

static int64_t guess_status_pts(AVBitStreamFilterContext *ctx, int status, AVRational link_time_base)
{
    unsigned i;
    int64_t r = INT64_MAX;

    for (i = 0; i < ctx->nb_inputs; i++) {
        AVBitStreamFilterLink * const link = ctx->inputs[i];
        BitStreamFilterLinkInternal * const li = ff_link_internal(link);
        if (li->status_out == status)
            r = FFMIN(r, av_rescale_q(link->current_pts, link->time_base, link_time_base));
    }
    if (r < INT64_MAX)
        return r;
    av_log(ctx, AV_LOG_WARNING, "EOF timestamp not reliable\n");
    for (i = 0; i < ctx->nb_inputs; i++) {
        AVBitStreamFilterLink * const link = ctx->inputs[i];
        BitStreamFilterLinkInternal * const li = ff_link_internal(link);
        r = FFMIN(r, av_rescale_q(li->status_in_pts, link->time_base, link_time_base));
    }
    if (r < INT64_MAX)
        return r;
    return AV_NOPTS_VALUE;
}

static int request_packet_to_filter(AVBitStreamFilterLink *link)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);
    int ret = -1;

    /* Assume the filter is blocked, let the method clear it if not */
    li->packet_blocked_in = 1;
    if (link->srcpad->request_packet)
        ret = link->srcpad->request_packet(link);
    else if (link->src->inputs[0])
        ret = ff_bsf_request_packet(link->src->inputs[0]);
    if (ret < 0) {
        if (ret != AVERROR(EAGAIN) && ret != li->status_in)
            ff_bsf_link_set_in_status(link, ret, guess_status_pts(link->src, ret, link->time_base));
        if (ret == AVERROR_EOF)
            ret = 0;
    }
    return ret;
}

static int default_filter_packet(AVBitStreamFilterLink *link, AVPacket *pkt)
{
    return ff_bsf_filter_packet(link->dst->outputs[0], pkt);
}

static int filter_packet_framed(AVBitStreamFilterLink *link, AVPacket *pkt)
{
    int (*filter)(AVBitStreamFilterLink *, AVPacket *);
    AVBitStreamFilterPad *dst = link->dstpad;
    int ret;

    if (!(filter = dst->filter))
        filter = default_filter_packet;

    if (dst->flags & FF_BSF_PAD_FLAG_NEEDS_WRITABLE) {
        ret = av_packet_make_writable(pkt);
        if (ret < 0)
            goto fail;
    }

    ret = filter(link, pkt);
    link->packet_count_out++;
    return ret;

fail:
    av_packet_free(&pkt);
    return ret;
}

int ff_bsf_filter_packet(AVBitStreamFilterLink *link, AVPacket *pkt)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);
    FFBitStreamFilterGraph * const graphi = ffbsffiltergraph(link->graph);
    int ret;

    li->packet_blocked_in = li->packet_wanted_out = 0;
    link->packet_count_in++;
    filter_unblock(link->dst);
    if (av_container_fifo_can_read(li->fifo) >= graphi->max_packet_queue)
        return AVERROR(ENOMEM);
    ret = av_container_fifo_write(li->fifo, pkt, 0);
    av_packet_free(&pkt);
    if (ret < 0)
        return ret;
    av_assert1(graphi->packets_queued < graphi->max_packet_queue);
    graphi->packets_queued++;
    ff_bsf_set_ready(link->dst, 300);
    return 0;
}

static int filter_packet_to_filter(AVBitStreamFilterLink *link)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);
    AVPacket *pkt = NULL;
    AVBitStreamFilterContext *dst = link->dst;
    int ret;

    av_assert1(av_container_fifo_can_read(li->fifo));
    ret = ff_bsf_inlink_consume_packet(link, &pkt);
    av_assert1(ret);
    if (ret < 0) {
        av_assert1(!pkt);
        return ret;
    }

    /* The filter will soon have received a new packet, that may allow it to
       produce one or more: unblock its outputs. */
    filter_unblock(dst);
    /* AVBitStreamFilterPad.filter_packet() expect packet_count_out to have the value
       before the packet; filter_packet_framed() will re-increment it. */
    link->packet_count_out--;
    ret = filter_packet_framed(link, pkt);
    if (ret < 0 && ret != li->status_out) {
        link_set_out_status(link, ret, AV_NOPTS_VALUE);
    } else {
        /* Run once again, to see if several packets were available, or if
           the input status has also changed, or any other reason. */
        ff_bsf_set_ready(dst, 300);
    }
    return ret;
}

static int forward_status_change(AVBitStreamFilterContext *filter, BitStreamFilterLinkInternal *li_in)
{
    AVBitStreamFilterLink *in = &li_in->l;
    unsigned out = 0, progress = 0;
    int ret;

    av_assert0(!li_in->status_out);
    if (!filter->nb_outputs) {
        /* not necessary with the current API and sinks */
        return 0;
    }
    while (!li_in->status_out) {
        BitStreamFilterLinkInternal *li_out = ff_link_internal(filter->outputs[out]);

        if (!li_out->status_in) {
            progress++;
            ret = request_packet_to_filter(filter->outputs[out]);
            if (ret < 0)
                return ret;
        }
        if (++out == filter->nb_outputs) {
            if (!progress) {
                /* Every output already closed: input no longer interesting. */
                link_set_out_status(in, li_in->status_in, li_in->status_in_pts);
                return 0;
            }
            progress = 0;
            out = 0;
        }
    }
    ff_bsf_set_ready(filter, 200);
    return 0;
}

static int filter_activate_default(AVBitStreamFilterContext *ctx)
{
    int i, nb_eofs = 0;

    for (i = 0; i < ctx->nb_outputs; i++)
        nb_eofs += ff_bsf_outlink_get_status(ctx->outputs[i]) == AVERROR_EOF;
    if (ctx->nb_outputs && nb_eofs == ctx->nb_outputs) {
        for (int j = 0; j < ctx->nb_inputs; j++)
            ff_bsf_inlink_set_status(ctx->inputs[j], AVERROR_EOF);
        return 0;
    }

    for (i = 0; i < ctx->nb_inputs; i++) {
        BitStreamFilterLinkInternal *li = ff_link_internal(ctx->inputs[i]);
        if (av_container_fifo_can_read(li->fifo)) {
            return filter_packet_to_filter(ctx->inputs[i]);
        }
    }
    for (i = 0; i < ctx->nb_inputs; i++) {
        BitStreamFilterLinkInternal * const li = ff_link_internal(ctx->inputs[i]);
        if (li->status_in && !li->status_out) {
            av_assert1(!av_container_fifo_can_read(li->fifo));
            return forward_status_change(ctx, li);
        }
    }
    for (i = 0; i < ctx->nb_outputs; i++) {
        BitStreamFilterLinkInternal * const li = ff_link_internal(ctx->outputs[i]);
        if (li->packet_wanted_out &&
            !li->packet_blocked_in) {
            return request_packet_to_filter(ctx->outputs[i]);
        }
    }
    for (i = 0; i < ctx->nb_outputs; i++) {
        BitStreamFilterLinkInternal * const li = ff_link_internal(ctx->outputs[i]);
        if (li->packet_wanted_out)
            return request_packet_to_filter(ctx->outputs[i]);
    }
    if (!ctx->nb_outputs) {
        ff_bsf_inlink_request_packet(ctx->inputs[0]);
        return 0;
    }
    return FFERROR_BSF_NOT_READY;
}

int ff_bsf_activate(AVBitStreamFilterContext *ctx)
{
    FFBitStreamFilterContext *ctxi = ffbsfctx(ctx);
    const FFBitStreamFilter *const fi = ff_bsf(ctx->filter);
    int ret;

    ctxi->ready = 0;
    ret = fi->activate ? fi->activate(ctx) : filter_activate_default(ctx);
    if (ret == FFERROR_BSF_NOT_READY)
        ret = 0;
    return ret;
}

int ff_bsf_inlink_acknowledge_status(AVBitStreamFilterLink *link, int *rstatus, int64_t *rpts)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);
    *rpts = link->current_pts;
    if (av_container_fifo_can_read(li->fifo))
        return *rstatus = 0;
    if (li->status_out)
        return *rstatus = li->status_out;
    if (!li->status_in)
        return *rstatus = 0;
    *rstatus = li->status_out = li->status_in;
    update_link_current_pts(li, li->status_in_pts);
    *rpts = link->current_pts;
    return 1;
}

size_t ff_bsf_inlink_queued_packets(AVBitStreamFilterLink *link)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);
    return av_container_fifo_can_read(li->fifo);
}

int ff_bsf_inlink_check_available_packet(AVBitStreamFilterLink *link)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);
    return av_container_fifo_can_read(li->fifo) > 0;
}

static void consume_update(BitStreamFilterLinkInternal *li, const AVPacket *pkt)
{
    update_link_current_pts(li, pkt->pts);
    li->l.packet_count_out++;
}

int ff_bsf_inlink_consume_packet(AVBitStreamFilterLink *link, AVPacket **rpkt)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);
    FFBitStreamFilterGraph * const graphi = ffbsffiltergraph(link->graph);
    AVPacket *pkt;

    *rpkt = NULL;
    if (!av_container_fifo_can_read(li->fifo))
        return 0;

    pkt = av_packet_alloc();
    if (!pkt)
        return AVERROR(ENOMEM);

    av_container_fifo_read(li->fifo, pkt, 0);
    consume_update(li, pkt);
    av_assert1(graphi->packets_queued > 0);
    graphi->packets_queued--;
    *rpkt = pkt;
    return 1;
}

void ff_bsf_inlink_request_packet(AVBitStreamFilterLink *link)
{
    BitStreamFilterLinkInternal *li = ff_link_internal(link);
    av_assert1(!li->status_in);
    av_assert1(!li->status_out);
    li->packet_wanted_out = 1;
    ff_bsf_set_ready(link->src, 100);
}

void ff_bsf_inlink_set_status(AVBitStreamFilterLink *link, int status)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);
    FFBitStreamFilterGraph * const graphi = ffbsffiltergraph(link->graph);
    if (li->status_out)
        return;
    li->packet_wanted_out = 0;
    li->packet_blocked_in = 0;
    link_set_out_status(link, status, AV_NOPTS_VALUE);
    while (av_container_fifo_can_read(li->fifo)) {
        AVPacket pkt;
        av_container_fifo_read(li->fifo, &pkt, 0);
        av_packet_unref(&pkt);
        av_assert1(graphi->packets_queued > 0);
        graphi->packets_queued--;
    }
    if (!li->status_in)
        li->status_in = status;
}

int ff_bsf_outlink_get_status(AVBitStreamFilterLink *link)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);
    return li->status_in;
}

int ff_bsf_outlink_packet_wanted(AVBitStreamFilterLink *link)
{
    BitStreamFilterLinkInternal * const li = ff_link_internal(link);
    return li->packet_wanted_out;
}

const AVBitStreamFilterPad ff_default_bsf_pad[1] = {
    {
        .name = "default",
    }
};
