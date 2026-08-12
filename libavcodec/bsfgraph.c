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

#include <stddef.h>
#include <string.h>

#include "libavutil/avassert.h"
#include "libavutil/error.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "bsf.h"
#include "bsf_internal.h"

#define OFFSET(x) offsetof(AVBitStreamFilterGraph, x)
#define FLAGS (AV_OPT_FLAG_BSF_PARAM|AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption filtergraph_options[] = {
    {"max_buffered_packets"  , "maximum number of buffered packets allowed", OFFSET(max_buffered_packets),
        AV_OPT_TYPE_UINT,   {.i64 = 0}, 0, UINT_MAX, FLAGS },
    { NULL },
};

static const AVClass filtergraph_class = {
    .class_name = "AVBitStreamFilterGraph",
    .item_name  = av_default_item_name,
    .version    = LIBAVUTIL_VERSION_INT,
    .option     = filtergraph_options,
    .category   = AV_CLASS_CATEGORY_BITSTREAM_FILTER,
};

AVBitStreamFilterGraph *av_bsf_graph_alloc(void)
{
    FFBitStreamFilterGraph *graph = av_mallocz(sizeof(*graph));
    AVBitStreamFilterGraph *ret;

    if (!graph)
        return NULL;

    ret = &graph->p;
    ret->av_class = &filtergraph_class;
    av_opt_set_defaults(ret);
    graph->max_packet_queue = SIZE_MAX;

    return ret;
}

void ff_bsf_graph_remove_filter(AVBitStreamFilterGraph *graph, AVBitStreamFilterContext *filter)
{
    int i, j;
    for (i = 0; i < graph->nb_filters; i++) {
        if (graph->filters[i] == filter) {
            FFSWAP(AVBitStreamFilterContext*, graph->filters[i],
                   graph->filters[graph->nb_filters - 1]);
            graph->nb_filters--;
            filter->graph = NULL;
            for (j = 0; j<filter->nb_outputs; j++)
                if (filter->outputs[j])
                    filter->outputs[j]->graph = NULL;

            return;
        }
    }
}

AVBitStreamFilterContext *av_bsf_graph_get_filter(AVBitStreamFilterGraph *graph, const char *name)
{
    int i;

    for (i = 0; i < graph->nb_filters; i++)
        if (graph->filters[i]->name && !strcmp(name, graph->filters[i]->name))
            return graph->filters[i];

    return NULL;
}

void av_bsf_graph_free(AVBitStreamFilterGraph **graphp)
{
    AVBitStreamFilterGraph *graph = *graphp;
    FFBitStreamFilterGraph *graphi = ffbsffiltergraph(graph);

    if (!graph)
        return;

    while (graph->nb_filters)
        ff_bsf_free(graph->filters[0]);

    av_freep(&graphi->sink_links);
    av_freep(&graphi->source_links);

    av_opt_free(graph);

    av_freep(&graph->filters);
    av_freep(graphp);
}

int av_bsf_graph_create_filter(AVBitStreamFilterContext **filt_ctx, const AVBitStreamFilter *filt,
                               const char *name, AVDictionary **options, AVBitStreamFilterGraph *graph_ctx)
{
    AVBitStreamFilterContext *s;
    int ret;

    ret = av_bsf_graph_alloc_filter(&s, filt, name, graph_ctx);
    if (ret < 0)
        return ret;

    ret = av_bsf_init_dict(s, options);
    if (ret < 0)
        goto fail;

    if (filt_ctx)
        *filt_ctx = s;

    return 0;

fail:
    ff_bsf_free(s);
    if (filt_ctx)
        *filt_ctx = NULL;
    return ret;
}

int av_bsf_graph_alloc_filter(AVBitStreamFilterContext **filt_ctx,
                              const AVBitStreamFilter *filter,
                              const char *name,
                              AVBitStreamFilterGraph *graph)
{
    AVBitStreamFilterContext **filters, *s;
    int ret;

    if (!ff_bsf(filter)->activate && !ff_bsf(filter)->nb_inputs && !ff_bsf(filter)->nb_outputs)
        return AVERROR(ENOTSUP);

    filters = av_realloc_array(graph->filters, graph->nb_filters + 1, sizeof(*filters));
    if (!filters)
        return AVERROR(ENOMEM);
    graph->filters = filters;

    ret = ff_bsf_alloc(filter, name, &s);
    if (ret < 0)
        return ret;

    graph->filters[graph->nb_filters++] = s;

    s->graph = graph;

    if (filt_ctx)
        *filt_ctx = s;

    return ret;
}

/**
 * Check for the validity of graph.
 *
 * A graph is considered valid if all its input and output pads are
 * connected.
 *
 * @return >= 0 in case of success, a negative value otherwise
 */
static int graph_check_validity(AVBitStreamFilterGraph *graph, void *log_ctx)
{
    AVBitStreamFilterContext *filt;
    int i, j;

    for (i = 0; i < graph->nb_filters; i++) {
        const AVBitStreamFilterPad *pad;
        filt = graph->filters[i];

        for (j = 0; j < filt->nb_inputs; j++) {
            if (!filt->inputs[j] || !filt->inputs[j]->src) {
                pad = &filt->input_pads[j];
                av_log(log_ctx, AV_LOG_ERROR,
                       "Input pad \"%s\" of the filter instance \"%s\" of %s not connected to any source\n",
                       pad->name, filt->name, filt->filter->name);
                return AVERROR(EINVAL);
            }
        }

        for (j = 0; j < filt->nb_outputs; j++) {
            if (!filt->outputs[j] || !filt->outputs[j]->dst) {
                pad = &filt->output_pads[j];
                av_log(log_ctx, AV_LOG_ERROR,
                       "Output pad \"%s\" of the filter instance \"%s\" of %s not connected to any destination\n",
                       pad->name, filt->name, filt->filter->name);
                return AVERROR(EINVAL);
            }
        }
    }

    return 0;
}

/**
 * Configure all the links of graphctx.
 *
 * @return >= 0 in case of success, a negative value otherwise
 */
static int graph_config_links(AVBitStreamFilterGraph *graph, void *log_ctx)
{
    AVBitStreamFilterContext *filt;
    int i, ret;

    for (i = 0; i < graph->nb_filters; i++) {
        filt = graph->filters[i];

        if (!filt->nb_outputs) {
            if ((ret = ff_bsf_config_links(filt)))
                return ret;
        }
    }

    return 0;
}

static int graph_config_pointers(AVBitStreamFilterGraph *graph, void *log_ctx)
{
    unsigned i, j;
    int sink_links_count = 0, source_links_count = 0, n = 0;
    AVBitStreamFilterContext *f;
    BitStreamFilterLinkInternal **sinks, **sources;

    for (i = 0; i < graph->nb_filters; i++) {
        f = graph->filters[i];
        for (j = 0; j < f->nb_inputs; j++) {
            ff_link_internal(f->inputs[j])->age_index  = -1;
        }
        for (j = 0; j < f->nb_outputs; j++) {
            ff_link_internal(f->outputs[j])->age_index = -1;
        }
        if (!f->nb_outputs) {
            if (f->nb_inputs > INT_MAX - sink_links_count)
                return AVERROR(EINVAL);
            sink_links_count += f->nb_inputs;
        }
        if (!f->nb_inputs && !strcmp(f->filter->name, "source")) {
            if (f->nb_outputs > INT_MAX - source_links_count)
                return AVERROR(EINVAL);
            source_links_count += f->nb_outputs;
        }
    }
    sinks = av_calloc(sink_links_count, sizeof(*sinks));
    if (!sinks)
        return AVERROR(ENOMEM);
    for (i = 0; i < graph->nb_filters; i++) {
        f = graph->filters[i];
        if (!f->nb_outputs) {
            for (j = 0; j < f->nb_inputs; j++) {
                sinks[n] = ff_link_internal(f->inputs[j]);
                sinks[n]->age_index = n;
                n++;
            }
        }
    }
    av_assert0(n == sink_links_count);
    ffbsffiltergraph(graph)->sink_links       = sinks;
    ffbsffiltergraph(graph)->sink_links_count = sink_links_count;

    sources = av_calloc(source_links_count, sizeof(*sources));
    if (!sources)
        return AVERROR(ENOMEM);
    for (i = 0, n = 0; i < graph->nb_filters; i++) {
        f = graph->filters[i];
        if (!f->nb_inputs && !strcmp(f->filter->name, "source")) {
            for (j = 0; j < f->nb_outputs; j++) {
                sources[n] = ff_link_internal(f->outputs[j]);
                n++;
            }
        }
    }
    av_assert0(n == source_links_count);
    ffbsffiltergraph(graph)->source_links       = sources;
    ffbsffiltergraph(graph)->source_links_count = source_links_count;

    return 0;
}

int av_bsf_graph_config(AVBitStreamFilterGraph *graphctx, void *log_ctx)
{
    int ret;

    if (graphctx->max_buffered_packets)
        ffbsffiltergraph(graphctx)->max_packet_queue = graphctx->max_buffered_packets;
    if ((ret = graph_check_validity(graphctx, log_ctx)))
        return ret;
    if ((ret = graph_config_links(graphctx, log_ctx)))
        return ret;
    if ((ret = graph_config_pointers(graphctx, log_ctx)))
        return ret;

    return 0;
}

static void heap_bubble_up(FFBitStreamFilterGraph *graph,
                           BitStreamFilterLinkInternal *li, int index)
{
    BitStreamFilterLinkInternal **links = graph->sink_links;

    av_assert0(index >= 0);

    while (index) {
        int parent = (index - 1) >> 1;
        if (links[parent]->l.current_pts_us >= li->l.current_pts_us)
            break;
        links[index] = links[parent];
        links[index]->age_index = index;
        index = parent;
    }
    links[index] = li;
    li->age_index = index;
}

static void heap_bubble_down(FFBitStreamFilterGraph *graph,
                             BitStreamFilterLinkInternal *li, int index)
{
    BitStreamFilterLinkInternal **links = graph->sink_links;

    av_assert0(index >= 0);

    while (1) {
        int child = 2 * index + 1;
        if (child >= graph->sink_links_count)
            break;
        if (child + 1 < graph->sink_links_count &&
            links[child + 1]->l.current_pts_us < links[child]->l.current_pts_us)
            child++;
        if (li->l.current_pts_us < links[child]->l.current_pts_us)
            break;
        links[index] = links[child];
        links[index]->age_index = index;
        index = child;
    }
    links[index] = li;
    li->age_index = index;
}

void ff_bsf_graph_update_heap(AVBitStreamFilterGraph *graph, BitStreamFilterLinkInternal *li)
{
    FFBitStreamFilterGraph  *graphi = ffbsffiltergraph(graph);

    heap_bubble_up  (graphi, li, li->age_index);
    heap_bubble_down(graphi, li, li->age_index);
}

int ff_bsf_graph_run_once(AVBitStreamFilterGraph *graph)
{
    FFBitStreamFilterContext *ctxi;
    unsigned i;

    av_assert0(graph->nb_filters);
    ctxi = ffbsfctx(graph->filters[0]);
    for (i = 1; i < graph->nb_filters; i++) {
        FFBitStreamFilterContext *ctxi_other = ffbsfctx(graph->filters[i]);

        if (ctxi_other->ready > ctxi->ready)
            ctxi = ctxi_other;
    }

    if (!ctxi->ready)
        return AVERROR(EAGAIN);

    ctxi->ready = 0;

    return ff_bsf_activate(&ctxi->p);
}

int av_bsf_graph_source_needs_input(const AVBitStreamFilterGraph *graph)
{
    const FFBitStreamFilterGraph *graphi = cffbsffiltergraph(graph);
    int nb_requests, nb_requests_max = -1;
    int best_input = AVERROR(EOF);

    for (int i = 0; i < graphi->source_links_count; i++) {
        const BitStreamFilterLinkInternal *sourcei = graphi->source_links[i];
        const AVBitStreamFilterLink *source = &sourcei->l;

        if (av_bsf_source_get_status(source->src) == AVERROR(EOF))
            continue;

        nb_requests = ff_bsf_source_get_nb_failed_requests(source->src);
        if (nb_requests > nb_requests_max) {
            nb_requests_max = nb_requests;
            best_input = i;
        }
    }

    return best_input;
}
