/*
 * This file is part of Librempeg.
 * Copyright (c) 2024-2025 Anton Khirnov
 *
 * Librempeg is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Librempeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with Librempeg.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "libavutil/avassert.h"
#include "libavutil/cpu.h"
#include "libavutil/fifo.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/thread.h"

#include "avfilter_internal.h"

#define MAX_AUTO_THREADS 4

enum ThreadState {
    THREAD_IDLE,
    THREAD_HAVE_INPUT,
};

struct WorkerThreadContext {
    FrameThreadingContext      *parent;
    FFFilterContext            *filter;

    pthread_mutex_t             lock;
    pthread_cond_t              cond;
    enum ThreadState            state;
    int                         die;

    pthread_t                   thread;
    int                         thread_started;

    AVFrame                    *frame_in;
    unsigned                    frame_in_idx;
    // these FIFOs hold output frames for each output link
    AVFifo                    **frames_out;
    int                         filter_err;
};

struct FrameThreadingContext {
    AVFilterContext            *parent;

    WorkerThreadContext        *threads;

    unsigned                    next_thread;
    unsigned                    startup;
};

static void thread_set_name(WorkerThreadContext *wt)
{
    const FrameThreadingContext *ft = wt->parent;
    const size_t idx = wt - ft->threads;
    char name[16];

    snprintf(name, sizeof(name), "av:%.7s:ff:%zu",
             ft->parent->filter->name, idx);

    ff_thread_setname(name);
}

static void *worker_thread(void *arg)
{
    WorkerThreadContext *wt = arg;
    AVFilterContext *ctx = &wt->filter->p;
    const FFFilter *filter = fffilter(ctx->filter);

    thread_set_name(wt);

    pthread_mutex_lock(&wt->lock);

    while (1) {
        int ret;

        while (wt->state == THREAD_IDLE && !wt->die)
            pthread_cond_wait(&wt->cond, &wt->lock);

        if (wt->die)
            break;


        if (filter->activate) {
            ret = filter->activate(ctx);
        } else {
            const AVFilterPad *in_pad = &ctx->input_pads[wt->frame_in_idx];

            av_assert0(in_pad->filter_frame);
            ret = in_pad->filter_frame(ctx->inputs[wt->frame_in_idx], wt->frame_in);
            wt->frame_in = NULL;
        }
        wt->filter_err = ret;

        wt->state = THREAD_IDLE;
        pthread_cond_signal(&wt->cond);
    }

    pthread_mutex_unlock(&wt->lock);

    return NULL;
}

int ff_filter_frame_thread_init(FFFilterContext *ctxi)
{
    AVFilterContext *ctx = &ctxi->p;
    FrameThreadingContext *ft;
    int ret, nb_threads;

    nb_threads = ff_filter_get_nb_threads(ctx);
    if (nb_threads <= 0) {
        nb_threads = av_cpu_count();
        nb_threads = FFMIN(nb_threads, MAX_AUTO_THREADS);
    }
    if (nb_threads <= 0) {
        ctx->thread_type = 0;
        ctx->nb_threads  = 0;
        return 0;
    }

    ctxi->ft = av_mallocz(sizeof(*ctxi->ft));
    if (!ctxi->ft)
        return AVERROR(ENOMEM);
    ft = ctxi->ft;

    ft->parent       = ctx;
    ctx->thread_type = AVFILTER_THREAD_FRAME_FILTER;

    ctx->nb_threads  = nb_threads;
    ft->threads      = av_calloc(ctx->nb_threads, sizeof(*ft->threads));
    if (!ft->threads)
        return AVERROR(ENOMEM);

    for (int i = 0; i < ctx->nb_threads; i++) {
        WorkerThreadContext *wt = &ft->threads[i];
        AVFilterContext *child;

        child = ff_filter_alloc(ctx->filter, ctx->name);
        if (!child)
            return AVERROR(ENOMEM);

        wt->parent     = ft;
        wt->filter     = fffilterctx(child);
        wt->filter->wt = wt;

        child->graph   = ctx->graph;
        wt->filter->is_frame_thread = 1;

        ret = av_opt_copy(child, ctx);
        if (ret < 0)
            return ret;

        if (ctx->filter->priv_class) {
            ret = av_opt_copy(child->priv, ctx->priv);
            if (ret < 0)
                return ret;
        }

        ret = avfilter_init_dict(child, NULL);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "Error initializing child thread %d\n", i);
            return ret;
        }

        wt->frames_out = av_calloc(ctx->nb_outputs, sizeof(*wt->frames_out));
        if (!wt->frames_out)
            return AVERROR(ENOMEM);

        for (int j = 0; j < ctx->nb_outputs; j++) {
            wt->frames_out[j] = av_fifo_alloc2(1, sizeof(AVFrame*),
                                               AV_FIFO_FLAG_AUTO_GROW);
            if (ret < 0)
                return ret;
        }

        // XXX: do this properly
        pthread_mutex_init(&wt->lock, NULL);
        pthread_cond_init(&wt->cond, NULL);

        ret = pthread_create(&wt->thread, NULL, worker_thread, wt);
        if (ret < 0)
            return AVERROR(ret);
        wt->thread_started = 1;
    }

    ft->startup = 1;

    return 0;
}

static void threads_stop(FFFilterContext *ctxi)
{
    AVFilterContext       *ctx = &ctxi->p;
    FrameThreadingContext *ft = ctxi->ft;

    for (int i = 0; i < ctx->nb_threads; i++) {
        WorkerThreadContext *wt = &ft->threads[i];

        if (!wt->thread_started)
            continue;

        pthread_mutex_lock(&wt->lock);

        wt->die = 1;
        pthread_cond_signal(&wt->cond);

        pthread_mutex_unlock(&wt->lock);

        pthread_join(wt->thread, NULL);
    }
}

void ff_filter_frame_thread_free(FFFilterContext *ctxi)
{
    AVFilterContext      *ctx = &ctxi->p;
    FrameThreadingContext *ft = ctxi->ft;

    if (!ft)
        return;

    threads_stop(ctxi);

    for (int i = 0; i < ctx->nb_threads && ft->threads; i++) {
        WorkerThreadContext *wt = &ft->threads[i];

        pthread_mutex_destroy(&wt->lock);
        pthread_cond_destroy(&wt->cond);

        if (wt->filter) {
            wt->filter->p.graph = NULL;
            avfilter_free(&wt->filter->p);
        }

        for (int j = 0; j < ctx->nb_outputs && wt->frames_out; j++) {
            AVFrame *f;

            while (av_fifo_read(wt->frames_out[j], &f, 1) >= 0)
                av_frame_free(&f);

            av_fifo_freep2(&wt->frames_out[j]);
        }
        av_freep(&wt->frames_out);

        av_frame_free(&wt->frame_in);
    }
    av_freep(&ft->threads);

    av_freep(&ctxi->ft);
}

void ff_filter_frame_thread_suspend(FFFilterContext *ctxi)
{
    FrameThreadingContext *ft = ctxi->ft;

    for (int i = 0; i < ctxi->p.nb_threads; i++) {
        WorkerThreadContext *wt = &ft->threads[i];

        if (!wt->thread_started)
            continue;

        pthread_mutex_lock(&wt->lock);

        while (wt->state != THREAD_IDLE)
            pthread_cond_wait(&wt->cond, &wt->lock);

        pthread_mutex_unlock(&wt->lock);
    }
}

int ff_filter_frame_thread_config_links(FFFilterContext *ctxi)
{
    AVFilterContext      *ctx = &ctxi->p;
    FrameThreadingContext *ft = ctxi->ft;
    int ret;

    for (int i = 0; i < ctx->nb_threads; i++) {
        WorkerThreadContext *wt = &ft->threads[i];
        AVFilterContext  *child = &wt->filter->p;

        for (unsigned j = 0; j < ctx->nb_inputs; j++) {
            AVFilterPad *pad = &child->input_pads[j];

            const AVFilterLink *l_src = ctx->inputs[j];
            AVFilterLink *l_dst;

            if (!child->inputs[j]) {
                FilterLinkInternal *li = ff_link_alloc(ctx->graph, pad->type);
                if (!li)
                    return AVERROR(ENOMEM);
                child->inputs[j] = &li->l.pub;
            }

            l_dst                      = child->inputs[j];
            l_dst->dst                 = child;
            l_dst->dstpad              = pad;
            l_dst->type                = l_src->type;
            l_dst->format              = l_src->format;
            l_dst->w                   = l_src->w;
            l_dst->h                   = l_src->h;
            l_dst->sample_aspect_ratio = l_src->sample_aspect_ratio;
            l_dst->colorspace          = l_src->colorspace;
            l_dst->color_range         = l_src->color_range;
            l_dst->sample_rate         = l_src->sample_rate;
            l_dst->time_base           = l_src->time_base;

            av_channel_layout_uninit(&l_dst->ch_layout);
            if (l_src->ch_layout.nb_channels) {
                ret = av_channel_layout_copy(&l_dst->ch_layout, &l_src->ch_layout);
                if (ret < 0)
                    return ret;
            }
        }

        for (unsigned j = 0; j < ctx->nb_outputs; j++) {
            AVFilterPad *pad = &child->output_pads[j];
            const AVFilterLink *l_src = ctx->outputs[j];
            AVFilterLink *l_dst;

            if (!child->outputs[j]) {
                FilterLinkInternal *li = ff_link_alloc(ctx->graph, pad->type);
                if (!li)
                    return AVERROR(ENOMEM);
                child->outputs[j]         = &li->l.pub;
            }

            l_dst                      = child->outputs[j];
            l_dst->src                 = child;
            l_dst->srcpad              = pad;
            l_dst->type                = l_src->type;
            l_dst->format              = l_src->format;
            l_dst->w                   = l_src->w;
            l_dst->h                   = l_src->h;
            l_dst->sample_aspect_ratio = l_src->sample_aspect_ratio;
            l_dst->colorspace          = l_src->colorspace;
            l_dst->color_range         = l_src->color_range;
            l_dst->sample_rate         = l_src->sample_rate;
            l_dst->time_base           = l_src->time_base;

            av_channel_layout_uninit(&l_dst->ch_layout);
            if (l_src->ch_layout.nb_channels) {
                ret = av_channel_layout_copy(&l_dst->ch_layout, &l_src->ch_layout);
                if (ret < 0)
                    return ret;
            }
        }

        for (unsigned j = 0; j < ctx->nb_inputs; j++) {
            AVFilterPad *pad = &child->input_pads[j];

            if (pad->config_props) {
                ret = pad->config_props(child->inputs[j]);
                if (ret < 0)
                    return ret;
            }
        }

        for (unsigned j = 0; j < ctx->nb_outputs; j++) {
            AVFilterPad *pad = &child->output_pads[j];

            if (pad->config_props) {
                ret = pad->config_props(child->outputs[j]);
                if (ret < 0)
                    return ret;
            }
        }
    }

    return 0;
}

static int thread_process_results(FFFilterContext *ctxi, WorkerThreadContext *wt)
{
    AVFilterContext *ctx = &ctxi->p;
    int ret = 0;

    for (unsigned i = 0; i < ctx->nb_outputs; i++) {
        AVFrame *f;

        while (av_fifo_read(wt->frames_out[i], &f, 1) >= 0) {
            ret = ff_filter_frame(ctx->outputs[i], f);
            if (ret < 0)
                return ret;
        }
    }

    ret = wt->filter_err;
    wt->filter_err = 0;

    return ret;
}

static int inlink_update_props(AVFilterLink *dst, const AVFilterLink *src)
{
    int ret;

    dst->time_base = src->time_base;
    dst->format    = src->format;

    switch (src->type) {
    case AVMEDIA_TYPE_VIDEO:
        dst->w                   = src->w;
        dst->h                   = src->h;
        dst->sample_aspect_ratio = src->sample_aspect_ratio;
        dst->colorspace          = src->colorspace;
        dst->color_range         = src->color_range;
        break;
    case AVMEDIA_TYPE_AUDIO:
        dst->sample_rate         = src->sample_rate;

        if (av_channel_layout_compare(&dst->ch_layout,
                                      &src->ch_layout)) {
            av_channel_layout_uninit(&dst->ch_layout);
            ret = av_channel_layout_copy(&dst->ch_layout,
                                         &src->ch_layout);
            if (ret < 0)
                return ret;
        }

        break;
    default: av_assert0(0);
    }

    return 0;
}

int ff_filter_frame_thread_submit(AVFilterLink *inlink, AVFrame *frame)
{
    FFFilterContext     *ctxi = fffilterctx(inlink->dst);
    AVFilterContext      *ctx = &ctxi->p;
    const FFFilter     *filter = fffilter(ctx->filter);
    FrameThreadingContext *ft = ctxi->ft;
    WorkerThreadContext   *wt = &ft->threads[ft->next_thread];
    const unsigned in_idx = FF_INLINK_IDX(inlink);
    int ret = 0;

    pthread_mutex_lock(&wt->lock);

    while (wt->state != THREAD_IDLE)
        pthread_cond_wait(&wt->cond, &wt->lock);

    // receive filtered results
    if (!ft->startup) {
        ret = thread_process_results(ctxi, wt);
        if (ret < 0)
            goto finish;
    }

    ret = inlink_update_props(wt->filter->p.inputs[in_idx], inlink);
    if (ret < 0)
        goto finish;

    // transfer state
    if (filter->transfer_state) {
        ret = filter->transfer_state(&wt->filter->p, ctx);
        if (ret < 0)
            return ret;
    }
    wt->filter->is_disabled = ctxi->is_disabled;

    av_assert0(!wt->frame_in);
    wt->frame_in     = frame;
    wt->frame_in_idx = in_idx;
    frame            = NULL;
    wt->state        = THREAD_HAVE_INPUT;

    pthread_cond_signal(&wt->cond);

    ft->next_thread = (ft->next_thread + 1) % ctx->nb_threads;
    if (ft->next_thread == 0)
        ft->startup = 0;

finish:
    pthread_mutex_unlock(&wt->lock);
    av_frame_free(&frame);

    return ret;
}

int ff_filter_frame_thread_frame_out(AVFilterLink *outlink, AVFrame *frame)
{
    FFFilterContext   *ctxi = fffilterctx(outlink->src);
    WorkerThreadContext *wt = ctxi->wt;
    const int           idx = FF_OUTLINK_IDX(outlink);
    int ret;

    ret = av_fifo_write(wt->frames_out[idx], &frame, 1);
    if (ret < 0) {
        av_frame_free(&frame);
        return ret;
    }

    return 0;
}

int ff_filter_frame_thread_activate(AVFilterContext *ctx)
{
    const FFFilter    *filter = fffilter(ctx->filter);
    FFFilterContext     *ctxi = fffilterctx(ctx);
    FrameThreadingContext *ft = ctxi->ft;
    WorkerThreadContext   *wt = &ft->threads[ft->next_thread];
    int ret = 0;

    pthread_mutex_lock(&wt->lock);

    while (wt->state != THREAD_IDLE)
        pthread_cond_wait(&wt->cond, &wt->lock);

    // receive filtered results
    if (!ft->startup) {
        ret = thread_process_results(ctxi, wt);
        if (ret < 0)
            goto finish;
    }

    // update input link properties
    for (int i = 0; i < ctx->nb_inputs; i++) {
        ret = inlink_update_props(wt->filter->p.inputs[i],
                                  ctx->inputs[i]);
        if (ret < 0)
            goto finish;
    }

    // transfer state
    if (filter->transfer_state) {
        ret = filter->transfer_state(&wt->filter->p, ctx);
        if (ret < 0)
            return ret;
    }
    wt->filter->is_disabled = ctxi->is_disabled;

    if (ft->startup)
        ff_filter_set_ready(ctx, 100);

    ft->next_thread = (ft->next_thread + 1) % ctx->nb_threads;
    if (ft->next_thread == 0)
        ft->startup = 0;

    wt->state = THREAD_HAVE_INPUT;
    pthread_cond_signal(&wt->cond);

finish:
    pthread_mutex_unlock(&wt->lock);

    return ret;
}

int ff_filter_frame_thread_get_buffer(AVFilterContext *ctx, AVFrame *frame,
                                      int out_idx, int align, unsigned flags)
{
    WorkerThreadContext   *wt = fffilterctx(ctx)->wt;
    FrameThreadingContext *ft = wt->parent;
    FFFilterGraph     *graphi = fffiltergraph(ctx->graph);
    int ret;

    ff_mutex_lock(&graphi->get_buffer_lock);

    ret = ff_filter_get_buffer_ext(ft->parent, frame, out_idx, align, flags);

    ff_mutex_unlock(&graphi->get_buffer_lock);

    return ret;
}

int ff_filter_frame_thread_flush(AVFilterContext *ctx)
{
    FFFilterContext     *ctxi = fffilterctx(ctx);
    FrameThreadingContext *ft = ctxi->ft;
    unsigned threads_flushed = 0;
    unsigned threads_to_flush = ft->startup ? ft->next_thread : ctx->nb_threads;
    unsigned thread_idx       = ft->startup ? 0               : ft->next_thread;
    int ret;

    while (threads_flushed < threads_to_flush) {
        WorkerThreadContext *wt = &ft->threads[thread_idx];

        pthread_mutex_lock(&wt->lock);

        while (wt->state != THREAD_IDLE)
            pthread_cond_wait(&wt->cond, &wt->lock);

        pthread_mutex_unlock(&wt->lock);

        ret = thread_process_results(ctxi, wt);
        if (ret < 0)
            return ret;

        threads_flushed++;
        thread_idx = (thread_idx + 1) % ctx->nb_threads;
    }

    return 0;
}
