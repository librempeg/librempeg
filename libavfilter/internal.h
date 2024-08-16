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

#ifndef AVFILTER_INTERNAL_H
#define AVFILTER_INTERNAL_H

/**
 * @file
 * internal API functions
 */

#include "libavutil/internal.h"
#include "avfilter.h"

/* Functions to parse audio format arguments */

/**
 * Parse a sample rate.
 *
 * @param ret unsigned integer pointer to where the value should be written
 * @param arg string to parse
 * @param log_ctx log context
 * @return >= 0 in case of success, a negative AVERROR code on error
 */
av_warn_unused_result
int ff_parse_sample_rate(int *ret, const char *arg, void *log_ctx);

/**
 * Parse a channel layout or a corresponding integer representation.
 *
 * @param ret 64bit integer pointer to where the value should be written.
 * @param nret integer pointer to the number of channels;
 *             if not NULL, then unknown channel layouts are accepted
 * @param arg string to parse
 * @param log_ctx log context
 * @return >= 0 in case of success, a negative AVERROR code on error
 */
av_warn_unused_result
int ff_parse_channel_layout(AVChannelLayout *ret, int *nret, const char *arg,
                            void *log_ctx);

/**
 * Negotiate the media format, dimensions, etc of all inputs to a filter.
 *
 * @param filter the filter to negotiate the properties for its inputs
 * @return       zero on successful negotiation
 */
int ff_filter_config_links(AVFilterContext *filter);

/* misc trace functions */

#define FF_TPRINTF_START(ctx, func) ff_tlog(NULL, "%-16s: ", #func)

#ifdef TRACE
void ff_tlog_link(void *ctx, AVFilterLink *link, int end);
#else
#define ff_tlog_link(ctx, link, end) do { } while(0)
#endif

/**
 * Append a new input/output pad to the filter's list of such pads.
 *
 * The *_free_name versions will set the AVFILTERPAD_FLAG_FREE_NAME flag
 * ensuring that the name will be freed generically (even on insertion error).
 */
int ff_append_inpad (AVFilterContext *f, AVFilterPad *p);
int ff_append_outpad(AVFilterContext *f, AVFilterPad *p);
int ff_append_inpad_free_name (AVFilterContext *f, AVFilterPad *p);
int ff_append_outpad_free_name(AVFilterContext *f, AVFilterPad *p);

/**
 * Request an input frame from the filter at the other end of the link.
 *
 * This function must not be used by filters using the activate callback,
 * use ff_link_set_frame_wanted() instead.
 *
 * The input filter may pass the request on to its inputs, fulfill the
 * request from an internal buffer or any other means specific to its function.
 *
 * When the end of a stream is reached AVERROR_EOF is returned and no further
 * frames are returned after that.
 *
 * When a filter is unable to output a frame for example due to its sources
 * being unable to do so or because it depends on external means pushing data
 * into it then AVERROR(EAGAIN) is returned.
 * It is important that a AVERROR(EAGAIN) return is returned all the way to the
 * caller (generally eventually a user application) as this step may (but does
 * not have to be) necessary to provide the input with the next frame.
 *
 * If a request is successful then some progress has been made towards
 * providing a frame on the link (through ff_filter_frame()). A filter that
 * needs several frames to produce one is allowed to return success if one
 * more frame has been processed but no output has been produced yet. A
 * filter is also allowed to simply forward a success return value.
 *
 * @param link the input link
 * @return     zero on success
 *             AVERROR_EOF on end of file
 *             AVERROR(EAGAIN) if the previous filter cannot output a frame
 *             currently and can neither guarantee that EOF has been reached.
 */
int ff_request_frame(AVFilterLink *link);

#define AVFILTER_DEFINE_CLASS_EXT(name, desc, options) \
    static const AVClass name##_class = {       \
        .class_name = desc,                     \
        .option     = options,                  \
        .version    = LIBAVUTIL_VERSION_INT,    \
        .category   = AV_CLASS_CATEGORY_FILTER, \
    }
#define AVFILTER_DEFINE_CLASS(fname) \
    AVFILTER_DEFINE_CLASS_EXT(fname, #fname, fname##_options)

/**
 * Find the index of a link.
 *
 * I.e. find i such that link == ctx->(in|out)puts[i]
 */
#define FF_INLINK_IDX(link)  ((int)((link)->dstpad - (link)->dst->input_pads))
#define FF_OUTLINK_IDX(link) ((int)((link)->srcpad - (link)->src->output_pads))

/**
 * Send a frame of data to the next filter.
 *
 * @param link   the output link over which the data is being sent
 * @param frame a reference to the buffer of data being sent. The
 *              receiving filter will free this reference when it no longer
 *              needs it or pass it on to the next filter.
 *
 * @return >= 0 on success, a negative AVERROR on error. The receiving filter
 * is responsible for unreferencing frame in case of error.
 */
int ff_filter_frame(AVFilterLink *link, AVFrame *frame);

/**
 * The filter is aware of hardware frames, and any hardware frame context
 * should not be automatically propagated through it.
 */
#define FF_FILTER_FLAG_HWFRAME_AWARE (1 << 0)

/**
 * Run one round of processing on a filter graph.
 */
int ff_filter_graph_run_once(AVFilterGraph *graph);

/**
 * Get number of threads for current filter instance.
 * This number is always same or less than graph->nb_threads.
 */
int ff_filter_get_nb_threads(AVFilterContext *ctx) av_pure;

/**
 * Generic processing of user supplied commands that are set
 * in the same way as the filter options.
 * NOTE: 'enable' option is handled separately, and not by
 * this function.
 */
int ff_filter_process_command(AVFilterContext *ctx, const char *cmd,
                              const char *arg, char *res, int res_len, int flags);

/**
 * Perform any additional setup required for hardware frames.
 *
 * link->hw_frames_ctx must be set before calling this function.
 * Inside link->hw_frames_ctx, the fields format, sw_format, width and
 * height must be set.  If dynamically allocated pools are not supported,
 * then initial_pool_size must also be set, to the minimum hardware frame
 * pool size necessary for the filter to work (taking into account any
 * frames which need to stored for use in operations as appropriate).  If
 * default_pool_size is nonzero, then it will be used as the pool size if
 * no other modification takes place (this can be used to preserve
 * compatibility).
 */
int ff_filter_init_hw_frames(AVFilterContext *avctx, AVFilterLink *link,
                             int default_pool_size);

#endif /* AVFILTER_INTERNAL_H */
