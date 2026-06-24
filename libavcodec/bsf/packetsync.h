/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef AVCODEC_BSF_PACKETSYNC_H
#define AVCODEC_BSF_PACKETSYNC_H

#include <stdint.h>

#include "libavutil/log.h"

#include "libavcodec/bsf.h"
#include "libavcodec/packet.h"

enum EOFAction {
    EOF_ACTION_ENDALL,
    EOF_ACTION_PASS
};

/**
 * This API is intended as a helper for filters that have several video
 * input and need to combine them somehow. If the inputs have different or
 * variable frame rate, getting the input packets to match requires a rather
 * complex logic and a few user-tunable options.
 *
 * In this API, when a set of synchronized input packets is ready to be
 * processed is called a packet event. packet event can be generated in
 * response to input packets on any or all inputs and the handling of
 * situations where some stream extend beyond the beginning or the end of
 * others can be configured.
 *
 * The basic working of this API is the following: set the on_event
 * callback, then call ff_packetsync_activate() from the filter's activate
 * callback.
 */

/**
 * Stream extrapolation mode
 *
 * Describe how the packets of a stream are extrapolated before the first one
 * and after EOF to keep sync with possibly longer other streams.
 */
enum FFPacketSyncExtMode {

    /**
     * Completely stop all streams with this one.
     */
    EXT_STOP,

    /**
     * Ignore this stream and continue processing the other ones.
     */
    EXT_NULL,
};

/**
 * Timestamp synchronization mode
 *
 * Describe how the packets of a stream are synchronized based on timestamp
 * distance.
 */
enum FFPacketTSSyncMode {

    /**
     * Sync to packets from secondary input with the nearest, lower or equal
     * timestamp to the packet event one.
     */
    TS_DEFAULT,

    /**
     * Sync to packets from secondary input with the absolute nearest timestamp
     * to the packet event one.
     */
    TS_NEAREST,
};

/**
 * Input stream structure
 */
typedef struct FFPacketSyncIn {

    /**
     * Extrapolation mode for timestamps before the first packet
     */
    enum FFPacketSyncExtMode before;

    /**
     * Extrapolation mode for timestamps after the last packet
     */
    enum FFPacketSyncExtMode after;

    /**
     * Time base for the incoming packets
     */
    AVRational time_base;

    /**
     * Current packet, may be NULL before the first one or after EOF
     */
    AVPacket *pkt;

    /**
     * Next packet, for internal use
     */
    AVPacket *pkt_next;

    /**
     * PTS of the current packet
     */
    int64_t pts;

    /**
     * PTS of the next packet, for internal use
     */
    int64_t pts_next;

    /**
     * Boolean flagging the next packet, for internal use
     */
    uint8_t have_next;

    /**
     * State: before first, in stream or after EOF, for internal use
     */
    uint8_t state;

    /**
     * Synchronization level: packets on input at the highest sync level will
     * generate output packet events.
     *
     * For example, if inputs #0 and #1 have sync level 2 and input #2 has
     * sync level 1, then a packet on either input #0 or #1 will generate a
     * packet event, but not a packet on input #2 until both inputs #0 and #1
     * have reached EOF.
     *
     * If sync is 0, no packet event will be generated.
     */
    unsigned sync;

    enum FFPacketTSSyncMode ts_mode;
} FFPacketSyncIn;

/**
 * Packet sync structure.
 */
typedef struct FFPacketSync {
    const AVClass *class;

    /**
     * Parent filter context.
     */
    AVBitStreamFilterContext *parent;

    /**
     * Number of input streams
     */
    unsigned nb_in;

    /**
     * Time base for the output events
     */
    AVRational time_base;

    /**
     * Timestamp of the current event
     */
    int64_t pts;

    /**
     * Callback called when a packet event is ready
     */
    int (*on_event)(struct FFPacketSync *fs);

    /**
     * Opaque pointer, not used by the API
     */
    void *opaque;

    /**
     * Index of the input that requires a request
     */
    unsigned in_request;

    /**
     * Synchronization level: only inputs with the same sync level are sync
     * sources.
     */
    unsigned sync_level;

    /**
     * Flag indicating that a packet event is ready
     */
    uint8_t pkt_ready;

    /**
     * Flag indicating that output has reached EOF.
     */
    uint8_t eof;

    /**
     * Pointer to array of inputs.
     */
    FFPacketSyncIn *in;

    int opt_eof_action;
    int opt_ts_sync_mode;

} FFPacketSync;

/**
 * Pre-initialize a packet sync structure.
 *
 * It sets the class pointer and inits the options to their default values.
 * The entire structure is expected to be already set to 0.
 * This step is optional, but necessary to use the options.
 */
void ff_packetsync_preinit(FFPacketSync *fs);

/**
 * Initialize a packet sync structure.
 *
 * The entire structure is expected to be already set to 0 or preinited.
 *
 * @param  fs      packet sync structure to initialize
 * @param  parent  parent AVBitStreamFilterContext object
 * @param  nb_in   number of inputs
 * @return  >= 0 for success or a negative error code
 */
int ff_packetsync_init(FFPacketSync *fs, AVBitStreamFilterContext *parent, unsigned nb_in);

/**
 * Configure a packet sync structure.
 *
 * Must be called after all options are set but before all use.
 *
 * @return  >= 0 for success or a negative error code
 */
int ff_packetsync_configure(FFPacketSync *fs);

/**
 * Free all memory currently allocated.
 */
void ff_packetsync_uninit(FFPacketSync *fs);

/**
 * Get the current packet in an input.
 *
 * @param fs      packet sync structure
 * @param in      index of the input
 * @param rpacket  used to return the current packet (or NULL)
 * @param get     if not zero, the calling code needs to get ownership of
 *                the returned packet; the current packet will either be
 *                duplicated or removed from the packetsync structure
 */
int ff_packetsync_get_packet(FFPacketSync *fs, unsigned in, AVPacket **rpacket,
                             unsigned get);

/**
 * Examine the packets in the filter's input and try to produce output.
 *
 * This function can be the complete implementation of the activate
 * method of a filter using packetsync.
 */
int ff_packetsync_activate(FFPacketSync *fs);

/**
 * Initialize a packet sync structure for dualinput.
 *
 * Compared to generic packetsync, dualinput assumes the first input is the
 * main one and the filtering is performed on it. The first input will be
 * the only one with sync set and generic timeline support will just pass it
 * unchanged when disabled.
 *
 * Equivalent to ff_packetsync_init(fs, parent, 2) then setting the time
 * base, sync and ext modes on the inputs.
 */
int ff_packetsync_init_dualinput(FFPacketSync *fs, AVBitStreamFilterContext *parent);

/**
 * @param f0  used to return the main packet
 * @param f1  used to return the second packet, or NULL if disabled
 * @return  >=0 for success or AVERROR code
 * @note  The packet returned in f0 belongs to the caller (get = 1 in
 * ff_packetsync_get_packet()) while the packet returned in f1 is still owned
 * by the packetsync structure.
 */
int ff_packetsync_dualinput_get(FFPacketSync *fs, AVPacket **f0, AVPacket **f1);

/**
 * Same as ff_packetsync_dualinput_get(), but make sure that f0 is writable.
 */
int ff_packetsync_dualinput_get_writable(FFPacketSync *fs, AVPacket **f0, AVPacket **f1);

const AVClass *ff_packetsync_child_class_iterate(void **iter);
extern const AVClass ff_packetsync_class;

#define PACKETSYNC_DEFINE_PURE_CLASS(name, desc, func_prefix, options)      \
static const AVClass name##_class = {                                       \
    .class_name          = desc,                                            \
    .item_name           = av_default_item_name,                            \
    .option              = options,                                         \
    .version             = LIBAVUTIL_VERSION_INT,                           \
    .category            = AV_CLASS_CATEGORY_BITSTREAM_FILTER,              \
    .child_class_iterate = ff_packetsync_child_class_iterate,               \
    .child_next          = func_prefix##_child_next,                        \
}

/* A filter that uses the *_child_next-function from this macro
 * is required to initialize the FFPacketSync structure in AVBitStreamFilter.preinit
 * via the *_packetsync_preinit function defined alongside it. */
#define PACKETSYNC_AUXILIARY_FUNCS(func_prefix, context, field)             \
static int func_prefix##_packetsync_preinit(AVBitStreamFilterContext *ctx)  \
{                                                                           \
    context *s = ctx->priv_data; \
    ff_packetsync_preinit(&s->field); \
    return 0; \
} \
static void *func_prefix##_child_next(void *obj, void *prev)                \
{                                                                           \
    context *s = obj; \
    return prev ? NULL : &s->field; \
}

#define PACKETSYNC_DEFINE_CLASS_EXT(name, context, field, options)          \
PACKETSYNC_AUXILIARY_FUNCS(name, context, field)                            \
PACKETSYNC_DEFINE_PURE_CLASS(name, #name, name, options)

#define PACKETSYNC_DEFINE_CLASS(name, context, field)                       \
PACKETSYNC_DEFINE_CLASS_EXT(name, context, field, name##_options)

#endif /* AVCODEC_BSF_PACKETSYNC_H */
