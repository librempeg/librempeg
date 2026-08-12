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

/**
 * @file
 * Packet sync API. Heavily based on libavfilter/framesync.c by Nicolas George
 */

#include "libavutil/avassert.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "libavcodec/bsf.h"
#include "filters.h"
#include "packetsync.h"

#define OFFSET(member) offsetof(FFPacketSync, member)
#define FLAGS (AV_OPT_FLAG_BSF_PARAM|AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_VIDEO_PARAM)

static const char *packetsync_name(void *ptr)
{
    return "packetsync";
}

static const AVOption packetsync_options[] = {
    { "eof_action", "Action to take when encountering EOF from secondary input ",
        OFFSET(opt_eof_action), AV_OPT_TYPE_INT, { .i64 = EOF_ACTION_PASS },
        EOF_ACTION_ENDALL, EOF_ACTION_PASS, .flags = FLAGS, .unit = "eof_action" },
        { "endall", "End both streams.",            0, AV_OPT_TYPE_CONST, { .i64 = EOF_ACTION_ENDALL }, .flags = FLAGS, .unit = "eof_action" },
        { "pass",   "Pass through the main input.", 0, AV_OPT_TYPE_CONST, { .i64 = EOF_ACTION_PASS },   .flags = FLAGS, .unit = "eof_action" },
    { "ts_sync_mode", "How strictly to sync streams based on secondary input timestamps",
        OFFSET(opt_ts_sync_mode), AV_OPT_TYPE_INT, { .i64 = TS_DEFAULT },
        TS_DEFAULT, TS_NEAREST, .flags = FLAGS, .unit = "ts_sync_mode" },
        { "default", "Packet from secondary input with the nearest lower or equal timestamp to the primary input packet",
            0, AV_OPT_TYPE_CONST, { .i64 = TS_DEFAULT }, .flags = FLAGS, .unit = "ts_sync_mode" },
        { "nearest", "Packet from secondary input with the absolute nearest timestamp to the primary input packet",
            0, AV_OPT_TYPE_CONST, { .i64 = TS_NEAREST }, .flags = FLAGS, .unit = "ts_sync_mode" },
    { NULL }
};
const AVClass ff_packetsync_class = {
    .version                   = LIBAVUTIL_VERSION_INT,
    .class_name                = "packetsync",
    .item_name                 = packetsync_name,
    .category                  = AV_CLASS_CATEGORY_BITSTREAM_FILTER,
    .option                    = packetsync_options,
    .parent_log_context_offset = OFFSET(parent),
};

const AVClass *ff_packetsync_child_class_iterate(void **iter)
{
    const AVClass *c = *iter ? NULL : &ff_packetsync_class;
    *iter = (void *)(uintptr_t)c;
    return c;
}

enum {
    STATE_BOF,
    STATE_RUN,
    STATE_EOF,
};

static int consume_from_fifos(FFPacketSync *fs);

void ff_packetsync_preinit(FFPacketSync *fs)
{
    if (fs->class)
        return;
    fs->class  = &ff_packetsync_class;
    av_opt_set_defaults(fs);
}

int ff_packetsync_init(FFPacketSync *fs, AVBitStreamFilterContext *parent, unsigned nb_in)
{
    /* For filters with several outputs, we will not be able to assume which
       output is relevant for ff_outlink_packet_wanted() and
       ff_bsf_link_set_in_status(). To be designed when needed. */
    av_assert0(parent->nb_outputs == 1);

    ff_packetsync_preinit(fs);
    fs->parent = parent;
    fs->nb_in  = nb_in;

    fs->in = av_calloc(nb_in, sizeof(*fs->in));
    if (!fs->in) {
        fs->nb_in = 0;
        return AVERROR(ENOMEM);
    }

    return 0;
}

static void packetsync_eof(FFPacketSync *fs, int64_t pts)
{
    fs->eof = 1;
    fs->pkt_ready = 0;
    ff_bsf_link_set_in_status(fs->parent->outputs[0], AVERROR_EOF, pts);
}

static void packetsync_sync_level_update(FFPacketSync *fs, int64_t eof_pts)
{
    unsigned i, level = 0;

    for (i = 0; i < fs->nb_in; i++)
        if (fs->in[i].state != STATE_EOF)
            level = FFMAX(level, fs->in[i].sync);
    av_assert0(level <= fs->sync_level);
    if (level < fs->sync_level)
        av_log(fs, AV_LOG_VERBOSE, "Sync level %u\n", level);
    if (fs->opt_ts_sync_mode > TS_DEFAULT) {
        for (i = 0; i < fs->nb_in; i++) {
            if (fs->in[i].sync < level)
                fs->in[i].ts_mode = fs->opt_ts_sync_mode;
            else
                fs->in[i].ts_mode = TS_DEFAULT;
        }
    }
    if (level)
        fs->sync_level = level;
    else
        packetsync_eof(fs, eof_pts);
}

int ff_packetsync_configure(FFPacketSync *fs)
{
    unsigned i;

    for (i = 1; i < fs->nb_in; i++) {
        fs->in[i].after = EXT_NULL;
        fs->in[i].sync  = 0;
    }
    if (fs->opt_eof_action == EOF_ACTION_ENDALL) {
        for (i = 0; i < fs->nb_in; i++)
            fs->in[i].after = EXT_STOP;
    }

    if (!fs->time_base.num) {
        for (i = 0; i < fs->nb_in; i++) {
            if (fs->in[i].sync) {
                if (fs->time_base.num) {
                    fs->time_base = av_gcd_q(fs->time_base, fs->in[i].time_base,
                                             AV_TIME_BASE / 2, AV_TIME_BASE_Q);
                } else {
                    fs->time_base = fs->in[i].time_base;
                }
            }
        }
        if (!fs->time_base.num) {
            av_log(fs, AV_LOG_ERROR, "Impossible to set time base\n");
            return AVERROR(EINVAL);
        }
        av_log(fs, AV_LOG_VERBOSE, "Selected %d/%d time base\n",
               fs->time_base.num, fs->time_base.den);
    }

    for (i = 0; i < fs->nb_in; i++)
        fs->in[i].pts = fs->in[i].pts_next = AV_NOPTS_VALUE;
    fs->sync_level = UINT_MAX;
    packetsync_sync_level_update(fs, AV_NOPTS_VALUE);

    return 0;
}

static int packetsync_advance(FFPacketSync *fs)
{
    unsigned i;
    int64_t pts;
    int ret;

    while (!(fs->pkt_ready || fs->eof)) {
        ret = consume_from_fifos(fs);
        if (ret <= 0)
            return ret;

        pts = INT64_MAX;
        for (i = 0; i < fs->nb_in; i++)
            if (fs->in[i].have_next && fs->in[i].pts_next < pts)
                pts = fs->in[i].pts_next;
        if (pts == INT64_MAX) {
            packetsync_eof(fs, AV_NOPTS_VALUE);
            break;
        }
        for (i = 0; i < fs->nb_in; i++) {
            if (fs->in[i].pts_next == pts ||
                (fs->in[i].ts_mode == TS_NEAREST &&
                 fs->in[i].have_next &&
                 fs->in[i].pts_next != INT64_MAX && fs->in[i].pts != AV_NOPTS_VALUE &&
                 fs->in[i].pts_next - pts < pts - fs->in[i].pts)) {
                av_packet_free(&fs->in[i].pkt);
                fs->in[i].pkt      = fs->in[i].pkt_next;
                fs->in[i].pts        = fs->in[i].pts_next;
                fs->in[i].pkt_next = NULL;
                fs->in[i].pts_next   = AV_NOPTS_VALUE;
                fs->in[i].have_next  = 0;
                fs->in[i].state      = fs->in[i].pkt ? STATE_RUN : STATE_EOF;
                if (fs->in[i].sync == fs->sync_level && fs->in[i].pkt)
                    fs->pkt_ready = 1;
                if (fs->in[i].state == STATE_EOF &&
                    fs->in[i].after == EXT_STOP)
                    packetsync_eof(fs, AV_NOPTS_VALUE);
            }
        }
        if (fs->pkt_ready)
            for (i = 0; i < fs->nb_in; i++)
                if ((fs->in[i].state == STATE_BOF &&
                     fs->in[i].before == EXT_STOP))
                    fs->pkt_ready = 0;
        fs->pts = pts;
    }
    return 0;
}

static int64_t packetsync_pts_extrapolate(FFPacketSync *fs, unsigned in,
                                         int64_t pts)
{
    return pts + 1;
}

static void packetsync_inject_packet(FFPacketSync *fs, unsigned in, AVPacket *pkt)
{
    int64_t pts;

    av_assert0(!fs->in[in].have_next);
    av_assert0(pkt);
    pts = av_rescale_q_rnd(pkt->pts, fs->in[in].time_base, fs->time_base, AV_ROUND_NEAR_INF | AV_ROUND_PASS_MINMAX);
    pkt->pts = pts;
    fs->in[in].pkt_next = pkt;
    fs->in[in].pts_next   = pts;
    fs->in[in].have_next  = 1;
}

static void packetsync_inject_status(FFPacketSync *fs, unsigned in, int status, int64_t eof_pts)
{
    av_assert0(!fs->in[in].have_next);
    fs->in[in].sync = 0;
    packetsync_sync_level_update(fs, status == AVERROR_EOF ? eof_pts : AV_NOPTS_VALUE);
    fs->in[in].pkt_next = NULL;
    fs->in[in].pts_next   = fs->in[in].state != STATE_RUN
                            ? INT64_MAX : packetsync_pts_extrapolate(fs, in, fs->in[in].pts);
    fs->in[in].have_next  = 1;
}

int ff_packetsync_get_packet(FFPacketSync *fs, unsigned in, AVPacket **rpkt,
                            unsigned get)
{
    AVPacket *pkt;
    unsigned need_copy = 0, i;
    int64_t pts_next;

    if (!fs->in[in].pkt) {
        *rpkt = NULL;
        return 0;
    }
    pkt = fs->in[in].pkt;
    if (get) {
        /* Find out if we need to copy the packet: is there another sync
           stream, and do we know if its current packet will outlast this one? */
        pts_next = fs->in[in].have_next ? fs->in[in].pts_next : INT64_MAX;
        for (i = 0; i < fs->nb_in && !need_copy; i++)
            if (i != in && fs->in[i].sync &&
                (!fs->in[i].have_next || fs->in[i].pts_next < pts_next))
                need_copy = 1;
        if (need_copy) {
            if (!(pkt = av_packet_clone(pkt)))
                return AVERROR(ENOMEM);
        } else {
            fs->in[in].pkt = NULL;
        }
        fs->pkt_ready = 0;
    }
    *rpkt = pkt;
    return 0;
}

void ff_packetsync_uninit(FFPacketSync *fs)
{
    unsigned i;

    for (i = 0; i < fs->nb_in; i++) {
        av_packet_free(&fs->in[i].pkt);
        av_packet_free(&fs->in[i].pkt_next);
    }

    av_freep(&fs->in);
}

static int consume_from_fifos(FFPacketSync *fs)
{
    AVBitStreamFilterContext *ctx = fs->parent;
    AVPacket *pkt = NULL;
    int64_t pts;
    unsigned i, nb_active, nb_miss;
    int ret, status;

    nb_active = nb_miss = 0;
    for (i = 0; i < fs->nb_in; i++) {
        if (fs->in[i].have_next || fs->in[i].state == STATE_EOF)
            continue;
        nb_active++;
        ret = ff_bsf_inlink_consume_packet(ctx->inputs[i], &pkt);
        if (ret < 0)
            return ret;
        if (ret) {
            av_assert0(pkt);
            packetsync_inject_packet(fs, i, pkt);
        } else {
            ret = ff_bsf_inlink_acknowledge_status(ctx->inputs[i], &status, &pts);
            if (ret > 0) {
                packetsync_inject_status(fs, i, status, pts);
            } else if (!ret) {
                nb_miss++;
            }
        }
    }
    if (nb_miss) {
        if (nb_miss == nb_active && !ff_bsf_outlink_packet_wanted(ctx->outputs[0]))
            return FFERROR_BSF_NOT_READY;
        for (i = 0; i < fs->nb_in; i++)
            if (!fs->in[i].have_next && fs->in[i].state != STATE_EOF)
                ff_bsf_inlink_request_packet(ctx->inputs[i]);
        return 0;
    }
    return 1;
}

int ff_packetsync_activate(FFPacketSync *fs)
{
    AVBitStreamFilterContext *ctx = fs->parent;
    int ret;

    ret = ff_bsf_outlink_get_status(ctx->outputs[0]);
    if (ret) {
        unsigned i;
        for (i = 0; i < ctx->nb_inputs; i++)
            ff_bsf_inlink_set_status(ctx->inputs[i], ret);
        return 0;
    }

    ret = packetsync_advance(fs);
    if (ret < 0)
        return ret;
    if (fs->eof || !fs->pkt_ready)
        return 0;
    ret = fs->on_event(fs);
    if (ret < 0)
        return ret;
    fs->pkt_ready = 0;

    return 0;
}

int ff_packetsync_init_dualinput(FFPacketSync *fs, AVBitStreamFilterContext *parent)
{
    int ret;

    ret = ff_packetsync_init(fs, parent, 2);
    if (ret < 0)
        return ret;
    fs->in[0].time_base = parent->inputs[0]->time_base;
    fs->in[1].time_base = parent->inputs[1]->time_base;
    fs->in[0].sync   = 2;
    fs->in[0].before = EXT_STOP;
    fs->in[0].after  = EXT_STOP;
    fs->in[1].sync   = 1;
    fs->in[1].before = EXT_NULL;
    fs->in[1].after  = EXT_NULL;
    return 0;
}

int ff_packetsync_dualinput_get(FFPacketSync *fs, AVPacket **f0, AVPacket **f1)
{
    AVBitStreamFilterContext *ctx = fs->parent;
    AVPacket *mainpic = NULL, *secondpic = NULL;
    int ret;

    if ((ret = ff_packetsync_get_packet(fs, 0, &mainpic,   1)) < 0 ||
        (ret = ff_packetsync_get_packet(fs, 1, &secondpic, 0)) < 0) {
        av_packet_free(&mainpic);
        return ret;
    }
    av_assert0(mainpic);
    mainpic->pts = av_rescale_q(fs->pts, fs->time_base, ctx->outputs[0]->time_base);
    *f0 = mainpic;
    *f1 = secondpic;
    return 0;
}

int ff_packetsync_dualinput_get_writable(FFPacketSync *fs, AVPacket **f0, AVPacket **f1)
{
    int ret;

    ret = ff_packetsync_dualinput_get(fs, f0, f1);
    if (ret < 0)
        return ret;
    ret = av_packet_make_writable(*f0);
    if (ret < 0) {
        av_packet_free(f0);
        *f1 = NULL;
        return ret;
    }
    return 0;
}
