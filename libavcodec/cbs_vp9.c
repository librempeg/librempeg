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

#include "libavutil/avassert.h"

#include "cbs.h"
#include "cbs_internal.h"
#include "cbs_vp9.h"


static int cbs_vp9_read_s(CodedBitstreamContext *ctx, GetBitContext *gbc,
                          int width, const char *name,
                          const int *subscripts, int32_t *write_to)
{
    uint32_t magnitude;
    int sign;
    int32_t value;

    CBS_TRACE_READ_START();

    if (get_bits_left(gbc) < width + 1) {
        av_log(ctx->log_ctx, AV_LOG_ERROR, "Invalid signed value at "
               "%s: bitstream ended.\n", name);
        return AVERROR_INVALIDDATA;
    }

    magnitude = get_bits(gbc, width);
    sign      = get_bits1(gbc);
    value     = sign ? -(int32_t)magnitude : magnitude;

    CBS_TRACE_READ_END();

    *write_to = value;
    return 0;
}

static int cbs_vp9_write_s(CodedBitstreamContext *ctx, PutBitContext *pbc,
                           int width, const char *name,
                           const int *subscripts, int32_t value)
{
    uint32_t magnitude;
    int sign;

    CBS_TRACE_WRITE_START();

    if (put_bits_left(pbc) < width + 1)
        return AVERROR(ENOSPC);

    sign      = value < 0;
    magnitude = sign ? -value : value;

    put_bits(pbc, width, magnitude);
    put_bits(pbc, 1, sign);

    CBS_TRACE_WRITE_END();

    return 0;
}

static int cbs_vp9_read_increment(CodedBitstreamContext *ctx, GetBitContext *gbc,
                                  uint32_t range_min, uint32_t range_max,
                                  const char *name, uint32_t *write_to)
{
    uint32_t value;

    CBS_TRACE_READ_START();

    av_assert0(range_min <= range_max && range_max - range_min < 32);

    for (value = range_min; value < range_max;) {
        if (get_bits_left(gbc) < 1) {
            av_log(ctx->log_ctx, AV_LOG_ERROR, "Invalid increment value at "
                   "%s: bitstream ended.\n", name);
            return AVERROR_INVALIDDATA;
        }
        if (get_bits1(gbc))
            ++value;
        else
            break;
    }

    CBS_TRACE_READ_END_NO_SUBSCRIPTS();

    *write_to = value;
    return 0;
}

static int cbs_vp9_write_increment(CodedBitstreamContext *ctx, PutBitContext *pbc,
                                   uint32_t range_min, uint32_t range_max,
                                   const char *name, uint32_t value)
{
    int len;

    CBS_TRACE_WRITE_START();

    av_assert0(range_min <= range_max && range_max - range_min < 8);
    if (value < range_min || value > range_max) {
        av_log(ctx->log_ctx, AV_LOG_ERROR, "%s out of range: "
               "%"PRIu32", but must be in [%"PRIu32",%"PRIu32"].\n",
               name, value, range_min, range_max);
        return AVERROR_INVALIDDATA;
    }

    if (value == range_max)
        len = range_max - range_min;
    else
        len = value - range_min + 1;
    if (put_bits_left(pbc) < len)
        return AVERROR(ENOSPC);

    if (len > 0)
        put_bits(pbc, len, (1 << len) - 1 - (value != range_max));

    CBS_TRACE_WRITE_END_NO_SUBSCRIPTS();

    return 0;
}

static int cbs_vp9_read_le(CodedBitstreamContext *ctx, GetBitContext *gbc,
                           int width, const char *name,
                           const int *subscripts, uint32_t *write_to)
{
    uint32_t value;
    int b;

    CBS_TRACE_READ_START();

    av_assert0(width % 8 == 0);

    if (get_bits_left(gbc) < width) {
        av_log(ctx->log_ctx, AV_LOG_ERROR, "Invalid le value at "
               "%s: bitstream ended.\n", name);
        return AVERROR_INVALIDDATA;
    }

    value = 0;
    for (b = 0; b < width; b += 8)
        value |= get_bits(gbc, 8) << b;

    CBS_TRACE_READ_END();

    *write_to = value;
    return 0;
}

static int cbs_vp9_write_le(CodedBitstreamContext *ctx, PutBitContext *pbc,
                            int width, const char *name,
                            const int *subscripts, uint32_t value)
{
    int b;

    CBS_TRACE_WRITE_START();

    av_assert0(width % 8 == 0);

    if (put_bits_left(pbc) < width)
        return AVERROR(ENOSPC);

    for (b = 0; b < width; b += 8)
        put_bits(pbc, 8, value >> b & 0xff);

    CBS_TRACE_WRITE_END();

    return 0;
}

#define HEADER(name) do { \
        ff_cbs_trace_header(ctx, name); \
    } while (0)

#define CHECK(call) do { \
        err = (call); \
        if (err < 0) \
            return err; \
    } while (0)

#define FUNC_NAME(rw, codec, name) cbs_ ## codec ## _ ## rw ## _ ## name
#define FUNC_VP9(rw, name) FUNC_NAME(rw, vp9, name)
#define FUNC(name) FUNC_VP9(READWRITE, name)

#define SUBSCRIPTS(subs, ...) (subs > 0 ? ((int[subs + 1]){ subs, __VA_ARGS__ }) : NULL)

#define s(width, name) \
        xs(width, name, current->name, 0, )
#define fs(width, name, subs, ...) \
        xf(width, name, current->name, subs, __VA_ARGS__)
#define ss(width, name, subs, ...) \
        xs(width, name, current->name, subs, __VA_ARGS__)

#define READ
#define READWRITE read
#define RWContext GetBitContext

#define f(width, name) do { \
        uint32_t value; \
        CHECK(ff_cbs_read_simple_unsigned(ctx, rw, width, #name, \
                                          &value)); \
        current->name = value; \
    } while (0)
#define xf(width, name, var, subs, ...) do { \
        uint32_t value; \
        CHECK(ff_cbs_read_unsigned(ctx, rw, width, #name, \
                                   SUBSCRIPTS(subs, __VA_ARGS__), \
                                   &value, 0, (1 << width) - 1)); \
        var = value; \
    } while (0)
#define xs(width, name, var, subs, ...) do { \
        int32_t value; \
        CHECK(cbs_vp9_read_s(ctx, rw, width, #name, \
                             SUBSCRIPTS(subs, __VA_ARGS__), &value)); \
        var = value; \
    } while (0)


#define increment(name, min, max) do { \
        uint32_t value; \
        CHECK(cbs_vp9_read_increment(ctx, rw, min, max, #name, &value)); \
        current->name = value; \
    } while (0)

#define fle(width, name, subs, ...) do { \
        CHECK(cbs_vp9_read_le(ctx, rw, width, #name, \
                              SUBSCRIPTS(subs, __VA_ARGS__), &current->name)); \
    } while (0)

#define delta_q(name) do { \
        uint8_t delta_coded; \
        int8_t delta_q; \
        xf(1, name.delta_coded, delta_coded, 0, ); \
        if (delta_coded) \
            xs(4, name.delta_q, delta_q, 0, ); \
        else \
            delta_q = 0; \
        current->name = delta_q; \
    } while (0)

#define prob(name, subs, ...) do { \
        uint8_t prob_coded; \
        uint8_t prob; \
        xf(1, name.prob_coded, prob_coded, subs, __VA_ARGS__); \
        if (prob_coded) \
            xf(8, name.prob, prob, subs, __VA_ARGS__); \
        else \
            prob = 255; \
        current->name = prob; \
    } while (0)

#define fixed(width, name, value) do { \
        av_unused uint32_t fixed_value; \
        CHECK(ff_cbs_read_unsigned(ctx, rw, width, #name, \
                                   0, &fixed_value, value, value)); \
    } while (0)

#define infer(name, value) do { \
        current->name = value; \
    } while (0)

#define byte_alignment(rw) (get_bits_count(rw) % 8)

#include "cbs_vp9_syntax_template.c"

#undef READ
#undef READWRITE
#undef RWContext
#undef f
#undef xf
#undef xs
#undef increment
#undef fle
#undef delta_q
#undef prob
#undef fixed
#undef infer
#undef byte_alignment


#define WRITE
#define READWRITE write
#define RWContext PutBitContext

#define f(width, name) do { \
        CHECK(ff_cbs_write_simple_unsigned(ctx, rw, width, #name, \
                                           current->name)); \
    } while (0)
#define xf(width, name, var, subs, ...) do { \
        CHECK(ff_cbs_write_unsigned(ctx, rw, width, #name, \
                                    SUBSCRIPTS(subs, __VA_ARGS__), \
                                    var, 0, (1 << width) - 1)); \
    } while (0)
#define xs(width, name, var, subs, ...) do { \
        CHECK(cbs_vp9_write_s(ctx, rw, width, #name, \
                              SUBSCRIPTS(subs, __VA_ARGS__), var)); \
    } while (0)

#define increment(name, min, max) do { \
        CHECK(cbs_vp9_write_increment(ctx, rw, min, max, #name, current->name)); \
    } while (0)

#define fle(width, name, subs, ...) do { \
        CHECK(cbs_vp9_write_le(ctx, rw, width, #name, \
                               SUBSCRIPTS(subs, __VA_ARGS__), current->name)); \
    } while (0)

#define delta_q(name) do { \
        xf(1, name.delta_coded, !!current->name, 0, ); \
        if (current->name) \
            xs(4, name.delta_q, current->name, 0, ); \
    } while (0)

#define prob(name, subs, ...) do { \
        xf(1, name.prob_coded, current->name != 255, subs, __VA_ARGS__); \
        if (current->name != 255) \
            xf(8, name.prob, current->name, subs, __VA_ARGS__); \
    } while (0)

#define fixed(width, name, value) do { \
        CHECK(ff_cbs_write_unsigned(ctx, rw, width, #name, \
                                    0, value, value, value)); \
    } while (0)

#define infer(name, value) do { \
        if (current->name != (value)) { \
            av_log(ctx->log_ctx, AV_LOG_WARNING, "Warning: " \
                   "%s does not match inferred value: " \
                   "%"PRId64", but should be %"PRId64".\n", \
                   #name, (int64_t)current->name, (int64_t)(value)); \
        } \
    } while (0)

#define byte_alignment(rw) (put_bits_count(rw) % 8)

#include "cbs_vp9_syntax_template.c"

#undef WRITE
#undef READWRITE
#undef RWContext
#undef f
#undef xf
#undef xs
#undef increment
#undef fle
#undef delta_q
#undef prob
#undef fixed
#undef infer
#undef byte_alignment


static int cbs_vp9_split_fragment(CodedBitstreamContext *ctx,
                                  CodedBitstreamFragment *frag,
                                  int header)
{
    uint8_t superframe_header;
    int err;

    if (frag->data_size == 0)
        return AVERROR_INVALIDDATA;

    // Last byte in the packet.
    superframe_header = frag->data[frag->data_size - 1];

    if ((superframe_header & 0xe0) == 0xc0) {
        VP9RawSuperframeIndex sfi = {0};
        GetBitContext gbc;
        size_t index_size, pos;
        int i;

        index_size = 2 + (((superframe_header & 0x18) >> 3) + 1) *
                          ((superframe_header & 0x07) + 1);

        if (index_size > frag->data_size)
            return AVERROR_INVALIDDATA;

        err = init_get_bits(&gbc, frag->data + frag->data_size - index_size,
                            8 * index_size);
        if (err < 0)
            return err;

        err = cbs_vp9_read_superframe_index(ctx, &gbc, &sfi);
        if (err < 0)
            return err;

        pos = 0;
        for (i = 0; i <= sfi.frames_in_superframe_minus_1; i++) {
            if (pos + sfi.frame_sizes[i] + index_size > frag->data_size) {
                av_log(ctx->log_ctx, AV_LOG_ERROR, "Frame %d too large "
                       "in superframe: %"PRIu32" bytes.\n",
                       i, sfi.frame_sizes[i]);
                return AVERROR_INVALIDDATA;
            }

            err = ff_cbs_append_unit_data(frag, 0,
                                          frag->data + pos,
                                          sfi.frame_sizes[i],
                                          frag->data_ref);
            if (err < 0)
                return err;

            pos += sfi.frame_sizes[i];
        }
        if (pos + index_size != frag->data_size) {
            av_log(ctx->log_ctx, AV_LOG_WARNING, "Extra padding at "
                   "end of superframe: %"SIZE_SPECIFIER" bytes.\n",
                   frag->data_size - (pos + index_size));
        }

        return 0;

    } else {
        err = ff_cbs_append_unit_data(frag, 0,
                                      frag->data, frag->data_size,
                                      frag->data_ref);
        if (err < 0)
            return err;
    }

    return 0;
}

static int cbs_vp9_read_unit(CodedBitstreamContext *ctx,
                             CodedBitstreamUnit *unit)
{
    VP9RawFrame *frame;
    GetBitContext gbc;
    int err, pos;

    err = init_get_bits(&gbc, unit->data, 8 * unit->data_size);
    if (err < 0)
        return err;

    err = ff_cbs_alloc_unit_content(ctx, unit);
    if (err < 0)
        return err;
    frame = unit->content;

    err = cbs_vp9_read_frame(ctx, &gbc, frame);
    if (err < 0)
        return err;

    pos = get_bits_count(&gbc);
    av_assert0(pos % 8 == 0);
    pos /= 8;
    av_assert0(pos <= unit->data_size);

    if (pos == unit->data_size) {
        // No data (e.g. a show-existing-frame frame).
    } else {
        frame->data_ref = av_buffer_ref(unit->data_ref);
        if (!frame->data_ref)
            return AVERROR(ENOMEM);

        frame->data      = unit->data      + pos;
        frame->data_size = unit->data_size - pos;
    }

    return 0;
}

static int cbs_vp9_write_unit(CodedBitstreamContext *ctx,
                              CodedBitstreamUnit *unit,
                              PutBitContext *pbc)
{
    VP9RawFrame *frame = unit->content;
    int err;

    err = cbs_vp9_write_frame(ctx, pbc, frame);
    if (err < 0)
        return err;

    // Frame must be byte-aligned.
    av_assert0(put_bits_count(pbc) % 8 == 0);

    if (frame->data) {
        if (frame->data_size > put_bits_left(pbc) / 8)
            return AVERROR(ENOSPC);

        flush_put_bits(pbc);
        memcpy(put_bits_ptr(pbc), frame->data, frame->data_size);
        skip_put_bytes(pbc, frame->data_size);
    }

    return 0;
}

static int cbs_vp9_assemble_fragment(CodedBitstreamContext *ctx,
                                     CodedBitstreamFragment *frag)
{
    int err;

    if (frag->nb_units == 1) {
        // Output is just the content of the single frame.

        CodedBitstreamUnit *frame = &frag->units[0];

        frag->data_ref = av_buffer_ref(frame->data_ref);
        if (!frag->data_ref)
            return AVERROR(ENOMEM);

        frag->data      = frame->data;
        frag->data_size = frame->data_size;

    } else {
        // Build superframe out of frames.

        VP9RawSuperframeIndex sfi;
        PutBitContext pbc;
        AVBufferRef *ref;
        uint8_t *data;
        size_t size, max, pos;
        int i, size_len;

        if (frag->nb_units > 8) {
            av_log(ctx->log_ctx, AV_LOG_ERROR, "Too many frames to "
                   "make superframe: %d.\n", frag->nb_units);
            return AVERROR(EINVAL);
        }

        max = 0;
        for (i = 0; i < frag->nb_units; i++)
            if (max < frag->units[i].data_size)
                max = frag->units[i].data_size;

        if (max < 2)
            size_len = 1;
        else
            size_len = av_log2(max) / 8 + 1;
        av_assert0(size_len <= 4);

        sfi.superframe_marker            = VP9_SUPERFRAME_MARKER;
        sfi.bytes_per_framesize_minus_1  = size_len - 1;
        sfi.frames_in_superframe_minus_1 = frag->nb_units - 1;

        size = 2;
        for (i = 0; i < frag->nb_units; i++) {
            size += size_len + frag->units[i].data_size;
            sfi.frame_sizes[i] = frag->units[i].data_size;
        }

        ref = av_buffer_alloc(size + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!ref)
            return AVERROR(ENOMEM);
        data = ref->data;
        memset(data + size, 0, AV_INPUT_BUFFER_PADDING_SIZE);

        pos = 0;
        for (i = 0; i < frag->nb_units; i++) {
            av_assert0(size - pos > frag->units[i].data_size);
            memcpy(data + pos, frag->units[i].data,
                   frag->units[i].data_size);
            pos += frag->units[i].data_size;
        }
        av_assert0(size - pos == 2 + frag->nb_units * size_len);

        init_put_bits(&pbc, data + pos, size - pos);

        err = cbs_vp9_write_superframe_index(ctx, &pbc, &sfi);
        if (err < 0) {
            av_log(ctx->log_ctx, AV_LOG_ERROR, "Failed to write "
                   "superframe index.\n");
            av_buffer_unref(&ref);
            return err;
        }

        av_assert0(put_bits_left(&pbc) == 0);
        flush_put_bits(&pbc);

        frag->data_ref  = ref;
        frag->data      = data;
        frag->data_size = size;
    }

    return 0;
}

static void cbs_vp9_flush(CodedBitstreamContext *ctx)
{
    CodedBitstreamVP9Context *vp9 = ctx->priv_data;

    memset(vp9->ref, 0, sizeof(vp9->ref));
}

static const CodedBitstreamUnitTypeDescriptor cbs_vp9_unit_types[] = {
    CBS_UNIT_TYPE_INTERNAL_REF(0, VP9RawFrame, data),
    CBS_UNIT_TYPE_END_OF_LIST
};

const CodedBitstreamType ff_cbs_type_vp9 = {
    .codec_id          = AV_CODEC_ID_VP9,

    .priv_data_size    = sizeof(CodedBitstreamVP9Context),

    .unit_types        = cbs_vp9_unit_types,

    .split_fragment    = &cbs_vp9_split_fragment,
    .read_unit         = &cbs_vp9_read_unit,
    .write_unit        = &cbs_vp9_write_unit,

    .flush             = &cbs_vp9_flush,

    .assemble_fragment = &cbs_vp9_assemble_fragment,
};
