/*
 * PSB demuxer
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

#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "riff.h"

typedef struct PSBData {
    uint32_t offset;
    uint32_t size;
} PSBData;

typedef union {
    int bln;
    int32_t num;
    int64_t num64;
    double dbl;
    float flt;
    const char *str;
    int count;
    PSBData data;
} PSBResult;

typedef struct PSBList {
    int bytes;
    int count;
    int esize;
    int64_t eoffset;
} PSBList;

typedef struct PSBHeader {
    int codec;

    int total_streams;
    int target_stream;

    int64_t stream_offset[6];
    uint32_t stream_size[6];
    uint32_t body_offset;
    uint32_t body_size;
    uint32_t intro_offset;
    uint32_t intro_size;
    uint32_t fmt_offset;
    uint32_t fmt_size;
    uint32_t dpds_offset;
    uint32_t dpds_size;

    int layers;
    int channels;
    int format;
    int sample_rate;
    int block_size;
    int avg_bitrate;
    int bps;

    uint32_t num_samples;
    int32_t body_samples;
    int32_t intro_samples;
    int32_t intro_skip;
    int32_t body_skip;

    const char *voice;
    const char *spec;
    const char *ext;
    const char *wav;
} PSBHeader;

typedef struct PSBContext {
    uint32_t header_id;
    uint16_t version;
    uint16_t encrypt_value;
    uint32_t encrypt_offset;
    uint32_t keys_offset;

    uint32_t strings_list_offset;
    uint32_t strings_data_offset;
    uint32_t data_offsets_offset;
    uint32_t data_sizes_offset;

    uint32_t data_offset;
    uint32_t root_offset;
    uint32_t unknown;

    uint8_t *strings_data;
    int strings_data_len;
    PSBList strings_list;
    PSBList data_offsets_list;
    PSBList data_sizes_list;

    char *keys;
    int *keys_pos;
    int keys_count;

    PSBHeader psb;
} PSBContext;

typedef struct PSBNode {
    PSBContext *ctx;
    AVIOContext *pb;
    int64_t offset;
} PSBNode;

typedef enum {
    PSB_ITYPE_NONE = 0x0,
    PSB_ITYPE_NULL = 0x1,
    PSB_ITYPE_TRUE = 0x2,
    PSB_ITYPE_FALSE = 0x3,

    PSB_ITYPE_INTEGER_0 = 0x4,
    PSB_ITYPE_INTEGER_8 = 0x5,
    PSB_ITYPE_INTEGER_16 = 0x6,
    PSB_ITYPE_INTEGER_24 = 0x7,
    PSB_ITYPE_INTEGER_32 = 0x8,
    PSB_ITYPE_INTEGER_40 = 0x9,
    PSB_ITYPE_INTEGER_48 = 0xA,
    PSB_ITYPE_INTEGER_56 = 0xB,
    PSB_ITYPE_INTEGER_64 = 0xC,

    PSB_ITYPE_LIST_8  = 0xD,
    PSB_ITYPE_LIST_16 = 0xE,
    PSB_ITYPE_LIST_24 = 0xF,
    PSB_ITYPE_LIST_32 = 0x10,
    PSB_ITYPE_LIST_40 = 0x11,
    PSB_ITYPE_LIST_48 = 0x12,
    PSB_ITYPE_LIST_56 = 0x13,
    PSB_ITYPE_LIST_64 = 0x14,

    PSB_ITYPE_STRING_8 = 0x15,
    PSB_ITYPE_STRING_16 = 0x16,
    PSB_ITYPE_STRING_24 = 0x17,
    PSB_ITYPE_STRING_32 = 0x18,

    PSB_ITYPE_DATA_8 = 0x19,
    PSB_ITYPE_DATA_16 = 0x1A,
    PSB_ITYPE_DATA_24 = 0x1B,
    PSB_ITYPE_DATA_32 = 0x1C,
    PSB_ITYPE_DATA_40 = 0x22,
    PSB_ITYPE_DATA_48 = 0x23,
    PSB_ITYPE_DATA_56 = 0x24,
    PSB_ITYPE_DATA_64 = 0x25,

    PSB_ITYPE_FLOAT_0 = 0x1D,
    PSB_ITYPE_FLOAT_32 = 0x1E,
    PSB_ITYPE_DOUBLE_64 = 0x1F,

    PSB_ITYPE_ARRAY = 0x20,
    PSB_ITYPE_OBJECT = 0x21,
} PSBIType;

typedef enum {
    PSB_TYPE_NULL = 0x0,
    PSB_TYPE_BOOL = 0x1,
    PSB_TYPE_INTEGER = 0x2,
    PSB_TYPE_FLOAT = 0x3,
    PSB_TYPE_STRING = 0x4,
    PSB_TYPE_DATA = 0x5,
    PSB_TYPE_ARRAY = 0x6,
    PSB_TYPE_OBJECT = 0x7,
    PSB_TYPE_UNKNOWN = 0x8,
} PSBType;

static uint32_t item_get_int(const int size, AVIOContext *pb)
{
    switch (size) {
    case 1:
        return avio_r8(pb);
    case 2:
        return avio_rl16(pb);
    case 3:
        return avio_rl24(pb);
    case 4:
        return avio_rl32(pb);
    default:
        return 0;
    }
}

static int read_probe(const AVProbeData *p)
{
    if (AV_RB32(p->buf) != MKBETAG('P','S','B','\x0'))
        return 0;

    if (AV_RL16(p->buf+4) != 2 && AV_RL16(p->buf+4) != 3)
        return 0;

    return AVPROBE_SCORE_MAX;
}

static int list_init(AVIOContext *pb, PSBList *lst, int64_t offset)
{
    int count_size, count, entry_size;
    uint8_t count_itype, entry_itype;

    avio_seek(pb, offset, SEEK_SET);

    count_itype = avio_r8(pb);

    switch (count_itype) {
    case PSB_ITYPE_LIST_8:
    case PSB_ITYPE_LIST_16:
    case PSB_ITYPE_LIST_24:
    case PSB_ITYPE_LIST_32:
        count_size = count_itype - PSB_ITYPE_LIST_8 + 1;
        count = item_get_int(count_size, pb);
        break;
    default:
        goto fail;
    }

    avio_seek(pb, offset + 1 + count_size, SEEK_SET);

    entry_itype = avio_r8(pb);

    switch (entry_itype) {
    case PSB_ITYPE_LIST_8:
    case PSB_ITYPE_LIST_16:
    case PSB_ITYPE_LIST_24:
    case PSB_ITYPE_LIST_32:
        entry_size = entry_itype - PSB_ITYPE_LIST_8 + 1;
        break;
    default:
        goto fail;
    }

    lst->bytes = 1 + count_size + 1 + entry_size * count;
    lst->count = count;
    lst->esize = entry_size;
    lst->eoffset = offset + 1 + count_size + 1;

    return 0;

fail:
    memset(lst, 0, sizeof(*lst));

    return AVERROR_INVALIDDATA;
}

static uint32_t list_get_entry(AVIOContext *pb,
                               PSBList *lst, uint32_t index)
{
    int64_t offset = lst->eoffset + index * lst->esize;

    avio_seek(pb, offset, SEEK_SET);

    return item_get_int(lst->esize, pb);
}

static int decode_key(AVFormatContext *s,
                      PSBList *kidx1, PSBList *kidx2, PSBList *kidx3,
                      char *str, int str_len, int index)
{
    AVIOContext *pb = s->pb;
    uint32_t entry_point;
    uint32_t point;
    int i;

    entry_point = list_get_entry(pb, kidx3, index);
    point = list_get_entry(pb, kidx2, entry_point);
    for (i = 0; i < str_len; i++) {
        uint32_t next;
        uint32_t diff;
        uint32_t curr;

        next = list_get_entry(pb, kidx2, point);
        diff = list_get_entry(pb, kidx1, next);
        curr = point - diff;

        str[i] = (char)curr;

        point = next;
        if (!point)
            break;
    }

    if (i == str_len) {
        return AVERROR_INVALIDDATA;
    } else {
        i++;
    }

    str[i] = '\0';

    return i;
}

static int init_keys(AVFormatContext *s)
{
    PSBContext *p = s->priv_data;
    AVIOContext *pb = s->pb;
    PSBList kidx1, kidx2, kidx3;
    int64_t keys_offset = p->keys_offset;
    int i, j, pos, ret;
    char key[256];
    int keys_size;

    ret = list_init(pb, &kidx1, keys_offset);
    if (ret < 0)
        return ret;

    ret = list_init(pb, &kidx2, keys_offset + kidx1.bytes);
    if (ret < 0)
        return ret;

    ret = list_init(pb, &kidx3, keys_offset + kidx1.bytes + kidx2.bytes);
    if (ret < 0)
        return ret;

    p->keys_count = kidx3.count;
    p->keys_pos = av_calloc(p->keys_count, sizeof(*p->keys_pos));
    if (!p->keys_pos)
        return AVERROR(ENOMEM);

    keys_size = (kidx1.bytes + kidx2.bytes + kidx3.bytes) * 2;
    p->keys = av_calloc(keys_size, sizeof(*p->keys));
    if (!p->keys)
        return AVERROR(ENOMEM);

    pos = 0;
    for (i = 0; i < kidx3.count; i++) {
        int key_len = decode_key(s, &kidx1, &kidx2, &kidx3, key, sizeof(key), i);

        if (pos + key_len > keys_size)
            return AVERROR_INVALIDDATA;

        for (j = 0; j < key_len; j++)
            p->keys[pos + key_len - 1 - j] = key[j];
        p->keys[pos + key_len] = '\0';

        p->keys_pos[i] = pos;

        pos += key_len + 1;
    }

    return 0;
}

static int list_get_count(int64_t offset, AVIOContext *pb)
{
    uint8_t itype;
    int size;

    avio_seek(pb, offset, SEEK_SET);
    itype = avio_r8(pb);

    switch (itype) {
    case PSB_ITYPE_LIST_8:
    case PSB_ITYPE_LIST_16:
    case PSB_ITYPE_LIST_24:
    case PSB_ITYPE_LIST_32:
        size = itype - PSB_ITYPE_LIST_8 + 1;
        return item_get_int(size, pb);
    default:
        return 0;
    }
}

static int psb_node_get_count(const PSBNode *node)
{
    int64_t offset;

    if (!node || !node->offset || !node->pb)
        goto fail;

    offset = node->offset;
    avio_seek(node->pb, offset, SEEK_SET);
    switch (avio_r8(node->pb)) {
    case PSB_ITYPE_ARRAY:
    case PSB_ITYPE_OBJECT:
        return list_get_count(offset + 1, node->pb);
    default:
        return 0;
    }
fail:
    return -1;
}

static const char *psb_node_get_key(const PSBNode *node, int index)
{
    int64_t offset;
    int pos;

    if (!node || !node->ctx || !node->offset)
        goto fail;

    offset = node->offset;
    avio_seek(node->pb, offset, SEEK_SET);
    switch (avio_r8(node->pb)) {
    case PSB_ITYPE_OBJECT: {
                               PSBList keys;
                               int keys_index;

                               list_init(node->pb, &keys, offset + 1);
                               keys_index = list_get_entry(node->pb, &keys, index);
                               if (keys_index < 0 || keys_index > node->ctx->keys_count)
                                   goto fail;

                               pos = node->ctx->keys_pos[keys_index];
                               return &node->ctx->keys[pos];
                           }

    default:
                               goto fail;
    }

fail:
    return NULL;
}

static void node_error(PSBNode *p_out)
{
    if (!p_out)
        return;

    p_out->pb = NULL;
    p_out->ctx = NULL;
    p_out->offset = 0;
}

static int psb_node_by_index(const PSBNode *node, int index, PSBNode *p_out)
{
    AVIOContext *pb;
    int64_t offset;

    if (!node || !node->offset || !node->pb)
        goto fail;

    pb = node->pb;
    offset = node->offset;
    avio_seek(pb, offset, SEEK_SET);
    switch (avio_r8(pb)) {
        case PSB_ITYPE_ARRAY: {
                                  PSBList offsets;
                                  int skip;

                                  list_init(pb, &offsets, offset + 1);
                                  skip = list_get_entry(pb, &offsets, index);

                                  p_out->pb = pb;
                                  p_out->ctx = node->ctx;
                                  p_out->offset = offset + 1 + offsets.bytes + skip;
                                  return 1;
                              }

        case PSB_ITYPE_OBJECT: {
                                   PSBList keys, offsets;
                                   int skip;

                                   list_init(pb, &keys, offset + 1);
                                   list_init(pb, &offsets, offset + 1 + keys.bytes);
                                   skip = list_get_entry(pb, &offsets, index);

                                   p_out->pb = pb;
                                   p_out->ctx = node->ctx;
                                   p_out->offset = offset + 1 + keys.bytes + offsets.bytes + skip;
                                   return 1;
                               }

        default:
                               goto fail;
    }
fail:
    node_error(p_out);
    return 0;
}

static int psb_node_by_key(const PSBNode *node, const char *key, PSBNode *p_out)
{
    int max;

    if (!node || !node->ctx)
        goto fail;

    max = psb_node_get_count(node);
    if (max < 0 || max > node->ctx->keys_count)
        goto fail;

    for (int i = 0; i < max; i++) {
        const char *key_test = psb_node_get_key(node, i);

        if (!key_test)
            goto fail;

        if (!strcmp(key_test, key))
            return psb_node_by_index(node, i, p_out);
    }

fail:
    node_error(p_out);
    return 0;
}

static PSBType psb_node_get_type(const PSBNode *node)
{
    int64_t offset;
    uint8_t itype;

    if (!node || !node->offset)
        goto fail;

    offset = node->offset;
    avio_seek(node->pb, offset, SEEK_SET);
    itype = avio_r8(node->pb);
    switch (itype) {
    case PSB_ITYPE_NULL:
        return PSB_TYPE_NULL;

    case PSB_ITYPE_TRUE:
    case PSB_ITYPE_FALSE:
        return PSB_TYPE_BOOL;

    case PSB_ITYPE_INTEGER_0:
    case PSB_ITYPE_INTEGER_8:
    case PSB_ITYPE_INTEGER_16:
    case PSB_ITYPE_INTEGER_24:
    case PSB_ITYPE_INTEGER_32:
    case PSB_ITYPE_INTEGER_40:
    case PSB_ITYPE_INTEGER_48:
    case PSB_ITYPE_INTEGER_56:
    case PSB_ITYPE_INTEGER_64:
        return PSB_TYPE_INTEGER;

    case PSB_ITYPE_STRING_8:
    case PSB_ITYPE_STRING_16:
    case PSB_ITYPE_STRING_24:
    case PSB_ITYPE_STRING_32:
        return PSB_TYPE_STRING;

    case PSB_ITYPE_DATA_8:
    case PSB_ITYPE_DATA_16:
    case PSB_ITYPE_DATA_24:
    case PSB_ITYPE_DATA_32:
    case PSB_ITYPE_DATA_40:
    case PSB_ITYPE_DATA_48:
    case PSB_ITYPE_DATA_56:
    case PSB_ITYPE_DATA_64:
        return PSB_TYPE_DATA;

    case PSB_ITYPE_FLOAT_0:
    case PSB_ITYPE_FLOAT_32:
    case PSB_ITYPE_DOUBLE_64:
        return PSB_TYPE_FLOAT;

    case PSB_ITYPE_ARRAY:
        return PSB_TYPE_ARRAY;

    case PSB_ITYPE_OBJECT:
        return PSB_TYPE_OBJECT;
    }

fail:
    return PSB_TYPE_UNKNOWN;
}

static int get_expected_node(const PSBNode *node, const char *key,
                             PSBNode *p_out, PSBType expected)
{
    if (!psb_node_by_key(node, key, p_out))
        goto fail;

    if (psb_node_get_type(p_out) != expected)
        goto fail;

    return 1;

fail:
    return 0;
}

static PSBResult psb_node_get_result(PSBNode *node)
{
    AVIOContext *pb;
    int64_t offset;
    uint8_t itype;
    PSBResult res = {0};
    int size, index, skip;

    if (!node || !node->ctx || !node->offset || !node->pb)
        goto fail;

    pb = node->pb;
    offset = node->offset;
    avio_seek(pb, offset, SEEK_SET);
    itype = avio_r8(pb);
    switch (itype) {
        case PSB_ITYPE_NULL:
            break;

        case PSB_ITYPE_TRUE:
        case PSB_ITYPE_FALSE:
            res.bln = (itype == PSB_ITYPE_TRUE);
            break;

        case PSB_ITYPE_INTEGER_0:
            res.num = 0;
            break;

        case PSB_ITYPE_INTEGER_8:
        case PSB_ITYPE_INTEGER_16:
        case PSB_ITYPE_INTEGER_24:
        case PSB_ITYPE_INTEGER_32:
            size = itype - PSB_ITYPE_INTEGER_8 + 1;

            res.num = item_get_int(size, pb);
            break;

        case PSB_ITYPE_INTEGER_40:
        case PSB_ITYPE_INTEGER_48:
        case PSB_ITYPE_INTEGER_56:
        case PSB_ITYPE_INTEGER_64:
            break;

        case PSB_ITYPE_STRING_8:
        case PSB_ITYPE_STRING_16:
        case PSB_ITYPE_STRING_24:
        case PSB_ITYPE_STRING_32:
            size = itype - PSB_ITYPE_STRING_8 + 1;
            index = item_get_int(size, pb);
            skip = list_get_entry(node->pb, &node->ctx->strings_list, index);

            res.str = (const char*)&node->ctx->strings_data[skip];
            if (skip >= node->ctx->strings_data_len)
                res.str = "";
            break;

        case PSB_ITYPE_DATA_8:
        case PSB_ITYPE_DATA_16:
        case PSB_ITYPE_DATA_24:
        case PSB_ITYPE_DATA_32:
            size = itype - PSB_ITYPE_DATA_8 + 1;
            index = item_get_int(size, pb);

            res.data.offset = list_get_entry(node->pb, &node->ctx->data_offsets_list, index);
            res.data.size = list_get_entry(node->pb, &node->ctx->data_sizes_list, index);

            res.data.offset += node->ctx->data_offset;

            break;

        case PSB_ITYPE_DATA_40:
        case PSB_ITYPE_DATA_48:
        case PSB_ITYPE_DATA_56:
        case PSB_ITYPE_DATA_64:
            break;

        case PSB_ITYPE_FLOAT_0:
            res.flt = 0.0f;
            break;

        case PSB_ITYPE_FLOAT_32:
            res.num = avio_rl32(pb);
            break;

        case PSB_ITYPE_DOUBLE_64:
            res.num64 = avio_rl64(pb);
            break;

        case PSB_ITYPE_ARRAY:
        case PSB_ITYPE_OBJECT:
            res.count = list_get_count(offset + 1, pb);
            break;

        default:
            goto fail;
    }

    return res;
fail:
    return res;
}

static const char *psb_node_get_string(const PSBNode *node, const char *key)
{
    PSBNode out = { 0 };

    if (!get_expected_node(node, key, &out, PSB_TYPE_STRING))
        return NULL;

    return psb_node_get_result(&out).str;
}

static void psb_get_root(PSBContext *p, AVIOContext *pb,
                         PSBNode *p_root)
{
    p_root->ctx = p;
    p_root->pb = pb;
    p_root->offset = p->root_offset;
}

static int psb_node_exists(const PSBNode *node, const char *key)
{
    PSBNode out;

    if (!psb_node_by_key(node, key, &out))
        return 0;

    return 1;
}

static PSBData psb_node_get_data(const PSBNode *node, const char *key)
{
    PSBNode out;

    if (!get_expected_node(node, key, &out, PSB_TYPE_DATA)) {
        PSBData data = { 0 };

        return data;
    }
    return psb_node_get_result(&out).data;
}

static int32_t psb_node_get_integer(const PSBNode *node, const char *key)
{
    PSBNode out;

    if (!get_expected_node(node, key, &out, PSB_TYPE_INTEGER))
        return 0;

    return psb_node_get_result(&out).num;
}

static float psb_node_get_float(const PSBNode *node, const char *key)
{
    PSBNode out;

    if (!get_expected_node(node, key, &out, PSB_TYPE_FLOAT))
        return 0.0f;

    return psb_node_get_result(&out).flt;
}

static int parse_psb_channels(PSBHeader *psb, PSBNode *nchans)
{
    PSBNode nchan, narch, node;

    psb->layers = psb_node_get_count(nchans);
    if (psb->layers == 0)
        goto fail;

    if (psb->layers > 6)
        goto fail;

    for (int i = 0; i < psb->layers; i++) {
        PSBData data;
        PSBType type;

        psb_node_by_index(nchans, i, &nchan);

        psb_node_by_key(&nchan, "archData", &narch);
        type = psb_node_get_type(&narch);
        switch (type) {
            case PSB_TYPE_DATA:
                data = psb_node_get_result(&narch).data;
                psb->stream_offset[i] = data.offset;
                psb->stream_size[i] = data.size;
                break;

            case PSB_TYPE_OBJECT:
                data = psb_node_get_data(&narch, "data");
                if (data.offset) {
                    psb->stream_offset[i] = data.offset;
                    psb->stream_size[i] = data.size;
                }

                data = psb_node_get_data(&narch, "fmt");
                if (data.offset) {
                    psb->fmt_offset = data.offset;
                    psb->fmt_size = data.size;
                }

                if (psb_node_by_key(&narch, "body", &node)) {
                    data = psb_node_get_data(&node, "data");
                    psb->body_offset = data.offset;
                    psb->body_size = data.size;
                    psb->body_samples = psb_node_get_integer(&node, "sampleCount");
                    psb->body_skip = psb_node_get_integer(&node, "skipSampleCount");
                }

                if (psb_node_by_key(&narch, "intro", &node)) {
                    data = psb_node_get_data(&node, "data");
                    psb->intro_offset = data.offset;
                    psb->intro_size = data.size;
                    psb->intro_samples = psb_node_get_integer(&node, "sampleCount");
                    psb->intro_skip = psb_node_get_integer(&node, "skipSampleCount");
                }

                data = psb_node_get_data(&narch, "dpds");
                if (data.offset) {
                    psb->dpds_offset = data.offset;
                    psb->dpds_size = data.size;
                }

                psb->channels = psb_node_get_integer(&narch, "channelCount");

                psb->sample_rate = (int)psb_node_get_float(&narch, "samprate");
                if (!psb->sample_rate)
                    psb->sample_rate = psb_node_get_integer(&narch, "samprate");

                psb->wav = psb_node_get_string(&narch, "wav");
                break;
            default:
                goto fail;
        }
    }

    return 1;
fail:
    return 0;
}

static int parse_psb_voice(PSBHeader *psb, PSBNode *nvoice)
{
    PSBNode nstream, nchans;

    psb->total_streams = psb_node_get_count(nvoice);
    if (psb->target_stream == 0)
        psb->target_stream = 1;
    if (psb->total_streams <= 0 || psb->target_stream > psb->total_streams)
        goto fail;

    if (!psb_node_by_index(nvoice, psb->target_stream - 1, &nstream))
        goto fail;

    psb->voice = psb_node_get_key(nvoice, psb->target_stream - 1);

    psb_node_by_key(&nstream, "channelList", &nchans);
    if (!parse_psb_channels(psb, &nchans))
        goto fail;

    if (psb_node_exists(&nstream, "device") <= 0)
        goto fail;

    return 1;
fail:
    return 0;
}

static int prepare_fmt(AVFormatContext *s, PSBHeader *psb)
{
    uint32_t offset = psb->fmt_offset;
    uint32_t size = psb->fmt_size;
    AVIOContext *pb = s->pb;

    if (!offset || size < 14)
        return 1;

    avio_seek(pb, offset, SEEK_SET);
    psb->format = avio_rl16(pb);
    if (psb->format == 0x6601) { /* X360 */
        psb->format = av_bswap16(psb->format);
        psb->channels = avio_rb16(pb);
        psb->sample_rate = avio_rb32(pb);
    } else {
        psb->channels = avio_rl16(pb);
        psb->sample_rate = avio_rl32(pb);
        psb->avg_bitrate = avio_rl32(pb);
        psb->block_size = avio_rl16(pb);
        if (size >= 16) {
            psb->bps = avio_rl16(pb);
        } else {
            psb->bps = 8;
        }
    }

    return 1;
}

static int prepare_codec(AVFormatContext *s, PSBHeader *psb)
{
    AVIOContext *pb = s->pb;
    const char *spec = psb->spec;
    const char *ext = psb->ext;

    if (psb->format != 0) {
        psb->codec = ff_codec_get_id(ff_codec_wav_tags, psb->format);

        if (psb->format == 0x0001)
            psb->codec = ff_get_pcm_codec_id(psb->bps, 0, 0, 0xFFFF);
        return 1;
    }

    if (!spec)
        goto fail;

    if (strcmp(spec, "nx") == 0) {
        if (!ext)
            goto fail;

        if (strcmp(ext, ".opus") == 0) {
            psb->codec = 0;

            psb->body_samples -= psb->body_skip;
            psb->num_samples = psb->intro_samples + psb->body_samples;
            return 1;
        }

        if (strcmp(ext, ".adpcm") == 0) {
            psb->codec = AV_CODEC_ID_ADPCM_THP_LE;

            psb->channels = psb->layers;

            avio_seek(pb, psb->stream_offset[0], SEEK_SET);
            psb->num_samples = avio_rl32(pb);

            return 1;
        }

        if (strcmp(ext, ".p16") == 0) {
            psb->codec = AV_CODEC_ID_PCM_S16LE;
            psb->bps = 16;

            psb->channels = psb->layers;

            return 1;
        }
    }

    if (strcmp(spec, "ps3") == 0) {
        psb->codec = AV_CODEC_ID_ATRAC3;
        return 1;
    }

    if (strcmp(spec, "vita") == 0 || strcmp(spec, "ps4") == 0) {
        avio_seek(pb, psb->stream_offset[0], SEEK_SET);

        if (avio_rb32(pb) == MKBETAG('R','I','F','F'))
            psb->codec = AV_CODEC_ID_ATRAC9;
        else
            psb->codec = 0;
        return 1;
    }

    if (strcmp(spec, "and") == 0) {
        if (!ext)
            goto fail;

        if (strcmp(ext, ".ogg") == 0) {
            psb->codec = 0;
            return 1;
        }

        if (strcmp(ext, ".wav") == 0) {
            psb->codec = 0;
            return 1;
        }
    }

fail:
    return 0;
}

static int prepare_psb_extra(AVFormatContext *s, PSBHeader *psb)
{
    if (!prepare_fmt(s, psb))
        goto fail;

    if (!prepare_codec(s, psb))
        goto fail;

    return 1;
fail:
    return 0;
}

static int read_header(AVFormatContext *s)
{
    PSBContext *p = s->priv_data;
    AVIOContext *pb = s->pb;
    PSBNode nroot, nvoice;
    const char *id;
    AVStream *st;
    int ret;

    p->header_id = avio_rb32(pb);
    p->version = avio_rl16(pb);
    p->encrypt_value = avio_rl16(pb);
    p->encrypt_offset = avio_rl32(pb);
    p->keys_offset = avio_rl32(pb);

    p->strings_list_offset = avio_rl32(pb);
    p->strings_data_offset = avio_rl32(pb);
    p->data_offsets_offset = avio_rl32(pb);
    p->data_sizes_offset = avio_rl32(pb);

    p->data_offset = avio_rl32(pb);
    p->root_offset = avio_rl32(pb);
    if (p->version >= 3)
        p->unknown = avio_rl32(pb);

    if (p->header_id != MKBETAG('P','S','B','\x0'))
        return AVERROR_INVALIDDATA;

    if (p->version != 2 && p->version != 3)
        p->unknown = avio_rl32(pb);

    if (p->encrypt_value != 0)
        return AVERROR_INVALIDDATA;

    if (p->encrypt_offset != 0 && p->encrypt_offset != p->keys_offset)
        return AVERROR_INVALIDDATA;

    if (p->keys_offset >= p->data_offset ||
        p->strings_list_offset >= p->data_offset ||
        p->strings_data_offset >= p->data_offset ||
        p->data_offsets_offset >= p->data_offset ||
        p->data_sizes_offset >= p->data_offset ||
        p->root_offset >= p->data_offset)
        return AVERROR_INVALIDDATA;

    ret = list_init(pb, &p->strings_list, p->strings_list_offset);
    if (ret < 0)
        return ret;

    p->strings_data_len = p->data_offsets_offset - p->strings_data_offset;
    if (p->strings_data_len < 0)
        return AVERROR_INVALIDDATA;

    p->strings_data = av_calloc(p->strings_data_len, sizeof(*p->strings_data));
    if (!p->strings_data)
        return AVERROR(ENOMEM);

    avio_seek(pb, p->strings_data_offset, SEEK_SET);
    ret = avio_read(pb, p->strings_data, p->strings_data_len);
    if (ret != p->strings_data_len)
        return AVERROR_INVALIDDATA;

    if (p->strings_data[p->strings_data_len - 1] != '\0')
        return AVERROR_INVALIDDATA;

    ret = list_init(pb, &p->data_offsets_list, p->data_offsets_offset);
    if (ret < 0)
        return ret;

    ret = list_init(pb, &p->data_sizes_list, p->data_sizes_offset);
    if (ret < 0)
        return ret;

    ret = init_keys(s);
    if (ret < 0)
        return ret;

    psb_get_root(p, pb, &nroot);

    id = psb_node_get_string(&nroot, "id");
    if (!id || strcmp(id, "sound_archive"))
        return AVERROR_INVALIDDATA;

    p->psb.spec = psb_node_get_string(&nroot, "spec");

    psb_node_by_key(&nroot, "voice", &nvoice);
    if (!parse_psb_voice(&p->psb, &nvoice))
        return AVERROR_INVALIDDATA;

    if (!prepare_psb_extra(s, &p->psb))
        return AVERROR_INVALIDDATA;

    if (p->psb.sample_rate <= 0 ||
        p->psb.channels <= 0 ||
        p->psb.block_size <= 0)
        return AVERROR_INVALIDDATA;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->start_time = 0;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = p->psb.codec;
    st->codecpar->bits_per_coded_sample = p->psb.bps;
    st->codecpar->ch_layout.nb_channels = p->psb.channels;
    st->codecpar->sample_rate = p->psb.sample_rate;
    st->codecpar->block_align = p->psb.block_size;
    if (st->codecpar->block_align / ((p->psb.bps + 7) / 8) <= p->psb.channels)
        st->codecpar->block_align *= 1024;
    st->codecpar->bit_rate = p->psb.avg_bitrate * 8LL;
    if (p->psb.num_samples > 0)
        st->duration = p->psb.num_samples;

    avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    if (st->codecpar->codec_id == AV_CODEC_ID_WMAV2) {
        ret = ff_alloc_extradata(st->codecpar, 6);
        if (ret < 0)
            return ret;

        memset(st->codecpar->extradata, 0, st->codecpar->extradata_size);
        AV_WL16(st->codecpar->extradata + 4, 0x1f);
    }

    avio_seek(pb, p->psb.stream_offset[0], SEEK_SET);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    PSBContext *p = s->priv_data;
    const int64_t end = p->psb.stream_offset[0] + p->psb.stream_size[0];
    AVIOContext *pb = s->pb;
    int64_t pos = avio_tell(pb);
    int ret;

    if (avio_feof(pb))
        return AVERROR_EOF;

    if (pos >= end)
        return AVERROR_EOF;

    ret = av_get_packet(pb, pkt, FFMIN(end - pos, s->streams[0]->codecpar->block_align));
    pkt->flags &= ~AV_PKT_FLAG_CORRUPT;
    pkt->stream_index = 0;

    return ret;
}

static int read_close(AVFormatContext *s)
{
    PSBContext *p = s->priv_data;

    av_freep(&p->strings_data);
    av_freep(&p->keys_pos);
    av_freep(&p->keys);

    return 0;
}

const FFInputFormat ff_psb_demuxer = {
    .p.name         = "psb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("PSB M2"),
    .priv_data_size = sizeof(PSBContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "psb",
    .read_probe     = read_probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_close     = read_close,
};
