/*
 * WwiseVorbis to Vorbis bitstream filter
 * Copyright (c) 2024 Paul B Mahol
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

/**
 * @file
 * WwiseVorbis to Vorbis bitstream filter.
 */

#include "libavutil/intreadwrite.h"
#include "bsf.h"
#include "bsf_internal.h"
#include "bsf/wwisevorbis_data.h"
#define BITSTREAM_READER_LE
#define BITSTREAM_WRITER_LE
#include "put_bits.h"
#include "get_bits.h"
#include "vorbis.h"

typedef struct WwVorbisContext {
    AVPacket *out_pkt;
    int is_wem;
    int setup_done;
    int header_done;
    int channels;
    int sample_rate;
    int bl0, bl1;

    uint8_t mode_blockflag[64+1];
    int mode_bits;
    int prev_blockflag;
} WwVorbisContext;

static av_cold int init(AVBSFContext *ctx)
{
    WwVorbisContext *s = ctx->priv_data;

    if (ctx->par_in->extradata_size >= 48) {
        s->bl0 = ctx->par_in->extradata[46];
        s->bl1 = ctx->par_in->extradata[47];
    }

    s->channels = ctx->par_in->ch_layout.nb_channels;
    s->sample_rate = ctx->par_in->sample_rate;
    s->is_wem = ctx->par_in->level == 1;
    ctx->par_out->codec_id = AV_CODEC_ID_VORBIS;

    s->out_pkt = av_packet_alloc();
    if (!s->out_pkt)
        return AVERROR(ENOMEM);

    return 0;
}

static int load_wvc(uint8_t *buf, size_t bufsize, uint32_t codebook_id, const int setup_type)
{
    const WVCInfo *wvc_list;
    int list_length;

    switch (setup_type) {
    case 0:
        wvc_list = wvc_list_standard;
        list_length = sizeof(wvc_list_standard) / sizeof(WVCInfo);
        break;
    case 1:
        wvc_list = wvc_list_aotuv603;
        list_length = sizeof(wvc_list_aotuv603) / sizeof(WVCInfo);
        break;
    default:
        goto fail;
    }

    for (int i = 0; i < list_length; i++) {
        if (wvc_list[i].id == codebook_id) {
            if (wvc_list[i].size > bufsize)
                goto fail;

            memcpy(buf, wvc_list[i].codebook, wvc_list[i].size);

            return wvc_list[i].size;
        }
    }
fail:
    return 0;
}

static unsigned int book_maptype1_quantvals(unsigned int entries, unsigned int dimensions)
{
    const int bits = ilog(entries);
    int vals = entries>>((bits-1)*(dimensions-1)/dimensions);

    while (1) {
        unsigned long acc = 1, acc1 = 1;

        for (int i = 0; i < dimensions; i++) {
            acc *= vals;
            acc1 *= vals+1;
        }

        if (acc <= entries && acc1 > entries) {
            return vals;
        } else {
            if (acc > entries) {
                vals--;
            } else {
                vals++;
            }
        }
    }
}

static int codebook_library_rebuild(PutBitContext *pb, GetBitContext *gb,
                                    size_t cb_size)
{
    uint32_t id = 0, dimensions = 0, entries;
    uint32_t ordered = 0, lookup_type;

    id = 0x564342; /* "VCB" */

    put_bits(pb, 24, id);
    dimensions = get_bits(gb, 4);
    put_bits(pb, 16, dimensions); /* 4 to 16 */
    entries = get_bits(gb, 14);
    put_bits(pb, 24, entries); /* 14 to 24 */

    /* codeword lengths */
    ordered = get_bits(gb, 1);
    put_bits(pb, 1, ordered);

    if (ordered) {
        uint32_t initial_length, current_entry;

        initial_length = get_bits(gb, 5);
        put_bits(pb, 5, initial_length);

        current_entry = 0;
        while (current_entry < entries) {
            uint32_t number;
            int numbebl_get = ilog(entries-current_entry);

            number = get_bits_long(gb, numbebl_get);
            put_bits(pb, numbebl_get, number);
            current_entry += number;
        }
        if (current_entry > entries) {
            av_log(NULL, AV_LOG_ERROR, "current_entry out of range\n");
            goto fail;
        }
    } else {
        uint32_t codeword_length_length, sparse;

        codeword_length_length = get_bits(gb, 3);
        sparse = get_bits(gb, 1);
        put_bits(pb, 1, sparse);

        if (0 == codeword_length_length || codeword_length_length > 5) {
            av_log(NULL, AV_LOG_ERROR, "nonsense codeword length\n");
            goto fail;
        }

        for (int i = 0; i < entries; i++) {
            uint32_t present_bool;

            present_bool = 1;
            if (sparse) {
                uint32_t present;

                present = get_bits(gb, 1);
                put_bits(pb, 1, present);

                present_bool = (0 != present);
            }

            if (present_bool) {
                uint32_t codeword_length;

                codeword_length = get_bits(gb, codeword_length_length);
                put_bits(pb, 5, codeword_length); /* max 7 (3b) to 5 */
            }
        }
    }

    lookup_type = get_bits1(gb);
    put_bits(pb, 4, lookup_type);

    if (lookup_type == 0) {
    } else if (lookup_type == 1) {
        uint32_t quantvals, min, max;
        uint32_t value_length, sequence_flag;

        min = get_bits_long(gb, 32);
        put_bits32(pb, min);
        max = get_bits_long(gb, 32);
        put_bits32(pb, max);
        value_length = get_bits(gb, 4);
        put_bits(pb, 4, value_length);
        sequence_flag = get_bits(gb, 1);
        put_bits(pb, 1, sequence_flag);

        quantvals = book_maptype1_quantvals(entries, dimensions);
        for (int i = 0; i < quantvals; i++) {
            uint32_t val, val_bits;

            val_bits = value_length+1;
            val = get_bits_long(gb, val_bits);
            put_bits(pb, val_bits, val);
        }
    } else if (lookup_type == 2) {
        av_log(NULL, AV_LOG_ERROR, "didn't expect lookup type 2\n");
        goto fail;
    } else {
        av_log(NULL, AV_LOG_ERROR, "invalid lookup type\n");
        goto fail;
    }

    return 1;
fail:
    return 0;
}

static int codebook_rebuild_by_id(PutBitContext *pb, int32_t codebook_id,
                                  const int setup_type)
{
    uint8_t ibuf[0x8000] = { 0 };
    size_t ibufsize = 0x8000;
    GetBitContext gb;
    size_t cb_size;
    int ret;

    cb_size = load_wvc(ibuf, ibufsize, codebook_id, setup_type);
    if (cb_size == 0)
        goto fail;

    ret = init_get_bits8(&gb, ibuf, ibufsize);
    if (ret < 0)
        return ret;

    return codebook_library_rebuild(pb, &gb, cb_size);
fail:
    return 0;
}

static int filter(AVBSFContext *ctx, AVPacket *pkt)
{
    WwVorbisContext *s = ctx->priv_data;
    const int channels = s->channels;
    GetBitContext gbc, *gb = &gbc;
    PutBitContext pbc, *pb = &pbc;
    int ret;

    ret = av_new_packet(s->out_pkt, 0x10000);
    if (ret < 0)
        return ret;

    if (s->header_done == 0) {
        uint8_t blocksizes;

        ret = ff_bsf_get_packet_ref(ctx, pkt);
        if (ret < 0)
            goto fail;

        ret = av_packet_copy_props(s->out_pkt, pkt);
        if (ret < 0)
            goto fail;

        init_put_bits(pb, s->out_pkt->data, s->out_pkt->size);

        blocksizes = (s->bl1 << 4) | (s->bl0);

        put_bits(pb, 8, 0x01);
        put_bits(pb, 8, 'v');
        put_bits(pb, 8, 'o');
        put_bits(pb, 8, 'r');
        put_bits(pb, 8, 'b');
        put_bits(pb, 8, 'i');
        put_bits(pb, 8, 's');

        put_bits32(pb, 0x00);
        put_bits(pb, 8, s->channels);
        put_bits32(pb, s->sample_rate);
        put_bits32(pb, 0x00);
        put_bits32(pb, 0x00);
        put_bits32(pb, 0x00);
        put_bits(pb, 8, blocksizes);
        put_bits(pb, 8, 0x01);

        flush_put_bits(pb);
        s->out_pkt->size = put_bytes_output(pb);

        s->header_done = 1;
    } else if (s->setup_done == 0) {
        uint32_t codebook_count_less1, codebook_count;

        ret = ff_bsf_get_packet_ref(ctx, pkt);
        if (ret < 0)
            goto fail;

        ret = av_packet_copy_props(s->out_pkt, pkt);
        if (ret < 0)
            goto fail;

        init_put_bits(pb, s->out_pkt->data, s->out_pkt->size);

        ret = init_get_bits8(gb, pkt->data, pkt->size);
        if (ret < 0)
            goto fail;

        skip_bits_long(gb, 2*8);

        put_bits(pb, 8, 5);
        put_bits(pb, 8, 'v');
        put_bits(pb, 8, 'o');
        put_bits(pb, 8, 'r');
        put_bits(pb, 8, 'b');
        put_bits(pb, 8, 'i');
        put_bits(pb, 8, 's');

        codebook_count_less1 = get_bits(gb, 8);
        put_bits(pb, 8, codebook_count_less1);
        codebook_count = codebook_count_less1 + 1;

        for (int i = 0; i < codebook_count; i++) {
            uint32_t codebook_id = get_bits(gb, 10);

            if (codebook_rebuild_by_id(pb, codebook_id, s->is_wem) == 0) {
                av_log(NULL, AV_LOG_ERROR, "failed to rebuild codebook\n");
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }
        }

        put_bits(pb, 6, 0);
        put_bits(pb, 16, 0);

        if (0) {
            while (get_bits_left(gb) > 0) {
                uint32_t bitly = get_bits1(gb);

                put_bits(pb, 1, bitly);
            }
        } else {
            uint32_t floor_count_less1, floor1_multiplier_less1, rangebits;
            uint32_t residue_count_less1, floor_count, residue_count;
            uint32_t mapping_count_less1, mapping_count;
            uint32_t mode_count_less1, mode_count;

            /* Floors */
            floor_count_less1 = get_bits(gb, 6);
            put_bits(pb, 6, floor_count_less1);
            floor_count = floor_count_less1 + 1;

            for (int i = 0; i < floor_count; i++) {
                uint32_t floor_type, floor1_partitions, maximum_class;
                uint32_t floor1_partition_class_list[32]; /* max 5b */
                uint32_t floor1_class_dimensions_list[16+1]; /* max 4b+1 */

                // Always floor type 1
                floor_type = 1;
                put_bits(pb, 16, floor_type);

                floor1_partitions = get_bits(gb, 5);
                put_bits(pb, 5, floor1_partitions);

                memset(floor1_partition_class_list, 0, sizeof(uint32_t)*32);

                maximum_class = 0;
                for (int j = 0; j < floor1_partitions; j++) {
                    uint32_t floor1_partition_class;

                    floor1_partition_class = get_bits(gb, 4);
                    put_bits(pb, 4, floor1_partition_class);

                    floor1_partition_class_list[j] = floor1_partition_class;

                    if (floor1_partition_class > maximum_class)
                        maximum_class = floor1_partition_class;
                }

                memset(floor1_class_dimensions_list, 0, sizeof(uint32_t)*(16+1));

                for (int j = 0; j <= maximum_class; j++) {
                    uint32_t class_dimensions_less1 = 0, class_subclasses = 0;

                    class_dimensions_less1 = get_bits(gb, 3);
                    put_bits(pb, 3, class_dimensions_less1);

                    floor1_class_dimensions_list[j] = class_dimensions_less1 + 1;

                    class_subclasses = get_bits(gb, 2);
                    put_bits(pb, 2, class_subclasses);

                    if (class_subclasses != 0) {
                        uint32_t masterbook = 0;

                        masterbook = get_bits(gb, 8);
                        put_bits(pb, 8, masterbook);

                        if (masterbook >= codebook_count) {
                            av_log(NULL, AV_LOG_ERROR, "invalid floor1 masterbook\n");
                            ret = AVERROR_INVALIDDATA;
                            goto fail;
                        }
                    }

                    for (int k = 0; k < (1U << class_subclasses); k++) {
                        uint32_t subclass_book_plus1 = 0;
                        int subclass_book = 0; /* this MUST be int */

                        subclass_book_plus1 = get_bits(gb, 8);
                        put_bits(pb, 8, subclass_book_plus1);

                        subclass_book = subclass_book_plus1 - 1;
                        if (subclass_book >= 0 && subclass_book >= codebook_count) {
                            av_log(NULL, AV_LOG_ERROR, "invalid floor1 subclass book\n");
                            ret = AVERROR_INVALIDDATA;
                            goto fail;
                        }
                    }
                }

                floor1_multiplier_less1 = get_bits(gb, 2);
                put_bits(pb, 2, floor1_multiplier_less1);

                rangebits = get_bits(gb, 4);
                put_bits(pb, 4, rangebits);

                for (int j = 0; j < floor1_partitions; j++) {
                    uint32_t current_class_number = 0;

                    current_class_number = floor1_partition_class_list[j];
                    for (int k = 0; k < floor1_class_dimensions_list[current_class_number]; k++) {
                        uint32_t X; /* max 4b (15) */

                        X = get_bits_long(gb, rangebits);
                        put_bits(pb, rangebits, X);
                    }
                }
            }

            residue_count_less1 = get_bits(gb, 6);
            put_bits(pb, 6, residue_count_less1);
            residue_count = residue_count_less1 + 1;

            for (int i = 0; i < residue_count; i++) {
                uint32_t residue_type = 0, residue_classifications = 0;
                uint32_t residue_begin = 0, residue_end = 0, residue_partition_size_less1 = 0, residue_classifications_less1 = 0, residue_classbook = 0;
                uint32_t residue_cascade[64+1]; /* 6b +1 */

                residue_type = get_bits(gb, 2);
                put_bits(pb, 16, residue_type); /* 2b to 16b */

                if (residue_type > 2) {
                    av_log(NULL, AV_LOG_ERROR, "invalid residue type\n");
                    ret = AVERROR_INVALIDDATA;
                    goto fail;
                }

                residue_begin = get_bits_long(gb, 24);
                put_bits(pb, 24, residue_begin);
                residue_end = get_bits_long(gb, 24);
                put_bits(pb, 24, residue_end);
                residue_partition_size_less1 = get_bits_long(gb, 24);
                put_bits(pb, 24, residue_partition_size_less1);
                residue_classifications_less1 = get_bits(gb, 6);
                put_bits(pb, 6, residue_classifications_less1);
                residue_classbook = get_bits(gb, 8);
                put_bits(pb, 8, residue_classbook);
                residue_classifications = residue_classifications_less1 + 1;

                if (residue_classbook >= codebook_count) {
                    av_log(NULL, AV_LOG_ERROR, "invalid residue classbook\n");
                    ret = AVERROR_INVALIDDATA;
                    goto fail;
                }

                memset(residue_cascade, 0, sizeof(uint32_t)*(64+1));

                for (int j = 0; j < residue_classifications; j++) {
                    uint32_t high_bits = 0, lobl_put = 0, bitflag = 0;

                    high_bits = 0;

                    lobl_put = get_bits(gb, 3);
                    put_bits(pb, 3, lobl_put);

                    bitflag = get_bits1(gb);
                    put_bits(pb, 1, bitflag);
                    if (bitflag) {
                        high_bits = get_bits(gb, 5);
                        put_bits(pb, 5, high_bits);
                    }

                    residue_cascade[j] = high_bits * 8 + lobl_put;
                }

                for (int j = 0; j < residue_classifications; j++) {
                    for (int k = 0; k < 8; k++) {
                        if (residue_cascade[j] & (1 << k)) {
                            uint32_t residue_book = 0;

                            residue_book = get_bits(gb, 8);
                            put_bits(pb, 8, residue_book);

                            if (residue_book >= codebook_count) {
                                av_log(NULL, AV_LOG_ERROR, "invalid residue book\n");
                                ret = AVERROR_INVALIDDATA;
                                goto fail;
                            }
                        }
                    }
                }
            }

            /* Mappings */
            mapping_count_less1 = get_bits(gb, 6);
            put_bits(pb, 6, mapping_count_less1);
            mapping_count = mapping_count_less1 + 1;

            for (int i = 0; i < mapping_count; i++) {
                uint32_t mapping_type, submaps_flag, submaps, square_polar_flag, mapping_reserved;

                mapping_type = 0;
                put_bits(pb, 16, mapping_type);

                submaps_flag = get_bits1(gb);
                put_bits(pb, 1, submaps_flag);

                submaps = 1;
                if (submaps_flag) {
                    uint32_t submaps_less1;

                    submaps_less1 = get_bits(gb, 4);
                    put_bits(pb, 4, submaps_less1);
                    submaps = submaps_less1 + 1;
                }

                square_polar_flag = get_bits1(gb);
                put_bits(pb, 1, square_polar_flag);

                if (square_polar_flag) {
                    uint32_t coupling_steps_less1, coupling_steps;

                    coupling_steps_less1 = get_bits(gb, 8);
                    put_bits(pb, 8, coupling_steps_less1);
                    coupling_steps = coupling_steps_less1 + 1;

                    for (int j = 0; j < coupling_steps; j++) {
                        uint32_t magnitude = 0, angle = 0;
                        int magnitude_bits = ilog(channels-1);
                        int angle_bits = ilog(channels-1);

                        magnitude = get_bits(gb, magnitude_bits);
                        put_bits(pb, magnitude_bits, magnitude);
                        angle = get_bits(gb, angle_bits);
                        put_bits(pb, angle_bits, angle);

                        if (angle == magnitude || magnitude >= channels || angle >= channels) {
                            av_log(NULL, AV_LOG_ERROR, "invalid coupling (angle=%i, mag=%i, ch=%i)\n", angle, magnitude,channels);
                            ret = AVERROR_INVALIDDATA;
                            goto fail;
                        }
                    }
                }

                // a rare reserved field not removed by Ak!
                mapping_reserved = get_bits(gb, 2);
                put_bits(pb, 2, mapping_reserved);
                if (0 != mapping_reserved) {
                    av_log(NULL, AV_LOG_ERROR, "mapping reserved field nonzero\n");
                    ret = AVERROR_INVALIDDATA;
                    goto fail;
                }

                if (submaps > 1) {
                    for (int j = 0; j < channels; j++) {
                        uint32_t mapping_mux;

                        mapping_mux = get_bits(gb, 4);
                        put_bits(pb, 4, mapping_mux);
                        if (mapping_mux >= submaps) {
                            av_log(NULL, AV_LOG_ERROR, "mapping_mux >= submaps\n");
                            ret = AVERROR_INVALIDDATA;
                            goto fail;
                        }
                    }
                }

                for (int j = 0; j < submaps; j++) {
                    uint32_t time_config, floor_number, residue_number;

                    time_config = get_bits(gb, 8);
                    put_bits(pb, 8, time_config);

                    floor_number = get_bits(gb, 8);
                    put_bits(pb, 8, floor_number);
                    if (floor_number >= floor_count) {
                        av_log(NULL, AV_LOG_ERROR, "invalid floor mapping\n");
                        ret = AVERROR_INVALIDDATA;
                        goto fail;
                    }

                    residue_number = get_bits(gb, 8);
                    put_bits(pb, 8, residue_number);
                    if (residue_number >= residue_count) {
                        av_log(NULL, AV_LOG_ERROR, "invalid residue mapping\n");
                        ret = AVERROR_INVALIDDATA;
                        goto fail;
                    }
                }
            }

            mode_count_less1 = get_bits(gb, 6);
            put_bits(pb, 6, mode_count_less1);
            mode_count = mode_count_less1 + 1;

            memset(s->mode_blockflag, 0, sizeof(uint8_t)*(64+1)); /* up to max mode_count */
            s->mode_bits = ilog(mode_count-1); /* for mod_packets */

            for (int i = 0; i < mode_count; i++) {
                uint32_t block_flag, windowtype, transformtype, mapping;

                block_flag = get_bits1(gb);
                put_bits(pb, 1, block_flag);

                s->mode_blockflag[i] = (block_flag != 0); /* for mod_packets */

                windowtype = 0;
                transformtype = 0;
                put_bits(pb, 16, windowtype);
                put_bits(pb, 16, transformtype);

                mapping = get_bits(gb, 8);
                put_bits(pb, 8, mapping);
                if (mapping >= mapping_count) {
                    av_log(NULL, AV_LOG_ERROR, "invalid mode mapping\n");
                    ret = AVERROR_INVALIDDATA;
                    goto fail;
                }
            }
        }

        put_bits(pb, 1, 1);

        flush_put_bits(pb);
        s->out_pkt->size = put_bytes_output(pb);

        s->setup_done = 1;
    } else {
        ret = ff_bsf_get_packet_ref(ctx, pkt);
        if (ret < 0)
            goto fail;

        ret = av_packet_copy_props(s->out_pkt, pkt);
        if (ret < 0)
            goto fail;

        if (1/* s->packet_type == WWV_MODIFIED */) {
            uint32_t packet_type, mode_number, remainder;

            init_put_bits(pb, s->out_pkt->data, s->out_pkt->size);

            ret = init_get_bits8(gb, pkt->data, pkt->size);
            if (ret < 0)
                goto fail;

            skip_bits_long(gb, 2*8);

            packet_type = 0;
            put_bits(pb, 1, packet_type);

            mode_number = get_bitsz(gb, s->mode_bits); /* max 6b */
            put_bits(pb, s->mode_bits, mode_number);

            remainder = get_bits(gb, 8-s->mode_bits);

            if (s->mode_blockflag[mode_number]) {
                uint32_t next_blockflag, prev_window_type, next_window_type;

                next_blockflag = 0;

                prev_window_type = s->prev_blockflag;
                put_bits(pb, 1, prev_window_type);

                next_window_type = next_blockflag;
                put_bits(pb, 1, next_window_type);
            }

            s->prev_blockflag = s->mode_blockflag[mode_number]; /* save for next packet */

            put_bits(pb, 8-s->mode_bits, remainder);

            for (int i = 0; i < pkt->size-1; i++)
                put_bits(pb, 8, get_bits(gb, 8));

            flush_put_bits(pb);
            s->out_pkt->size = put_bytes_output(pb);
        } else {
            memcpy(s->out_pkt->data, pkt->data, pkt->size);
            s->out_pkt->size = pkt->size;
        }
    }

    av_packet_unref(pkt);
    av_packet_move_ref(pkt, s->out_pkt);

    return 0;

fail:
    av_packet_unref(s->out_pkt);

    return ret;
}

static void flush(AVBSFContext *ctx)
{
    WwVorbisContext *s = ctx->priv_data;

    av_packet_unref(s->out_pkt);
    s->prev_blockflag = 0;
}

static void close(AVBSFContext *ctx)
{
    WwVorbisContext *s = ctx->priv_data;

    av_packet_free(&s->out_pkt);
}

const FFBitStreamFilter ff_wwvorbis_to_vorbis_bsf = {
    .p.name         = "wwvorbis_to_vorbis",
    .p.codec_ids    = (const enum AVCodecID []){ AV_CODEC_ID_WWVORBIS, AV_CODEC_ID_NONE },
    .priv_data_size = sizeof(WwVorbisContext),
    .init           = init,
    .filter         = filter,
    .flush          = flush,
    .close          = close,
};
