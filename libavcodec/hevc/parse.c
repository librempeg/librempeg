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

#include "bytestream.h"
#include "h2645_parse.h"
#include "hevc.h"
#include "parse.h"

static int hevc_decode_nal_units(const uint8_t *buf, int buf_size, HEVCParamSets *ps,
                                 HEVCSEI *sei, int is_nalff, int nal_length_size,
                                 int err_recognition, int apply_defdispwin, void *logctx)
{
    int i;
    int ret = 0;
    int flags = (H2645_FLAG_IS_NALFF * !!is_nalff) | H2645_FLAG_SMALL_PADDING;
    H2645Packet pkt = { 0 };

    ret = ff_h2645_packet_split(&pkt, buf, buf_size, logctx,
                                nal_length_size, AV_CODEC_ID_HEVC, flags);
    if (ret < 0) {
        goto done;
    }

    for (i = 0; i < pkt.nb_nals; i++) {
        H2645NAL *nal = &pkt.nals[i];
        /* ignore everything except parameter sets and VCL NALUs */
        switch (nal->type) {
        case HEVC_NAL_VPS:
            ret = ff_hevc_decode_nal_vps(&nal->gb, logctx, ps);
            if (ret < 0)
                goto done;
            break;
        case HEVC_NAL_SPS:
            ret = ff_hevc_decode_nal_sps(&nal->gb, logctx, ps,
                                         nal->nuh_layer_id, apply_defdispwin);
            if (ret < 0)
                goto done;
            break;
        case HEVC_NAL_PPS:
            ret = ff_hevc_decode_nal_pps(&nal->gb, logctx, ps);
            if (ret < 0)
                goto done;
            break;
        case HEVC_NAL_SEI_PREFIX:
        case HEVC_NAL_SEI_SUFFIX:
            ret = ff_hevc_decode_nal_sei(&nal->gb, logctx, sei, ps, nal->type);
            if (ret < 0)
                goto done;
            break;
        default:
            av_log(logctx, AV_LOG_VERBOSE, "Ignoring NAL type %d in extradata\n", nal->type);
            break;
        }
    }

done:
    ff_h2645_packet_uninit(&pkt);
    if (err_recognition & AV_EF_EXPLODE)
        return ret;

    return 0;
}

int ff_hevc_decode_extradata(const uint8_t *data, int size, HEVCParamSets *ps,
                             HEVCSEI *sei, int *is_nalff, int *nal_length_size,
                             int err_recognition, int apply_defdispwin, void *logctx)
{
    int ret = 0;
    GetByteContext gb;

    bytestream2_init(&gb, data, size);

    /* data[0] == 1 is configurationVersion from 14496-15.
     * data[0] == 0 is for backward compatibility predates the standard.
     *
     * Minimum number of bytes of hvcC with 0 numOfArrays is 23.
     */
    if (size >= 23 && ((data[0] == 1) || (data[0] == 0 && (data[1] || data[2] > 1)))) {
        /* It seems the extradata is encoded as hvcC format. */
        int i, j, num_arrays, nal_len_size;

        *is_nalff = 1;

        bytestream2_skip(&gb, 21);
        nal_len_size = (bytestream2_get_byte(&gb) & 3) + 1;
        num_arrays   = bytestream2_get_byte(&gb);

        /* nal units in the hvcC always have length coded with 2 bytes,
         * so put a fake nal_length_size = 2 while parsing them */
        *nal_length_size = 2;

        /* Decode nal units from hvcC. */
        for (i = 0; i < num_arrays; i++) {
            int type = bytestream2_get_byte(&gb) & 0x3f;
            int cnt  = bytestream2_get_be16(&gb);

            for (j = 0; j < cnt; j++) {
                // +2 for the nal size field
                int nalsize = bytestream2_peek_be16(&gb) + 2;
                if (bytestream2_get_bytes_left(&gb) < nalsize) {
                    av_log(logctx, AV_LOG_ERROR,
                           "Invalid NAL unit size in extradata.\n");
                    return AVERROR_INVALIDDATA;
                }

                ret = hevc_decode_nal_units(gb.buffer, nalsize, ps, sei, *is_nalff,
                                            *nal_length_size, err_recognition, apply_defdispwin,
                                            logctx);
                if (ret < 0) {
                    av_log(logctx, AV_LOG_ERROR,
                           "Decoding nal unit %d %d from hvcC failed\n",
                           type, i);
                    return ret;
                }
                bytestream2_skip(&gb, nalsize);
            }
        }

        /* Now store right nal length size, that will be used to parse
         * all other nals */
        *nal_length_size = nal_len_size;
    } else {
        *is_nalff = 0;
        ret = hevc_decode_nal_units(data, size, ps, sei, *is_nalff, *nal_length_size,
                                    err_recognition, apply_defdispwin, logctx);
        if (ret < 0)
            return ret;
    }

    return ret;
}
