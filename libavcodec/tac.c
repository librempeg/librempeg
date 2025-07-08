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

#include "libavutil/channel_layout.h"
#include "libavutil/mem_internal.h"
#include "libavutil/thread.h"
#include "libavutil/tx.h"

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "bytestream.h"
#include "tac_data.h"

#define TAC_BLOCK_SIZE 0x4E000
#define TAC_FRAME_SAMPLES 1024
#define TAC_CHANNELS 2

#define TAC_CODED_BANDS     27
#define TAC_CODED_COEFS     32
#define TAC_TOTAL_POINTS    32
#define TAC_SCALE_TABLE_MAX_INDEX 511

typedef struct TACHeader {
    uint32_t huffman_offset;    /* setup */
    uint16_t loop_frame;        /* aligned to block start */
    uint16_t loop_discard;      /* discarded start samples in loop frame (lower = outputs more) */
    uint16_t frame_count;       /* number of valid frames ("block end" frames not included) */
    uint16_t frame_last;        /* valid samples in final frame - 1 (lower = outputs less, 0 = outputs 1), even for non-looped files */
    uint32_t loop_offset;       /* points to a block; file size if not looped */
    uint32_t file_size;         /* block aligned; actual file size can be a bit smaller if last block is truncated */
    uint32_t joint_stereo;      /* usually 0 and rarely 1 ("MSStereoMode") */
} TACHeader;

typedef struct TACFrameHeader {
    uint16_t frame_crc;
    uint16_t huff_flag;
    uint16_t frame_size;
    uint16_t frame_id;
    uint16_t huff_count;
    uint32_t huff_cfg;
} TACFrameHeader;

typedef struct TACContext {
    AVClass *class;
    TACHeader header;
    TACFrameHeader frame_header;
    AVCodecContext *avctx;
    int got_keyframe;

    int16_t huff_table_1[257];
    int16_t huff_table_2[TAC_CHANNELS][32]; /* saved between (some) frames */
    int16_t huff_table_3[258];
    uint8_t huff_table_4[16384];

    int16_t codes[TAC_CHANNELS][TAC_FRAME_SAMPLES];

    REG_VF spectrum[TAC_CHANNELS][TAC_FRAME_SAMPLES / 4]; /* temp huffman-to-coefs */
    REG_VF wave[TAC_CHANNELS][TAC_FRAME_SAMPLES / 4]; /* final samples, in vector form */
    REG_VF hist[TAC_CHANNELS][TAC_FRAME_SAMPLES / 4]; /* saved between frames */

    DECLARE_ALIGNED(32, REG_VF, tmp)[16+2];
    DECLARE_ALIGNED(32, REG_VF, in)[16+2];

    AVTXContext *tx[TAC_CHANNELS];
    av_tx_fn tx_fn[TAC_CHANNELS];
} TACContext;

static int init_huffman(AVCodecContext *avctx)
{
    TACContext *s = avctx->priv_data;
    const uint8_t *src = avctx->extradata+32;
    int left = avctx->extradata_size-32;
    unsigned idx;

    left -= 256;
    if (left <= 0)
        return AVERROR_INVALIDDATA;

    for (int i = 0; i < 256; i++) {
        int16_t n = *src++;

        if (n & 0x80) {
            n &= 0x7F;
            if (left <= 0)
                return AVERROR_INVALIDDATA;

            left--;
            n |= *src++ << 7;
        }

        s->huff_table_1[i] = n;
    }

    for (int i = 0; i < TAC_CHANNELS; i++)
        memset(s->huff_table_2[i], 0, sizeof(s->huff_table_2[i]));

    s->huff_table_1[256] = 1;
    s->huff_table_3[0] = 0;
    for (int i = 1, j = 0; i < 258; i++, j++)
        s->huff_table_3[i] = s->huff_table_3[j] + s->huff_table_1[j];

    for (idx = 0; !s->huff_table_1[idx]; idx++);

    for (int i = 0; i < 16383; i++) {
        if (idx+1 >= FF_ARRAY_ELEMS(s->huff_table_3))
            return AVERROR_INVALIDDATA;

        if (i >= s->huff_table_3[idx+1]) {
            while (!s->huff_table_1[++idx]) {
                ;
            }
        }

        s->huff_table_4[i] = idx;
    }

    return 0;
}

static int parse_extradata(AVCodecContext *avctx)
{
    const uint8_t *buf = avctx->extradata;
    TACContext *s = avctx->priv_data;
    TACHeader *h = &s->header;

    if (avctx->extradata_size < 32)
        return AVERROR_INVALIDDATA;

    h->huffman_offset  = AV_RL32(buf+0x00);
    h->loop_frame      = AV_RL16(buf+0x08);
    h->loop_discard    = AV_RL16(buf+0x0A);
    h->frame_count     = AV_RL16(buf+0x0C);
    h->frame_last      = AV_RL16(buf+0x0E);
    h->loop_offset     = AV_RL32(buf+0x10);
    h->file_size       = AV_RL32(buf+0x14);
    h->joint_stereo    = AV_RL32(buf+0x18);

    if (h->huffman_offset < 0x20 || h->huffman_offset > TAC_BLOCK_SIZE-256)
        return AVERROR_INVALIDDATA;

    return init_huffman(avctx);
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    TACContext *s = avctx->priv_data;
    int ret;

    av_channel_layout_uninit(&avctx->ch_layout);
    av_channel_layout_default(&avctx->ch_layout, TAC_CHANNELS);
    avctx->sample_fmt = AV_SAMPLE_FMT_FLTP;
    avctx->sample_rate = 48000;

    for (int ch = 0; ch < TAC_CHANNELS; ch++) {
        float scale = 1.f;

        ret = av_tx_init(&s->tx[ch], &s->tx_fn[ch], AV_TX_FLOAT_MDCT, 1, 32, &scale, AV_TX_FULL_IMDCT);
        if (ret < 0)
            return ret;
    }

    return parse_extradata(avctx);
}

static int parse_frame_header(AVCodecContext *avctx, GetByteContext *gb)
{
    TACContext *s = avctx->priv_data;
    TACFrameHeader *h = &s->frame_header;

    if (bytestream2_peek_le32(gb) == 0xFFFFFFFFu) {
        bytestream2_skip(gb, 4);
        while (!bytestream2_peek_byte(gb)) {
            bytestream2_skip(gb, 1);
            if (bytestream2_get_bytes_left(gb) <= 0)
                break;
        }
    }

    if (bytestream2_get_bytes_left(gb) <= 0)
        return 1;

    h->frame_crc = bytestream2_get_le16(gb);
    h->huff_flag = bytestream2_peek_le16(gb) >> 15;
    h->frame_size = bytestream2_get_le16(gb) & 0x7FFF;
    h->frame_id = bytestream2_get_le16(gb);
    h->huff_count = bytestream2_get_le16(gb);
    h->huff_cfg = bytestream2_get_be32(gb);

    if (bytestream2_tell(gb) + 8 + h->frame_size > TAC_BLOCK_SIZE)
        return AVERROR_INVALIDDATA;

    return 0;
}

static int read_codes(AVCodecContext *avctx, GetByteContext *gb,
                      uint16_t huff_flag, uint32_t huff_cfg)
{
    TACContext *s = avctx->priv_data;
    int huff_count = 0;
    uint32_t unkA = 0;
    uint32_t unkB = huff_cfg;
    uint32_t unkC = 0;
    uint32_t unkD = 0xFFFFFFFF;
    uint16_t *huff_table_1 = s->huff_table_1;
    uint16_t *huff_table_3 = s->huff_table_3;
    uint8_t *huff_table_4 = s->huff_table_4;

    for (int ch = 0; ch < TAC_CHANNELS; ch++) {
        uint16_t *huff_table_2 = s->huff_table_2[ch];
        int huff_done = 0;
        int huff_todo = 28;
        int16_t huff_val = 0;

        for (; huff_done < huff_todo; huff_done++) {
            int offset;

            unkD = unkD >> 14;
            if (unkD == 0)
                return AVERROR_INVALIDDATA;
            offset = (unkB - unkC) / unkD;
            if (offset < 0 || offset >= FF_ARRAY_ELEMS(s->huff_table_4))
                return AVERROR_INVALIDDATA;

            unkA = huff_table_4[offset];
            unkC += huff_table_3[unkA] * unkD;
            unkD *= huff_table_1[unkA];

            while (0xFFFFFF >= (unkC ^ (unkC + unkD))) {
                if (bytestream2_get_bytes_left(gb) <= 0)
                    break;
                unkB = (unkB << 8) | bytestream2_get_byteu(gb);
                unkD = (unkD << 8);
                unkC = (unkC << 8);
            }

            while (0xFFFF >= unkD) {
                if (bytestream2_get_bytes_left(gb) <= 0)
                    break;
                unkD = (((~unkC) + 1) & 0xFFFF) << 8;
                unkB = (unkB << 8) | bytestream2_get_byteu(gb);
                unkC = (unkC << 8);
            }

            if (unkA >= 0xFE) {
                uint32_t unkT;
                unkT = unkA == 0xFE;
                unkD = unkD >> (unkT ? 8 : 13);
                if (unkD == 0)
                    return AVERROR_INVALIDDATA;
                unkA = (unkB - unkC) / unkD;
                unkC = unkC + (unkA * unkD);
                if (unkT)
                    unkA += 0xFE;

                while (0xFFFFFF >= (unkC ^ (unkC + unkD))) {
                    if (bytestream2_get_bytes_left(gb) <= 0)
                        break;
                    unkB = (unkB << 8) | bytestream2_get_byteu(gb);
                    unkD = (unkD << 8);
                    unkC = (unkC << 8);
                }

                while (0xFFFF >= unkD) {
                    if (bytestream2_get_bytes_left(gb) <= 0)
                        break;
                    unkD = (((~unkC) + 1) & 0xFFFF) << 8;
                    unkB = (unkB << 8) | bytestream2_get_byteu(gb);
                    unkC = (unkC << 8);
                }
            }

            if (unkA & 1) {
                huff_val = -((((int16_t)unkA) + 1) / 2);
            } else {
                huff_val = unkA / 2;
            }

            if (huff_done < 28) {
                if (huff_flag)
                    huff_val += huff_table_2[huff_done];

                huff_table_2[huff_done] = huff_val;

                if (huff_done != 0 && (huff_val << 16) != 0)
                    huff_todo += 32;
            }

            s->codes[ch][huff_done] = huff_val;
        }

        huff_count += huff_done;
    }

    return huff_count;
}

static const int _xyzw = 0xF;
static const int _x_z_ = 0xA;
static const int _x___ = 0x8;
static const int __yzw = 0x7;
static const int __y_w = 0x5;
static const int __y__ = 0x4;
static const int ___z_ = 0x2;
static const int ____w = 0x1;

static REG_VF VECTOR_ZERO = { .f = {0.0f, 0.0f, 0.0f, 0.0f} };
static REG_VF VECTOR_ONE = { .f = {1.0f, 1.0f, 1.0f, 1.0f} };

static inline void MADD(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fd->f.x + (fs->f.x * ft->f.x);
    if (dest & __y__) fd->f.y = fd->f.y + (fs->f.y * ft->f.y);
    if (dest & ___z_) fd->f.z = fd->f.z + (fs->f.z * ft->f.z);
    if (dest & ____w) fd->f.w = fd->f.w + (fs->f.w * ft->f.w);
}

static inline void MADDx(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fd->f.x + (fs->f.x * ft->f.x);
    if (dest & __y__) fd->f.y = fd->f.y + (fs->f.y * ft->f.x);
    if (dest & ___z_) fd->f.z = fd->f.z + (fs->f.z * ft->f.x);
    if (dest & ____w) fd->f.w = fd->f.w + (fs->f.w * ft->f.x);
}

static inline void MADDy(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fd->f.x + (fs->f.x * ft->f.y);
    if (dest & __y__) fd->f.y = fd->f.y + (fs->f.y * ft->f.y);
    if (dest & ___z_) fd->f.z = fd->f.z + (fs->f.z * ft->f.y);
    if (dest & ____w) fd->f.w = fd->f.w + (fs->f.w * ft->f.y);
}

static inline void MADDw(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fd->f.x + (fs->f.x * ft->f.w);
    if (dest & __y__) fd->f.y = fd->f.y + (fs->f.y * ft->f.w);
    if (dest & ___z_) fd->f.z = fd->f.z + (fs->f.z * ft->f.w);
    if (dest & ____w) fd->f.w = fd->f.w + (fs->f.w * ft->f.w);
}

static inline void MADDz(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fd->f.x + (fs->f.x * ft->f.z);
    if (dest & __y__) fd->f.y = fd->f.y + (fs->f.y * ft->f.z);
    if (dest & ___z_) fd->f.z = fd->f.z + (fs->f.z * ft->f.z);
    if (dest & ____w) fd->f.w = fd->f.w + (fs->f.w * ft->f.z);
}

static inline void STORE(uint8_t dest, REG_VF* dst, const REG_VF *fs, int pos)
{
    if (dest & _x___) dst[pos].f.x = fs->f.x;
    if (dest & __y__) dst[pos].f.y = fs->f.y;
    if (dest & ___z_) dst[pos].f.z = fs->f.z;
    if (dest & ____w) dst[pos].f.w = fs->f.w;
}

static inline void MUL(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fs->f.x * ft->f.x;
    if (dest & __y__) fd->f.y = fs->f.y * ft->f.y;
    if (dest & ___z_) fd->f.z = fs->f.z * ft->f.z;
    if (dest & ____w) fd->f.w = fs->f.w * ft->f.w;
}

static inline void MULx(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fs->f.x * ft->f.x;
    if (dest & __y__) fd->f.y = fs->f.y * ft->f.x;
    if (dest & ___z_) fd->f.z = fs->f.z * ft->f.x;
    if (dest & ____w) fd->f.w = fs->f.w * ft->f.x;
}

static inline void MULy(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fs->f.x * ft->f.y;
    if (dest & __y__) fd->f.y = fs->f.y * ft->f.y;
    if (dest & ___z_) fd->f.z = fs->f.z * ft->f.y;
    if (dest & ____w) fd->f.w = fs->f.w * ft->f.y;
}

static inline void MULw(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fs->f.x * ft->f.w;
    if (dest & __y__) fd->f.y = fs->f.y * ft->f.w;
    if (dest & ___z_) fd->f.z = fs->f.z * ft->f.w;
    if (dest & ____w) fd->f.w = fs->f.w * ft->f.w;
}

static inline void MULz(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fs->f.x * ft->f.z;
    if (dest & __y__) fd->f.y = fs->f.y * ft->f.z;
    if (dest & ___z_) fd->f.z = fs->f.z * ft->f.z;
    if (dest & ____w) fd->f.w = fs->f.w * ft->f.z;
}

static inline void COPY(uint8_t dest, REG_VF *fd, const int16_t* buf)
{
    if (dest & _x___) fd->f.x = buf[0];
    if (dest & __y__) fd->f.y = buf[1];
    if (dest & ___z_) fd->f.z = buf[2];
    if (dest & ____w) fd->f.w = buf[3];
}

static inline void ABS(uint8_t dest, REG_VF *ft, const REG_VF *fs)
{
    if (dest & _x___) ft->f.x = fabsf(fs->f.x);
    if (dest & __y__) ft->f.y = fabsf(fs->f.y);
    if (dest & ___z_) ft->f.z = fabsf(fs->f.z);
    if (dest & ____w) ft->f.w = fabsf(fs->f.w);
}

static inline void FMUL(uint8_t dest, REG_VF *fd, const REG_VF *fs, const float I_F)
{
    if (dest & _x___) fd->f.x = fs->f.x * I_F;
    if (dest & __y__) fd->f.y = fs->f.y * I_F;
    if (dest & ___z_) fd->f.z = fs->f.z * I_F;
    if (dest & ____w) fd->f.w = fs->f.w * I_F;
}

static inline void FMULf(uint8_t dest, REG_VF *fd, const float fs)
{
    if (dest & _x___) fd->f.x = fd->f.x * fs;
    if (dest & __y__) fd->f.y = fd->f.y * fs;
    if (dest & ___z_) fd->f.z = fd->f.z * fs;
    if (dest & ____w) fd->f.w = fd->f.w * fs;
}

static inline void ADD(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fs->f.x + ft->f.x;
    if (dest & __y__) fd->f.y = fs->f.y + ft->f.y;
    if (dest & ___z_) fd->f.z = fs->f.z + ft->f.z;
    if (dest & ____w) fd->f.w = fs->f.w + ft->f.w;
}

static inline void ADDx(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fs->f.x + ft->f.x;
    if (dest & __y__) fd->f.y = fs->f.y + ft->f.x;
    if (dest & ___z_) fd->f.z = fs->f.z + ft->f.x;
    if (dest & ____w) fd->f.w = fs->f.w + ft->f.x;
}

static inline void ADDy(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fs->f.x + ft->f.y;
    if (dest & __y__) fd->f.y = fs->f.y + ft->f.y;
    if (dest & ___z_) fd->f.z = fs->f.z + ft->f.y;
    if (dest & ____w) fd->f.w = fs->f.w + ft->f.y;
}

static inline void ADDw(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fs->f.x + ft->f.w;
    if (dest & __y__) fd->f.y = fs->f.y + ft->f.w;
    if (dest & ___z_) fd->f.z = fs->f.z + ft->f.w;
    if (dest & ____w) fd->f.w = fs->f.w + ft->f.w;
}

static inline void ADDz(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fs->f.x + ft->f.z;
    if (dest & __y__) fd->f.y = fs->f.y + ft->f.z;
    if (dest & ___z_) fd->f.z = fs->f.z + ft->f.z;
    if (dest & ____w) fd->f.w = fs->f.w + ft->f.z;
}

static inline void SUB(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fs->f.x - ft->f.x;
    if (dest & __y__) fd->f.y = fs->f.y - ft->f.y;
    if (dest & ___z_) fd->f.z = fs->f.z - ft->f.z;
    if (dest & ____w) fd->f.w = fs->f.w - ft->f.w;
}

static inline void FTOI0(uint8_t dest, REG_VF *ft, const REG_VF *fs)
{
    if (dest & _x___) ft->i.x = (int32_t)fs->f.x;
    if (dest & __y__) ft->i.y = (int32_t)fs->f.y;
    if (dest & ___z_) ft->i.z = (int32_t)fs->f.z;
    if (dest & ____w) ft->i.w = (int32_t)fs->f.w;
}

static inline void ITOF0(uint8_t dest, REG_VF *ft, const REG_VF *fs)
{
    if (dest & _x___) ft->f.x = (float)fs->i.x;
    if (dest & __y__) ft->f.y = (float)fs->i.y;
    if (dest & ___z_) ft->f.z = (float)fs->i.z;
    if (dest & ____w) ft->f.w = (float)fs->i.w;
}

static inline void MOVE(uint8_t dest, REG_VF *fd, const REG_VF *fs)
{
    if (dest & _x___) fd->f.x = fs->f.x;
    if (dest & __y__) fd->f.y = fs->f.y;
    if (dest & ___z_) fd->f.z = fs->f.z;
    if (dest & ____w) fd->f.w = fs->f.w;
}

static inline void MOVEx(uint8_t dest, REG_VF *fd, const REG_VF *fs)
{
    if (dest & _x___) fd->f.x = fs->f.x;
    if (dest & __y__) fd->f.y = fs->f.x;
    if (dest & ___z_) fd->f.z = fs->f.x;
    if (dest & ____w) fd->f.w = fs->f.x;
}

static inline void SIGN(uint8_t dest, REG_VF *fd, const REG_VF *fs)
{
    if (dest & _x___) if (fs->f.x < 0) fd->f.x = -fd->f.x;
    if (dest & __y__) if (fs->f.y < 0) fd->f.y = -fd->f.y;
    if (dest & ___z_) if (fs->f.z < 0) fd->f.z = -fd->f.z;
    if (dest & ____w) if (fs->f.w < 0) fd->f.w = -fd->f.w;
}

static inline void LOAD(uint8_t dest, REG_VF *ft, REG_VF* src, int pos)
{
    if (dest & _x___) ft->f.x = src[pos].f.x;
    if (dest & __y__) ft->f.y = src[pos].f.y;
    if (dest & ___z_) ft->f.z = src[pos].f.z;
    if (dest & ____w) ft->f.w = src[pos].f.w;
}

static inline void MSUBx(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fd->f.x - (fs->f.x * ft->f.x);
    if (dest & __y__) fd->f.y = fd->f.y - (fs->f.y * ft->f.x);
    if (dest & ___z_) fd->f.z = fd->f.z - (fs->f.z * ft->f.x);
    if (dest & ____w) fd->f.w = fd->f.w - (fs->f.w * ft->f.x);
}

static inline void MSUBy(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fd->f.x - (fs->f.x * ft->f.y);
    if (dest & __y__) fd->f.y = fd->f.y - (fs->f.y * ft->f.y);
    if (dest & ___z_) fd->f.z = fd->f.z - (fs->f.z * ft->f.y);
    if (dest & ____w) fd->f.w = fd->f.w - (fs->f.w * ft->f.y);
}

static inline void MSUBz(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fd->f.x - (fs->f.x * ft->f.z);
    if (dest & __y__) fd->f.y = fd->f.y - (fs->f.y * ft->f.z);
    if (dest & ___z_) fd->f.z = fd->f.z - (fs->f.z * ft->f.z);
    if (dest & ____w) fd->f.w = fd->f.w - (fs->f.w * ft->f.z);
}

static inline void MSUBw(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    if (dest & _x___) fd->f.x = fd->f.x - (fs->f.x * ft->f.w);
    if (dest & __y__) fd->f.y = fd->f.y - (fs->f.y * ft->f.w);
    if (dest & ___z_) fd->f.z = fd->f.z - (fs->f.z * ft->f.w);
    if (dest & ____w) fd->f.w = fd->f.w - (fs->f.w * ft->f.w);
}

static inline void MR32(uint8_t dest, REG_VF *ft, const REG_VF *fs)
{
    float x = fs->f.x;
    if (dest & _x___) ft->f.x = fs->f.y;
    if (dest & __y__) ft->f.y = fs->f.z;
    if (dest & ___z_) ft->f.z = fs->f.w;
    if (dest & ____w) ft->f.w = x;
}

static inline void DIV(uint8_t dest, REG_VF *fd, const REG_VF *fs, const REG_VF *ft)
{
    fd->f.x = fs->f.x / ft->f.x;
    fd->f.y = fs->f.y / ft->f.y;
    fd->f.z = fs->f.z / ft->f.z;
    fd->f.w = fs->f.w / ft->f.w;
}

static void unpack_code4(REG_VF* spectrum, const REG_VF* spc1, const REG_VF* spc2, const REG_VF* code, const REG_VF* idx, int out_pos)
{
    const REG_VF *ST = SCALE_TABLE;
    REG_VF tbc1, tbc2, out;

    /* copy table coefs .N, unless huffman code was 0 */
    if (code->f.x != 0) {
        MOVEx(_x___, &tbc1, &ST[idx->i.x + 0]);
        MOVEx(_x___, &tbc2, &ST[idx->i.x + 1]);
    } else {
        MOVEx(_x___, &tbc1, &VECTOR_ZERO);
        MOVEx(_x___, &tbc2, &VECTOR_ZERO);
    }

    if (code->f.y != 0) {
        MOVEx(__y__, &tbc1, &ST[idx->i.y + 0]);
        MOVEx(__y__, &tbc2, &ST[idx->i.y + 1]);
    } else {
        MOVEx(__y__, &tbc1, &VECTOR_ZERO);
        MOVEx(__y__, &tbc2, &VECTOR_ZERO);
    }

    if (code->f.z != 0) {
        MOVEx(___z_, &tbc1, &ST[idx->i.z + 0]);
        MOVEx(___z_, &tbc2, &ST[idx->i.z + 1]);
    } else {
        MOVEx(___z_, &tbc1, &VECTOR_ZERO);
        MOVEx(___z_, &tbc2, &VECTOR_ZERO);
    }

    if (code->f.w != 0) {
        MOVEx(____w, &tbc1, &ST[idx->i.w + 0]);
        MOVEx(____w, &tbc2, &ST[idx->i.w + 1]);
    } else {
        MOVEx(____w, &tbc1, &VECTOR_ZERO);
        MOVEx(____w, &tbc2, &VECTOR_ZERO);
    }

    /* out = [signed] (scp1/scp2) * (tbc2 - tbc1) + tbc1 */
    DIV  (_xyzw, &out, spc1, spc2);
    SUB  (_xyzw, &tbc2, &tbc2, &tbc1);
    MUL  (_xyzw, &out, &out, &tbc2);
    ADD  (_xyzw, &out, &out, &tbc1);
    SIGN (_xyzw, &out, code);

    STORE(_xyzw, spectrum, &out, out_pos);
}

static void unpack_band(REG_VF *spectrum, const int16_t *codes, int band_pos, int *code_pos, int out_pos)
{
    const REG_VF *ST = SCALE_TABLE;
    int i;
    int16_t base_index = codes[0]; /* table index, max ~35 */
    int16_t band_index = codes[band_pos]; /* table too */
    REG_VF scale;

    /* bad values should be caught by CRC check but for completeness */
    base_index = av_clip(base_index, 0, TAC_SCALE_TABLE_MAX_INDEX);
    band_index = av_clip(band_index, 0, TAC_SCALE_TABLE_MAX_INDEX-128);

    /* index zero = band is not coded and all of its coefs are 0 */
    if (band_index == 0) {
        for (i = 0; i < (TAC_CODED_COEFS / 4); i++) {
            STORE(_xyzw, spectrum, &VECTOR_ZERO, out_pos+i);
        }
        return;
    }

    /* put final band scale at .y */
    MULy (__y__, &scale, &ST[128 + band_index], &ST[base_index]);

    /* unpack coefs */
    for (i = 0; i < 8; i++) {
        REG_VF code, idx, tm01, tm02, tm03;
        REG_VF spc1, spc2;

        COPY (_xyzw, &code, &codes[(*code_pos)]);
        (*code_pos) += 4;

        /* scale coef then round down to int to get table indexes (!!!) */
        ABS  (_xyzw, &tm01, &code);
        MULy (_xyzw, &tm01, &tm01, &scale);
        FMUL (_xyzw, &tm02, &tm01, 512.0); /* 512 = SCALE_TABLE max */
        ADD  (_xyzw, &tm03, &tm02, &VECTOR_ONE);

        FTOI0(_xyzw, &idx, &tm02); /* keep idx as int for later (probably could use (int)f.N too) */
        ITOF0(_xyzw, &tm02, &idx);
        FMULf(_xyzw, &tm02, 0.00195313);

        FTOI0(_xyzw, &tm03, &tm03);
        ITOF0(_xyzw, &tm03, &tm03);
        FMULf(_xyzw, &tm03, 0.00195313);

        SUB  (_xyzw, &spc1, &tm01, &tm02);
        SUB  (_xyzw, &spc2, &tm03, &tm02);

        /* Also just in case. In rare cases index may access 511+1 but table takes this into account */
        idx.i.x = av_clip(idx.i.x, 0, TAC_SCALE_TABLE_MAX_INDEX);
        idx.i.y = av_clip(idx.i.y, 0, TAC_SCALE_TABLE_MAX_INDEX);
        idx.i.z = av_clip(idx.i.z, 0, TAC_SCALE_TABLE_MAX_INDEX);
        idx.i.w = av_clip(idx.i.w, 0, TAC_SCALE_TABLE_MAX_INDEX);

        unpack_code4(spectrum, &spc1, &spc2, &code, &idx, out_pos + i);
    }
}

static void unpack_antialias(REG_VF* spectrum)
{
    const REG_VF* AT = ANTIALIASING_TABLE;
    int pos_lo = 0x7;
    int pos_hi = 0x8;

    for (int i = 0; i < TAC_CODED_BANDS; i++) {
        for (int j = 0; j < 4; j++) {
            REG_VF lo_in, hi_in, lo_out, hi_out;

            LOAD (_xyzw, &lo_in, spectrum, pos_lo - j);
            LOAD (_xyzw, &hi_in, spectrum, pos_hi + j);

            MULx (____w, &lo_out, &lo_in,     &AT[0x0+j]);
            MSUBx(____w, &lo_out, &AT[0xF-j], &hi_in);
            MULw (_x___, &hi_out, &hi_in,     &AT[0x7-j]);
            MADDw(_x___, &hi_out, &AT[0x8+j], &lo_in);

            MULy (___z_, &lo_out, &lo_in,     &AT[0x0+j]);
            MSUBy(___z_, &lo_out, &AT[0xF-j], &hi_in);
            MULz (__y__, &hi_out, &hi_in,     &AT[0x7-j]);
            MADDz(__y__, &hi_out, &AT[0x8+j], &lo_in);

            MULz (__y__, &lo_out, &lo_in,     &AT[0x0+j]);
            MSUBz(__y__, &lo_out, &AT[0xF-j], &hi_in);
            MULy (___z_, &hi_out, &hi_in,     &AT[0x7-j]);
            MADDy(___z_, &hi_out, &AT[0x8+j], &lo_in);

            MULw (_x___, &lo_out, &lo_in,  &AT[0x0+j]);
            MSUBw(_x___, &lo_out, &AT[0xF-j], &hi_in);
            MULx (____w, &hi_out, &hi_in,  &AT[0x7-j]);
            MADDx(____w, &hi_out, &AT[0x8+j], &lo_in);

            STORE(_xyzw, spectrum, &lo_out, pos_lo - j);
            STORE(_xyzw, spectrum, &hi_out, pos_hi + j);
        }

        pos_lo += 0x8;
        pos_hi += 0x8;
    }
}

static void unpack_channel(REG_VF *spectrum, const int16_t *codes)
{
    /* Huffman codes has 1 base scale + 27 bands scales + N coefs (up to 27*32).
     * Not all bands store codes so an index is needed, after scales */
    int code_pos = TAC_CODED_BANDS + 1;
    int out_pos = 0x00;

    /* unpack bands */
    for (int i = 1; i < TAC_CODED_BANDS + 1; i++) {
        unpack_band(spectrum, codes, i, &code_pos, out_pos);
        out_pos += (TAC_CODED_COEFS / 4); /* 8 vectors of 4 coefs, per band */
    }

    /* memset rest up to max (27*32/4)..(32*32/4) */
    for (int i = 0xD8; i < 0x100; i++)
       STORE (_xyzw, spectrum, &VECTOR_ZERO, i);

    unpack_antialias(spectrum);
}

static void transform_dot_product(REG_VF *mac, const REG_VF *spectrum, const REG_VF *TT, int pos_i, int pos_t)
{
    MUL  (_xyzw, mac, &spectrum[pos_i+0], &TT[pos_t+0]);
    MADD (_xyzw, mac, &spectrum[pos_i+1], &TT[pos_t+1]);
    MADD (_xyzw, mac, &spectrum[pos_i+2], &TT[pos_t+2]);
    MADD (_xyzw, mac, &spectrum[pos_i+3], &TT[pos_t+3]);
    MADD (_xyzw, mac, &spectrum[pos_i+4], &TT[pos_t+4]);
    MADD (_xyzw, mac, &spectrum[pos_i+5], &TT[pos_t+5]);
    MADD (_xyzw, mac, &spectrum[pos_i+6], &TT[pos_t+6]);
    MADD (_xyzw, mac, &spectrum[pos_i+7], &TT[pos_t+7]);
}

static void transform(REG_VF *wave, const REG_VF *spectrum)
{
    const REG_VF *TT = TRANSFORM_TABLE;
    int pos_t = 0;
    int pos_o = 0;

    for (int i = 0; i < TAC_TOTAL_POINTS; i++) {
        int pos_i = 0;
        REG_VF mac, ror, out;

        for (int j = 0; j < 8; j++) {
            transform_dot_product(&mac, spectrum, TT, pos_i, pos_t);
            pos_i += 8;
            MR32 (_xyzw, &ror, &mac);
            ADD  (_x_z_, &ror, &ror, &mac);
            ADDz (_x___, &out, &ror, &ror);

            transform_dot_product(&mac, spectrum, TT, pos_i, pos_t);
            pos_i += 8;
            MR32 (_xyzw, &ror, &mac);
            ADD  (__y_w, &ror, &ror, &mac);
            ADDw (__y__, &out, &ror, &ror);

            transform_dot_product(&mac, spectrum, TT, pos_i, pos_t);
            pos_i += 8;
            MR32 (_xyzw, &ror, &mac);
            ADD  (_x_z_, &ror, &ror, &mac);
            ADDx (___z_, &out, &ror, &ror);

            transform_dot_product(&mac, spectrum, TT, pos_i, pos_t);
            pos_i += 8;
            MR32 (_xyzw, &ror, &mac);
            ADD  (__y_w, &ror, &ror, &mac);
            ADDy (____w, &out, &ror, &ror);

            FMULf(_xyzw, &out, 0.25);
            STORE(_xyzw, wave, &out, pos_o++);
        }

        pos_t += 0x08;
    }
}

static void process(AVCodecContext *avctx, REG_VF *wave, REG_VF *hist,
                    const int ch)
{
    TACContext *s = avctx->priv_data;
    int pos_o = 0;
    int pos_w = 0;
    int pos_h;
    int pos_r = 0x200; /* rolls down to 0, becoming 0x10 steps (0x00, 0xF0, 0xE0, ..., 0x00, 0xF0, ...) */

    for (int i = 0; i < TAC_TOTAL_POINTS; i++) {
        REG_VF *tmp = s->tmp;
        REG_VF *in = s->in;

        pos_h = pos_r & 0xFF;
        pos_r = pos_r - 0x10;

        memcpy(in, wave+pos_w, sizeof(float) * 32);
        s->tx_fn[ch](s->tx[ch], tmp, in, sizeof(float));

        pos_w += 8;

        STORE(_xyzw, hist, tmp + 0x0, pos_h + 0x0);
        STORE(_xyzw, hist, tmp + 0x1, pos_h + 0x1);
        STORE(_xyzw, hist, tmp + 0x2, pos_h + 0x2);
        STORE(_xyzw, hist, tmp + 0x3, pos_h + 0x3);
        STORE(_xyzw, hist, tmp + 0x4, pos_h + 0x4);
        STORE(_xyzw, hist, tmp + 0x5, pos_h + 0x5);
        STORE(_xyzw, hist, tmp + 0x6, pos_h + 0x6);
        STORE(_xyzw, hist, tmp + 0x7, pos_h + 0x7);
        STORE(_xyzw, hist, tmp + 0x8, pos_h + 0x8);
        STORE(_xyzw, hist, tmp + 0x9, pos_h + 0x9);
        STORE(_xyzw, hist, tmp + 0xA, pos_h + 0xA);
        STORE(_xyzw, hist, tmp + 0xB, pos_h + 0xB);
        STORE(_xyzw, hist, tmp + 0xC, pos_h + 0xC);
        STORE(_xyzw, hist, tmp + 0xD, pos_h + 0xD);
        STORE(_xyzw, hist, tmp + 0xE, pos_h + 0xE);
        STORE(_xyzw, hist, tmp + 0xF, pos_h + 0xF);

        /* hist/window overlap and update final wave */
        for (int j = 0; j < 8; j++) {
            const REG_VF *WT = WINDOW_TABLE;
            REG_VF out;

            MUL  (_xyzw, &out, &hist[(pos_h + 0x00) & 0xFF], &WT[0x00+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0x18) & 0xFF], &WT[0x08+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0x20) & 0xFF], &WT[0x10+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0x38) & 0xFF], &WT[0x18+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0x40) & 0xFF], &WT[0x20+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0x58) & 0xFF], &WT[0x28+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0x60) & 0xFF], &WT[0x30+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0x78) & 0xFF], &WT[0x38+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0x80) & 0xFF], &WT[0x40+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0x98) & 0xFF], &WT[0x48+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0xA0) & 0xFF], &WT[0x50+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0xB8) & 0xFF], &WT[0x58+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0xC0) & 0xFF], &WT[0x60+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0xD8) & 0xFF], &WT[0x68+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0xE0) & 0xFF], &WT[0x70+j]);
            MADD (_xyzw, &out, &hist[(pos_h + 0xF8) & 0xFF], &WT[0x78+j]);

            pos_h++;

            STORE(_xyzw, wave, &out, pos_o++);
        }
    }
}

static void decode_vu1(AVCodecContext *avctx)
{
    TACContext *s = avctx->priv_data;

    for (int ch = 0; ch < TAC_CHANNELS; ch++) {
        unpack_channel(s->spectrum[ch], s->codes[ch]);

        transform(s->wave[ch], s->spectrum[ch]);

        process(avctx, s->wave[ch], s->hist[ch], ch);
    }
}

static void finalize_output(AVCodecContext *avctx, AVFrame *frame)
{
    TACContext *s = avctx->priv_data;

    if (s->header.joint_stereo) {
        REG_VF* wave_l = s->wave[0];
        REG_VF* wave_r = s->wave[1];

        /* Combine joint stereo channels that encode diffs in L/R ("MS stereo"). In pseudo-mono files R has */
        /* all samples as 0 (R only saves 28 huffman codes, signalling no coefs per 1+27 bands) */
        for (int i = 0; i < TAC_TOTAL_POINTS * 8; i++) {
            REG_VF samples_l, samples_r;

            ADD  (_xyzw, &samples_l, &wave_l[i], &wave_r[i]);
            SUB  (_xyzw, &samples_r, &wave_l[i], &wave_r[i]);
            MOVE (_xyzw, &wave_l[i], &samples_l);
            MOVE (_xyzw, &wave_r[i], &samples_r);
        }
    }

    for (int ch = 0; ch < TAC_CHANNELS; ch++)
        memcpy(frame->extended_data[ch], s->wave[ch], TAC_FRAME_SAMPLES * sizeof(float));
}

#define CRC16_INIT     0xFFFF
#define CRC16_POLY     0x1021
#define CRC16_XOR_OUT  0xFFFF

static uint16_t crc16(const uint8_t *data, int length)
{
    uint16_t i, crc = CRC16_INIT;

    while (length--) {
        for (crc ^= *data++ << 8, i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? crc << 1 ^ CRC16_POLY : crc << 1;
        }
    }

    return crc ^ CRC16_XOR_OUT;
}

static int decode_frame(AVCodecContext *avctx, AVFrame *frame,
                        int *got_frame_ptr, AVPacket *avpkt)
{
    TACContext *s = avctx->priv_data;
    GetByteContext gbc;
    GetByteContext *gb = &gbc;
    int ret;

    bytestream2_init(gb, avpkt->data, avpkt->size);
    ret = parse_frame_header(avctx, gb);
    if (ret < 0)
        return ret;
    if (ret == 1)
        return avpkt->size;

    if (!s->got_keyframe)
        s->got_keyframe = !s->frame_header.huff_flag;
    if (!s->got_keyframe)
        return avpkt->size;

    if (avctx->err_recognition & AV_EF_CRCCHECK) {
        if (crc16(avpkt->data + bytestream2_tell(gb) - 8, s->frame_header.frame_size + 4) != s->frame_header.frame_crc) {
            if (avctx->err_recognition & AV_EF_EXPLODE)
                return AVERROR_INVALIDDATA;
        }
    }

    ret = read_codes(avctx, gb,
                     s->frame_header.huff_flag,
                     s->frame_header.huff_cfg);
    if (ret != s->frame_header.huff_count)
        return AVERROR_INVALIDDATA;

    frame->nb_samples = TAC_FRAME_SAMPLES;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    decode_vu1(avctx);
    finalize_output(avctx, frame);

    if (s->frame_header.huff_flag)
        frame->flags &= ~AV_FRAME_FLAG_KEY;
    else
        frame->flags |= AV_FRAME_FLAG_KEY;

    *got_frame_ptr = 1;

    return avpkt->size;
}

static av_cold void decode_flush(AVCodecContext *avctx)
{
    TACContext *s = avctx->priv_data;

    s->got_keyframe = 0;
    for (int ch = 0; ch < TAC_CHANNELS; ch++) {
        memset(s->huff_table_2[ch], 0, sizeof(s->huff_table_2[ch]));
        memset(s->hist[ch], 0, sizeof(s->hist[ch]));
    }
}

static av_cold int decode_end(AVCodecContext *avctx)
{
    TACContext * s = avctx->priv_data;

    for (int ch = 0; ch < TAC_CHANNELS; ch++)
        av_tx_uninit(&s->tx[ch]);

    return 0;
}

const FFCodec ff_tac_decoder = {
    .p.name         = "tac",
    CODEC_LONG_NAME("tri-Ace audio"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_TAC,
    .priv_data_size = sizeof(TACContext),
    .init           = decode_init,
    FF_CODEC_DECODE_CB(decode_frame),
    .flush          = decode_flush,
    .close          = decode_end,
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_CHANNEL_CONF,
};
