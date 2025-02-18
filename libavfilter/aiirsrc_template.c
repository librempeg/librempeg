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

#include "avfilter.h"
#include "audio.h"

#undef itype
#undef ftype
#undef SAMPLE_FORMAT
#if DEPTH == 16
#define ftype float
#define itype int16_t
#define SAMPLE_FORMAT s16p
#elif DEPTH == 32
#define ftype double
#define itype int32_t
#define SAMPLE_FORMAT s32p
#elif DEPTH == 33
#define ftype float
#define itype float
#define SAMPLE_FORMAT fltp
#elif DEPTH == 65
#define ftype double
#define itype double
#define SAMPLE_FORMAT dblp
#endif

#define F(x) ((ftype)(x))

#define fn3(a,b)   a##_##b
#define fn2(a,b)   fn3(a,b)
#define fn(a)      fn2(a, SAMPLE_FORMAT)

typedef struct fn(StateContext) {
    ftype *coefs_arr;
    ftype *stage_arr;
} fn(StateContext);

static void fn(src_uninit)(AVFilterContext *ctx)
{
    AudioIIRSRCContext *s = ctx->priv;
    const int channels = s->channels;
    fn(StateContext) *state;

    if (s->state) {
        state = s->state;
        for (int ch = 0; ch < channels; ch++) {
            fn(StateContext) *stc = &state[ch];

            av_freep(&stc->coefs_arr);
            av_freep(&stc->stage_arr);
        }

        av_freep(&s->state);
    }
}

static int fn(src_init)(AVFilterContext *ctx)
{
    const int channels = ctx->inputs[0]->ch_layout.nb_channels;
    AudioIIRSRCContext *s = ctx->priv;
    fn(StateContext) *state;

    s->channels = channels;
    s->state = av_calloc(channels, sizeof(*state));
    if (!s->state)
        return AVERROR(ENOMEM);
    state = s->state;

    for (int ch = 0; ch < channels; ch++) {
        fn(StateContext) *stc = &state[ch];

        stc->coefs_arr = av_calloc(s->nbr_coefs, sizeof(*stc->coefs_arr));
        if (!stc->coefs_arr)
            return AVERROR(ENOMEM);

        stc->stage_arr = av_calloc(s->nbr_coefs + 2, sizeof(*stc->stage_arr));
        if (!stc->stage_arr)
            return AVERROR(ENOMEM);

        for (int i = 0; i < s->nbr_coefs; i++)
            stc->coefs_arr[i] = s->coefs[i];
    }

    return 0;
}

static void fn(process_sample_pos1)(const int nbr_coefs, ftype *spl_0, ftype *spl_1,
                                    ftype *stage_arr, const ftype *coefs_arr)
{
    const int cnt = nbr_coefs + 2 - 1;

    ftype tmp_0 = spl_0[0];
    tmp_0 -= stage_arr[cnt    ];
    tmp_0 *= coefs_arr[cnt - 2];
    tmp_0 += stage_arr[cnt - 2];

    stage_arr[cnt - 2] = spl_0[0];
    stage_arr[cnt - 1] = spl_1[0];
    stage_arr[cnt    ] = tmp_0;

    spl_0[0] = tmp_0;
}

static void fn(process_sample_pos0)(const int nbr_coefs, ftype *spl_0, ftype *spl_1,
                                    ftype *stage_arr)
{
    const int cnt = nbr_coefs + 2;

    stage_arr[cnt - 2] = spl_0[0];
    stage_arr[cnt - 1] = spl_1[0];
}

static void fn(process_sample_pos)(const int stage, const int nbr_coefs,
                                   ftype *spl_0, ftype *spl_1, ftype *stage_arr,
                                   const ftype *coefs_arr)
{
    const int cnt = nbr_coefs + 2 - stage;

    ftype tmp_0 = spl_0[0];
    tmp_0 -= stage_arr[cnt    ];
    tmp_0 *= coefs_arr[cnt - 2];
    tmp_0 += stage_arr[cnt - 2];

    ftype tmp_1 = spl_1[0];
    tmp_1 -= stage_arr[cnt + 1];
    tmp_1 *= coefs_arr[cnt - 1];
    tmp_1 += stage_arr[cnt - 1];

    stage_arr[cnt - 2] = spl_0[0];
    stage_arr[cnt - 1] = spl_1[0];

    spl_0[0] = tmp_0;
    spl_1[0] = tmp_1;
}

#undef NBR
#define NBR 1
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 2
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 3
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 4
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 5
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 6
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 7
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 8
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 9
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 10
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 11
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 12
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 13
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 14
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 15
#include "aiirsrc_template_nbr.c"

#undef NBR
#define NBR 16
#include "aiirsrc_template_nbr.c"
