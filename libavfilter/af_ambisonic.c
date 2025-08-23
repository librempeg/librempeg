/*
 * Copyright (c) 2022 Paul B Mahol
 * Copyright (c) 2017 Sanchit Sinha
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

#include <float.h>
#include <math.h>
#include <stdio.h>

#include "libavutil/channel_layout.h"
#include "libavutil/float_dsp.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/avassert.h"
#include "audio.h"
#include "avfilter.h"
#include "formats.h"

#define EVEN 0
#define ODD 1
#define MAX_ORDER 3
#define SQR(x) ((x) * (x))
#define MAX_CHANNELS SQR(MAX_ORDER + 1)

enum A_NAME {
    A_W, A_Y, A_Z, A_X, A_V, A_T, A_R, A_S, A_U, A_Q, A_O, A_M, A_K, A_L, A_N, A_P,
};

enum NearFieldType {
    NF_AUTO = -1,
    NF_NONE,
    NF_IN,
    NF_OUT,
    NB_NFTYPES,
};

enum PrecisionType {
    P_AUTO = -1,
    P_SINGLE,
    P_DOUBLE,
    NB_PTYPES,
};

enum PTypes {
    PT_AMP,
    PT_RMS,
    PT_ENERGY,
    PT_NBTYPES,
};

enum NormType {
    N3D,
    SN3D,
    FUMA,
    NB_NTYPES,
};

enum DirectionType {
    D_X,
    D_Y,
    D_Z,
    D_C,
    NB_DTYPES,
};

enum SequenceType {
    M_ACN,
    M_FUMA,
    M_SID,
    NB_MTYPES,
};

enum Layouts {
    MONO,
    STEREO,
    STEREO_DOWNMIX,
    SURROUND,
    L2_1,
    TRIANGLE,
    QUAD,
    SQUARE,
    L4_0,
    L5_0,
    L5_0_SIDE,
    L6_0,
    L7_0,
    TETRA,
    CUBE,
    NB_LAYOUTS,
};

enum Rotation { YAW, PITCH, ROLL };

typedef struct SOSSection {
    double b[3];
    double a[2];
    double z[2];
} SOSSection;

typedef struct NearField {
    SOSSection sos[MAX_ORDER];
} NearField;

typedef struct Xover {
    double b[3];
    double a[3];
    double w[2];
} Xover;

static const double gains_2d[][4] =
{
    { 1 },
    { 1, 0.707107 },
    { 1, 0.866025, 0.5 },
    { 1, 0.92388, 0.707107, 0.382683 },
};

static const double gains_3d[][4] =
{
    { 1 },
    { 1, 0.57735027 },
    { 1, 0.774597, 0.4 },
    { 1, 0.861136, 0.612334, 0.304747 },
};

static const double same_distance[] =
{
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
};

static const double cube_azimuth[] =
{
    315, 45, 135, 225, 315, 45, 135, 225,
};

static const double cube_elevation[] =
{
     35.26439,  35.26439,  35.26439,  35.26439,
    -35.26439, -35.26439, -35.26439, -35.26439
};

static const struct {
    const int              order;
    const int              inputs;
    const int              speakers;
    const int              near_field;
    const int              type;
    const double           xover;
    const AVChannelLayout  outlayout;
    const double          *speakers_azimuth;
    const double          *speakers_elevation;
    const double          *speakers_distance;
} ambisonic_tab[] = {
    [MONO] = {
        .order = 0,
        .inputs = 1,
        .speakers = 1,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = (AVChannelLayout)AV_CHANNEL_LAYOUT_MONO,
        .speakers_azimuth = (const double[1]){ 0. },
        .speakers_distance = (const double[1]){ 1. },
    },
    [STEREO] = {
        .order = 1,
        .inputs = 4,
        .speakers = 2,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO,
        .speakers_azimuth = (const double[2]){ -30, 30},
        .speakers_distance = same_distance,
    },
    [STEREO_DOWNMIX] = {
        .order = 1,
        .inputs = 4,
        .speakers = 2,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO_DOWNMIX,
        .speakers_azimuth = (const double[2]){ -90, 90 },
        .speakers_distance = same_distance,
    },
    [SURROUND] = {
        .order = 1,
        .inputs = 4,
        .speakers = 3,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = (AVChannelLayout)AV_CHANNEL_LAYOUT_SURROUND,
        .speakers_azimuth = (const double[3]){ -45, 45, 0 },
        .speakers_distance = same_distance,
    },
    [L2_1] = {
        .order = 1,
        .inputs = 4,
        .speakers = 3,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = (AVChannelLayout)AV_CHANNEL_LAYOUT_2_1,
        .speakers_azimuth = (const double[3]){ -45, 45, 180 },
        .speakers_distance = same_distance,
    },
    [TRIANGLE] = {
        .order = 1,
        .inputs = 4,
        .speakers = 3,
        .type = 1,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = AV_CHANNEL_LAYOUT_MASK(3, AV_CH_FRONT_CENTER|
                                               AV_CH_BACK_LEFT|
                                               AV_CH_BACK_RIGHT),
        .speakers_azimuth = (const double[3]){ 0, -120, 120 },
        .speakers_distance = same_distance,
    },
    [QUAD] = {
        .order = 1,
        .inputs = 4,
        .speakers = 4,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = (AVChannelLayout)AV_CHANNEL_LAYOUT_QUAD,
        .speakers_azimuth = (const double[4]){ -45, 45, -135, 135 },
        .speakers_distance = same_distance,
    },
    [SQUARE] = {
        .order = 1,
        .inputs = 4,
        .speakers = 4,
        .type = 1,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = AV_CHANNEL_LAYOUT_MASK(4, AV_CH_FRONT_CENTER|
                                               AV_CH_BACK_CENTER|
                                               AV_CH_SIDE_LEFT|
                                               AV_CH_SIDE_RIGHT),
        .speakers_azimuth = (const double[4]){ 0, -90, 180, 90 },
        .speakers_distance = same_distance,
    },
    [L4_0] = {
        .order = 1,
        .inputs = 4,
        .speakers = 4,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = (AVChannelLayout)AV_CHANNEL_LAYOUT_4POINT0,
        .speakers_azimuth = (const double[4]){ -30, 30, 0, 180 },
        .speakers_distance = same_distance,
    },
    [L5_0] = {
        .order = 1,
        .inputs = 4,
        .speakers = 5,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = (AVChannelLayout)AV_CHANNEL_LAYOUT_5POINT0_BACK,
        .speakers_azimuth = (const double[5]){ -30, 30, 0, -145, 145 },
        .speakers_distance = same_distance,
    },
    [L5_0_SIDE] = {
        .order = 1,
        .inputs = 4,
        .speakers = 5,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = (AVChannelLayout)AV_CHANNEL_LAYOUT_5POINT0,
        .speakers_azimuth = (const double[5]){ -30, 30, 0, -110, 110 },
        .speakers_distance = same_distance,
    },
    [L6_0] = {
        .order = 1,
        .inputs = 4,
        .speakers = 6,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = (AVChannelLayout)AV_CHANNEL_LAYOUT_6POINT0,
        .speakers_azimuth = (const double[6]){ -30, 30, 0, 180, -110, 110 },
        .speakers_distance = same_distance,
    },
    [L7_0] = {
        .order = 1,
        .inputs = 4,
        .speakers = 7,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = (AVChannelLayout)AV_CHANNEL_LAYOUT_7POINT0,
        .speakers_azimuth = (const double[7]){ -30, 30, 0, -145, 145, -110, 110 },
        .speakers_distance = same_distance,
    },
    [TETRA] = {
        .order = 1,
        .inputs = 4,
        .speakers = 4,
        .type = 2,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = AV_CHANNEL_LAYOUT_MASK(4, AV_CH_SIDE_LEFT|
                                               AV_CH_SIDE_RIGHT|
                                               AV_CH_TOP_CENTER|
                                               AV_CH_BOTTOM_FRONT_CENTER),
        .speakers_azimuth = (const double[4]){ -90, 90, 0, 180 },
        .speakers_elevation = (const double[4]){ -35.3, -35.3, 35.3, 35.3 },
        .speakers_distance = same_distance,
    },
    [CUBE] = {
        .order = 1,
        .inputs = 4,
        .speakers = 8,
        .type = 2,
        .near_field = NF_NONE,
        .xover = 0.,
        .outlayout = (AVChannelLayout)AV_CHANNEL_LAYOUT_CUBE,
        .speakers_azimuth = cube_azimuth,
        .speakers_elevation = cube_elevation,
        .speakers_distance = same_distance,
    },
};

typedef struct AmbisonicContext {
    const AVClass *class;
    int order;                    /* Order of ambisonic */
    int level;                    /* Output Level compensation */
    enum Layouts layout;          /* Output speaker layout */
    enum NormType scaling_norm;   /* Normalization Type */
    enum PrecisionType precision; /* Processing Precision Type */
    enum SequenceType seq;        /* Input Channel sequence type */
    enum NearFieldType near_field; /* Near Field compensation type */

    int invert[NB_DTYPES];        /* Axis Odd/Even Invert */
    double gain[2][NB_DTYPES];    /* Axis Odd/Even Gains */
    double pgains[2][MAX_ORDER+1];/* LF/HF perceptual gains */

    double rotate[3]; /* Angles for yaw(x), pitch(y), roll(z) rotation */

    double distance;
    double proximity;

    int pgtype;
    int max_channels;             /* Max Channels */
    int matrix_norm;
    double matching;

    double temp;
    double xover_freq;
    double xover_ratio;

    Xover xover[2][MAX_CHANNELS];
    NearField nf[2][MAX_CHANNELS];

    int    seq_tab[NB_MTYPES][MAX_CHANNELS];
    int    seq_map[MAX_CHANNELS];
    double norm_tab[NB_NTYPES][MAX_CHANNELS];
    double rotate_mat[MAX_CHANNELS][MAX_CHANNELS];
    double dominance_mat[4][4];
    double direction_mat[4][4];
    double zoom_mat[4][4];
    double focus_mat[4][4];
    double push_mat[4][4];
    double press_mat[4][4];
    double asymmetry_mat[4][4];
    double mirror_mat[MAX_CHANNELS];
    double transform_mat[MAX_CHANNELS][MAX_CHANNELS];
    double decode_mat[MAX_CHANNELS][MAX_CHANNELS];
    double norm_decode_mat[MAX_CHANNELS][MAX_CHANNELS];
    double u[MAX_CHANNELS][MAX_CHANNELS];
    double v[MAX_CHANNELS][MAX_CHANNELS];
    double w[MAX_CHANNELS];
    double level_tab[MAX_CHANNELS];
    double gains_tab[2][MAX_CHANNELS];
    double dominance[4];
    double direction[4];
    double zoom[4];
    double focus[4];
    double push[4];
    double press[4];
    double asymmetry;

    AVFrame *sframe;
    AVFrame *rframe;
    AVFrame *frame2;

    void (*nf_init[MAX_ORDER])(NearField *nf, double r0, double r1,
                               double speed, double rate,
                               double gain);
    void (*nf_process[MAX_ORDER])(NearField *nf,
                                  AVFrame *frame,
                                  int ch, int add,
                                  double gain);
    void (*process)(AVFilterContext *ctx, AVFrame *in, AVFrame *out);

    AVFloatDSPContext *fdsp;
} AmbisonicContext;

#define OFFSET(x) offsetof(AmbisonicContext,x)
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define AFT AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM

static const AVOption ambisonic_options[] = {
    { "layout", "layout of output", OFFSET(layout), AV_OPT_TYPE_INT, {.i64=STEREO}, 0, NB_LAYOUTS-1, AF , "lyt"},
    {   "mono",   "mono layout",   0, AV_OPT_TYPE_CONST, {.i64=MONO},   0, 0, AF , "lyt"},
    {   "stereo", "stereo layout", 0, AV_OPT_TYPE_CONST, {.i64=STEREO}, 0, 0, AF , "lyt"},
    {   "downmix","stereo downmix", 0, AV_OPT_TYPE_CONST, {.i64=STEREO_DOWNMIX}, 0, 0, AF , "lyt"},
    {   "3.0",    "3.0 layout",    0, AV_OPT_TYPE_CONST, {.i64=SURROUND}, 0, 0, AF , "lyt"},
    {   "3.0(back)","3.0(back) layout", 0, AV_OPT_TYPE_CONST, {.i64=L2_1}, 0, 0, AF , "lyt"},
    {   "triangle","triangle layout", 0, AV_OPT_TYPE_CONST, {.i64=TRIANGLE}, 0, 0, AF , "lyt"},
    {   "quad",   "quad layout",   0, AV_OPT_TYPE_CONST, {.i64=QUAD},   0, 0, AF , "lyt"},
    {   "square", "square layout", 0, AV_OPT_TYPE_CONST, {.i64=SQUARE}, 0, 0, AF , "lyt"},
    {   "4.0",    "4.0 layout",    0, AV_OPT_TYPE_CONST, {.i64=L4_0},   0, 0, AF , "lyt"},
    {   "5.0",    "5.0 layout",    0, AV_OPT_TYPE_CONST, {.i64=L5_0},   0, 0, AF , "lyt"},
    {   "5.0(side)", "5.0(side) layout", 0, AV_OPT_TYPE_CONST, {.i64=L5_0_SIDE}, 0, 0, AF , "lyt"},
    {   "6.0",    "6.0 layout",    0, AV_OPT_TYPE_CONST, {.i64=L6_0},   0, 0, AF , "lyt"},
    {   "7.0",    "7.0 layout",    0, AV_OPT_TYPE_CONST, {.i64=L7_0},   0, 0, AF , "lyt"},
    {   "tetra", "tetrahedron layout", 0, AV_OPT_TYPE_CONST, {.i64=TETRA},   0, 0, AF , "lyt"},
    {   "cube",   "cube layout",   0, AV_OPT_TYPE_CONST, {.i64=CUBE},   0, 0, AF , "lyt"},
    { "sequence", "input channel sequence", OFFSET(seq), AV_OPT_TYPE_INT, {.i64=M_ACN},  0, NB_MTYPES-1, AF, "seq"},
    {   "acn",  "ACN",  0, AV_OPT_TYPE_CONST, {.i64=M_ACN},  0, 0, AF, "seq"},
    {   "fuma", "FuMa", 0, AV_OPT_TYPE_CONST, {.i64=M_FUMA}, 0, 0, AF, "seq"},
    {   "sid",  "SID",  0, AV_OPT_TYPE_CONST, {.i64=M_SID},  0, 0, AF, "seq"},
    { "scaling", "input scaling format", OFFSET(scaling_norm), AV_OPT_TYPE_INT,   {.i64=SN3D}, 0, NB_NTYPES-1, AF, "scl"},
    {   "n3d",  "N3D scaling (normalised)",       0, AV_OPT_TYPE_CONST, {.i64=N3D},  0, 0, AF, "scl"},
    {   "sn3d", "SN3D scaling (semi-normalised)", 0, AV_OPT_TYPE_CONST, {.i64=SN3D}, 0, 0, AF, "scl"},
    {   "fuma", "furse malham scaling",           0, AV_OPT_TYPE_CONST, {.i64=FUMA}, 0, 0, AF, "scl"},
    { "nearfield", "near-field compensation", OFFSET(near_field), AV_OPT_TYPE_INT, {.i64=NF_AUTO}, NF_AUTO, NB_NFTYPES-1, AF, "nf"},
    {   "auto", "auto", 0, AV_OPT_TYPE_CONST, {.i64=NF_AUTO}, 0, 0, AF, "nf"},
    {   "none", "none", 0, AV_OPT_TYPE_CONST, {.i64=NF_NONE}, 0, 0, AF, "nf"},
    {   "in",   "in",   0, AV_OPT_TYPE_CONST, {.i64=NF_IN},   0, 0, AF, "nf"},
    {   "out",  "out",  0, AV_OPT_TYPE_CONST, {.i64=NF_OUT},  0, 0, AF, "nf"},
    { "matching", "set matching for decode matrix", OFFSET(matching), AV_OPT_TYPE_DOUBLE, {.dbl=0}, 0., 1., AF, "matching" },
    {   "mode",   "set exact mode matching",  0, AV_OPT_TYPE_CONST, {.dbl=0}, 0, 0, AF, "matching" },
    {   "energy", "set even energy matching", 0, AV_OPT_TYPE_CONST, {.dbl=1}, 0, 0, AF, "matching" },
    { "xoverfreq", "cross-over frequency", OFFSET(xover_freq), AV_OPT_TYPE_DOUBLE, {.dbl=-1.}, -1., 800., AF },
    { "xoverratio", "cross-over HF/LF ratio", OFFSET(xover_ratio), AV_OPT_TYPE_DOUBLE, {.dbl=0.}, -30., 30., AF },
    { "pgtype", "set the perceptual LF/HF gains type", OFFSET(pgtype), AV_OPT_TYPE_INT, {.i64=PT_RMS}, 0, PT_NBTYPES-1, AF, "pgt" },
    {   "amplitude", NULL, 0, AV_OPT_TYPE_CONST, {.i64=PT_AMP},    0, 0, AF, "pgt" },
    {   "rms",       NULL, 0, AV_OPT_TYPE_CONST, {.i64=PT_RMS},    0, 0, AF, "pgt" },
    {   "energy",    NULL, 0, AV_OPT_TYPE_CONST, {.i64=PT_ENERGY}, 0, 0, AF, "pgt" },
    { "precision", "processing precision", OFFSET(precision), AV_OPT_TYPE_INT, {.i64=P_AUTO}, P_AUTO, NB_PTYPES-1, AF, "pre"},
    {   "auto",   "auto",                             0, AV_OPT_TYPE_CONST, {.i64=P_AUTO}, 0, 0, AF, "pre"},
    {   "float",  "single floating-point precision",  0, AV_OPT_TYPE_CONST, {.i64=P_SINGLE}, 0, 0, AF, "pre"},
    {   "double", "double floating-point precision" , 0, AV_OPT_TYPE_CONST, {.i64=P_DOUBLE}, 0, 0, AF, "pre"},
    { "temp", "set the temperature °C", OFFSET(temp), AV_OPT_TYPE_DOUBLE, {.dbl=20.}, -50., 50., AF },
    { "yaw",    "angle for yaw (x-axis)",   OFFSET(rotate[YAW]),   AV_OPT_TYPE_DOUBLE, {.dbl=0.}, -180., 180., AFT },
    { "pitch",  "angle for pitch (y-axis)", OFFSET(rotate[PITCH]), AV_OPT_TYPE_DOUBLE, {.dbl=0.}, -180., 180., AFT },
    { "roll",   "angle for roll (z-axis)",  OFFSET(rotate[ROLL]),  AV_OPT_TYPE_DOUBLE, {.dbl=0.}, -180., 180., AFT },
    { "level",  "output level compensation", OFFSET(level), AV_OPT_TYPE_BOOL, {.i64=1}, 0, 1, AFT },
    { "norm",   "enable matrix normalization", OFFSET(matrix_norm), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, AFT },
    { "distance",  "set distance",  OFFSET(distance),  AV_OPT_TYPE_DOUBLE, {.dbl=1.0}, 0.1, 33, AF },
    { "proximity", "set proximity", OFFSET(proximity), AV_OPT_TYPE_DOUBLE, {.dbl=1.0}, 0.1, 33, AF },
    { "invert_x", "invert X", OFFSET(invert[D_X]), AV_OPT_TYPE_FLAGS, {.i64=0}, 0, 3, AFT, "ix"},
    {   "odd",  "invert odd harmonics",  0, AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, AFT, "ix"},
    {   "even", "invert even harmonics", 0, AV_OPT_TYPE_CONST, {.i64=2}, 0, 0, AFT, "ix"},
    { "invert_y", "invert Y", OFFSET(invert[D_Y]), AV_OPT_TYPE_FLAGS, {.i64=0}, 0, 3, AFT, "iy"},
    {   "odd",  "invert odd harmonics",  0, AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, AFT, "iy"},
    {   "even", "invert even harmonics", 0, AV_OPT_TYPE_CONST, {.i64=2}, 0, 0, AFT, "iy"},
    { "invert_z", "invert Z", OFFSET(invert[D_Z]), AV_OPT_TYPE_FLAGS, {.i64=0}, 0, 3, AFT, "iz"},
    {   "odd",  "invert odd harmonics",  0, AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, AFT, "iz"},
    {   "even", "invert even harmonics", 0, AV_OPT_TYPE_CONST, {.i64=2}, 0, 0, AFT, "iz"},
    { "invert_c", "circular invert", OFFSET(invert[D_C]), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, AFT},
    { "x_odd",  "X odd harmonics gain",  OFFSET(gain[ODD][D_X]),  AV_OPT_TYPE_DOUBLE, {.dbl=1.}, 0, 2., AFT },
    { "x_even", "X even harmonics gain", OFFSET(gain[EVEN][D_X]), AV_OPT_TYPE_DOUBLE, {.dbl=1.}, 0, 2., AFT },
    { "y_odd",  "Y odd harmonics gain",  OFFSET(gain[ODD][D_Y]),  AV_OPT_TYPE_DOUBLE, {.dbl=1.}, 0, 2., AFT },
    { "y_even", "Y even harmonics gain", OFFSET(gain[EVEN][D_Y]), AV_OPT_TYPE_DOUBLE, {.dbl=1.}, 0, 2., AFT },
    { "z_odd",  "Z odd harmonics gain",  OFFSET(gain[ODD][D_Z]),  AV_OPT_TYPE_DOUBLE, {.dbl=1.}, 0, 2., AFT },
    { "z_even", "Z even harmonics gain", OFFSET(gain[EVEN][D_Z]), AV_OPT_TYPE_DOUBLE, {.dbl=1.}, 0, 2., AFT },
    { "c_gain", "set the circular gain",     OFFSET(gain[0][D_C]),    AV_OPT_TYPE_DOUBLE, {.dbl=1.}, 0, 2., AFT },
    { "f_dom",  "set the forward dominance", OFFSET(dominance[A_X]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-12,12., AFT },
    { "s_dom",  "set the side dominance",    OFFSET(dominance[A_Y]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-12,12., AFT },
    { "v_dom",  "set the vertical dominance",OFFSET(dominance[A_Z]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-12,12., AFT },
    { "o_dir",  "set the origin soundfield directivity", OFFSET(direction[A_W]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},0,90., AFT },
    { "x_dir",  "set the X-axis soundfield directivity", OFFSET(direction[A_X]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},0,90., AFT },
    { "y_dir",  "set the Y-axis soundfield directivity", OFFSET(direction[A_Y]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},0,90., AFT },
    { "z_dir",  "set the Z-axis soundfield directivity", OFFSET(direction[A_Z]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},0,90., AFT },
    { "x_zoom", "set the X-axis soundfield zoom", OFFSET(zoom[A_X]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    { "y_zoom", "set the Y-axis soundfield zoom", OFFSET(zoom[A_Y]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    { "z_zoom", "set the Z-axis soundfield zoom", OFFSET(zoom[A_Z]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    { "x_focus", "set the X-axis soundfield focus", OFFSET(focus[A_X]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    { "y_focus", "set the Y-axis soundfield focus", OFFSET(focus[A_Y]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    { "z_focus", "set the Z-axis soundfield focus", OFFSET(focus[A_Z]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    { "x_push", "set the X-axis soundfield push", OFFSET(push[A_X]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    { "y_push", "set the Y-axis soundfield push", OFFSET(push[A_Y]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    { "z_push", "set the Z-axis soundfield push", OFFSET(push[A_Z]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    { "x_press", "set the X-axis soundfield press", OFFSET(press[A_X]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    { "y_press", "set the Y-axis soundfield press", OFFSET(press[A_Y]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    { "z_press", "set the Z-axis soundfield press", OFFSET(press[A_Z]), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    { "asymmetry", "set the soundfield asymmetry", OFFSET(asymmetry), AV_OPT_TYPE_DOUBLE, {.dbl=0.},-90,90., AFT },
    {NULL}
};

static double db2a(double db)
{
    return pow(10., db / 20.);
}

static double pythag(double a, double b)
{
    double absa = fabs(a);
    double absb = fabs(b);

    if (absa > absb) {
        return absa * sqrt(1.0+SQR(absb/absa));
    } else {
        if (absb == 0.0)
            return 0.0;
        else
            return absb * sqrt(1.0+SQR(absa/absb));
    }
}

static void mstep(int m, int n, double h, int l, int i,
                  double u[MAX_CHANNELS][MAX_CHANNELS])
{
    for (int j = l; j < n; j++) {
        double s = 0.0, f;

        for (int k = i; k < m; k++)
            s += u[k][i] * u[k][j];
        f = s / h;
        for (int k = i; k < m; k++)
            u[k][j] += f * u[k][i];
    }
}

#define LOG_MATRIX(name, mat, m, n) \
{ \
    av_log(ctx, AV_LOG_DEBUG, name " matrix:\n"); \
    for (int i = 0; i < m; i++) { \
        for (int j = 0; j < n; j++) \
            av_log(ctx, AV_LOG_DEBUG, "\t%g", mat[i][j]); \
        av_log(ctx, AV_LOG_DEBUG, "\n"); \
    } \
}

static void svdcmp(AVFilterContext *ctx,
                   double u[MAX_CHANNELS][MAX_CHANNELS],
                   int m, int n,
                   double *q, double v[MAX_CHANNELS][MAX_CHANNELS])
{
    double e[MAX_CHANNELS] = { 0. };
    double g, x, s, f, h, z, c, y;
    double eps = 1e-15;
    const double tol = 1e-64 / eps;
    const int itmax = 50;
    int l, l1;

    av_assert0(1.0 + eps > 1.0);
    av_assert0(tol > 0.0);

    g = 0.0;
    x = 0.0;

    for (int i = 0; i < n; i++) {
        s = 0.0;

        e[i] = g;
        l = i + 1;
        for (int j = i; j < m; j++)
            s += SQR(u[j][i]);
        if (s <= tol) {
            g = 0.0;
        } else {
            f = u[i][i];
            g = f < 0.0 ? sqrt(s) : -sqrt(s);
            h = f * g - s;
            u[i][i] = f - g;
            mstep(m, n, h, l, i, u);
        }

        q[i] = g;
        s = 0.0;
        for (int j = l; j < n; j++)
            s += u[i][j]*u[i][j];
        if (s <= tol) {
            g = 0.0;
        } else {
            f = u[i][i+1];
            g = f < 0.0 ? sqrt(s) : -sqrt(s);
            h = f*g - s;
            u[i][i+1] = f-g;
            for (int j = l; j < n; j++)
                e[j] = u[i][j] / h;
            for (int j = l; j < m; j++) {
                s = 0.0;
                for (int k = l; k < n; k++)
                     s += u[j][k]*u[i][k];
                for (int k = l; k < n; k++)
                     u[j][k] += s * e[k];
            }
        }

        y = fabs(q[i])+fabs(e[i]);
        if (y > x)
            x = y;
    }

    for (int i = n - 1; i > -1; i--) {
        if (g != 0.0) {
            h = g*u[i][i+1];

            for (int j = l; j < n; j++)
                 v[j][i] = u[i][j] / h;

            for (int j = l; j < n; j++) {
                s = 0.0;
                for (int k = l; k < n; k++)
                    s += u[i][k] * v[k][j];
                for (int k = l; k < n; k++)
                    v[k][j] += s * v[k][i];
            }
        }

        for (int j = l; j < n; j++) {
            v[i][j] = 0.0;
            v[j][i] = 0.0;
        }

        v[i][i] = 1.0;
        g = e[i];
        l = i;
    }

    for (int i = n - 1; i > -1; i--) {
        l = i+1;
        g = q[i];
        for (int j = l; j < n; j++)
            u[i][j] = 0.0;
        if (g != 0.0) {
            h = u[i][i] * g;
            mstep(m, n, h, l, i, u);
            for (int j = i; j < m; j++)
                u[j][i] = u[j][i] / g;
        } else {
            for (int j = i; j < m; j++)
                u[j][i] = 0.0;
        }
        u[i][i] += 1.0;
    }

    eps = eps * x;
    for (int k = n-1; k >= 0; k--) {
        for (int iteration = 0; iteration < itmax; iteration++) {
            int goto_test_f_convergence = 0;

            for (l = k; l >= 0; l--) {
                goto_test_f_convergence = 0;
                if (fabs(e[l]) <= eps) {
                    goto_test_f_convergence = 1;
                    break;
                }

                av_assert0(l > 0);
                if (fabs(q[l-1]) <= eps)
                    break;
            }
            if (!goto_test_f_convergence) {
                c = 0.0;
                s = 1.0;
                l1 = l-1;
                av_assert0(l1 >= 0);
                for (int i = l; i <= k; i++) {
                    f = s*e[i];
                    e[i] = c*e[i];

                    if (fabs(f) <= eps)
                        break;

                    g = q[i];
                    h = pythag(f,g);
                    q[i] = h;
                    c = g/h;
                    s = -f/h;
                    for (int j = 0; j < m; j++) {
                        y = u[j][l1];
                        z = u[j][i];
                        u[j][l1] = y*c+z*s;
                        u[j][i] = -y*s+z*c;
                    }
                }
            }
            z = q[k];
            if (l == k) {
                if (z <= 0.0) {
                    q[k] = -z;
                    for (int j = 0; j < n; j++)
                        v[j][k] = -v[j][k];
                }
                break;
            }

            if (iteration >= itmax-1)
                break;

            x = q[l];
            av_assert0(k > 0);
            y = q[k-1];
            g = e[k-1];
            h = e[k];
            f = ((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
            g = pythag(f,1.0);
            if (f < 0.)
                f = ((x-z)*(x+z)+h*(y/(f-g)-h))/x;
            else
                f = ((x-z)*(x+z)+h*(y/(f+g)-h))/x;
            c = 1.0;
            s = 1.0;
            for (int i = l+1; i < k+1; i++) {
                g = e[i];
                y = q[i];
                h = s*g;
                g = c*g;
                z = pythag(f,h);
                e[i-1] = z;
                c = f/z;
                s = h/z;
                f = x*c+g*s;
                g = -x*s+g*c;
                h = y*s;
                y = y*c;
                for (int j = 0; j < n; j++) {
                    x = v[j][i-1];
                    z = v[j][i];
                    v[j][i-1] = x*c+z*s;
                    v[j][i] = -x*s+z*c;
                }
                z = pythag(f,h);
                q[i-1] = z;
                c = f/z;
                s = h/z;
                f = c*g+s*y;
                x = -s*g+c*y;
                for (int j = 0; j < m; j++) {
                    y = u[j][i-1];
                    z = u[j][i];
                    u[j][i-1] = y*c+z*s;
                    u[j][i] = -y*s+z*c;
                }
            }

            e[l] = 0.0;
            e[k] = f;
            q[k] = x;
        }
    }

    LOG_MATRIX("um", u, m, m)

    av_log(ctx, AV_LOG_DEBUG, "wv:\n");
    for (int i = 0; i < n; i++)
        av_log(ctx, AV_LOG_DEBUG, "\t%g,", q[i]);
    av_log(ctx, AV_LOG_DEBUG, "\n");

    LOG_MATRIX("vm", v, n, n)
}

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AmbisonicContext *s = ctx->priv;
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *outlayouts = NULL;
    AVFilterChannelLayouts *inlayouts = NULL;
    AVChannelLayout *outlayout = (AVChannelLayout *)&ambisonic_tab[s->layout].outlayout;
    AVChannelLayout inlayout = AV_CHANNEL_LAYOUT_AMBISONIC_FIRST_ORDER;
    int ret = 0;

    if (s->precision == P_AUTO) {
        ret = ff_add_format(&formats, AV_SAMPLE_FMT_FLTP);
        if (ret)
            return ret;
        ret = ff_add_format(&formats, AV_SAMPLE_FMT_DBLP);
    } else if (s->precision == P_SINGLE) {
        ret = ff_add_format(&formats, AV_SAMPLE_FMT_FLTP);
    } else if (s->precision == P_DOUBLE) {
        ret = ff_add_format(&formats, AV_SAMPLE_FMT_DBLP);
    }
    if (ret)
        return ret;
    ret = ff_set_common_formats2(ctx, cfg_in, cfg_out, formats);
    if (ret)
        return ret;

    ret = ff_add_channel_layout(&outlayouts, outlayout);
    if (ret)
        return ret;

    ret = ff_channel_layouts_ref(outlayouts, &cfg_out[0]->channel_layouts);
    if (ret)
        return ret;

    ret = ff_add_channel_layout(&inlayouts, &inlayout);
    if (ret)
        return ret;

    return ff_channel_layouts_ref(inlayouts, &cfg_in[0]->channel_layouts);
}

static void acn_to_level_order(int acn, int *level, int *order)
{
    *level = floor(sqrt(acn));
    *order = acn - *level * *level - *level;
}

static void calc_acn_sequence(AmbisonicContext *s)
{
    int *dst = s->seq_tab[M_ACN];

    for (int n = 0, i = 0; n <= s->order; n++) {
        for (int m = -n; m <= n; m++, i++)
            dst[i] = n * n + n + m;
    }
}

static void calc_fuma_sequence(AmbisonicContext *s)
{
    int *dst = s->seq_tab[M_FUMA];

    for (int n = 0, i = 0; n <= s->order; n++) {
        if (n < 2) {
            for (int m = -n; m <= n; m++)
                dst[i++] = n * n + 2 * (n - FFABS(m)) + (m < 0);
        } else {
            for (int m = -n; m <= n; m++)
                dst[i++] = SQR(n) + FFABS(m) * 2 - (m > 0);
        }
    }
}

static void calc_sid_sequence(AmbisonicContext *s)
{
    int *dst = s->seq_tab[M_SID];

    for (int n = 0, i = 0; n <= s->order; n++) {
        for (int m = -n; m <= n; m++, i++)
            dst[i] = n * n + 2 * (n - FFABS(m)) + (m < 0);
    }
}

static void calc_ch_map(AmbisonicContext *s)
{
    const int *src0 = s->seq_tab[M_ACN];
    const int *src1 = s->seq_tab[s->seq];
    int *dst = s->seq_map;

    for (int n = 0; n < SQR(s->order + 1); n++)
        dst[src0[n]] = src1[n];
}

static double factorial(int x)
{
    double prod = 1.;

    for (int i = 1; i <= x; i++)
        prod *= i;

    return prod;
}

static double n3d_norm(int i)
{
    int n, m;

    acn_to_level_order(i, &n, &m);

    return sqrt((2 * n + 1) * (2 - (m == 0)) * factorial(n - FFABS(m)) / factorial(n + FFABS(m)));
}

static double sn3d_norm(int i)
{
    int n, m;

    acn_to_level_order(i, &n, &m);

    return sqrt((2 - (m == 0)) * factorial(n - FFABS(m)) / factorial(n + FFABS(m)));
}

static void calc_norm_matrix(AVFilterContext *ctx, AmbisonicContext *s)
{
    const int speakers = ambisonic_tab[s->layout].speakers;
    const int inputs = ambisonic_tab[s->layout].inputs;

    if (!s->matrix_norm) {
        memcpy(s->norm_decode_mat, s->decode_mat, sizeof(s->decode_mat));
        return;
    }

    for (int y = 0; y < speakers; y++) {
        double scale = 0.;

        for (int x = 0; x < inputs; x++)
            scale += fabs(s->decode_mat[y][x]);
        if (scale > 1.)
            scale = 1. / scale;
        else
            scale = 1.;

        for (int x = 0; x < inputs; x++)
            s->norm_decode_mat[y][x] = s->decode_mat[y][x] * scale;
    }

    LOG_MATRIX("norm decode", s->norm_decode_mat, speakers, inputs)
}

static void calc_sn3d_scaling(AmbisonicContext *s)
{
    double *dst = s->norm_tab[SN3D];

    for (int i = 0; i < s->max_channels; i++)
        dst[i] = 1.;
}

static void calc_n3d_scaling(AmbisonicContext *s)
{
    double *dst = s->norm_tab[N3D];

    for (int i = 0; i < s->max_channels; i++)
        dst[i] = n3d_norm(i) / sn3d_norm(i);
}

static void calc_fuma_scaling(AmbisonicContext *s)
{
    double *dst = s->norm_tab[FUMA];

    for (int i = 0; i < s->max_channels; i++) {
        dst[i] = sn3d_norm(i);

        switch (i) {
        case 0:
            dst[i] *= 1. / M_SQRT2;
        case 1:
        case 2:
        case 3:
        case 12:
        default:
            break;
        case 4:
            dst[i] *= 2. / sqrt(3.);
            break;
        case 5:
            dst[i] *= 2. / sqrt(3.);
            break;
        case 6:
            break;
        case 7:
            dst[i] *= 2. / sqrt(3.);
            break;
        case 8:
            dst[i] *= 2. / sqrt(3.);
            break;
        case 9:
            dst[i] *= sqrt(8. / 5.);
            break;
        case 10:
            dst[i] *= 3. / sqrt(5.);
            break;
        case 11:
            dst[i] *= sqrt(45. / 32.);
            break;
        case 13:
            dst[i] *= sqrt(45. / 32.);
            break;
        case 14:
            dst[i] *= 3. / sqrt(5.);
            break;
        case 15:
            dst[i] *= sqrt(8./5.);
            break;
        }
    }
}

static void multiply_mat(double out[3][3],
                         const double a[3][3],
                         const double b[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double sum = 0.;

            for (int k = 0; k < 3; k++)
                sum += a[i][k] * b[k][j];

            out[i][j] = sum;
        }
    }
}

static double P(int i, int l, int mu, int m_, double R_1[3][3],
                double R_lm1[2 * MAX_ORDER + 1][2 * MAX_ORDER + 1])
{
    double ret = 0.;
    double ri1  = R_1[i + 1][2];
    double rim1 = R_1[i + 1][0];
    double ri0  = R_1[i + 1][1];

    if (m_ == -l) {
        ret = ri1 * R_lm1[mu + l - 1][0] + rim1 * R_lm1[mu + l- 1][2 * l - 2];
    } else {
        if (m_ == l)
            ret = ri1 * R_lm1[mu + l - 1][2 * l - 2] - rim1 * R_lm1[mu + l - 1][0];
        else
            ret = ri0 * R_lm1[mu + l - 1][m_ + l - 1];
    }
    return ret;
}

static double U(int l, int m, int n, double R_1[3][3],
                double R_lm1[2 * MAX_ORDER + 1][2 * MAX_ORDER + 1])
{
    return P(0, l, m, n, R_1, R_lm1);
}

static double V(int l, int m, int n, double R_1[3][3],
                double R_lm1[2 * MAX_ORDER + 1][2 * MAX_ORDER + 1])
{
    double ret = 0.;

    if (m == 0) {
        double p0 = P( 1, l,  1, n, R_1, R_lm1);
        double p1 = P(-1, l, -1, n, R_1, R_lm1);
        ret = p0+p1;
    } else {
        if (m > 0) {
            int d = (m == 1) ? 1 : 0;
            double p0 = P( 1, l,  m - 1, n, R_1, R_lm1);
            double p1 = P(-1, l, -m + 1, n, R_1, R_lm1);

            ret = p0 * sqrt(1 + d) - p1 * (1 - d);
        } else {
            int d = (m == -1) ? 1 : 0;
            double p0 = P( 1, l,  m + 1, n, R_1, R_lm1);
            double p1 = P(-1, l, -m - 1, n, R_1, R_lm1);

            ret = p0 * (1 - d) + p1 * sqrt(1 + d);
        }
    }
    return ret;
}

static double W(int l, int m, int n, double R_1[3][3],
                double R_lm1[2 * MAX_ORDER + 1][2 * MAX_ORDER + 1])
{
    double ret = 0.;

    if (m != 0) {
        if (m > 0) {
            double p0 = P( 1, l, m + 1, n, R_1, R_lm1);
            double p1 = P(-1, l,-m - 1, n, R_1, R_lm1);

            ret = p0 + p1;
        } else {
            double p0 = P( 1, l,  m - 1, n, R_1, R_lm1);
            double p1 = P(-1, l, -m + 1, n, R_1, R_lm1);

            ret = p0 - p1;
        }
    }

    return ret;
}

static void calc_rotation_mat(AVFilterContext *ctx,
                              AmbisonicContext *s,
                              double rotate[3])
{
    double X[3][3] = {{0.}}, Y[3][3] = {{0.}}, Z[3][3] = {{0.}}, R[3][3], t[3][3];
    double R_lm1[2 * MAX_ORDER + 1][2 * MAX_ORDER + 1] = {{0.}};
    double yaw, pitch, roll;
    double R_1[3][3];

    yaw   = (M_PI / 180.) * rotate[YAW];
    pitch = (M_PI / 180.) * rotate[PITCH];
    roll  = (M_PI / 180.) * rotate[ROLL];

    X[0][0] = 1.;
    X[1][1] = X[2][2] = cos(roll);
    X[1][2] = sin(roll);
    X[2][1] = -X[1][2];

    Y[0][0] = Y[2][2] = cos(pitch);
    Y[0][2] = sin(pitch);
    Y[2][0] = -Y[0][2];
    Y[1][1] = 1.;

    Z[0][0] = Z[1][1] = cos(yaw);
    Z[0][1] = sin(yaw);
    Z[1][0] = -Z[0][1];
    Z[2][2] = 1.;

    multiply_mat(t, X, Y);
    multiply_mat(R, t, Z);

    R_1[0][0] = R[1][1];
    R_1[0][1] = R[1][2];
    R_1[0][2] = R[1][0];
    R_1[1][0] = R[2][1];
    R_1[1][1] = R[2][2];
    R_1[1][2] = R[2][0];
    R_1[2][0] = R[0][1];
    R_1[2][1] = R[0][2];
    R_1[2][2] = R[0][0];

    memset(s->rotate_mat, 0, sizeof(s->rotate_mat));

    s->rotate_mat[0][0] = 1.;
    s->rotate_mat[1][1] = R_1[0][0];
    s->rotate_mat[1][2] = R_1[0][1];
    s->rotate_mat[1][3] = R_1[0][2];
    s->rotate_mat[2][1] = R_1[1][0];
    s->rotate_mat[2][2] = R_1[1][1];
    s->rotate_mat[2][3] = R_1[1][2];
    s->rotate_mat[3][1] = R_1[2][0];
    s->rotate_mat[3][2] = R_1[2][1];
    s->rotate_mat[3][3] = R_1[2][2];

    R_lm1[0][0] = R_1[0][0];
    R_lm1[0][1] = R_1[0][1];
    R_lm1[0][2] = R_1[0][2];
    R_lm1[1][0] = R_1[1][0];
    R_lm1[1][1] = R_1[1][1];
    R_lm1[1][2] = R_1[1][2];
    R_lm1[2][0] = R_1[2][0];
    R_lm1[2][1] = R_1[2][1];
    R_lm1[2][2] = R_1[2][2];

    for (int l = 2; l <= s->order; l++) {
        double R_l[2 * MAX_ORDER + 1][2 * MAX_ORDER + 1] = {{0.}};

        for (int m = -l; m <= l; m++) {
            for (int n = -l; n <= l; n++) {
                int d = (m == 0) ? 1 : 0;
                double denom = FFABS(n) == l ? (2 * l) * (2 * l - 1) : l * l - n * n;
                double u = sqrt((l * l - m * m) / denom);
                double v = sqrt((1. + d) * (l + FFABS(m) - 1.) * (l + FFABS(m)) / denom) * (1. - 2. * d) * 0.5;
                double w = sqrt((l - FFABS(m) - 1.)*(l - FFABS(m)) / denom) * (1. - d) * -0.5;

                if (u)
                    u *= U(l, m, n, R_1, R_lm1);
                if (v)
                    v *= V(l, m, n, R_1, R_lm1);
                if (w)
                    w *= W(l, m, n, R_1, R_lm1);

                R_l[m + l][n + l] = u + v + w;
            }
        }

        for (int i = 0; i < 2 * l + 1; i++) {
            for (int j = 0; j < 2 * l + 1; j++)
                s->rotate_mat[l * l + i][l * l + j] = R_l[i][j];
        }

        memcpy(R_lm1, R_l, sizeof(R_l));
    }

    av_log(ctx, AV_LOG_DEBUG, "rotation matrix:\n");
    for (int i = 0; i < SQR(s->order + 1); i++) {
        for (int j = 0; j < SQR(s->order + 1); j++) {
            if (fabs(s->rotate_mat[i][j]) < 1e-6f)
                s->rotate_mat[i][j] = 0.;
            av_log(ctx, AV_LOG_DEBUG, "\t%g", s->rotate_mat[i][j]);
        }
        av_log(ctx, AV_LOG_DEBUG, "\n");
    }
}

static void calc_mirror_mat(AmbisonicContext *s)
{
    for (int i = 0; i < s->max_channels; i++) {
        double gain = 1.;
        int level, order;

        acn_to_level_order(i, &level, &order);

        if (i == 0 || (!((level + order) & 1))) {
            gain *= s->gain[EVEN][D_Z];

            if (s->invert[D_Z] & 2)
                gain *= -1.;
        }

        if ((level + order) & 1) {
            gain *= s->gain[ODD][D_Z];

            if (s->invert[D_Z] & 1)
                gain *= -1.;
        }

        if (order >= 0) {
            gain *= s->gain[EVEN][D_Y];

            if (s->invert[D_Y] & 2)
                gain *= -1.;
        }

        if (order < 0) {
            gain *= s->gain[ODD][D_Y];

            if (s->invert[D_Y] & 1)
                gain *= -1.;
        }


        if (((order < 0) && (order & 1)) || ((order >= 0) && !(order & 1)) ) {
            gain *= s->gain[EVEN][D_X];

            if (s->invert[D_X] & 2)
                gain *= -1.;
        }

        if (((order < 0) && !(order & 1)) || ((order >= 0) && (order & 1))) {
            gain *= s->gain[ODD][D_X];

            if (s->invert[D_X] & 1)
                gain *= -1.;
        }

        if (level == order || level == -order) {
            gain *= s->gain[0][D_C];

            if (s->invert[D_C])
                gain *= -1.;
        }

        s->mirror_mat[i] = gain;
    }
}

static void multiply_mat16(double out[16][16],
                           const double a[16][16],
                           const double b[16][16], int x)
{
    for (int i = 0; i < x; i++) {
        for (int j = 0; j < x; j++) {
            double sum = 0.;

            for (int k = 0; k < x; k++)
                sum += a[i][k] * b[k][j];

            out[i][j] = sum;
        }
    }
}

static void multiply_matx(double out[16][16],
                          const double a[16][16],
                          const double b[4][4])
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double sum = 0.;

            for (int k = 0; k < 4; k++)
                sum += a[i][k] * b[k][j];

            out[i][j] = sum;
        }
    }
}

static void multiply_mat4(double out[4][4],
                          const double a[4][4],
                          const double b[4][4])
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double sum = 0.;

            for (int k = 0; k < 4; k++)
                sum += a[i][k] * b[k][j];

            out[i][j] = sum;
        }
    }
}

static void dominance_mat(double d, int index, double m[4][4])
{
    double g0, g1, k, kr;

    k = db2a(d);
    kr = 1. / k;

    g0 = (k + kr) * 0.5;
    g1 = (k - kr) / M_SQRT2;

    m[A_W][A_W]     = g0;
    m[A_W][index]   = g1 * 0.5;
    m[index][A_W]   = g1;
    m[index][index] = g0;
}

static void direct_mat(double angle, int index, double m[4][4])
{
    double g0, g1;

    g0 = sqrt(1. + sin((M_PI/180.) * angle));
    g1 = sqrt(1. - sin((M_PI/180.) * angle));

    if (index == 0)
        FFSWAP(double, g0, g1);

    for (int i = 0; i < 4; i++)
        m[i][i] = index == i ? g1 : g0;
}

static void zoom_mat(double angle, int index, double m[4][4])
{
    double g0, g1;

    angle = angle * (M_PI / 180.);

    g0 = sin(angle);
    g1 = cos(angle);

    for (int i = 0; i < 4; i++) {
        if (i == 0) {
            m[i][i] = 1.;
        } else if (i == index) {
            m[i][i] = 1.;
            m[A_W][i] = g0 / M_SQRT2;
            m[i][A_W] = g0 * M_SQRT2;
        } else {
            m[i][i] = g1;
        }
    }
}

static void focus_mat(double angle, int index, double m[4][4])
{
    double g0, g1, g2;

    angle = angle * (M_PI / 180.);

    g0 = 1. / (1. + sin(fabs(angle)));
    g1 = M_SQRT2 * sin(angle) * g0;
    g2 = cos(angle) * g0;

    for (int i = 0; i < 4; i++) {
        if (i == 0) {
            m[i][i] = g0;
        } else if (i == index) {
            m[i][i] = g0;
            m[A_W][i] = g1 * 0.5;
            m[i][A_W] = g1;
        } else {
            m[i][i] = g2;
        }
    }
}

static void push_mat(double angle, int index, double m[4][4])
{
    double g0, g1;

    angle = angle * (M_PI / 180.);

    g0 = M_SQRT2 * sin(angle) * sin(fabs(angle));
    g1 = SQR(cos(angle));

    for (int i = 0; i < 4; i++) {
        if (i == 0) {
            m[i][i] = 1.;
        } else if (i == index) {
            m[i][i] = g1;
            m[i][A_W] = g0;
        } else {
            m[i][i] = g1;
        }
    }
}

static void press_mat(double angle, int index, double m[4][4])
{
    double g0, g1, g2;

    angle = angle * (M_PI / 180.);

    g0 = M_SQRT2 * sin(angle) * sin(fabs(angle));
    g2 = cos(angle);
    g1 = SQR(g2);

    for (int i = 0; i < 4; i++) {
        if (i == 0) {
            m[i][i] = 1.;
        } else if (i == index) {
            m[i][i] = g1;
            m[i][A_W] = g0;
        } else {
            m[i][i] = g2;
        }
    }
}

static const double i_mat[4][4] =
{
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 },
};

static void calc_asymmetry_mat(AVFilterContext *ctx, AmbisonicContext *s)
{
    double angle = s->asymmetry * (M_PI / 180.);
    double g0 = -sin(angle);
    double g1 = SQR(sin(angle));
    double g2 = SQR(cos(angle));
    double g3 = cos(angle) * sin(angle);
    double g4 = cos(angle);

    s->asymmetry_mat[0][0] = 1.0;
    s->asymmetry_mat[0][2] = g0 / M_SQRT2;
    s->asymmetry_mat[1][0] = g1 * M_SQRT2;
    s->asymmetry_mat[1][1] = g2;
    s->asymmetry_mat[1][2] = g0;
    s->asymmetry_mat[2][0] =-g3 * M_SQRT2;
    s->asymmetry_mat[2][1] = g3;
    s->asymmetry_mat[2][2] = g4;
    s->asymmetry_mat[3][3] = g4;

    LOG_MATRIX("asymmetry", s->asymmetry_mat, 4, 4)
}

static void calc_press_mat(AVFilterContext *ctx, AmbisonicContext *s)
{
    double x_mat[4][4] = { 0 };
    double y_mat[4][4] = { 0 };
    double z_mat[4][4] = { 0 };
    double o_mat[4][4];

    press_mat(s->press[A_X], A_X, x_mat);
    press_mat(s->press[A_Y], A_Y, y_mat);
    press_mat(s->press[A_Z], A_Z, z_mat);

    multiply_mat4(o_mat, x_mat, y_mat);
    multiply_mat4(s->press_mat, o_mat, z_mat);

    LOG_MATRIX("press", s->press_mat, 4, 4)
}

static void calc_push_mat(AVFilterContext *ctx, AmbisonicContext *s)
{
    double x_mat[4][4] = { 0 };
    double y_mat[4][4] = { 0 };
    double z_mat[4][4] = { 0 };
    double o_mat[4][4];

    push_mat(s->push[A_X], A_X, x_mat);
    push_mat(s->push[A_Y], A_Y, y_mat);
    push_mat(s->push[A_Z], A_Z, z_mat);

    multiply_mat4(o_mat, x_mat, y_mat);
    multiply_mat4(s->push_mat, o_mat, z_mat);

    LOG_MATRIX("push", s->push_mat, 4, 4)
}

static void calc_dominance_mat(AVFilterContext *ctx, AmbisonicContext *s)
{
    double x_mat[4][4];
    double y_mat[4][4];
    double z_mat[4][4];
    double o_mat[4][4];

    memcpy(x_mat, i_mat, sizeof(i_mat));
    memcpy(y_mat, i_mat, sizeof(i_mat));
    memcpy(z_mat, i_mat, sizeof(i_mat));

    dominance_mat(s->dominance[A_X], A_X, x_mat);
    dominance_mat(s->dominance[A_Y], A_Y, y_mat);
    dominance_mat(s->dominance[A_Z], A_Z, z_mat);

    multiply_mat4(o_mat, x_mat, y_mat);
    multiply_mat4(s->dominance_mat, o_mat, z_mat);

    LOG_MATRIX("dominance", s->dominance_mat, 4, 4)
}

static void calc_direction_mat(AVFilterContext *ctx, AmbisonicContext *s)
{
    double w_mat[4][4] = { 0 };
    double x_mat[4][4] = { 0 };
    double y_mat[4][4] = { 0 };
    double z_mat[4][4] = { 0 };
    double t_mat[4][4];

    direct_mat(s->direction[A_W], A_W, w_mat);
    direct_mat(s->direction[A_X], A_X, x_mat);
    direct_mat(s->direction[A_Y], A_Y, y_mat);
    direct_mat(s->direction[A_Z], A_Z, z_mat);

    multiply_mat4(t_mat, w_mat, x_mat);
    multiply_mat4(w_mat, t_mat, y_mat);
    multiply_mat4(s->direction_mat, w_mat, z_mat);

    LOG_MATRIX("direction", s->direction_mat, 4, 4)
}

static void calc_zoom_mat(AVFilterContext *ctx, AmbisonicContext *s)
{
    double x_mat[4][4] = { 0 };
    double y_mat[4][4] = { 0 };
    double z_mat[4][4] = { 0 };
    double t_mat[4][4];

    zoom_mat(s->zoom[A_X], A_X, x_mat);
    zoom_mat(s->zoom[A_Y], A_Y, y_mat);
    zoom_mat(s->zoom[A_Z], A_Z, z_mat);

    multiply_mat4(t_mat, x_mat, y_mat);
    multiply_mat4(s->zoom_mat, t_mat, z_mat);

    LOG_MATRIX("zoom", s->zoom_mat, 4, 4)
}

static void calc_focus_mat(AVFilterContext *ctx, AmbisonicContext *s)
{
    double x_mat[4][4] = { 0 };
    double y_mat[4][4] = { 0 };
    double z_mat[4][4] = { 0 };
    double t_mat[4][4];

    focus_mat(s->focus[A_X], A_X, x_mat);
    focus_mat(s->focus[A_Y], A_Y, y_mat);
    focus_mat(s->focus[A_Z], A_Z, z_mat);

    multiply_mat4(t_mat, x_mat, y_mat);
    multiply_mat4(s->focus_mat, t_mat, z_mat);

    LOG_MATRIX("focus", s->focus_mat, 4, 4)
}

static void calc_transform_mat(AVFilterContext *ctx, AmbisonicContext *s)
{
    const int inputs = ambisonic_tab[s->layout].inputs;
    double tt_mat[16][16] = { 0 };
    double o_mat[16][16] = { 0 };
    double t_mat[4][4];
    double r_mat[4][4];

    for (int i = 0; i < inputs; i++)
        tt_mat[i][i] = s->mirror_mat[i];

    multiply_mat4(t_mat, s->dominance_mat, s->direction_mat);
    multiply_mat4(r_mat, s->zoom_mat, t_mat);
    multiply_mat4(t_mat, s->focus_mat, r_mat);
    multiply_mat4(r_mat, s->push_mat, t_mat);
    multiply_mat4(t_mat, s->press_mat, r_mat);
    multiply_mat4(r_mat, s->asymmetry_mat, t_mat);

    multiply_matx(o_mat, tt_mat, r_mat);

    multiply_mat16(s->transform_mat, s->rotate_mat, o_mat, inputs);

    LOG_MATRIX("transform", s->transform_mat, inputs, inputs)
}

static void near_field(AmbisonicContext *s, AVFrame *frame,
                       const int out, const int add)
{
    for (int ch = 1 - out; ch < frame->ch_layout.nb_channels; ch++) {
        int n, m;

        acn_to_level_order(ch, &n, &m);

        if (!s->nf_process[n - 1])
            break;

        s->nf_process[n - 1](&s->nf[out][ch], frame, ch, add, 1.);
    }
}

typedef struct ThreadData {
    const double *gains_tab;
    int nb_channels;
    AVFrame *in, *out;
} ThreadData;

#define DEPTH 32
#include "ambisonic_template.c"

#undef DEPTH
#define DEPTH 64
#include "ambisonic_template.c"

static double speed_of_sound(double temp)
{
    return 1.85325 * (643.95 * sqrt(((temp + 273.15) / 273.15))) * 1000.0 / (60. * 60.);
}

static void nfield1_init(NearField *nf, double distance, double proximity,
                         double speed, double rate,
                         double gain)
{
    const double w0 = 2.0 * rate * proximity / speed;
    const double w1 = 2.0 * rate * distance  / speed;
    const double c0 = -1.0 / w0;
    const double c1 = -1.0 / w1;

    nf->sos[0].b[0] = (1.0-c0) / (1.0 - c1);
    nf->sos[0].b[1] = (-1.0*(1.0+c0)) / (1.0 - c1);
    nf->sos[0].b[2] = 0.0;
    nf->sos[0].a[0] = (-1.0 * (1.0 + c1)) / (1.0 - c1);
    nf->sos[0].a[1] = 0.0;
}

static void near_field_init(AmbisonicContext *s, int out,
                            double speed, double rate, double gain)
{
    for (int ch = 1 - out; ch < s->max_channels; ch++) {
        int n, m;

        acn_to_level_order(ch, &n, &m);

        if (!s->nf_init[n - 1])
            break;

        s->nf_init[n - 1](&s->nf[out][ch], s->distance, s->proximity,
                          speed, rate, gain);
    }
}

static void calc_level_tab(AmbisonicContext *s)
{
    double max_distance = 0.;

    for (int spkr = 0; spkr < ambisonic_tab[s->layout].speakers; spkr++) {
        const double spkr_distance = ambisonic_tab[s->layout].speakers_distance[spkr];

        if (spkr_distance > max_distance)
            max_distance = spkr_distance;
    }

    for (int spkr = 0; spkr < ambisonic_tab[s->layout].speakers; spkr++) {
        const double scale = s->level ? ambisonic_tab[s->layout].speakers_distance[spkr] / max_distance : 1.;

        s->level_tab[spkr] = scale;
    }
}

static void calc_pgains_tab(AmbisonicContext *s, int type)
{
    const int order = s->order;

    if (!type) {
        for (int level = 0; level < order + 1; level++)
            s->pgains[0][level] = s->pgains[1][level] = 1.0;
    } else if (type == 1 || type == 2) {
        const int components = type == 1 ? 2 * order + 1 : SQR(order + 1);
        const int speakers = ambisonic_tab[s->layout].speakers;
        double E_gain = 0;
        double g, g2 = 0.;

        for (int level = 0; level < order + 1; level++) {
            const double f = type == 1 ? 1 + (level > 0):
                                         2 * SQR(level) + 1;
            const double e = type == 1 ? gains_2d[order][level]:
                                         gains_3d[order][level];
            E_gain += SQR(e) * f;
        }

        if (s->pgtype == PT_ENERGY) {
            g2 = speakers / E_gain;
        } else if (s->pgtype == PT_RMS) {
            g2 = components / E_gain;
        } else if (s->pgtype == PT_AMP) {
            g2 = 1.;
        }

        g = sqrt(g2);

        for (int level = 0; level < order + 1; level++) {
            const double e = type == 1 ? gains_2d[order][level]:
                                         gains_3d[order][level];
            s->pgains[0][level] = 1.0;
            s->pgains[1][level] = g * e;
        }
    }
}

static void calc_gains_tab(AVFilterContext *ctx,
                           AmbisonicContext *s, double xover_ratio)
{
    const int inputs = ambisonic_tab[s->layout].inputs;
    const double xover_gain = db2a(xover_ratio);

    for (int level = 0, ch = 0; level < s->order + 1; level++) {
        for (int i = 0; i < 1 + level * 2; i++, ch++) {
            const double lf_gain = s->pgains[0][level];
            const double hf_gain = s->pgains[1][level];

            s->gains_tab[0][ch] = lf_gain / xover_gain;
            s->gains_tab[1][ch] = hf_gain * xover_gain;
        }
    }

    av_log(ctx, AV_LOG_DEBUG, "gains tab:\n");
    for (int ch = 0; ch < inputs; ch++)
        av_log(ctx, AV_LOG_DEBUG, "\t%g", s->gains_tab[0][ch]);
    av_log(ctx, AV_LOG_DEBUG, "\n");
    for (int ch = 0; ch < inputs; ch++)
        av_log(ctx, AV_LOG_DEBUG, "\t%g", s->gains_tab[1][ch]);
    av_log(ctx, AV_LOG_DEBUG, "\n");
}

static void xover_init_input(AVFilterContext *ctx, Xover *xover, double freq, double rate, int hf)
{
    double k = tan(M_PI * freq / rate);
    double k2 = k * k;
    double d = k2 + 2. * k + 1.;

    if (hf) {
        xover->b[0] = -1. / d;
        xover->b[1] =  2. / d;
        xover->b[2] = -1. / d;
    } else {
        xover->b[0] = k2 / d;
        xover->b[1] = 2. * k2 / d;
        xover->b[2] = k2 / d;
    }

    xover->a[0] = 1.;
    xover->a[1] = -2 * (k2 - 1.) / d;
    xover->a[2] = -(k2 - 2 * k + 1.) / d;

    av_log(ctx, AV_LOG_DEBUG, "shelf: %g %g %g / %g %g %g\n",
           xover->b[0], xover->b[1], xover->b[2],
           xover->a[0], xover->a[1], xover->a[2]);
}

static void xover_init(AVFilterContext *ctx, AmbisonicContext *s, double freq, double rate, int channels)
{
    for (int ch = 0; ch < channels; ch++) {
        xover_init_input(ctx, &s->xover[0][ch], freq, rate, 0);
        xover_init_input(ctx, &s->xover[1][ch], freq, rate, 1);
    }
}

static void calc_factor(double *factors, int inputs,
                        double a, double e, int m)
{
    const double cos_a = cos(a);
    const double sin_a = sin(a);
    const double cos_e = cos(e);
    const double sin_e = sin(e);
    const double sqrt3 = sqrt(3.);

    factors[A_W] = 1.;

    if (inputs <= 1)
        return;

    factors[A_Y] = sin_a * cos_e;
    factors[A_Z] = sin_e * m;
    factors[A_X] = cos_a * cos_e;

    if (inputs <= 4)
        return;

    factors[A_V] = sqrt3 * sin_a * SQR(cos_e) * cos_a;
    factors[A_T] = 0.25 * sqrt3 * (cos(2. * e - a) - cos(2. * e + a));
    factors[A_R] = (3./2.) * SQR(sin_e) - 0.5;
    factors[A_S] = 0.25 * sqrt3 * (sin(2. * e - a) + sin(2. * e + a));
    factors[A_U] = 0.5 * sqrt3 * SQR(cos_e)*cos(2. * a);

    if (inputs <= 9)
        return;

    factors[A_Q] = 0.25 * sqrt(10.) * (-4. * SQR(sin_a) + 3.) * sin_a * SQR(cos_e) * cos_e;
    factors[A_O] = sqrt(15.) * sin_e * sin_a * SQR(cos_e) * cos_a;
    factors[A_M] = 0.25 * sqrt(6.) * (5. * SQR(sin_e) - 1.) * sin_a * cos_e;
    factors[A_K] = 0.5 * (5*SQR(sin_e) - 3)*sin_e;
    factors[A_L] = 0.25 * sqrt(6.) * (5. * SQR(sin_e) - 1.) * cos_e * cos_a;
    factors[A_N] = 0.5 * sqrt(15.) * sin_e * SQR(cos_e) * cos(2. * a);
    factors[A_P] = 0.25 * sqrt(10.)*(-4. * SQR(sin_a) + 1.) * SQR(cos_e) * cos_e * cos_a;
}

static void calc_factors(AVFilterContext *ctx,
                         AmbisonicContext *s)
{
    const int speakers = ambisonic_tab[s->layout].speakers;
    const double *elevation = ambisonic_tab[s->layout].speakers_elevation;
    const double *azimuth = ambisonic_tab[s->layout].speakers_azimuth;
    const int inputs = ambisonic_tab[s->layout].inputs;

    if (elevation) {
        for (int ch = 0; ch < speakers; ch++)
            calc_factor(s->decode_mat[ch], inputs,
                        (M_PI / 180.) * azimuth[ch] * -1.,
                        (M_PI / 180.) * elevation[ch], 1);
    } else {
        for (int ch = 0; ch < speakers; ch++)
            calc_factor(s->decode_mat[ch], inputs,
                        (M_PI / 180.) * azimuth[ch] * -1.,
                        0., 0);
    }

    LOG_MATRIX("factors", s->decode_mat, speakers, inputs)
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AmbisonicContext *s = ctx->priv;
    const int type = ambisonic_tab[s->layout].type;
    const int inputs = ambisonic_tab[s->layout].inputs;
    const int speakers = ambisonic_tab[s->layout].speakers;
    const double matching = s->matching;
    double w_mean = 0.;

    s->order = ambisonic_tab[s->layout].order;
    s->max_channels = SQR(s->order + 1);

    if (s->near_field == NF_AUTO)
        s->near_field = ambisonic_tab[s->layout].near_field;
    if (s->xover_freq < 0)
        s->xover_freq = ambisonic_tab[s->layout].xover;

    calc_factors(ctx, s);

    memset(s->v, 0, sizeof(s->v));
    memset(s->w, 0, sizeof(s->w));

    memcpy(s->u, s->decode_mat, sizeof(s->u));
    svdcmp(ctx, s->u, speakers, inputs, s->w, s->v);

    for (int x = 0; x < inputs; x++) {
        s->w[x] = s->w[x] > 1e-9 ? 1. / s->w[x] : 0.f;
        w_mean += s->w[x];
    }

    w_mean /= inputs;
    for (int x = 0; x < inputs; x++)
        s->w[x] = s->w[x] * (1. - matching) + matching * w_mean;

    for (int y = 0; y < inputs; y++) {
        for (int x = 0; x < inputs; x++)
            s->v[y][x] *= s->w[x];
    }

    for (int y = 0; y < speakers; y++) {
        for (int x = 0; x < inputs; x++) {
            double sum = 0.;

            for (int z = 0; z < inputs; z++)
                sum += s->v[x][z] * s->u[y][z];
            s->decode_mat[y][x] = sum;
        }
    }

    LOG_MATRIX("decode", s->decode_mat, speakers, inputs)

    calc_norm_matrix(ctx, s);
    calc_sn3d_scaling(s);
    calc_n3d_scaling(s);
    calc_fuma_scaling(s);

    calc_acn_sequence(s);
    calc_fuma_sequence(s);
    calc_sid_sequence(s);

    calc_ch_map(s);

    near_field_init(s, 0, speed_of_sound(s->temp), outlink->sample_rate, 1.);
    near_field_init(s, 1, speed_of_sound(s->temp), outlink->sample_rate, 1.);

    calc_rotation_mat(ctx, s, s->rotate);
    calc_mirror_mat(s);

    calc_asymmetry_mat(ctx, s);
    calc_direction_mat(ctx, s);
    calc_dominance_mat(ctx, s);
    calc_focus_mat(ctx, s);
    calc_press_mat(ctx, s);
    calc_push_mat(ctx, s);
    calc_zoom_mat(ctx, s);

    calc_level_tab(s);
    calc_pgains_tab(s, type);
    calc_gains_tab(ctx, s, s->xover_ratio);
    xover_init(ctx, s, s->xover_freq, outlink->sample_rate, s->max_channels);

    switch (s->precision) {
    case P_AUTO:
        s->nf_process[0] = outlink->format == AV_SAMPLE_FMT_FLTP ? nfield1_process_float : nfield1_process_double;
        s->process = outlink->format == AV_SAMPLE_FMT_FLTP ? process_float : process_double;
        break;
    case P_SINGLE:
        s->nf_process[0] = nfield1_process_float;
        s->process = process_float;
        break;
    case P_DOUBLE:
        s->nf_process[0] = nfield1_process_double;
        s->process = process_double;
        break;
    default: av_assert0(0);
    }

    calc_transform_mat(ctx, s);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AmbisonicContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out;

    if (!s->rframe || s->rframe->nb_samples < in->nb_samples) {
        ff_graph_frame_free(ctx, &s->sframe);
        ff_graph_frame_free(ctx, &s->rframe);
        ff_graph_frame_free(ctx, &s->frame2);
        s->sframe = ff_get_audio_buffer(inlink, in->nb_samples);
        s->rframe = ff_get_audio_buffer(inlink, in->nb_samples);
        s->frame2 = ff_get_audio_buffer(inlink, in->nb_samples);
        if (!s->sframe || !s->rframe || !s->frame2) {
            av_frame_free(&s->sframe);
            av_frame_free(&s->rframe);
            av_frame_free(&s->frame2);
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
    }

    out = ff_get_audio_buffer(outlink, in->nb_samples);
    if (!out) {
        av_frame_free(&in);
        return AVERROR(ENOMEM);
    }
    av_frame_copy_props(out, in);

    s->process(ctx, in, out);

    ff_graph_frame_free(ctx, &in);
    return ff_filter_frame(outlink, out);
}

static av_cold int init(AVFilterContext *ctx)
{
    AmbisonicContext *s = ctx->priv;

    s->nf_init[0] = nfield1_init;

    s->fdsp = avpriv_float_dsp_alloc(0);
    if (!s->fdsp)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AmbisonicContext *s = ctx->priv;

    av_freep(&s->fdsp);
    av_frame_free(&s->sframe);
    av_frame_free(&s->rframe);
    av_frame_free(&s->frame2);
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

static int process_command(AVFilterContext *ctx, const char *cmd, const char *arg)
{
    AmbisonicContext *s = ctx->priv;
    int matrix_norm = s->matrix_norm;
    double asymmetry = s->asymmetry;
    int level = s->level;
    double gain[2][NB_DTYPES];
    int invert[NB_DTYPES];
    double direction[4];
    double dominance[4];
    double focus[4];
    double press[4];
    double push[4];
    double rotate[3];
    double zoom[4];
    int ret;

    memcpy(direction, s->direction, sizeof(direction));
    memcpy(dominance, s->dominance, sizeof(dominance));
    memcpy(focus, s->focus, sizeof(focus));
    memcpy(gain, s->gain, sizeof(gain));
    memcpy(invert, s->invert, sizeof(invert));
    memcpy(press, s->press, sizeof(press));
    memcpy(push, s->push, sizeof(push));
    memcpy(rotate, s->rotate, sizeof(rotate));
    memcpy(zoom, s->zoom, sizeof(zoom));

    ret = ff_filter_process_command(ctx, cmd, arg);
    if (ret < 0)
        return ret;

    if (memcmp(rotate, s->rotate, sizeof(rotate)))
        calc_rotation_mat(ctx, s, s->rotate);

    if (memcmp(gain, s->gain, sizeof(gain)) ||
        memcmp(invert, s->invert, sizeof(invert)))
        calc_mirror_mat(s);

    if (asymmetry != s->asymmetry)
        calc_asymmetry_mat(ctx, s);

    if (memcmp(dominance, s->dominance, sizeof(dominance)))
        calc_dominance_mat(ctx, s);

    if (memcmp(direction, s->direction, sizeof(direction)))
        calc_direction_mat(ctx, s);

    if (memcmp(zoom, s->zoom, sizeof(zoom)))
        calc_zoom_mat(ctx, s);

    if (memcmp(focus, s->focus, sizeof(focus)))
        calc_focus_mat(ctx, s);

    if (memcmp(push, s->push, sizeof(push)))
        calc_push_mat(ctx, s);

    if (memcmp(press, s->press, sizeof(press)))
        calc_press_mat(ctx, s);

    if (level != s->level)
        calc_level_tab(s);

    if (matrix_norm != s->matrix_norm)
        calc_norm_matrix(ctx, s);

    calc_transform_mat(ctx, s);

    return 0;
}

AVFILTER_DEFINE_CLASS(ambisonic);

const FFFilter ff_af_ambisonic = {
    .p.name          = "ambisonic",
    .p.description   = NULL_IF_CONFIG_SMALL("Ambisonic decoder"),
    .p.priv_class    = &ambisonic_class,
    .priv_size       = sizeof(AmbisonicContext),
    .init            = init,
    .uninit          = uninit,
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags         = AVFILTER_FLAG_SLICE_THREADS,
    .process_command = process_command,
};
