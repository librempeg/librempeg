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

#include <float.h>

#include "libavutil/ffmath.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "audio.h"
#include "filters.h"
#include "formats.h"

enum DetectionModes {
    DET_UNSET = 0,
    DET_DISABLED,
    DET_OFF,
    DET_ON,
    DET_ADAPTIVE,
    NB_DMODES,
};

enum DirectionModes {
    BELOW,
    ABOVE,
};

enum FilterModes {
    LISTEN = -1,
    CUT,
    BOOST,
    NB_FMODES,
};

enum DetectFilter {
    DBANDPASS,
    DLOWPASS,
    DHIGHPASS,
    DPEAK,
    NB_DFILTERS
};

enum DetectThresholdFilter {
    DTDISABLED,
    DTABSOLUTE,
    DTRELATIVE,
    NB_DTFILTERS
};

enum TargetFilter {
    TBELL,
    TLOWSHELF,
    THIGHSHELF,
    NB_TFILTERS
};

typedef struct AudioDynamicEqualizerContext {
    const AVClass *class;

    double *threshold;
    unsigned nb_threshold;

    double *dfrequency;
    unsigned nb_dfrequency;

    double *dqfactor;
    unsigned nb_dqfactor;

    int *dftype;
    unsigned nb_dftype;

    int *dttype;
    unsigned nb_dttype;

    double *tfrequency;
    unsigned nb_tfrequency;

    double *tqfactor;
    unsigned nb_tqfactor;

    double *tattack;
    unsigned nb_tattack;

    double *trelease;
    unsigned nb_trelease;

    int *tftype;
    unsigned nb_tftype;

    double *ratio;
    unsigned nb_ratio;

    double *range;
    unsigned nb_range;

    int *direction;
    unsigned nb_direction;

    double *makeup;
    unsigned nb_makeup;

    int *detection;
    unsigned nb_detection;

    int *mode;
    unsigned nb_mode;

    int *active;
    unsigned nb_active;

    AVChannelLayout *channel;
    unsigned nb_channel;

    double *fdetection;
    unsigned nb_fdetection;

    int sidechain;
    int precision;
    int format;
    int nb_bands;
    int nb_channels;

    int (*filter_prepare)(AVFilterContext *ctx);
    void (*fill_fdetection)(AVFilterContext *ctx);
    int (*filter_channels)(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs);
    int (*init_state)(AVFilterContext *ctx, int nb_channels, int nb_bands);
    void (*uninit_state)(AVFilterContext *ctx);

    AVFrame *in, *sc;

    void *bc;
} AudioDynamicEqualizerContext;

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const AudioDynamicEqualizerContext *s = ctx->priv;
    static const enum AVSampleFormat sample_fmts[3][3] = {
        { AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_NONE },
        { AV_SAMPLE_FMT_FLTP, AV_SAMPLE_FMT_NONE },
        { AV_SAMPLE_FMT_DBLP, AV_SAMPLE_FMT_NONE },
    };
    int ret;

    if ((ret = ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out,
                                                sample_fmts[s->precision])) < 0)
        return ret;

    return 0;
}

typedef struct ThreadData {
    AVFrame *in, *out, *sc;
} ThreadData;

#define DEPTH 32
#include "adynamicequalizer_template.c"

#undef DEPTH
#define DEPTH 64
#include "adynamicequalizer_template.c"

static av_cold int init(AVFilterContext *ctx)
{
    AudioDynamicEqualizerContext *s = ctx->priv;

    if (s->sidechain) {
        AVFilterPad pad = { NULL };

        pad.type = AVMEDIA_TYPE_AUDIO;
        pad.name = "sidechain";
        return ff_append_inpad(ctx, &pad);
    }

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AudioDynamicEqualizerContext *s = ctx->priv;
    int ret, nb_bands;

    s->format = outlink->format;
    switch (s->format) {
    case AV_SAMPLE_FMT_DBLP:
        s->init_state      = init_state_double;
        s->uninit_state    = uninit_state_double;
        s->fill_fdetection  = fill_fdetection_double;
        s->filter_prepare  = filter_prepare_double;
        s->filter_channels = filter_channels_double;
        break;
    case AV_SAMPLE_FMT_FLTP:
        s->init_state      = init_state_float;
        s->uninit_state    = uninit_state_float;
        s->fill_fdetection  = fill_fdetection_float;
        s->filter_prepare  = filter_prepare_float;
        s->filter_channels = filter_channels_float;
        break;
    default:
        return AVERROR_BUG;
    }

    nb_bands = s->nb_tfrequency;
    ret = s->init_state(ctx, outlink->ch_layout.nb_channels, nb_bands);
    if (ret < 0)
        return ret;
    s->nb_channels = outlink->ch_layout.nb_channels;

    s->nb_fdetection = nb_bands;
    s->fdetection = av_calloc(s->nb_fdetection, sizeof(*s->fdetection));
    if (!s->fdetection)
        return AVERROR(ENOMEM);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in, AVFrame *sc)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    AudioDynamicEqualizerContext *s = ctx->priv;
    ThreadData td;
    AVFrame *out;

    if (av_frame_is_writable(in)) {
        out = in;
    } else {
        out = ff_get_audio_buffer(outlink, in->nb_samples);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
    }

    td.in = in;
    td.out = out;
    td.sc = sc;
    s->filter_prepare(ctx);
    ff_filter_execute(ctx, s->filter_channels, &td, NULL,
                      FFMIN(s->nb_channels, ff_filter_get_nb_threads(ctx)));
    s->fill_fdetection(ctx);

    if (out != in)
        av_frame_free(&s->in);
    s->in = NULL;
    av_frame_free(&s->sc);
    return ff_filter_frame(outlink, out);
}

static int activate(AVFilterContext *ctx)
{
    AudioDynamicEqualizerContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFilterLink *inlink = ctx->inputs[0];

    FF_FILTER_FORWARD_STATUS_BACK_ALL(outlink, ctx);

    if (!s->in) {
        int ret = ff_inlink_consume_frame(inlink, &s->in);
        if (ret < 0)
            return ret;
    }

    if (s->in) {
        if (s->sidechain && !s->sc) {
            AVFilterLink *sclink = ctx->inputs[1];
            int ret = ff_inlink_consume_samples(sclink, s->in->nb_samples,
                                                s->in->nb_samples, &s->sc);
            if (ret < 0)
                return ret;

            if (!ret) {
                FF_FILTER_FORWARD_STATUS(sclink, outlink);
                FF_FILTER_FORWARD_WANTED(outlink, sclink);
                return 0;
            }
        }

        return filter_frame(inlink, s->in, s->sc);
    }

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    AudioDynamicEqualizerContext *s = ctx->priv;

    av_frame_free(&s->in);
    av_frame_free(&s->sc);

    if (s->uninit_state)
        s->uninit_state(ctx);
}

#define OFFSET(x) offsetof(AudioDynamicEqualizerContext, x)
#define AF AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_RUNTIME_PARAM
#define AR AV_OPT_TYPE_FLAG_ARRAY
#define XR AV_OPT_FLAG_EXPORT|AV_OPT_FLAG_READONLY

static const AVOptionArrayDef def_threshold  = {.def="0",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_dfrequency = {.def="1000",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_dqfactor   = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_tfrequency = {.def="1000",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_tqfactor   = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_dftype     = {.def="bandpass",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_dttype     = {.def="absolute",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_tattack    = {.def="20",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_trelease   = {.def="200",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_tftype     = {.def="bell",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_ratio      = {.def="1",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_makeup     = {.def="0",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_range      = {.def="50",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_direction  = {.def="below",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_mode       = {.def="cut",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_auto       = {.def="off",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_active     = {.def="true",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_channel    = {.def="24c",.size_min=1,.sep=' '};
static const AVOptionArrayDef def_fdetection  = {.def="0",.size_min=1,.sep=' '};

static const AVOption adynamicequalizer_options[] = {
    { "threshold",  "set detection threshold", OFFSET(threshold),  AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_threshold}, 0, 100,     FLAGS },
    { "dfrequency", "set detection frequency", OFFSET(dfrequency), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_dfrequency},2, 1000000, FLAGS },
    { "dqfactor",   "set detection Q factor",  OFFSET(dqfactor),   AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_dqfactor},  0.001, 1000,FLAGS },
    { "dftype",     "set detection filter type",OFFSET(dftype),    AV_OPT_TYPE_INT|AR, {.arr=&def_dftype}, 0,NB_DFILTERS-1,FLAGS, .unit = "dftype" },
    {   "bandpass", 0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=DBANDPASS},0, 0,       FLAGS, .unit = "dftype" },
    {   "lowpass",  0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=DLOWPASS}, 0, 0,       FLAGS, .unit = "dftype" },
    {   "highpass", 0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=DHIGHPASS},0, 0,       FLAGS, .unit = "dftype" },
    {   "peak",     0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=DPEAK},    0, 0,       FLAGS, .unit = "dftype" },
    { "dttype",     "set detection threshold type",OFFSET(dttype), AV_OPT_TYPE_INT|AR, {.arr=&def_dttype}, 0,NB_DTFILTERS-1,FLAGS, .unit = "dttype" },
    {   "disabled", "disable detection threshold",0,               AV_OPT_TYPE_CONST,  {.i64=DTDISABLED},0, 0,      FLAGS, .unit = "dttype" },
    {   "absolute", "set the absolute threshold", 0,               AV_OPT_TYPE_CONST,  {.i64=DTABSOLUTE},0, 0,      FLAGS, .unit = "dttype" },
    {   "relative", "set the relative threshold", 0,               AV_OPT_TYPE_CONST,  {.i64=DTRELATIVE},0, 0,      FLAGS, .unit = "dttype" },
    { "tfrequency", "set target frequency",    OFFSET(tfrequency), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_tfrequency},2, 1000000, FLAGS },
    { "tqfactor",   "set target Q factor",     OFFSET(tqfactor),   AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_tqfactor},  0.001, 1000,FLAGS },
    { "attack", "set target attack duration",  OFFSET(tattack),    AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_tattack},   0.01, 2000, FLAGS },
    { "release","set target release duration", OFFSET(trelease),   AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_trelease},  0.01, 2000, FLAGS },
    { "tftype",     "set target filter type",  OFFSET(tftype),     AV_OPT_TYPE_INT|AR, {.arr=&def_tftype}, 0, NB_TFILTERS-1,FLAGS, .unit = "tftype" },
    {   "bell",     0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=TBELL},    0, 0,       FLAGS, .unit = "tftype" },
    {   "lowshelf", 0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=TLOWSHELF},0, 0,       FLAGS, .unit = "tftype" },
    {   "highshelf",0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=THIGHSHELF},0,0,       FLAGS, .unit = "tftype" },
    { "ratio",      "set ratio factor",        OFFSET(ratio),      AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_ratio},     0, 30,      FLAGS },
    { "makeup",     "set makeup gain",         OFFSET(makeup),     AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_makeup},    0, 1000,    FLAGS },
    { "range",      "set max gain",            OFFSET(range),      AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_range},     0, 2000,    FLAGS },
    { "direction",  "set filtering direction", OFFSET(direction),  AV_OPT_TYPE_INT|AR, {.arr=&def_direction}, BELOW, ABOVE,  FLAGS, .unit = "direction" },
    {   "below",    "filter below threshold",  0,                  AV_OPT_TYPE_CONST,  {.i64=BELOW},             0, 0,       FLAGS, .unit = "direction" },
    {   "above",    "filter above threshold",  0,                  AV_OPT_TYPE_CONST,  {.i64=ABOVE},             0, 0,       FLAGS, .unit = "direction" },
    { "mode",       "set filtering mode",      OFFSET(mode),       AV_OPT_TYPE_INT|AR, {.arr=&def_mode}, LISTEN,NB_FMODES-1,FLAGS, .unit = "mode" },
    {     "listen", 0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=LISTEN},            0, 0,       FLAGS, .unit = "mode" },
    {        "cut", 0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=CUT},               0, 0,       FLAGS, .unit = "mode" },
    {      "boost", 0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=BOOST},             0, 0,       FLAGS, .unit = "mode" },
    { "auto",       "set auto threshold",      OFFSET(detection),  AV_OPT_TYPE_INT|AR, {.arr=&def_auto},DET_DISABLED,NB_DMODES-1,FLAGS, .unit = "auto" },
    {   "disabled", 0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=DET_DISABLED}, 0, 0,   FLAGS, .unit = "auto" },
    {   "off",      0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=DET_OFF},      0, 0,   FLAGS, .unit = "auto" },
    {   "on",       0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=DET_ON},       0, 0,   FLAGS, .unit = "auto" },
    {   "adaptive", 0,                         0,                  AV_OPT_TYPE_CONST,  {.i64=DET_ADAPTIVE}, 0, 0,   FLAGS, .unit = "auto" },
    { "active",     "set the band activity",   OFFSET(active),     AV_OPT_TYPE_BOOL|AR,{.arr=&def_active},  0, 1,   FLAGS },
    { "channel",    "set the channels values per band", OFFSET(channel), AV_OPT_TYPE_CHLAYOUT|AR, {.arr=&def_channel}, 0, 0, FLAGS },
    { "precision", "set processing precision", OFFSET(precision),  AV_OPT_TYPE_INT,    {.i64=0},            0, 2,   AF, .unit = "precision" },
    {   "auto",  "set auto processing precision",                  0, AV_OPT_TYPE_CONST, {.i64=0},          0, 0,   AF, .unit = "precision" },
    {   "float", "set single-floating point processing precision", 0, AV_OPT_TYPE_CONST, {.i64=1},          0, 0,   AF, .unit = "precision" },
    {   "double","set double-floating point processing precision", 0, AV_OPT_TYPE_CONST, {.i64=2},          0, 0,   AF, .unit = "precision" },
    { "sidechain",  "enable sidechain input",  OFFSET(sidechain),  AV_OPT_TYPE_BOOL,   {.i64=0},            0, 1,   AF },
    { "fdetection", "filtered detection value",OFFSET(fdetection), AV_OPT_TYPE_DOUBLE|AR, {.arr=&def_fdetection}, 0, 100, AF|XR },
    { NULL }
};

AVFILTER_DEFINE_CLASS(adynamicequalizer);

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_output,
    },
};

const FFFilter ff_af_adynamicequalizer = {
    .p.name          = "adynamicequalizer",
    .p.description   = NULL_IF_CONFIG_SMALL("Apply Dynamic Equalization of input audio."),
    .p.priv_class    = &adynamicequalizer_class,
    .priv_size       = sizeof(AudioDynamicEqualizerContext),
    .init            = init,
    .activate        = activate,
    .uninit          = uninit,
    FILTER_INPUTS(ff_audio_default_filterpad),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC2(query_formats),
    .p.flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL |
                       AVFILTER_FLAG_DYNAMIC_INPUTS |
                       AVFILTER_FLAG_SLICE_THREADS,
    .process_command = ff_filter_process_command,
};
