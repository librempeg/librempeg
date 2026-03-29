#ifndef AVFILTER_FFFRAMEPOOL_H
#define AVFILTER_FFFRAMEPOOL_H

/**
 * Frame pool. This structure must be initialized with
 * ff_frame_pool_{video,audio}_reinit() and freed with ff_frame_pool_uninit().
 */
typedef struct FFFramePool {

    enum AVMediaType type;
    union {
        enum AVPixelFormat pix_fmt;
        enum AVSampleFormat sample_fmt;
    };

    /* video */
    int width;
    int height;

    /* audio */
    int planes;
    int channels;
    int nb_samples;

    /* common */
    int align;
    int linesize[4];
    AVBufferPool *pools[4]; /* for audio, only pools[0] is used */

    AVMutex mutex;

} FFFramePool;

#endif
