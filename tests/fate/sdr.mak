FATE_SDR = fate-sdr-am \
           fate-sdr-fm \

FATE_SAMPLES_FFMPEG += $(FATE_SDR)

fate-sdr-am: CMD = framecrc -mode all_mode -video_size 400x200 -i $(TARGET_SAMPLES)/sdr/am.sdr -map 0 -filter:a aresample -c:a pcm_u8
fate-sdr-fm: CMD = framecrc -mode all_mode -video_size 400x200 -i $(TARGET_SAMPLES)/sdr/fm.sdr -map 0 -filter:a aresample -c:a pcm_u8

FATE_FFMPEG += $(FATE_SDR_FFMPEG-yes)

fate-sdr: $(FATE_SDR) $(FATE_SDR_FFMPEG-yes)
