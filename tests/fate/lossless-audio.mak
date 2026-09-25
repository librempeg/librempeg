FATE_SAMPLES_LOSSLESS_AUDIO-$(call DEMDEC, MOV, ALAC, ASF2SF_FILTER) += fate-lossless-alac
fate-lossless-alac: CMD = md5 -i $(TARGET_SAMPLES)/lossless-audio/inside.m4a -f s16le -af asf2sf

FATE_SAMPLES_LOSSLESS_AUDIO-$(call DEMDEC, MLP, MLP) += fate-lossless-meridianaudio
fate-lossless-meridianaudio: CMD = md5 -i $(TARGET_SAMPLES)/lossless-audio/luckynight-partial.mlp -f s16le

FATE_SAMPLES_LOSSLESS_AUDIO-$(call DEMDEC, RM, RALF, ASF2SF_FILTER) += fate-ralf
fate-ralf: CMD = md5 -i $(TARGET_SAMPLES)/lossless-audio/luckynight-partial.rmvb -vn -f s16le -af asf2sf

FATE_SAMPLES_LOSSLESS_AUDIO-$(call DEMDEC, SHORTEN, SHORTEN, ASF2SF_FILTER) += fate-lossless-shorten
fate-lossless-shorten: CMD = md5 -i $(TARGET_SAMPLES)/lossless-audio/luckynight-partial.shn -f s16le -af asf2sf

FATE_SAMPLES_LOSSLESS_AUDIO-$(call DEMDEC, RKA, RKA, ASF2SF_FILTER) += fate-lossless-rka
fate-lossless-rka: CMD = md5 -i $(TARGET_SAMPLES)/lossless-audio/luckynight-partial.rka -f s16le -af asf2sf

FATE_SAMPLES_LOSSLESS_AUDIO-$(call DEMDEC, OSQ, OSQ, ASF2SF_FILTER) += fate-lossless-osq
fate-lossless-osq: CMD = md5 -i $(TARGET_SAMPLES)/lossless-audio/luckynight-partial.osq -f s16le -af asf2sf

FATE_SAMPLES_LOSSLESS_AUDIO-$(call CRC, TAK, TAK, ASF2SF_FILTER) += fate-lossless-tak
fate-lossless-tak: CMD = crc -i $(TARGET_SAMPLES)/lossless-audio/luckynight-partial.tak -af asf2sf

FATE_SAMPLES_LOSSLESS_AUDIO-$(call CRC, TTA, TTA) += fate-lossless-tta
fate-lossless-tta: CMD = crc -i $(TARGET_SAMPLES)/lossless-audio/inside.tta

FATE_SAMPLES_LOSSLESS_AUDIO-$(call CRC, TTA, TTA) += fate-lossless-tta-encrypted
fate-lossless-tta-encrypted: CMD = crc -password ffmpeg -i $(TARGET_SAMPLES)/lossless-audio/encrypted.tta

FATE_SAMPLES_LOSSLESS_AUDIO-$(call DEMDEC, ASF, WMALOSSLESS, ASF2SF_FILTER) += fate-lossless-wma fate-lossless-wma24-1 fate-lossless-wma24-2 fate-lossless-wma24-rawtile
fate-lossless-wma: CMD = md5 -i $(TARGET_SAMPLES)/lossless-audio/luckynight-partial.wma -f s16le -frames 209 -af asf2sf
fate-lossless-wma24-1: CMD = md5 -i $(TARGET_SAMPLES)/lossless-audio/master_audio_2.0_24bit.wma -f s24le -af asf2sf
fate-lossless-wma24-2: CMD = md5 -i $(TARGET_SAMPLES)/lossless-audio/Mega_Weird_Audio_Test_24bit.wma -f s24le -af asf2sf
fate-lossless-wma24-rawtile: CMD = md5 -i $(TARGET_SAMPLES)/lossless-audio/g2_24bit.wma -f s24le -af asf2sf
fate-lossless-wmall: fate-lossless-wma fate-lossless-wma24-1 fate-lossless-wma24-2 fate-lossless-wma24-rawtile

FATE_SAMPLES_LOSSLESS_AUDIO += $(FATE_SAMPLES_LOSSLESS_AUDIO-yes)

FATE_SAMPLES_FFMPEG += $(FATE_SAMPLES_LOSSLESS_AUDIO)

fate-lossless-audio: $(FATE_SAMPLES_LOSSLESS_AUDIO)
