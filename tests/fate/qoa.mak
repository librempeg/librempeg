FATE_QOA-$(call FRAMECRC, QOA, QOA, ASF2SF_FILTER) += fate-qoa-152
fate-qoa-152: CMD = framecrc -i $(TARGET_SAMPLES)/qoa/coin_48_1_152.qoa -af asf2sf

FATE_QOA-$(call FRAMECRC, QOA, QOA, ASF2SF_FILTER) += fate-qoa-278
fate-qoa-278: CMD = framecrc -i $(TARGET_SAMPLES)/qoa/vibra_44_2_278.qoa -af asf2sf

FATE_QOA-$(call FRAMECRC, QOA, QOA, ASF2SF_FILTER) += fate-qoa-303
fate-qoa-303: CMD = framecrc -i $(TARGET_SAMPLES)/qoa/banjo_48_2_303.qoa -af asf2sf

FATE_SAMPLES_FFMPEG += $(FATE_QOA-yes)
fate-audio fate-qoa: fate-qoa-152 fate-qoa-278 fate-qoa-303
