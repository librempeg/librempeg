#include "libavutil/mem.h"
#include "ipu_instructions.h"

void ff_ipu_init_inst_exec_list(IPUInstructionExecutionContext *ie, int w, int h)
{
    if (ie && !(w % 16) && !(h % 16)) {
        ie->inst = ((w+15)/16) * ((h+15)/16);

        size_t inst_d_size;
        if (!av_size_mult(ie->inst, sizeof(IPUBlockCoordContext), &inst_d_size)) {
            ie->blk_d = av_mallocz(inst_d_size);
            if (ie->blk_d) {
                int w_clc = 0, h_clc = 0;

                for (int i = 0; i < ie->inst; i++) {
                    IPUBlockCoordContext *blk_d = ie->blk_d + i;
                    blk_d->starting_x = w_clc;
                    blk_d->starting_y = h_clc;

                    w_clc += 16;
                    blk_d->ending_x = w_clc;
                    if (w_clc == w) {
                        h_clc += 16;
                        blk_d->ending_y = h_clc;
                        w_clc ^= w_clc;
                    }
                }
            }
        }
    }
}

void ff_ipu_init_inst_exec_comm_list(IPUInstructionExecutionCommunicationContext *iec, int ipu_flags_exist, IPUControlRegister ctrl, int w, int h)
{
    if (iec && !(w % 16) && !(h % 16)) {
        iec->ipu_flags_exist = (ipu_flags_exist) ? 1 : 0;
        iec->inst = ((w+15)/16) * ((h+15)/16);

        size_t inst_d_size;
        if (!av_size_mult(iec->inst, sizeof(IPUBlockInstructionCommunicationContext), &inst_d_size)) {
            iec->inst_d = av_mallocz(inst_d_size);
            if (iec->inst_d) {
                int wh2i = 0;

                for (int i = 0; i < iec->inst; i++) {
                    IPUBlockInstructionCommunicationContext *blk_inst_cm_i = iec->inst_d + i;
                    blk_inst_cm_i->sync_inst_id = wh2i++;
                }
            }
        }
    }
}

void ff_ipu_set_ctrl_from_inst_exec_comm_list(IPUInstructionExecutionCommunicationContext *iec, IPUControlRegister ctrl)
{
    if (iec) {
        /*
         * allowed values of intra_dc_precision are as follows: 
         * 0 (8-bit), 1 (9-bit), 2 (10-bit), 3 (reserved)
         */
        if (ctrl.intra_dc_precision < 0)
            iec->ctrl.intra_dc_precision = 0;
        else if (ctrl.intra_dc_precision > 3)
            iec->ctrl.intra_dc_precision = 3;
        else
            iec->ctrl.intra_dc_precision = ctrl.intra_dc_precision;

        /* alternate_scan is either 0 (zigzag scanning) or 1 (alternate scanning) */
        if (ctrl.alternate_scan < 0)
            iec->ctrl.alternate_scan = 0;
        else if (ctrl.alternate_scan > 1)
            iec->ctrl.alternate_scan = 1;
        else
            iec->ctrl.alternate_scan = ctrl.alternate_scan;

        /* intra_vlc_format is either:
         * - 0 (MPEG1-compatible 2-dimensional VLC table)
         * - 1 (2-dimensional VLC table specially for intra macro block)
         */
        if (ctrl.intra_vlc_format < 0)
            iec->ctrl.intra_vlc_format = 0;
        else if (ctrl.intra_vlc_format > 1)
            iec->ctrl.intra_vlc_format = 1;
        else
            iec->ctrl.intra_vlc_format = ctrl.intra_vlc_format;

        /* q_scale_step is either 0 (linear step) or 1 (non-linear step) */
        if (ctrl.q_scale_step < 0)
            iec->ctrl.q_scale_step = 0;
        else if (ctrl.q_scale_step > 1)
            iec->ctrl.q_scale_step = 1;
        else
            iec->ctrl.q_scale_step = ctrl.q_scale_step;

        /* mpeg1_bitstream is either 0 (MPEG-2 bit stream) or 1 (MPEG-1 bit stream) */
        if (ctrl.mpeg1_bitstream < 0)
            iec->ctrl.mpeg1_bitstream = 0;
        else if (ctrl.mpeg1_bitstream > 1)
            iec->ctrl.mpeg1_bitstream = 1;
        else
            iec->ctrl.mpeg1_bitstream = ctrl.mpeg1_bitstream;

        /*
         * allowed values of picture_type are as follows:
         * 0 (reserved), 1 (I-picture), 2 (P-picture), 3 (B-picture), 4 (D-picture)
         */
        if (ctrl.picture_type < 0)
            iec->ctrl.picture_type = 0;
        else if (ctrl.picture_type > 4)
            iec->ctrl.picture_type = 4;
        else
            iec->ctrl.picture_type = ctrl.picture_type;

        // whether or not the entire IPU decoding process should be reset.
        if (ctrl.reset < 0)
            iec->ctrl.reset = 0;
        else if (ctrl.reset > 1)
            iec->ctrl.reset = 1;
        iec->ctrl.reset = ctrl.reset;
    }
}

void ff_ipu_set_inst_to_exec_from_start(IPUInstructionExecutionCommunicationContext *iec, IPUBlockCommand cmd, IPUIBDECArguments args, int w, int h, int max_w, int max_h)
{
    if (iec && !(w % 16) && !(h % 16)) {
        int expected_inst_c = ((max_w+15)/16) * ((max_h+15)/16);
        if (iec->inst == expected_inst_c) {
            int bic_goto = ((w+15)/16);
            if (bic_goto == 0)
                bic_goto = ((h+15)/16) * ((max_w+15)/16) + ((w+15)/16);

            IPUBlockInstructionCommunicationContext *bic = NULL;
            for (int i = bic_goto; i < iec->inst; i++) {
                bic = iec->inst_d + i;
                bic->cmd = cmd;
                if ((cmd == IPU_CMD_IDEC) | (cmd == IPU_CMD_BDEC)) {
                    bic->ibd_args.qscale_code = args.qscale_code;
                    if (cmd == IPU_CMD_IDEC)
                        bic->ibd_args.dtd = args.dtd;
                    else if (cmd == IPU_CMD_BDEC) {
                        bic->ibd_args.dct_type = args.dct_type;
                        bic->ibd_args.reset_dc_pred = args.reset_dc_pred;
                        bic->ibd_args.macro_block_intra = args.macro_block_intra;
                    }
                }
            }
        }
    }
}

void ff_ipu_close_block_inst_comm(IPUInstructionExecutionCommunicationContext *iec)
{
    if (iec) {
        if (iec->inst_d)
            av_freep(&iec->inst_d);
    }
}