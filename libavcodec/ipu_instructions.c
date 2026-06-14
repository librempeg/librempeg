#include "libavutil/mem.h"
#include "ipu_instructions.h"

size_t ff_ipu_calc_priv_data_skip_to_ipu_iec_ctx(AVCodecContext *avctx)
{
    size_t ret = 0;
    int ofst_sz_fld_sz = (sizeof(size_t)*2);

    if (avctx->priv_data) {
        if (*(size_t*)((char*)avctx->priv_data + (ofst_sz_fld_sz*1)))
            ret = *(size_t*)((char*)avctx->priv_data + (ofst_sz_fld_sz*1));
    }

    return ret;
}

void ff_ipu_init_inst_exec_list(AVCodecContext *avctx, int x, int y)
{
    size_t skip = ff_ipu_calc_priv_data_skip_to_ipu_iec_ctx(avctx);
    if (skip) {
        IPUInstructionExecutionContext *ie = (IPUInstructionExecutionContext*)((char*)avctx->priv_data + skip);

        if ((!(x % 16)) && (!(y % 16))) {
            ie->inst = ((x+15)/16) * ((y+15)/16);

            size_t inst_d_size;
            if (!av_size_mult(ie->inst, sizeof(IPUBlockInstructionContext), &inst_d_size)) {
                if (!ie->inst_d)
                    ie->inst_d = av_mallocz(inst_d_size);

                if (ie->inst_d) {
                    int x_clc = 0, y_clc = 0;
                    IPUBlockInstructionContext *inst_d = NULL;

                    for (int i = 0; i < ie->inst; i++) {
                        inst_d = ie->inst_d + i;
                        inst_d->starting_x = x_clc;
                        inst_d->starting_y = y_clc;

                        x_clc += 16;
                        inst_d->ending_x = x_clc;
                        if (x_clc == x) {
                            y_clc += 16;
                            inst_d->ending_y = y_clc;
                            x_clc ^= x_clc;
                        }
                    }
                }
            }
        }
    }
}

void ff_ipu_ctrl_byte_signal(AVCodecContext *avctx, int ipu_flags_exist)
{
    size_t skip = ff_ipu_calc_priv_data_skip_to_ipu_iec_ctx(avctx);
    if (skip) {
        IPUInstructionExecutionContext *ie = (IPUInstructionExecutionContext*)((char*)avctx->priv_data + skip);

        if (ipu_flags_exist < 0)
            ie->ipu_flags_exist = 0;
        else if (ipu_flags_exist > 1)
            ie->ipu_flags_exist = 1;
        else
            ie->ipu_flags_exist = ipu_flags_exist;
    }
}

void ff_ipu_iq_comms_signal(AVCodecContext *avctx, int iq_comms)
{
    size_t skip = ff_ipu_calc_priv_data_skip_to_ipu_iec_ctx(avctx);
    if (skip) {
        IPUInstructionExecutionContext *ie = (IPUInstructionExecutionContext*)((char*)avctx->priv_data + skip);

        if (iq_comms < 0)
            ie->iq_comms = 0;
        else if (iq_comms > 1)
            ie->iq_comms = 1;
        else
            ie->iq_comms = iq_comms;
    }
}

void ff_ipu_set_ctrl(AVCodecContext *avctx, IPUControlRegister ctrl)
{
    size_t skip = ff_ipu_calc_priv_data_skip_to_ipu_iec_ctx(avctx);
    if (skip) {
        IPUInstructionExecutionContext *ie = (IPUInstructionExecutionContext*)((char*)avctx->priv_data + skip);
        IPUControlRegister *ctrl_in = &ie->ctrl;

        /*
         * allowed values of intra_dc_precision are as follows: 
         * 0 (8-bit), 1 (9-bit), 2 (10-bit), 3 (reserved)
         */
        if (ctrl.intra_dc_precision < 0)
            ctrl_in->intra_dc_precision = 0;
        else if (ctrl.intra_dc_precision > 3)
            ctrl_in->intra_dc_precision = 3;
        else
            ctrl_in->intra_dc_precision = ctrl.intra_dc_precision;

        /* alternate_scan is either 0 (zigzag scanning) or 1 (alternate scanning) */
        if (ctrl.alternate_scan < 0)
            ctrl_in->alternate_scan = 0;
        else if (ctrl.alternate_scan > 1)
            ctrl_in->alternate_scan = 1;
        else
            ctrl_in->alternate_scan = ctrl.alternate_scan;

        /* intra_vlc_format is either:
         * - 0 (MPEG1-compatible 2-dimensional VLC table)
         * - 1 (2-dimensional VLC table specially for intra macro block)
         */
        if (ctrl.intra_vlc_format < 0)
            ctrl_in->intra_vlc_format = 0;
        else if (ctrl.intra_vlc_format > 1)
            ctrl_in->intra_vlc_format = 1;
        else
            ctrl_in->intra_vlc_format = ctrl.intra_vlc_format;

        /* q_scale_step is either 0 (linear step) or 1 (non-linear step) */
        if (ctrl.q_scale_step < 0)
            ctrl_in->q_scale_step = 0;
        else if (ctrl.q_scale_step > 1)
            ctrl_in->q_scale_step = 1;
        else
            ctrl_in->q_scale_step = ctrl.q_scale_step;

        /* mpeg1_bitstream is either 0 (MPEG-2 bit stream) or 1 (MPEG-1 bit stream) */
        if (ctrl.mpeg1_bitstream < 0)
            ctrl_in->mpeg1_bitstream = 0;
        else if (ctrl.mpeg1_bitstream > 1)
            ctrl_in->mpeg1_bitstream = 1;
        else
            ctrl_in->mpeg1_bitstream = ctrl.mpeg1_bitstream;

        /*
         * allowed values of picture_type are as follows:
         * 0 (reserved), 1 (I-picture), 2 (P-picture), 3 (B-picture), 4 (D-picture)
         */
        if (ctrl.picture_type < 0)
            ctrl_in->picture_type = 0;
        else if (ctrl.picture_type > 4)
            ctrl_in->picture_type = 4;
        else
            ctrl_in->picture_type = ctrl.picture_type;

        // whether or not the entire IPU decoding process should be reset.
        if (ctrl.reset < 0)
            ctrl_in->reset = 0;
        else if (ctrl.reset > 1)
            ctrl_in->reset = 1;
        ctrl_in->reset = ctrl.reset;
    }
}

void ff_ipu_set_inst_to_exec_from_start(AVCodecContext *avctx, IPUBlockCommand cmd, IPUIBDECArguments args, int x, int y, int max_x, int max_y)
{
    size_t skip = ff_ipu_calc_priv_data_skip_to_ipu_iec_ctx(avctx);
    if (skip) {
        IPUInstructionExecutionContext *ie = (IPUInstructionExecutionContext*)((char*)avctx->priv_data + skip);
        IPUBlockInstructionContext *bic = NULL;
        IPUIBDECArguments *args_in = NULL;
        int is_expected_targeted_res = (!(x % 16)) && (!(y % 16));
        int is_expected_max_res = (!(max_x % 16)) && (!(max_y % 16));

        if (is_expected_targeted_res && is_expected_max_res) {
            int expected_inst_c = ((max_x+15)/16) * ((max_y+15)/16);
            if (ie->inst == expected_inst_c) {
                int bic_goto = ((x+15)/16);
                if (bic_goto == 0)
                    bic_goto = ((y+15)/16) * ((max_x+15)/16) + ((x+15)/16);

                for (int i = bic_goto; i < ie->inst; i++) {
                    bic = ie->inst_d + i;
                    bic->cmd = cmd;

                    args_in = &bic->args;
                    if ((cmd == IPU_CMD_IDEC) | (cmd == IPU_CMD_BDEC)) {
                        args_in->qscale_code = args.qscale_code;
                        if (cmd == IPU_CMD_IDEC)
                            args_in->dtd = args.dtd;
                        else if (cmd == IPU_CMD_BDEC) {
                            args_in->dct_type = args.dct_type;
                            args_in->reset_dc_pred = args.reset_dc_pred;
                            args_in->macro_block_intra = args.macro_block_intra;
                        }
                    }
                }
            }
        }
    }
}

void ff_ipu_close_block_inst_comm(AVCodecContext *avctx)
{
    size_t skip = ff_ipu_calc_priv_data_skip_to_ipu_iec_ctx(avctx);
    if (skip) {
        IPUInstructionExecutionContext *ie = (IPUInstructionExecutionContext*)((char*)avctx->priv_data + skip);
    
        if (ie->inst_d)
            av_freep(&ie->inst_d);
    }
}

void ff_ipu_queue_ctrl_exec(AVPacket *avpkt, IPUControlRegister ctrl, int is_busy)
{
    IPUInstructionQueueContext *iq = (IPUInstructionQueueContext*)avpkt->side_data->data;
    IPUControlRegister *ctrl_in = &iq->next_ctrl;

    iq->iq_type = IPU_IQ_CTRL;

    if (is_busy < 0)
        iq->busy_until_next_ctrl_execution = 0;
    else if (is_busy > 1)
        iq->busy_until_next_ctrl_execution = 1;
    else
        iq->busy_until_next_ctrl_execution = is_busy;

    /*
     * allowed values of intra_dc_precision are as follows: 
     * 0 (8-bit), 1 (9-bit), 2 (10-bit), 3 (reserved)
     */
    if (ctrl.intra_dc_precision < 0)
        ctrl_in->intra_dc_precision = 0;
    else if (ctrl.intra_dc_precision > 3)
        ctrl_in->intra_dc_precision = 3;
    else
        ctrl_in->intra_dc_precision = ctrl.intra_dc_precision;

    /* alternate_scan is either 0 (zigzag scanning) or 1 (alternate scanning) */
    if (ctrl.alternate_scan < 0)
        ctrl_in->alternate_scan = 0;
    else if (ctrl.alternate_scan > 1)
        ctrl_in->alternate_scan = 1;
    else
        ctrl_in->alternate_scan = ctrl.alternate_scan;

    /* intra_vlc_format is either:
     * - 0 (MPEG1-compatible 2-dimensional VLC table)
     * - 1 (2-dimensional VLC table specially for intra macro block)
     */
    if (ctrl.intra_vlc_format < 0)
        ctrl_in->intra_vlc_format = 0;
    else if (ctrl.intra_vlc_format > 1)
        ctrl_in->intra_vlc_format = 1;
    else
        ctrl_in->intra_vlc_format = ctrl.intra_vlc_format;

    /* q_scale_step is either 0 (linear step) or 1 (non-linear step) */
    if (ctrl.q_scale_step < 0)
        ctrl_in->q_scale_step = 0;
    else if (ctrl.q_scale_step > 1)
        ctrl_in->q_scale_step = 1;
    else
        ctrl_in->q_scale_step = ctrl.q_scale_step;

    /* mpeg1_bitstream is either 0 (MPEG-2 bit stream) or 1 (MPEG-1 bit stream) */
    if (ctrl.mpeg1_bitstream < 0)
        ctrl_in->mpeg1_bitstream = 0;
    else if (ctrl.mpeg1_bitstream > 1)
        ctrl_in->mpeg1_bitstream = 1;
    else
        ctrl_in->mpeg1_bitstream = ctrl.mpeg1_bitstream;

    /*
     * allowed values of picture_type are as follows:
     * 0 (reserved), 1 (I-picture), 2 (P-picture), 3 (B-picture), 4 (D-picture)
     */
    if (ctrl.picture_type < 0)
        ctrl_in->picture_type = 0;
    else if (ctrl.picture_type > 4)
        ctrl_in->picture_type = 4;
    else
        ctrl_in->picture_type = ctrl.picture_type;

    // whether or not the entire IPU decoding process should be reset.
    if (ctrl.reset < 0)
        ctrl_in->reset = 0;
    else if (ctrl.reset > 1)
        ctrl_in->reset = 1;
    ctrl_in->reset = ctrl.reset;
}

void ff_ipu_queue_inst_exec_from_start(AVPacket *avpkt, IPUBlockCommand next_cmd, IPUIBDECArguments next_args, int x, int y, int max_x, int max_y, int is_busy)
{
    IPUInstructionQueueContext *iq = (IPUInstructionQueueContext*)avpkt->side_data->data;
    IPUIBDECArguments *args_in = &iq->next_args;

    iq->iq_type = IPU_IQ_CMD;

    if (is_busy < 0)
        iq->busy_until_next_cmd_instruction = 0;
    else if (is_busy > 1)
        iq->busy_until_next_cmd_instruction = 1;
    else
        iq->busy_until_next_cmd_instruction = is_busy;

    iq->next_cmd = next_cmd;

    if ((next_cmd >= IPU_CMD_IDEC) && (next_cmd <= IPU_CMD_VDEC)) {
        if (next_args.qscale_code < 0)
            args_in->qscale_code = 0;
        else if (next_args.qscale_code > 31)
            args_in->qscale_code = 31;
        else
            args_in->qscale_code = next_args.qscale_code;

        if (next_cmd == IPU_CMD_IDEC) {
            if (next_args.dtd < 0)
                args_in->dtd = 0;
            else if (next_args.dtd > 1)
                args_in->dtd = 1;
            else
                args_in->dtd = next_args.dtd;
        } else if (next_cmd == IPU_CMD_BDEC) {
            if (next_args.dct_type < 0)
                args_in->dct_type = 0;
            else if (next_args.dct_type > 1)
                args_in->dct_type = 1;
            else
                args_in->dct_type = next_args.dct_type;
            //
            if (next_args.reset_dc_pred < 0)
                args_in->reset_dc_pred = 0;
            else if (next_args.reset_dc_pred > 1)
                args_in->reset_dc_pred = 1;
            else
                args_in->reset_dc_pred = next_args.reset_dc_pred;
            //
            if (next_args.macro_block_intra < 0)
                args_in->macro_block_intra = 0;
            else if (next_args.macro_block_intra > 1)
                args_in->macro_block_intra = 1;
            else
                args_in->macro_block_intra = next_args.macro_block_intra;
        }
    }

    int is_expected_targeted_res = (!(x % 16)) && (!(y % 16));
    int is_expected_max_res = (!(max_x % 16)) && (!(max_y % 16));

    if (is_expected_targeted_res && is_expected_max_res) {
        int expected_inst_c = ((max_x+15)/16) * ((max_y+15)/16);
        int bic_goto = ((x+15)/16);
        if (bic_goto == 0)
            bic_goto = ((y+15)/16) * ((max_x+15)/16) + ((x+15)/16);

        if ((bic_goto >= 0) && (bic_goto <= expected_inst_c)) {
            iq->blk_i = bic_goto;
            iq->blk_i_end = -1;
        }
    }
}

void ff_ipu_execute_queued_ctrl(IPUInstructionQueueType iq_type, IPUInstructionQueueContext *iq, IPUInstructionExecutionContext *ie)
{
    if (iq && ie) {
        if ((iq_type == IPU_IQ_CTRL) && (iq->busy_until_next_ctrl_execution) && (!(ie->executing))) {
            ie->ctrl.intra_dc_precision = iq->next_ctrl.intra_dc_precision;
            ie->ctrl.alternate_scan = iq->next_ctrl.alternate_scan;
            ie->ctrl.intra_vlc_format = iq->next_ctrl.intra_vlc_format;
            ie->ctrl.q_scale_step = iq->next_ctrl.q_scale_step;
            ie->ctrl.mpeg1_bitstream = iq->next_ctrl.mpeg1_bitstream;
            ie->ctrl.picture_type = iq->next_ctrl.picture_type;
            ie->ctrl.reset = ie->ctrl.reset;

            iq->busy_until_next_ctrl_execution = 0;
        }
    }
}

void ff_ipu_execute_queued_cmd(IPUInstructionQueueType iq_type, IPUInstructionQueueContext *iq, IPUInstructionExecutionContext *ie)
{
    if (iq && ie) {
        if ((iq_type == IPU_IQ_CMD) && (iq->busy_until_next_cmd_instruction) && (!(ie->executing))) {
            if (iq->blk_i_end == -1) {
                for (int i = iq->blk_i; i < ie->inst; i++) {
                    IPUBlockInstructionContext *inst_d = ie->inst_d + i;

                    inst_d->cmd = iq->next_cmd;
                    if ((iq->next_cmd >= IPU_CMD_IDEC) && (iq->next_cmd <= IPU_CMD_BDEC)) {
                        IPUIBDECArguments *args_d = &inst_d->args;

                        args_d->qscale_code = iq->next_args.qscale_code;
                        if (iq->next_cmd == IPU_CMD_IDEC)
                            args_d->dtd = iq->next_args.dtd;
                        else if (iq->next_cmd == IPU_CMD_BDEC) {
                            args_d->dct_type = iq->next_args.dct_type;
                            args_d->reset_dc_pred = iq->next_args.reset_dc_pred;
                            args_d->macro_block_intra = iq->next_args.macro_block_intra;
                        }
                    }
                }
            }

            iq->busy_until_next_cmd_instruction = 0;
        }
    }
}