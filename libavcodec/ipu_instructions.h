#include "avcodec.h"
#include "packet_internal.h"

#ifndef AVCODEC_IPU_INSTRUCTIONS_H
#define AVCODEC_IPU_INSTRUCTIONS_H

/**
 * IPUBlockCommand
 */
typedef enum IPUBlockCommand {
    IPU_CMD_dummy00 = 0,
    IPU_CMD_IDEC,
    IPU_CMD_BDEC,
    IPU_CMD_VDEC,
    IPU_CMD_end
} IPUBlockCommand;

typedef struct IPUIBDECArguments {
    int qscale_code; // IPU IDEC/BDEC command
    int dtd; // IPU IDEC command only (DT (DCT Type) Decode)
    int dct_type; // IPU BDEC command only
    int reset_dc_pred; // IPU BDEC command only
    int macro_block_intra; // IPU BDEC command only
} IPUIBDECArguments;

/**
 * IPUControlRegister
 */
typedef struct IPUControlRegister {
    int coded_block_pattern;
    int intra_dc_precision;
    int alternate_scan;
    int intra_vlc_format;
    int q_scale_step;
    int mpeg1_bitstream;
    int picture_type;
    int reset;
} IPUControlRegister;

/**
 * IPUBlockInstructionContext
 */
typedef struct IPUBlockInstructionContext {
    IPUBlockCommand cmd; // IPU block command
    IPUIBDECArguments args; // IDEC/BDEC arguments
    // the following 4 vars shall be divisible by 16, otherwise not happening.
    int starting_x;
    int starting_y;
    int ending_x;
    int ending_y;
} IPUBlockInstructionContext;

/**
 * IPUInstructionExecutionContext
 */
typedef struct IPUInstructionExecutionContext {
    int ipu_flags_exist; // denotes the existence of a byte consisting of IPU bitstream flags.
    int iq_comms; // denotes the existence of an instruction queue.
    int executing;
    IPUControlRegister ctrl; // bitstream decoding options.
    int inst; // how many instructions do we need from a given frame
    IPUBlockInstructionContext *inst_d; // instruction data.
} IPUInstructionExecutionContext;

/**
 * Fulfills the execution list of an IPU frame.
 * This is meant to be used in the .init function of an existing FFCodec implementation.
 *
 * @param avctx The specified address of an AVCodecContext object, if it exists.
 *
 * @param x The specified width of an IPU frame, by a multiple of 16.
 *
 * @param y The specified height of an IPU frame, by a multiple of 16.
 *
 */
void ff_ipu_init_inst_exec_list(AVCodecContext *avctx, int x, int y);

/**
 * Signals that there is a control byte at the start of an IPU-encoded frame.
 *
 * @param avctx The specified address of an AVCodecContext object, if it exists.
 *
 * @param ipu_flags_exist Indicates that such a byte exists; 0 if it doesn't, and 1 if it does.
 *
 */
void ff_ipu_ctrl_byte_signal(AVCodecContext *avctx, int ipu_flags_exist);

void ff_ipu_iq_comms_signal(AVCodecContext *avctx, int iq_comms);

/**
 * Fulfills the execution communication list of an IPU frame.
 * This is meant to be used in the .read_probe function of an existing FFInputFormat implementation.
 *
 * @param avctx The specified address of an AVCodecContext object, if it exists.
 *
 * @param ipu_flags_exists
 *        Denotes whether there is a "IPU bitstream flags" byte
 *        at the beginning of an IPU-encoded frame.
 *        A positive number in the argument (or even just 1) indicate that it exists,
 *        while a negative number (or even just 0) indicate that it doesn't.
 *
 * @param x The specified width of an IPU frame, by a multiple of 16.
 *
 * @param y The specified height of an IPU frame, by a multiple of 16.
 *
 */
void ff_ipu_init_inst_exec_comm_list(AVCodecContext *avctx, int ipu_flags_exist, IPUControlRegister ctrl, int x, int y);

/**
 * Uploads the control registers into the IPU instruction execution context.
 *
 * @param avctx The specified address of an AVCodecContext object, if it exists.
 *
 * @param ctrl
 *        The specified address of an IPUControlRegister object, if it exists.
 *        If null, nothing will be done about it.
 *        Otherwise, it is expected that all aspects of said object be explicitly specified defined from the outset.
 *
 */
void ff_ipu_set_ctrl(AVCodecContext *avctx, IPUControlRegister ctrl);

//IPUInstructionExecutionContext *ff_ipu_retrieve_inst_exec_context();
//void ff_ipu_set_inst_to_exec(IPUInstructionExecutionCommunicationContext *iec, IPUBlockCommand cmd, int w, int h);

/**
 * Sets the instruction execution communication list of an IPU frame from a specified width and height.
 * Can be used in either (or both of) the .read_probe and .read_packet functions
 * of an existing FFInputFormat implementation.
 *
 * @param avctx The specified address of an AVCodecContext object, if it exists.
 *
 * @param cmd The specified IPU block command.
 *
 * @param args The specified IPU IDEC/BDEC command arguments,
 *             taking the form of a IPUIBDECArguments object structure.
 *
 * @param x The specified width of an IPU frame, by a multiple of 16.
 *
 * @param y The specified height of an IPU frame, by a multiple of 16.
 *
 * @param max_x The specified maximum width of an IPU frame, by a multiple of 16.
 *              Said width must already exist beforehand and be greater than w.
 *
 * @param max_y The specified maximum height of an IPU frame, by a multiple of 16.
 *              Said height must already exist beforehand and be greater than h.
 *
 */
void ff_ipu_set_inst_to_exec_from_start(AVCodecContext *avctx, IPUBlockCommand cmd, IPUIBDECArguments args, int x, int y, int max_x, int max_y);
//void ff_ipu_set_inst_to_exec_from_start_to_finish(IPUInstructionExecutionContext *ie, IPUBlockCommand cmd, int starting_i, int ending_i);


void ff_ipu_close_block_inst_comm(AVCodecContext *avctx);

/**
 * IPUInstructionQueueType
 */
typedef enum IPUInstructionQueueType {
    IPU_IQ_dummy00 = 0,
    IPU_IQ_CTRL,
    IPU_IQ_CMD,
    IPU_IQ_end
} IPUInstructionQueueType;

/**
 * IPUInstructionExecutionContext
 */
typedef struct IPUInstructionQueueContext {
    IPUInstructionQueueType iq_type;
    int busy_until_next_ctrl_execution; // signals the need to wait until the entire frame is fully executed (decoded).
    IPUControlRegister next_ctrl; // bitstream decoding options.
    int busy_until_next_cmd_instruction; // signals the need to wait until the entire macroblock is fully executed (decoded).
    IPUBlockCommand next_cmd; // next IPU block command
    IPUIBDECArguments next_args; // next IDEC/BDEC arguments
    int blk_i; // target instruction.
    int blk_i_end; // last instruction;
} IPUInstructionQueueContext;

void ff_ipu_queue_ctrl_exec(AVPacket *avpkt, IPUControlRegister ctrl, int is_busy);

void ff_ipu_queue_inst_exec_from_start(AVPacket *avpkt, IPUBlockCommand next_cmd, IPUIBDECArguments next_args, int x, int y, int max_x, int max_y, int is_busy);

void ff_ipu_execute_queued_ctrl(IPUInstructionQueueType iq_type, IPUInstructionQueueContext *iq, IPUInstructionExecutionContext *ie);

void ff_ipu_execute_queued_cmd(IPUInstructionQueueType iq_type, IPUInstructionQueueContext *iq, IPUInstructionExecutionContext *ie);

size_t ff_ipu_calc_priv_data_skip_to_ipu_iec_ctx(AVCodecContext *avctx);

#endif /* AVCODEC_IPU_H */