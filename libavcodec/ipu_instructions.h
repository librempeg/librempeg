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
 * IPUBlockInstructionContext
 */
typedef struct IPUBlockCoordContext {
    // the following 4 vars shall be divisible by 16, otherwise not happening.
    int starting_x;
    int starting_y;
    int ending_x;
    int ending_y;
} IPUBlockCoordContext;

/**
 * IPUInstructionExecutionContext
 */
typedef struct IPUInstructionExecutionContext {
    int inst; // how many instructions do we need from a given frame
    IPUBlockCoordContext *blk_d; // instruction data
} IPUInstructionExecutionContext;

/**
 * Fulfills the execution list of an IPU frame.
 * This is meant to be used in the .init function of an existing FFCodec implementation.
 *
 * @param ie The pointer to the execution context.
 *           Shall come from an existing object beforehand.
 *
 * @param w The specified width of an IPU frame, by a multiple of 16.
 *
 * @param h The specified height of an IPU frame, by a multiple of 16.
 *
 */
void ff_ipu_init_inst_exec_list(IPUInstructionExecutionContext *ie, int w, int h);

/**
 * IPUBlockInstructionCommunicationContext
 */
typedef struct IPUBlockInstructionCommunicationContext {
    IPUBlockCommand cmd; // IPU block command
    IPUIBDECArguments ibd_args; // IDEC/BDEC arguments
    int sync_inst_id; // instruction number synchronization ID
} IPUBlockInstructionCommunicationContext;

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
 * IPUInstructionExecutionCommunicationContext
 */
typedef struct IPUInstructionExecutionCommunicationContext {
    int ipu_flags_exist; // denotes the existence of a byte consisting of IPU bitstream flags.
    IPUControlRegister ctrl; // bitstream decoding options.
    int inst; // how many instructions do we need from a given frame.
    IPUBlockInstructionCommunicationContext *inst_d; // instruction comms data.
} IPUInstructionExecutionCommunicationContext;

/**
 * Fulfills the execution communication list of an IPU frame.
 * This is meant to be used in the .read_probe function of an existing FFInputFormat implementation.
 *
 * @param iec The specified address of an IPUInstructionExecutionCommunicationContext object,
 *            if it exists.
 *
 * @param ipu_flags_exists
 *        Denotes whether there is a "IPU bitstream flags" byte
 *        at the beginning of an IPU-encoded frame.
 *        A positive number in the argument (or even just 1) indicate that it exists,
 *        while a negative number (or even just 0) indicate that it doesn't.
 *
 * @param w The specified width of an IPU frame, by a multiple of 16.
 *
 * @param h The specified height of an IPU frame, by a multiple of 16.
 *
 */
void ff_ipu_init_inst_exec_comm_list(IPUInstructionExecutionCommunicationContext *iec, int ipu_flags_exist, IPUControlRegister ctrl, int w, int h);

/**
 * Fulfills the execution communication list of an IPU frame.
 * This is meant to be used in the .read_probe function of an existing FFInputFormat implementation.
 *
 * @param iec The specified address of an IPUInstructionExecutionCommunicationContext object,
 *            if it exists.
 *
 * @param ctrl
 *        The specified address of an IPUControlRegister object, if it exists.
 *        If null, nothing will be done about it.
 *        Otherwise, it is expected that all aspects of said object be explicitly specified defined from the outset.
 *
 */
void ff_ipu_set_ctrl_from_inst_exec_comm_list(IPUInstructionExecutionCommunicationContext *iec, IPUControlRegister ctrl);

//IPUInstructionExecutionContext *ff_ipu_retrieve_inst_exec_context();
//void ff_ipu_set_inst_to_exec(IPUInstructionExecutionCommunicationContext *iec, IPUBlockCommand cmd, int w, int h);

/**
 * Sets the instruction execution communication list of an IPU frame from a specified width and height.
 * Can be used in either (or both of) the .read_probe and .read_packet functions
 * of an existing FFInputFormat implementation.
 *
 * @param iec The pointer to the execution communication context.
 *            Shall come from an existing object beforehand.
 *
 * @param cmd The specified IPU block command.
 *
 * @param args The specified IPU IDEC/BDEC command arguments,
 *             taking the form of a IPUIBDECArguments object structure.
 *
 * @param w The specified width of an IPU frame, by a multiple of 16.
 *
 * @param h The specified height of an IPU frame, by a multiple of 16.
 *
 * @param max_w The specified maximum width of an IPU frame, by a multiple of 16.
 *              Said width must already exist beforehand and be greater than w.
 *
 * @param max_h The specified maximum height of an IPU frame, by a multiple of 16.
 *              Said height must already exist beforehand and be greater than h.
 *
 */
void ff_ipu_set_inst_to_exec_from_start(IPUInstructionExecutionCommunicationContext *iec, IPUBlockCommand cmd, IPUIBDECArguments args, int w, int h, int max_w, int max_h);
//void ff_ipu_set_inst_to_exec_from_start_to_finish(IPUInstructionExecutionContext *ie, IPUBlockCommand cmd, int starting_i, int ending_i);


void ff_ipu_close_block_inst_comm(IPUInstructionExecutionCommunicationContext *iec);

#endif /* AVCODEC_IPU_H */