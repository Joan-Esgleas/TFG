/* -----------------------------------------------
* Project Name   : DRAC
* File           : control_unit.sv
* Organization   : Barcelona Supercomputing Center
* Author(s)      : Guillem Lopez Paradis
* Email(s)       : guillem.lopez@bsc.es
* References     : 
* -----------------------------------------------
* Revision History
*  Revision   | Author     | Commit | Description
*  0.1        | Guillem.LP | 
*  0.2        | Max D.     |
* -----------------------------------------------
*/


module control_unit
    import drac_pkg::*;
    import riscv_pkg::*;
(
    input logic             clk_i,
    input logic             rstn_i,
    input logic             miss_icache_i,
    input logic             ready_icache_i,
    input logic             if2_cu_valid_i,
    input id_cu_t           id_cu_i,
    input rr_cu_t           rr_cu_i,
    input exe_cu_t          exe_cu_i,
    input wb_cu_t           wb_cu_i,
    input resp_csr_cpu_t    csr_cu_i,
    input logic             correct_branch_pred_i,
    input logic             debug_wr_valid_i,
    input debug_contr_in_t  debug_contr_i,

    output pipeline_ctrl_t  pipeline_ctrl_o,
    output pipeline_flush_t pipeline_flush_o,
    output cu_if_t          cu_if_o,
    output logic            invalidate_icache_o,
    output logic            invalidate_buffer_o,
    output debug_contr_out_t debug_contr_o,
    output logic             debug_csr_halt_ack_o,
    output cu_rr_t          cu_rr_o
);
    logic jump_enable_int;
    logic exception_enable_q, exception_enable_d;
    pipeline_ctrl_t pipeline_ctrl_int;

    logic flush_step_inst;
    logic step_inst_in_pipeline;
    logic step_inst_in_if2;
    logic step_inst_in_id;

    logic csr_enable_d, csr_enable_q;  

    debug_contr_state_t state_debug_q, state_debug_d;
    logic on_halt_state;
    logic debug_progbuf_xcpt_d, debug_progbuf_xcpt_q;  

    logic pipeline_empty;
    assign pipeline_empty = !(if2_cu_valid_i | id_cu_i.valid | rr_cu_i.valid | exe_cu_i.valid | wb_cu_i.valid);

    // Debug mode
    always_comb begin
        debug_contr_o.resume_ack = 1'b0;
        debug_contr_o.halt_ack = 1'b0;
        state_debug_d = state_debug_q;
        on_halt_state = 1'b0;
        debug_contr_o.halted = 1'b0;
        debug_contr_o.running = 1'b0;
        debug_contr_o.parked = 1'b0; 
        debug_contr_o.unavail = 1'b0;
        debug_contr_o.progbuf_ack = 1'b0;
        debug_contr_o.progbuf_xcpt = debug_progbuf_xcpt_q;
        debug_contr_o.havereset = 1'b0;
        debug_csr_halt_ack_o = 1'b0;
        debug_progbuf_xcpt_d = debug_progbuf_xcpt_q;

        case (state_debug_q)
            DEBUG_STATE_RESET: begin
                state_debug_d = DEBUG_STATE_HAVERESET;
                on_halt_state = 1'b1;
            end
            DEBUG_STATE_HAVERESET: begin
                debug_contr_o.havereset = 1'b1;
                if (debug_contr_i.halt_on_reset || debug_contr_i.halt_req) begin
                    state_debug_d = DEBUG_STATE_HALTED;
                    debug_contr_o.halt_ack = 1'b1;
                    on_halt_state = 1'b1;
                    debug_csr_halt_ack_o = 1'b1;
                end else begin
                    state_debug_d = DEBUG_STATE_RESUME;
                    on_halt_state = 1'b0;
                end
            end
            DEBUG_STATE_RESUME: begin
                state_debug_d = DEBUG_STATE_RUNNING;
                debug_contr_o.resume_ack = 1'b1;
                debug_contr_o.running = 1'b1;
            end
            DEBUG_STATE_RUNNING: begin
                if (csr_cu_i.debug_ebreak) begin
                    state_debug_d = DEBUG_STATE_HALTED;
                    debug_contr_o.halt_ack = 1'b1;
                end else if (debug_contr_i.halt_req) begin
                    state_debug_d = DEBUG_STATE_HALTING;
                end 
                on_halt_state = 1'b0;
                debug_contr_o.running = 1'b1;
            end
            DEBUG_STATE_HALTING: begin
                if (pipeline_empty || csr_cu_i.debug_ebreak) begin
                    state_debug_d = DEBUG_STATE_HALTED;
                    debug_contr_o.halt_ack = 1'b1;
                    if (pipeline_empty) begin
                        debug_csr_halt_ack_o = 1'b1;
                    end 
                end 
                on_halt_state = 1'b1;
                debug_contr_o.running = 1'b1;
            end
            DEBUG_STATE_HALTED: begin
                if (debug_contr_i.resume_req) begin
                    state_debug_d = DEBUG_STATE_RESUME;
                end else if (debug_contr_i.progbuf_req) begin
                    state_debug_d = DEBUG_STATE_PROGBUFF;
                    debug_contr_o.progbuf_ack = 1'b1;
                    debug_progbuf_xcpt_d = 1'b0;
                end
                on_halt_state = 1'b1;
                debug_contr_o.halted = 1'b1;
                debug_contr_o.parked = 1'b1;
            end
            DEBUG_STATE_PROGBUFF: begin
                if (debug_contr_i.resume_req) begin
                    state_debug_d = DEBUG_STATE_RESUME;
                end else if (csr_cu_i.debug_ebreak || exception_enable_q) begin
                    state_debug_d = DEBUG_STATE_HALTED;
                    debug_contr_o.halt_ack = 1'b1;
                    debug_progbuf_xcpt_d = exception_enable_q;
                end 
                on_halt_state = 1'b0;
                debug_contr_o.halted = 1'b1;
            end
        endcase
    end

    assign step_inst_in_if2 = (if2_cu_valid_i && csr_cu_i.debug_step && ((state_debug_q == DEBUG_STATE_RUNNING) || (state_debug_q == DEBUG_STATE_RESUME)));
    assign step_inst_in_id = (id_cu_i.valid && csr_cu_i.debug_step && ((state_debug_q == DEBUG_STATE_RUNNING) || (state_debug_q == DEBUG_STATE_RESUME)));

    always_ff@(posedge clk_i, negedge rstn_i)
    begin
        if (~rstn_i)
            step_inst_in_pipeline <= 0;
        else if (flush_step_inst)
            step_inst_in_pipeline <= 0;
        else if (step_inst_in_id)
            step_inst_in_pipeline <= 1;
    end

    // jump enable logic
    always_comb begin
        jump_enable_int =   (wb_cu_i.valid && wb_cu_i.xcpt) ||
                            // branch at exe
                            (exe_cu_i.valid && ~correct_branch_pred_i && !pipeline_ctrl_int.stall_exe) ||
                            // valid jal
                            id_cu_i.valid_jal ||
                            // jump to evec when eret
                            csr_cu_i.csr_eret ||
                            // jump to evec when xcpt from csr
                            csr_cu_i.csr_exception ||
                            // jump to evec when ecall
                            (wb_cu_i.valid && wb_cu_i.ecall_taken);
                            // we can add here the debug jump also
    end

    // set the exception state that will stall the pipeline on cycle to reduce the delay of the CSRs
    assign exception_enable_d = exception_enable_q ? 1'b0 : ((wb_cu_i.valid && wb_cu_i.xcpt) ||
                                                            csr_cu_i.csr_eret ||
                                                            csr_cu_i.csr_exception ||
                                                            csr_cu_i.debug_ebreak ||
                                                            debug_contr_o.halt_ack ||
                                                            (wb_cu_i.valid && wb_cu_i.ecall_taken));



    // set the exception state that will stall the pipeline on cycle to reduce the delay of the CSRs
    assign csr_enable_d = csr_enable_q ? 1'b0 : (wb_cu_i.valid && wb_cu_i.stall_csr_fence) &&
                                                            !((wb_cu_i.valid && wb_cu_i.xcpt) || 
                                                            csr_cu_i.csr_eret || 
                                                            csr_cu_i.csr_exception ||  
                                                            csr_cu_i.debug_ebreak ||
                                                            debug_contr_o.halt_ack ||
                                                            (wb_cu_i.valid && wb_cu_i.ecall_taken));


    // logic enable write register file at commit
    always_comb begin
        // we don't allow regular reads/writes if not halted
        if ((wb_cu_i.valid &&
            !wb_cu_i.xcpt &&
            !csr_cu_i.csr_exception &&
            wb_cu_i.write_enable)
            || (debug_wr_valid_i && on_halt_state)) 
        begin
            cu_rr_o.write_enable = 1'b1;
        end else begin
            cu_rr_o.write_enable = 1'b0;
        end
    end

    always_comb begin
        // we don't allow regular reads/writes if not halted
        if (debug_wr_valid_i && on_halt_state) begin
            cu_rr_o.write_enable_dbg = 1'b1;
        end else begin
            cu_rr_o.write_enable_dbg = 1'b0;
        end
    end

    // logic to select the next pc
    always_comb begin
        // branches or valid jal
        if (jump_enable_int || exception_enable_q || csr_enable_q) begin
            cu_if_o.next_pc = NEXT_PC_SEL_JUMP;
        end else if (pipeline_ctrl_int.stall_if_1 ||
                     step_inst_in_if2         ||
                     id_cu_i.stall_csr_fence  ||
                     rr_cu_i.stall_csr_fence  ||
                     exe_cu_i.stall_csr_fence ||
                     step_inst_in_pipeline    || 
                     (wb_cu_i.valid && wb_cu_i.fence) ||
                     on_halt_state)  begin
            cu_if_o.next_pc = NEXT_PC_SEL_KEEP_PC;
        end else begin
            cu_if_o.next_pc = NEXT_PC_SEL_BP_OR_PC_4;
        end
    end

    // logic to select which pc to use in fetch
    always_comb begin
        // if exception or eret select from csr
        if (exception_enable_q) begin
            pipeline_ctrl_int.sel_addr_if = SEL_JUMP_CSR;
        end else if (csr_enable_q) begin
            pipeline_ctrl_int.sel_addr_if = SEL_JUMP_CSR_RW;
        end else if (exe_cu_i.valid && ~correct_branch_pred_i && !pipeline_ctrl_int.stall_exe) begin
            pipeline_ctrl_int.sel_addr_if = SEL_JUMP_EXECUTION;
        end else begin
            pipeline_ctrl_int.sel_addr_if = SEL_JUMP_DECODE;
        end
    end

    // logic invalidate icache
    // when there is a fence, it could be a self modifying code
    // invalidate icache
    assign invalidate_icache_o = (wb_cu_i.valid && wb_cu_i.fence_i);
    // logic invalidate buffer and repeat fetch
    // when a fence, invalidate buffer and also when csr eret
    // when it is a csr it should be checked more?
    assign invalidate_buffer_o = (wb_cu_i.valid && (wb_cu_i.fence_i | 
                                                    exception_enable_q |
                                                    debug_contr_o.halt_ack |
                                                    (wb_cu_i.stall_csr_fence & !wb_cu_i.fence)));


    // logic about flush the pipeline if branch
    always_comb begin
        // if exception
        pipeline_flush_o.flush_if  = 1'b0;
        pipeline_flush_o.flush_id  = 1'b0;
        pipeline_flush_o.flush_rr  = 1'b0;
        pipeline_flush_o.flush_exe = 1'b0;
        pipeline_flush_o.flush_wb  = 1'b0;
        flush_step_inst            = 1'b0;
        if ((wb_cu_i.xcpt & wb_cu_i.valid) || exception_enable_q) begin
            pipeline_flush_o.flush_if  = 1'b1;
            pipeline_flush_o.flush_id  = 1'b1;
            pipeline_flush_o.flush_rr  = 1'b1;
            pipeline_flush_o.flush_exe = 1'b1;
            pipeline_flush_o.flush_wb  = 1'b0;
            flush_step_inst            = 1'b1;
        end else if (csr_enable_q) begin
            pipeline_flush_o.flush_if  = 1'b1;
            pipeline_flush_o.flush_id  = 1'b1;
            pipeline_flush_o.flush_rr  = 1'b1;
            pipeline_flush_o.flush_exe = 1'b1;
            pipeline_flush_o.flush_wb  = 1'b0;
        end else if ((exe_cu_i.valid && ~correct_branch_pred_i && !pipeline_ctrl_int.stall_exe)) begin
            pipeline_flush_o.flush_if  = 1'b1;
            pipeline_flush_o.flush_id  = 1'b1;
            pipeline_flush_o.flush_rr  = 1'b1;
            pipeline_flush_o.flush_exe = 1'b0;
            pipeline_flush_o.flush_wb  = 1'b0;
        end else if ((id_cu_i.stall_csr_fence |
                     rr_cu_i.stall_csr_fence  |
                     exe_cu_i.stall_csr_fence |
                     step_inst_in_pipeline    |
                     wb_cu_i.stall_csr_fence )  && !(rr_cu_i.data_dependency || csr_cu_i.csr_stall || exe_cu_i.stall)) begin
            pipeline_flush_o.flush_if  = 1'b1;
            pipeline_flush_o.flush_id  = 1'b0;
            pipeline_flush_o.flush_rr  = 1'b0;
            pipeline_flush_o.flush_exe = 1'b0;
            pipeline_flush_o.flush_wb  = 1'b0;
        end else if ((id_cu_i.valid_jal ||
                    (wb_cu_i.valid && wb_cu_i.fence)) && !(rr_cu_i.data_dependency || csr_cu_i.csr_stall || exe_cu_i.stall)) begin
            pipeline_flush_o.flush_if  = 1'b1;
            pipeline_flush_o.flush_id  = 1'b0;
            pipeline_flush_o.flush_rr  = 1'b0;
            pipeline_flush_o.flush_exe = 1'b0;
            pipeline_flush_o.flush_wb  = 1'b0;
        end
    end


    // logic about stall the pipeline if branch
    always_comb begin
        // if exception
        if (csr_cu_i.csr_stall) begin
            pipeline_ctrl_int.stall_if_1  = 1'b1;
            pipeline_ctrl_int.stall_if_2  = 1'b1;
            pipeline_ctrl_int.stall_id  = 1'b1;
            pipeline_ctrl_int.stall_rr  = 1'b1;
            pipeline_ctrl_int.stall_exe = 1'b1;
            pipeline_ctrl_int.stall_wb  = 1'b0;
        end else if (exe_cu_i.stall) begin
            pipeline_ctrl_int.stall_if_1  = 1'b1;
            pipeline_ctrl_int.stall_if_2  = 1'b1;
            pipeline_ctrl_int.stall_id  = 1'b1;
            pipeline_ctrl_int.stall_rr  = 1'b1;
            pipeline_ctrl_int.stall_exe = 1'b0;
            pipeline_ctrl_int.stall_wb  = 1'b0;
        end else if (rr_cu_i.data_dependency) begin
            pipeline_ctrl_int.stall_if_1  = 1'b1;
            pipeline_ctrl_int.stall_if_2  = 1'b1;
            pipeline_ctrl_int.stall_id  = 1'b1;
            pipeline_ctrl_int.stall_rr  = 1'b0;
            pipeline_ctrl_int.stall_exe = 1'b0;
            pipeline_ctrl_int.stall_wb  = 1'b0;
        end else if (miss_icache_i) begin
            pipeline_ctrl_int.stall_if_1  = 1'b1;
            pipeline_ctrl_int.stall_if_2  = 1'b1;
            pipeline_ctrl_int.stall_id  = 1'b0;
            pipeline_ctrl_int.stall_rr  = 1'b0;
            pipeline_ctrl_int.stall_exe = 1'b0;
            pipeline_ctrl_int.stall_wb  = 1'b0;
        end else if (wb_cu_i.valid && wb_cu_i.stall_csr_fence ||
                     (!miss_icache_i && !ready_icache_i) || 
                     step_inst_in_pipeline               ||
                     on_halt_state                       ||
                     step_inst_in_if2) begin
            pipeline_ctrl_int.stall_if_1  = 1'b1;
            pipeline_ctrl_int.stall_if_2  = 1'b0;
            pipeline_ctrl_int.stall_id  = 1'b0;
            pipeline_ctrl_int.stall_rr  = 1'b0;
            pipeline_ctrl_int.stall_exe = 1'b0;
            pipeline_ctrl_int.stall_wb  = 1'b0;
        end else begin
            pipeline_ctrl_int.stall_if_1  = 1'b0;
            pipeline_ctrl_int.stall_if_2  = 1'b0;
            pipeline_ctrl_int.stall_id  = 1'b0;
            pipeline_ctrl_int.stall_rr  = 1'b0;
            pipeline_ctrl_int.stall_exe = 1'b0;
            pipeline_ctrl_int.stall_wb  = 1'b0;
        end
    end

    assign pipeline_ctrl_o = pipeline_ctrl_int;

    always_ff @(posedge clk_i, negedge rstn_i) begin
        if(!rstn_i) begin
            exception_enable_q <= 1'b0;
            csr_enable_q <= 1'b0;
        end else begin 
            exception_enable_q <= exception_enable_d;
            csr_enable_q <= csr_enable_d;
        end
    end
    always_ff @(posedge clk_i, negedge rstn_i) begin
        if(!rstn_i) begin
            state_debug_q <= DEBUG_STATE_RESET;
            debug_progbuf_xcpt_q <= 1'b0;
        end else begin 
            state_debug_q <= state_debug_d;
            debug_progbuf_xcpt_q <= debug_progbuf_xcpt_d;
        end
    end

endmodule
