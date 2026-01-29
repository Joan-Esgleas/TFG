/* -----------------------------------------------
* Project Name   : DRAC
* File           : datapath.sv
* Organization   : Barcelona Supercomputing Center
* Author(s)      : Guillem Lopez Paradis
* Email(s)       : guillem.lopez@bsc.es
* References     : 
* -----------------------------------------------
* Revision History
*  Revision   | Author     | Commit | Description
*  0.1        | Guillem.LP | 
* -----------------------------------------------
*/


module datapath
    import drac_pkg::*;
    import riscv_pkg::*;
#(
    parameter drac_pkg::drac_cfg_t DracCfg     = drac_pkg::DracDefaultConfig
)(
    input logic             clk_i,
    input logic             rstn_i,
    input addr_t            reset_addr_i,
    // icache/dcache/CSR interface input
    input resp_icache_cpu_t resp_icache_cpu_i,
    input resp_dcache_cpu_t resp_dcache_cpu_i,
    input resp_csr_cpu_t    resp_csr_cpu_i,
    input logic             en_translation_i,
    input logic             en_ld_st_translation_i,
    input debug_reg_in_t    debug_reg_i,
    input debug_contr_in_t  debug_contr_i,
    input logic [1:0]       csr_priv_lvl_i,
    input logic             req_icache_ready_i,
    input tlb_cache_comm_t  dtlb_comm_i,
    // icache/dcache/CSR interface output
    output req_cpu_dcache_t req_cpu_dcache_o, 
    output req_cpu_icache_t req_cpu_icache_o,
    output req_cpu_csr_t    req_cpu_csr_o,
    output debug_reg_out_t  debug_reg_o,
    output debug_contr_out_t debug_contr_o,
    output logic            debug_csr_halt_ack_o,
    output cache_tlb_comm_t dtlb_comm_o,
    //--PMU   
    output to_PMU_t         pmu_flags_o
);

`ifdef SIM_COMMIT_LOG
    logic commit_valid;
    commit_data_t commit_data;
`endif

    bus64_t pc_if, pc_id, pc_rr, pc_exe, pc_wb;
    logic valid_if, valid_id, valid_rr, valid_exe, valid_wb;

    pipeline_ctrl_t control_int;
    pipeline_flush_t flush_int;
    cu_if_t cu_if_int;
    addrPC_t pc_jump_if_int;
    addrPC_t pc_evec_q;
    addrPC_t pc_next_csr_q;

    // Pipelines stages data
    // Fetch
    if_1_if_2_stage_t stage_if_1_if_2_d; // this is the saving in the current cycle
    if_1_if_2_stage_t stage_if_1_if_2_q; // this is the next or output of reg
    if_id_stage_t stage_if_2_id_d; 
    if_id_stage_t stage_if_2_id_q; 
    logic invalidate_icache_int;
    logic invalidate_buffer_int;
    logic retry_fetch;
    // Decode
    instr_entry_t stage_id_rr_d;
    instr_entry_t stage_id_rr_q;
    // RR
    rr_exe_instr_t stage_rr_exe_d;
    rr_exe_instr_t stage_rr_exe_q;

    // Control Unit Decode
    id_cu_t id_cu_int;
    jal_id_if_t jal_id_if_int;

    // Exe
    logic stall_exe_out;
    /* verilator lint_off UNOPTFLAT */
    exe_cu_t exe_cu_int;
    /* verilator lint_on UNOPTFLAT */
    exe_wb_instr_t exe_to_wb_exe;
    exe_wb_instr_t exe_to_wb_wb;

    exe_if_branch_pred_t exe_if_branch_pred_int;

    //Bypass wires
    wb_exe_instr_t wb_to_exe_exe;
    logic wb_xcpt;

    // WB->Commit
    wb_cu_t wb_cu_int;
    rr_cu_t rr_cu_int;
    cu_rr_t cu_rr_int;

    // wb csr
    logic   wb_csr_ena_int;

    // data to write to RR from wb
    bus64_t data_wb_rr_int;

    // codifies if the branch was correctly predicted
    // this signal goes from exe stage to fetch stage
    logic correct_branch_pred;

    // Debug signals
    bus64_t  reg_wr_data;
    //logic    reg_wr_enable;
    logic [REGFILE_WIDTH-1:0] reg_wr_addr;
    logic [REGFILE_WIDTH-1:0] reg_rd1_addr;
    // stall IF
    logic stall_if;
    logic miss_icache;
    `ifdef SIM_KONATA_DUMP
        bus64_t id_fetch;
    `endif

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// CONTROL UNIT                                                                                 /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Control Unit
    control_unit control_unit_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .miss_icache_i(miss_icache),
        .if2_cu_valid_i(stage_if_1_if_2_q.valid),
        .ready_icache_i(req_icache_ready_i),
        .rr_cu_i(rr_cu_int),
        .cu_rr_o(cu_rr_int),
        .wb_cu_i(wb_cu_int),
        .exe_cu_i(exe_cu_int),
        .csr_cu_i(resp_csr_cpu_i),
        .pipeline_ctrl_o(control_int),
        .pipeline_flush_o(flush_int),
        .cu_if_o(cu_if_int),
        .invalidate_icache_o(invalidate_icache_int),
        .invalidate_buffer_o(invalidate_buffer_int),
        .id_cu_i(id_cu_int),
        .correct_branch_pred_i(correct_branch_pred),
        .debug_wr_valid_i(debug_reg_i.rf_we),
        .debug_contr_i(debug_contr_i),
        .debug_contr_o(debug_contr_o),
        .debug_csr_halt_ack_o(debug_csr_halt_ack_o)
    );

    // Combinational logic select the jump addr
    // from decode or wb 
    always_comb begin
        retry_fetch = 1'b0;
        if (control_int.sel_addr_if == SEL_JUMP_EXECUTION) begin
            pc_jump_if_int = exe_if_branch_pred_int.branch_addr_result_exe;
        end else if (control_int.sel_addr_if == SEL_JUMP_CSR) begin
            pc_jump_if_int = pc_evec_q;
            retry_fetch = 1'b1;
        end else if (control_int.sel_addr_if == SEL_JUMP_CSR_RW) begin
            pc_jump_if_int = pc_next_csr_q;
            retry_fetch = 1'b1;   
        end else if (control_int.sel_addr_if == SEL_JUMP_DECODE) begin
            pc_jump_if_int = jal_id_if_int.jump_addr;
        end else begin
            `ifdef ASSERTIONS
                assert (1 == 0);
            `endif
        end
    end

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// FETCH                  STAGE                                                                 /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // IF Stage
    if_stage_1 #(
        .DracCfg(DracCfg)
    ) if_stage_1_inst (
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .reset_addr_i(reset_addr_i),
        .stall_debug_i(debug_contr_o.parked),
        .stall_i(control_int.stall_if_1),
        .cu_if_i(cu_if_int),
        .invalidate_icache_i(invalidate_icache_int),
        .invalidate_buffer_i(invalidate_buffer_int),
        .en_translation_i(en_translation_i), 
        .pc_jump_i(pc_jump_if_int),
        .retry_fetch_i(retry_fetch),
        .req_cpu_icache_o(req_cpu_icache_o),
        .fetch_o(stage_if_1_if_2_d),
        `ifdef SIM_KONATA_DUMP
        .id_o(id_fetch),
        `endif
        .exe_if_branch_pred_i(exe_if_branch_pred_int)
    );

    register #($bits(if_1_if_2_stage_t)) reg_if_1_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .flush_i(flush_int.flush_if),
        .load_i(!control_int.stall_if_1),
        .input_i(stage_if_1_if_2_d),
        .output_o(stage_if_1_if_2_q)
    );

    if_stage_2 if_stage_2_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .fetch_i(stage_if_1_if_2_q),
        .stall_i(control_int.stall_if_2),
        .flush_i(flush_int.flush_if),
        .resp_icache_cpu_i(resp_icache_cpu_i),
        .fetch_o(stage_if_2_id_d),
        .stall_o(miss_icache)
    );

    // Register IF to ID
    register #($bits(if_id_stage_t)) reg_if_2_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .flush_i(flush_int.flush_if),
        .load_i(!control_int.stall_id),
        .input_i(stage_if_2_id_d),
        .output_o(stage_if_2_id_q)
    );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// DECODER                           STAGE                                                      /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // ID Stage
    decoder id_decode_inst(
        .decode_i(stage_if_2_id_q),
        .decode_instr_o(stage_id_rr_d),
        .debug_mode_en_i(debug_contr_o.halted),
        .jal_id_if_o(jal_id_if_int)
    );

    // valid jal in decode
    assign id_cu_int.valid = stage_id_rr_d.valid;
    assign id_cu_int.valid_jal = jal_id_if_int.valid;
    assign id_cu_int.stall_csr_fence = stage_id_rr_d.stall_csr_fence && stage_id_rr_d.valid;

    // Register ID to RR
    register #($bits(instr_entry_t)) reg_id_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .flush_i(flush_int.flush_id),
        .load_i(!control_int.stall_id),
        .input_i(stage_id_rr_d),
        .output_o(stage_id_rr_q)
    );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// READ REGISTER  STAGE                                                                         /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    assign reg_wr_data   = (debug_reg_i.rf_we  && debug_contr_o.parked) ? debug_reg_i.rf_wdata : data_wb_rr_int;
    assign reg_wr_addr   = (debug_reg_i.rf_we  && debug_contr_o.parked)  ? debug_reg_i.rf_preg : exe_to_wb_wb.rd;
    assign reg_rd1_addr  = (debug_reg_i.rf_en  && debug_contr_o.parked)  ? debug_reg_i.rf_preg : stage_id_rr_q.rs1;

    // check data dependency between RR with EXE and WB
    // if there is a dependency, stall the pipeline

    `ifdef BYPASS_ENABLE
        // When the bypass is enabled, we do not need to check the data dependency
        assign rr_cu_int.data_dependency = 1'b0;
    `else
        // TODO: complete the data dependency logic for the lab4
        // Cheching data dependency
        // 1. check dependency with EXE stage
        // 2. check dependency with WB stage
        assign rr_cu_int.data_dependency = ...;
    `endif

    // RR Stage
    regfile rr_stage_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),

        .write_enable_i(cu_rr_int.write_enable | cu_rr_int.write_enable_dbg),
        .write_addr_i(reg_wr_addr),
        .write_data_i(reg_wr_data),
        
        .read_addr1_i(reg_rd1_addr),
        .read_addr2_i(stage_id_rr_q.rs2),
        .read_data1_o(stage_rr_exe_d.data_rs1),
        .read_data2_o(stage_rr_exe_d.data_rs2)
    );

    assign stage_rr_exe_d.csr_interrupt_cause = resp_csr_cpu_i.csr_interrupt_cause;
    assign stage_rr_exe_d.csr_interrupt = resp_csr_cpu_i.csr_interrupt;

    always_comb begin
        if (rr_cu_int.data_dependency) begin
            stage_rr_exe_d.instr = '0;
        end else begin
            stage_rr_exe_d.instr = stage_id_rr_q;
        end
    end

    // We do not have renaming (preg == vreg)
    assign debug_reg_o.rnm_read_resp = debug_reg_i.rnm_read_reg;
    assign debug_reg_o.rf_rdata = stage_rr_exe_d.data_rs1;


    assign rr_cu_int.stall_csr_fence = stage_rr_exe_d.instr.stall_csr_fence && stage_rr_exe_d.instr.valid;
    assign rr_cu_int.valid = stage_rr_exe_d.instr.valid;

    // Register RR to EXE
    register #($bits(stage_rr_exe_d)) reg_rr_inst (
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .flush_i(flush_int.flush_rr),
        .load_i(!control_int.stall_rr),
        .input_i(stage_rr_exe_d),
        .output_o(stage_rr_exe_q)
    );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////// EXECUTION STAGE                                                                              /////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Bypass from WB to EXE
    rr_exe_instr_t from_rr_exe;
    
    always_comb begin
        from_rr_exe = stage_rr_exe_q;
        `ifdef BYPASS_ENABLE
            // TODO: complete the bypass logic for the lab4
            // bypass from WB to EXE
            from_rr_exe.data_rs1 = ...;
            from_rr_exe.data_rs2 = ...;
        `else
            from_rr_exe.data_rs1 = stage_rr_exe_q.data_rs1;
            from_rr_exe.data_rs2 = stage_rr_exe_q.data_rs2;
        `endif
    end

    exe_stage #(
        .DracCfg(DracCfg)
    ) exe_stage_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),

        .flush_i(flush_int.flush_exe),
        .csr_interrupt_i(from_rr_exe.csr_interrupt),
        .csr_interrupt_cause_i(from_rr_exe.csr_interrupt_cause),

        .from_rr_i(from_rr_exe),

        .resp_dcache_cpu_i(resp_dcache_cpu_i),
        .dtlb_comm_i    (dtlb_comm_i),

        .en_ld_st_translation_i(en_ld_st_translation_i),
        .to_wb_o(exe_to_wb_exe),
        .stall_o(exe_cu_int.stall),
        .priv_lvl_i(csr_priv_lvl_i),

        .req_cpu_dcache_o(req_cpu_dcache_o),
        .dtlb_comm_o    (dtlb_comm_o),
        .exe_if_branch_pred_o(exe_if_branch_pred_int),
        .correct_branch_pred_o(correct_branch_pred),

        //PMU Neiel-Leyva
        .pmu_is_branch_o        ( pmu_flags_o.is_branch     ),      
        .pmu_branch_taken_o     ( pmu_flags_o.branch_taken  ),   
        .pmu_miss_prediction_o  ( pmu_flags_o.branch_miss   ),
        .pmu_stall_mul_o        ( pmu_flags_o.stall_rr      ),
        .pmu_stall_div_o        ( pmu_flags_o.stall_exe     ),
        .pmu_stall_mem_o        ( pmu_flags_o.stall_wb      )
    );

    assign exe_cu_int.valid = from_rr_exe.instr.valid;
    assign exe_cu_int.change_pc_ena = from_rr_exe.instr.change_pc_ena;
    assign exe_cu_int.stall_csr_fence = from_rr_exe.instr.stall_csr_fence && from_rr_exe.instr.valid;

    register #($bits(exe_wb_instr_t)) reg_exe_inst(
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .flush_i(flush_int.flush_exe),
        .load_i(!control_int.stall_exe),
        .input_i(exe_to_wb_exe),
        .output_o(exe_to_wb_wb)
    );

    bus64_t debug_pc_int;
    // pc of the ebreak_instruction or pc-4 if single stepping, because an additional ebreak was inserted
    assign debug_pc_int = stage_if_1_if_2_d.pc_inst;

    csr_interface csr_interface_inst(
        .wb_xcpt_i(wb_xcpt),
        .exe_to_wb_wb_i(exe_to_wb_wb),
        .stall_exe_i(control_int.stall_exe),
        .wb_csr_ena_int_o(wb_csr_ena_int),
        .debug_pc_valid_i((resp_csr_cpu_i.debug_step || debug_contr_o.halt_ack)),
        .debug_pc_i(debug_pc_int),
        .debug_mode_en_i(debug_contr_o.halted),
        .req_cpu_csr_o(req_cpu_csr_o)
    );

    // if there is an exception that can be from:
    // the instruction itself or the interrupt
    assign wb_xcpt = exe_to_wb_wb.ex.valid;
    
        // Delay the PC_EVEC treatment one cycle
    always_ff @(posedge clk_i, negedge rstn_i) begin
        if(!rstn_i) begin
            pc_evec_q <= 'b0;
            pc_next_csr_q <= 'b0;
        end else begin 
            pc_evec_q <= resp_csr_cpu_i.csr_evec;
            pc_next_csr_q <= exe_to_wb_wb.pc + 64'h4;
        end
    end


    // data to write to regfile at WB from CSR or exe stage
    assign data_wb_rr_int = (wb_csr_ena_int) ?  resp_csr_cpu_i.csr_rw_rdata :
                                                exe_to_wb_wb.result;

    // For bypasses
    // IMPORTANT: since we can not do bypassig of a  CSR, we will not take into acount the case 
    // of forwarding the result of a CSR to increasse the frequency
    assign wb_to_exe_exe.valid  = exe_to_wb_wb.regfile_we && exe_to_wb_wb.valid;
    assign wb_to_exe_exe.rd     = exe_to_wb_wb.rd;
    assign wb_to_exe_exe.data   = exe_to_wb_wb.result;

    // Control Unit
    assign wb_cu_int.valid = exe_to_wb_wb.valid;//; & !control_int.stall_wb; // and not flush???
    assign wb_cu_int.change_pc_ena = exe_to_wb_wb.change_pc_ena;
    assign wb_cu_int.csr_enable_wb = wb_csr_ena_int;
    assign wb_cu_int.stall_csr_fence = exe_to_wb_wb.stall_csr_fence && exe_to_wb_wb.valid;
    assign wb_cu_int.xcpt = wb_xcpt;
    assign wb_cu_int.write_enable = exe_to_wb_wb.regfile_we;
    assign wb_cu_int.ecall_taken = ~(debug_contr_o.halted) && (exe_to_wb_wb.instr_type == ECALL ||
                                    exe_to_wb_wb.instr_type == MRTS  ||
                                    exe_to_wb_wb.instr_type == EBREAK );
    // tell cu that there is a fence or fence_i
    assign wb_cu_int.fence = (exe_to_wb_wb.instr_type == FENCE_I || 
                              exe_to_wb_wb.instr_type == FENCE || 
                              exe_to_wb_wb.instr_type == SFENCE_VMA);
    // tell cu there is a fence i to flush the icache
    assign wb_cu_int.fence_i = (exe_to_wb_wb.instr_type == FENCE_I || 
                                exe_to_wb_wb.instr_type == SFENCE_VMA);

    // Debug Ring signals Output
    assign debug_reg_o.rf_rdata = stage_rr_exe_d.data_rs1;

`ifdef SIM_COMMIT_LOG
    // Debug signals
    assign commit_valid     = exe_to_wb_wb.valid;
    assign commit_pc        = (exe_to_wb_wb.valid) ? exe_to_wb_wb.pc : 64'b0;
    assign commit_data      = (exe_to_wb_wb.valid) ? data_wb_rr_int  : 64'b0;
    assign commit_addr_reg  = exe_to_wb_wb.rd;
    assign commit_reg_we    = exe_to_wb_wb.regfile_we && exe_to_wb_wb.valid;
    assign commit_branch_taken = exe_to_wb_wb.branch_taken;

    // PC
    assign pc_if1  = stage_if_1_if_2_d.pc_inst;
    assign pc_if2  = stage_if_2_id_d.pc_inst;
    assign pc_id  = (valid_id)  ? stage_id_rr_d.pc : 64'b0;
    assign pc_rr  = (valid_rr)  ? stage_rr_exe_d.instr.pc : 64'b0;
    assign pc_exe = (valid_exe) ? from_rr_exe.instr.pc : 64'b0;
    assign pc_wb  = (valid_wb)  ? exe_to_wb_wb.pc : 64'b0;
    
    // Valid
    assign valid_if1  = stage_if_1_if_2_d.valid;
    assign valid_if2  = stage_if_2_id_d.valid;
    assign valid_id  = stage_id_rr_d.valid;
    assign valid_rr  = stage_rr_exe_d.instr.valid;
    assign valid_exe = from_rr_exe.instr.valid;
    assign valid_wb  = exe_to_wb_wb.valid;

    // Debug signals
    always_comb begin 

        commit_valid = exe_to_wb_wb.valid;

        commit_data.pc              = (exe_to_wb_wb.valid) ? exe_to_wb_wb.pc : 64'b0;
        commit_data.dst             = exe_to_wb_wb.rd;
        commit_data.reg_wr_valid    = exe_to_wb_wb.regfile_we && exe_to_wb_wb.valid && exe_to_wb_wb.rd != 5'b0;
        commit_data.csr_wr_valid    =
            (exe_to_wb_wb.instr_type inside {CSRRW, CSRRWI} ) ||
            (exe_to_wb_wb.instr_type inside {CSRRS, CSRRC, CSRRSI, CSRRCI} && exe_to_wb_wb.rs1 != 5'b0);
        commit_data.csr_dst         = req_cpu_csr_o.csr_rw_addr;
        commit_data.csr_data        = req_cpu_csr_o.csr_rw_data;
        commit_data.inst            = exe_to_wb_wb.inst;
        commit_data.xcpt            = exe_to_wb_wb.ex.valid;
        commit_data.xcpt_cause      = exe_to_wb_wb.ex.cause;
        commit_data.csr_priv_lvl    = csr_priv_lvl_i;
        commit_data.csr_rw_data     = req_cpu_csr_o.csr_rw_data;
        commit_data.csr_xcpt        = resp_csr_cpu_i.csr_exception;
        commit_data.csr_xcpt_cause  = resp_csr_cpu_i.csr_exception_cause;
        commit_data.csr_tval        = resp_csr_cpu_i.csr_tval;
        commit_data.mem_type        = exe_to_wb_wb.mem_type;
        commit_data.mem_addr        = exe_to_wb_wb.addr;
        commit_data.data = data_wb_rr_int;
    end


    // Module that generates the signature of the core to compare with spike
    `ifdef SIM_COMMIT_LOG_DPI
        commit_log_behav commit_log
        (
            .clk(clk_i),
            .rst(rstn_i),
            .commit_valid_i(commit_valid),
            .commit_data_i(commit_data)
        );
    `endif

    `ifdef SIM_KONATA_DUMP
    konata_dump_behav konata_dump
    (
        .clk(clk_i),
        .rst(rstn_i),
        .if1_valid(valid_if1),
        .if1_id(id_fetch), 
        .if1_stall(control_int.stall_if_1),
        .if1_flush(flush_int.flush_if),

        .if2_valid(valid_if2),
        .if2_id(stage_if_2_id_d.id),
        .if2_stall(control_int.stall_if_2),
        .if2_flush(flush_int.flush_if),

        .id_valid(valid_id),
        .id_inst(stage_if_2_id_q.inst),
        .id_pc(pc_id),
        .id_id(stage_if_2_id_q.id),
        .id_stall(control_int.stall_id),
        .id_flush(flush_int.flush_id),

        .rr_valid(valid_rr),
        .rr_id(stage_id_rr_q.id),
        .rr_stall(control_int.stall_rr),
        .rr_flush(flush_int.flush_rr),

        .exe_valid(valid_exe),
        .exe_id(from_rr_exe.instr.id),
        .exe_stall(control_int.stall_exe),
        .exe_flush(flush_int.flush_exe),
        .exe_unit(from_rr_exe.instr.unit),

        .wb_valid(valid_wb),
        .wb_id(exe_to_wb_wb.id),
        .wb_unit(exe_to_wb_wb.unit)
    );
    `endif
`endif

    //PMU
    assign pmu_flags_o.stall_if     = control_int.stall_if_1; 
    assign pmu_flags_o.stall_id     = control_int.stall_if_2; 
    assign pmu_flags_o.stall_rr     = control_int.stall_id; 
    assign pmu_flags_o.stall_exe    = control_int.stall_rr;
    assign pmu_flags_o.stall_data_dependency = rr_cu_int.data_dependency && !control_int.stall_rr;
    assign pmu_flags_o.stall_structural = exe_cu_int.stall;
endmodule
