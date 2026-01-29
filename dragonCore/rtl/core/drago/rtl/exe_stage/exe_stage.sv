/* -----------------------------------------------
 * Project Name   : DRAC
 * File           : execution.v
 * Organization   : Barcelona Supercomputing Center
 * Author(s)      : Rubén Langarita
 * Email(s)       : ruben.langarita@bsc.es
 * -----------------------------------------------
 * Revision History
 *  Revision   | Author    | Description
 *  0.1        | Victor SP | Remove dcache interface
 * -----------------------------------------------
 */

module exe_stage 
    import drac_pkg::*;
    import riscv_pkg::*;
#(
    parameter drac_pkg::drac_cfg_t DracCfg     = drac_pkg::DracDefaultConfig
)(
    input logic             clk_i,
    input logic             rstn_i,
    input logic             flush_i,

    input logic             csr_interrupt_i, // interrupt detected on the csr
    input bus64_t           csr_interrupt_cause_i,  // which interrupt has been detected

    // INPUTS
    input rr_exe_instr_t    from_rr_i,
    input resp_dcache_cpu_t resp_dcache_cpu_i, // Response from dcache interface
    input logic             en_ld_st_translation_i,
    input logic [1:0]       priv_lvl_i,

    input tlb_cache_comm_t       dtlb_comm_i,
    output cache_tlb_comm_t      dtlb_comm_o,

    // OUTPUTS
    output exe_wb_instr_t   to_wb_o,
    output logic            stall_o,
    
    output req_cpu_dcache_t req_cpu_dcache_o, // Request to dcache interface 
    output logic                     correct_branch_pred_o, // Decides if the branch prediction was correct  
    output exe_if_branch_pred_t      exe_if_branch_pred_o , // Branch prediction (taken, target) and result (take, target)

    //--PMU
    output logic  pmu_is_branch_o       ,
    output logic  pmu_branch_taken_o    ,                    
    output logic  pmu_miss_prediction_o ,
    output logic  pmu_stall_mul_o       ,
    output logic  pmu_stall_div_o       ,
    output logic  pmu_stall_mem_o       
);

    function logic is_inst_branch(instr_type_t instr_type_int);
        return  instr_type_int == BLT  |
                instr_type_int == BLTU |
                instr_type_int == BGE  |
                instr_type_int == BGEU |
                instr_type_int == BEQ  |
                instr_type_int == BNE;

    endfunction

    // Declarations
    bus64_t rs1_alu_data;
    bus64_t rs2_alu_data;

    bus64_t result_alu;
    bus64_t result_mul;
    logic stall_mul;
    bus64_t result_div;
    bus64_t result_rmd;
    logic stall_div;

    branch_decision_t taken_branch;
    addrPC_t result_branch;
    addrPC_t linked_pc;

    bus64_t result_mem;
    logic stall_mem;

    exception_t exception_mem;

    // Bypasses
    `ifdef ASSERTIONS
        always @(posedge clk_i) begin
            if(from_rr_i.instr.rs1 == 0)
                assert from_rr_i.data_rs1==0;
            if(from_rr_i.instr.rs2 == 0)
                assert from_rr_i.data_rs2==0;
        end
    `endif

    // Select rs2 from imm to avoid bypasses
    assign rs1_alu_data = from_rr_i.instr.use_pc ? from_rr_i.instr.pc : from_rr_i.data_rs1;
    assign rs2_alu_data = from_rr_i.instr.use_imm ? from_rr_i.instr.result : from_rr_i.data_rs2;

    alu alu_inst (
        .data_rs1_i     (rs1_alu_data),
        .data_rs2_i     (rs2_alu_data),
        .instr_type_i   (from_rr_i.instr.instr_type),
        .result_o       (result_alu)
    );

    mul_unit mul_unit_inst (
        .clk_i          (clk_i),
        .rstn_i         (rstn_i),
        .kill_mul_i     (kill_i),
        .request_i      (from_rr_i.instr.unit == UNIT_MUL && from_rr_i.instr.valid),
        .func3_i        (from_rr_i.instr.mem_size),
        .int_32_i       (from_rr_i.instr.op_32),
        .src1_i         (from_rr_i.data_rs1),
        .src2_i         (from_rr_i.data_rs2),

        .result_o       (result_mul),
        .stall_o        (stall_mul)
    );

    div_unit div_unit_inst (
        .clk_i          (clk_i),
        .rstn_i         (rstn_i),
        .kill_div_i     (kill_i),
        .request_i      (from_rr_i.instr.unit == UNIT_DIV && from_rr_i.instr.valid),
        .int_32_i       (from_rr_i.instr.op_32),
        .signed_op_i    (from_rr_i.instr.signed_op),
        .dvnd_i         (from_rr_i.data_rs1),
        .dvsr_i         (rs2_alu_data),

        .quo_o          (result_div),
        .rmd_o          (result_rmd),
        .stall_o        (stall_div)
    );

    branch_unit branch_unit_inst (
        .instr_type_i       (from_rr_i.instr.instr_type),
        .pc_i               (from_rr_i.instr.pc),
        .data_rs1_i         (from_rr_i.data_rs1),
        .data_rs2_i         (from_rr_i.data_rs2),
        .imm_i              (from_rr_i.instr.result),

        .taken_o            (taken_branch),
        .result_o           (result_branch),
        .link_pc_o          (linked_pc)
    );

    mem_unit mem_unit_inst (
        .clk_i          (clk_i),
        .rstn_i         (rstn_i),
        .flush_i        (flush_i),
        .valid_i        (from_rr_i.instr.unit == UNIT_MEM && from_rr_i.instr.valid),
        .instruction_i  (from_rr_i),
        .src1_i         (from_rr_i.data_rs1),
        .src2_i         (from_rr_i.data_rs2),
        .en_ld_st_translation_i(en_ld_st_translation_i),
        .priv_lvl_i     (priv_lvl_i),
        .resp_dcache_cpu_i(resp_dcache_cpu_i),
        .dtlb_comm_i    (dtlb_comm_i),
        .req_cpu_dcache_o(req_cpu_dcache_o),
        .dtlb_comm_o    (dtlb_comm_o),
        .result_o       (result_mem),
        .exception_o    (exception_mem),
        .stall_o        (stall_mem)
    );




    //------------------------------------------------------------------------------
    // DATA  TO WRITE_BACK
    //------------------------------------------------------------------------------

    assign to_wb_o.valid = from_rr_i.instr.valid && !stall_o;
    assign to_wb_o.pc = from_rr_i.instr.pc;
    assign to_wb_o.bpred = from_rr_i.instr.bpred;
    assign to_wb_o.rs1 = from_rr_i.instr.rs1;
    assign to_wb_o.rd = from_rr_i.instr.rd;
    assign to_wb_o.change_pc_ena = from_rr_i.instr.change_pc_ena;
    assign to_wb_o.regfile_we = from_rr_i.instr.regfile_we && !stall_o;
    assign to_wb_o.instr_type = from_rr_i.instr.instr_type;
    assign to_wb_o.stall_csr_fence = from_rr_i.instr.stall_csr_fence;
    assign to_wb_o.csr_addr = from_rr_i.instr.result[CSR_ADDR_SIZE-1:0];

    `ifdef SIM_COMMIT_LOG
    assign to_wb_o.inst = from_rr_i.instr.inst;
    `endif
    `ifdef SIM_KONATA_DUMP
    assign to_wb_o.id = from_rr_i.instr.id;
    assign to_wb_o.unit = from_rr_i.instr.unit;
    `endif

    always_comb begin
        to_wb_o.ex.cause  = INSTR_ADDR_MISALIGNED;
        to_wb_o.ex.origin = 0;
        to_wb_o.ex.valid  = 0;
        if(from_rr_i.instr.ex.valid) begin // Bypass exception from previous stages
            to_wb_o.ex = from_rr_i.instr.ex;
        end else if(from_rr_i.instr.valid) begin // Check exceptions in exe stage
            //Interrupt comming from csr, if there are a memory operation is better to finish it
            if(from_rr_i.instr.unit != UNIT_MEM && csr_interrupt_i) begin 
                to_wb_o.ex.cause = exception_cause_t'(csr_interrupt_cause_i);
                to_wb_o.ex.origin = 64'b0;
                to_wb_o.ex.valid = 1;
            end else if (exception_mem.valid) begin
                // mem exception
                to_wb_o.ex.cause = exception_mem.cause;
                to_wb_o.ex.origin = exception_mem.origin;
                to_wb_o.ex.valid = exception_mem.valid;
            end else if (result_branch[1:0] != 0 && 
                        from_rr_i.instr.unit == UNIT_BRANCH && 
                        (from_rr_i.instr.instr_type == JALR || 
                        ( is_inst_branch(from_rr_i.instr.instr_type) &&
                        taken_branch )) &&
                        from_rr_i.instr.valid) begin // invalid address
                to_wb_o.ex.cause = INSTR_ADDR_MISALIGNED;
                to_wb_o.ex.origin = result_branch;
                to_wb_o.ex.valid = 1;
            end 
        end
    end


    always_comb begin
        to_wb_o.branch_taken = 1'b0;
        case(from_rr_i.instr.unit)
            UNIT_ALU: begin
                to_wb_o.result      = result_alu;
            end
            UNIT_MUL: begin
                to_wb_o.result      = result_mul;
            end
            UNIT_DIV: begin
                case(from_rr_i.instr.instr_type)
                    DIV,DIVU,DIVW,DIVUW: begin
                        to_wb_o.result = result_div;
                    end
                    REM,REMU,REMW,REMUW: begin
                        to_wb_o.result = result_rmd;
                    end
                    default: begin
                        to_wb_o.result = 0;
                    end
                endcase
            end
            UNIT_BRANCH: begin
                to_wb_o.branch_taken    = taken_branch;
                to_wb_o.result          = linked_pc;
            end
            UNIT_MEM: begin
                to_wb_o.result      = result_mem;
            end
            UNIT_SYSTEM: begin
                to_wb_o.result      = from_rr_i.data_rs1;
            end
            default: begin
                to_wb_o.result      = 0;
            end
        endcase
    end

    // Branch predictor required signals
    // Program counter at Execution Stage
    assign exe_if_branch_pred_o.pc_execution = from_rr_i.instr.pc; 
    assign to_wb_o.result_pc     = result_branch;

    branch_prediction_checker branch_prediction_checker_inst (
        .stall_o(stall_o),
        .instr_type(from_rr_i.instr.instr_type),
        .bpred(from_rr_i.instr.bpred),
        .taken_branch(taken_branch),
        .result_branch(result_branch),
        .correct_branch_pred_o(correct_branch_pred_o)
    );

    // Address generated by branch in Execution Stage
    assign exe_if_branch_pred_o.branch_addr_result_exe = result_branch; 
    // Taken or not taken branch result in Execution Stage
    assign exe_if_branch_pred_o.branch_taken_result_exe = taken_branch == TAKEN;   
    // The instruction in the Execution Stage is a branch
    assign exe_if_branch_pred_o.is_branch_exe = is_inst_branch(from_rr_i.instr.instr_type);

    assign stall_o = (from_rr_i.instr.valid & from_rr_i.instr.unit == UNIT_MUL) ? stall_mul :
                    (from_rr_i.instr.valid & from_rr_i.instr.unit == UNIT_DIV) ? stall_div :
                    (from_rr_i.instr.valid & from_rr_i.instr.unit == UNIT_MEM) ? stall_mem :
                    0;


    //-PMU 
    assign pmu_is_branch_o       = is_inst_branch(from_rr_i.instr.instr_type) && from_rr_i.instr.valid;
    assign pmu_branch_taken_o    = is_inst_branch(from_rr_i.instr.instr_type) && taken_branch && 
                                from_rr_i.instr.valid            ;
    assign pmu_miss_prediction_o = !correct_branch_pred_o;

    assign pmu_stall_mul_o = from_rr_i.instr.valid & from_rr_i.instr.unit == UNIT_MUL & stall_mul;
    assign pmu_stall_div_o = from_rr_i.instr.valid & from_rr_i.instr.unit == UNIT_DIV & stall_div;
    assign pmu_stall_mem_o = from_rr_i.instr.valid & from_rr_i.instr.unit == UNIT_MEM & stall_mem; 


endmodule

