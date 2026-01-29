/* -----------------------------------------------
 * Project Name   : DRAC
 * File           : mem_unit.sv
 * Organization   : Barcelona Supercomputing Center
 * Author(s)      : Max Doblas
 * Email(s)       : max.doblas@bsc.es
 * -----------------------------------------------
 * Revision History
 *  Revision   | Author   | Description
 *  0.1        | Max D    |  
 * -----------------------------------------------
 */

module mem_unit 
    import drac_pkg::*;
    import riscv_pkg::*;
#(
    parameter drac_pkg::drac_cfg_t DracCfg     = drac_pkg::DracDefaultConfig
)(
    input  wire                  clk_i,                  // Clock signal
    input  wire                  rstn_i,                 // Reset signal
    input logic                  valid_i,                // Valid
    input logic                  flush_i,                // Flush signal
    input rr_exe_instr_t         instruction_i,          // Interface to add new instuction

    input bus64_t                src1_i,
    input bus64_t                src2_i,

    input logic                  en_ld_st_translation_i,
    input resp_dcache_cpu_t      resp_dcache_cpu_i,      // Response from dcache
    input tlb_cache_comm_t       dtlb_comm_i,

    output req_cpu_dcache_t      req_cpu_dcache_o,       // Request to dcache
    output bus64_t               result_o,               // Output instruction
    output exception_t           exception_o,            // Exception of the commit instruction
    output logic                 stall_o,                // Mem unit is able to accept more petitions
    output cache_tlb_comm_t      dtlb_comm_o,

    input logic [1:0] priv_lvl_i
);

    logic [1:0] state;
    logic [1:0] next_state;
    bus64_t vaddr;
    bus64_t paddr;
    logic translate_enable;

    bus64_t src1_q, src1_d;
    bus64_t src2_q, src2_d;

    logic reg_valid_req;
    logic reg_ready_resp;
    logic killed_dcache_req_d;
    logic killed_dcache_req_q;
    logic req_cpu_dcache_valid_int;
    logic stall_after_flush_lsq;

    logic is_load;
    logic is_store;
    logic is_amo;

    assign is_load = instruction_i.instr.mem_type == LOAD;
    assign is_store = instruction_i.instr.mem_type == STORE;
    assign is_amo = instruction_i.instr.mem_type == AMO;

    logic [6:0] tag_id;
    logic [6:0] tag_id_inflight;

    logic [63:0] data_dword;
    logic [31:0] data_word;
    logic [15:0] data_half;
    logic [7:0]  data_byte;

    // Possible states of the control automata
    parameter ResetState  = 2'b00,
            Idle = 2'b01,
            WaitRready = 2'b10,
            WaitResponse = 2'b11;
    

    /////////////////////////////////////////////////////////////////////
    /// Update State Machine and Stored Instruction
    always_ff @(posedge clk_i, negedge rstn_i) begin
        if(~rstn_i) begin
            state <= ResetState;
            src1_q <= '0;
            src2_q <= '0;
        end else begin
            state <= next_state;            
            src1_q <= src1_d;
            src2_q <= src2_d;
        end
    end

    // Mealy Output and Next State
    always_comb begin
        req_cpu_dcache_valid_int    = 1'b0;     // No Request
        next_state                  = Idle; // Next state Read Head
        stall_o                     = 1'b0;
        src1_d                      = src1_q;
        src2_d                      = src2_q;
        if (flush_i) begin
            req_cpu_dcache_valid_int    = 1'b0;     // No Request
            next_state                  = Idle; // Next state Read Head
        end else begin
            case(state)
                ////////////////////////////////////////////////// Reset state
                ResetState: begin
                    req_cpu_dcache_valid_int    = 1'b0; // No Request
                    next_state                  = Idle ; // Next state Read Head
                    stall_o = valid_i;
                end
                ////////////////////////////////////////////////// New request
                Idle: begin
                    src1_d = src1_i;
                    src2_d = src2_i;
                    if (valid_i & !exception_o.valid & translate_enable & !flush_i & ~stall_after_flush_lsq) begin
                        // Request Logic
                        next_state = WaitResponse;
                        req_cpu_dcache_valid_int = 1'b1;
                        stall_o = 1'b1;
                    end else if (valid_i & !exception_o.valid & !flush_i) begin
                        next_state = WaitRready;
                        req_cpu_dcache_valid_int = 1'b0;
                        stall_o = 1'b1;
                    end else begin
                        next_state = Idle;
                        req_cpu_dcache_valid_int = 1'b0;
                        stall_o = 1'b0;
                    end
                end
                WaitRready: begin
                    if (valid_i & !exception_o.valid & translate_enable & !flush_i & ~stall_after_flush_lsq) begin
                        // Request Logic
                        next_state = WaitResponse;
                        req_cpu_dcache_valid_int = 1'b1;
                        stall_o = 1'b1;
                    end else if (valid_i & !exception_o.valid & !flush_i) begin
                        next_state = WaitRready;
                        req_cpu_dcache_valid_int = 1'b0;
                        stall_o = 1'b1;
                    end else begin
                        next_state = Idle;
                        req_cpu_dcache_valid_int = 1'b0;
                        stall_o = 1'b0;
                    end
                end
                ////////////////////////////////////////////////// Waiting response
                WaitResponse: begin
                    if (resp_dcache_cpu_i.valid) begin
                        next_state = Idle; 
                        stall_o = 1'b0;
                    end else begin
                        next_state = WaitResponse; 
                        stall_o = 1'b1;
                    end
                end
            endcase
        end
    end

    // Update tag identifier for the dcache access
    always_ff @(posedge clk_i, negedge rstn_i) begin
        if(~rstn_i) begin
            tag_id <= 'h0;
            tag_id_inflight <= 'h0;
        end else begin
            if (req_cpu_dcache_o.valid & resp_dcache_cpu_i.ready) begin // Sent or killed request
                tag_id <= tag_id + 7'h1;
                tag_id_inflight <= tag_id;
            end
        end
    end

    assign killed_dcache_req_d = ~reg_ready_resp & reg_valid_req & ~req_cpu_dcache_valid_int;
    // Update State Machine and Stored Instruction
    always_ff @(posedge clk_i, negedge rstn_i) begin
        if(~rstn_i) begin
            reg_ready_resp <= 1'b0;
            reg_valid_req <= 1'b0;
            killed_dcache_req_q <= 1'b0;
            stall_after_flush_lsq <= 1'b0;
        end else begin
            if (req_cpu_dcache_o.valid & resp_dcache_cpu_i.ready) begin // Sent or killed request
                killed_dcache_req_q <= 1'b0;
                stall_after_flush_lsq <= 1'b0;
            end else if (killed_dcache_req_d) begin
                killed_dcache_req_q <= 1'b1;
                stall_after_flush_lsq <= flush_i;
            end
            reg_ready_resp <= resp_dcache_cpu_i.ready;
            reg_valid_req <= req_cpu_dcache_o.valid;
        end
    end

    // Address calculation
    assign vaddr = is_amo ? src1_d : src1_d + instruction_i.instr.result;

    // TLB access
    assign dtlb_comm_o.vm_enable = en_ld_st_translation_i;
    assign dtlb_comm_o.priv_lvl = priv_lvl_i;
    assign dtlb_comm_o.req.valid = valid_i && (state == Idle);
    assign dtlb_comm_o.req.vpn = vaddr[PHY_VIRT_MAX_ADDR_SIZE-1:12];
    assign dtlb_comm_o.req.passthrough = 1'b0;
    assign dtlb_comm_o.req.instruction = 1'b0;
    assign dtlb_comm_o.req.asid = '0;
    assign dtlb_comm_o.req.store = is_amo || is_store; // TODO: Check this, might not be exactly right...

    assign translate_enable = dtlb_comm_i.tlb_ready & !dtlb_comm_i.resp.miss & instruction_i.instr.valid;

    assign paddr = {dtlb_comm_i.resp.ppn, vaddr[11:0]};

    //// Send request to the DCACHE interface
    always_comb begin
        req_cpu_dcache_o.valid = req_cpu_dcache_valid_int | killed_dcache_req_d | killed_dcache_req_q;
        req_cpu_dcache_o.data_rs1        = paddr;
        req_cpu_dcache_o.data_rs2        = src2_d;
        req_cpu_dcache_o.instr_type      = (killed_dcache_req_d) ? ADD : instruction_i.instr.instr_type;
        req_cpu_dcache_o.mem_size        = instruction_i.instr.mem_size;
        req_cpu_dcache_o.rd              = tag_id;
        req_cpu_dcache_o.is_amo_or_store = is_store || is_amo;
        req_cpu_dcache_o.is_amo          = is_amo;
        req_cpu_dcache_o.is_store        = is_store;
    end

    // Exception treatment
    always_comb begin 
        logic is_load_reserved;
        is_load_reserved = (instruction_i.instr.instr_type == AMO_LRW) || (instruction_i.instr.instr_type == AMO_LRD);
        exception_o = '0;

        if (valid_i) begin
            if (((instruction_i.instr.mem_size == 4'b0001) & (|vaddr[0:0])) |
                ((instruction_i.instr.mem_size == 4'b0010) & (|vaddr[1:0])) |
                ((instruction_i.instr.mem_size == 4'b0011) & (|vaddr[2:0])) |
                ((instruction_i.instr.mem_size == 4'b0101) & (|vaddr[0:0])) |
                ((instruction_i.instr.mem_size == 4'b0110) & (|vaddr[1:0])) |
                ((instruction_i.instr.mem_size == 4'b0111) & (|vaddr[2:0]))) begin // Misaligned address
                exception_o.cause       = ((is_amo || is_store) && ~is_load_reserved) ? ST_AMO_ADDR_MISALIGNED : LD_ADDR_MISALIGNED;
                exception_o.origin      = vaddr;
                exception_o.valid       = 1'b1;
            end else if (dtlb_comm_i.resp.xcpt.store & (is_amo || is_store) & ~is_load_reserved) begin // Page fault store
                exception_o.cause       = ST_AMO_PAGE_FAULT;
                exception_o.origin      = vaddr;
                exception_o.valid       = 1'b1;
            end else if (dtlb_comm_i.resp.xcpt.load) begin // Page fault load
                exception_o.cause       = LD_PAGE_FAULT;
                exception_o.origin      = vaddr;
                exception_o.valid       = 1'b1;
            end else if ((en_ld_st_translation_i && (vaddr[VIRT_ADDR_SIZE-1] ? !(&vaddr[63:VIRT_ADDR_SIZE]) : | vaddr[63:VIRT_ADDR_SIZE])) ||
                        (~en_ld_st_translation_i && (~is_inside_mapped_sections(DracCfg, vaddr) || (vaddr >= PHISIC_MEM_LIMIT))) ||
                        (en_ld_st_translation_i && translate_enable && (~is_inside_mapped_sections(DracCfg, paddr) || (paddr >= PHISIC_MEM_LIMIT)))) begin // invalid address
                exception_o.cause  = ((is_amo || is_store) && ~is_load_reserved) ? ST_AMO_ACCESS_FAULT : LD_ACCESS_FAULT;
                exception_o.origin = vaddr;
                exception_o.valid  = 1'b1;
            end
        end
    end 

    // Select bits from whole double word depending on offset
    assign data_dword= (DCACHE_MAXELEM == 8)  ? resp_dcache_cpu_i.data : resp_dcache_cpu_i.data[{vaddr[DCACHE_MAXELEM_LOG-1+(DCACHE_MAXELEM<=8):3],   6'b0} +: 64];
    assign data_word = resp_dcache_cpu_i.data[{vaddr[DCACHE_MAXELEM_LOG-1:2], 5'b0} +: 32];
    assign data_half = resp_dcache_cpu_i.data[{vaddr[DCACHE_MAXELEM_LOG-1:1], 4'b0} +: 16];
    assign data_byte = resp_dcache_cpu_i.data[{vaddr[DCACHE_MAXELEM_LOG-1:0], 3'b0} +: 8];

    // Select data depending on memory size & do sign extension if needed
    always_comb begin
        result_o = 'h0;
        case (instruction_i.instr.mem_size)
            4'b0000: result_o = {{(XLEN-8){data_byte[7]}},data_byte};
            4'b0001: result_o = {{(XLEN-16){data_half[15]}},data_half};
            4'b0010: result_o = {{(XLEN-32){data_word[31]}},data_word};
            4'b0011: result_o = data_dword;
            4'b0100: result_o = data_byte;
            4'b0101: result_o = data_half;
            4'b0110: result_o = data_word;
            4'b0111: result_o = data_dword;
            default: result_o = resp_dcache_cpu_i.data[XLEN-1:0];
        endcase
    end


endmodule