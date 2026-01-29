/* -----------------------------------------------
* Project Name   : DRAC
* File           : tb_icache_interface.v
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
package drac_pkg;

import riscv_pkg::*;

`ifndef CONF_DRAGO_PHY_ADDR_SIZE
    `define CONF_DRAGO_PHY_ADDR_SIZE 32
`endif
parameter PHY_ADDR_SIZE = `CONF_DRAGO_PHY_ADDR_SIZE;

parameter VIRT_ADDR_SIZE = 39;
parameter PHY_VIRT_MAX_ADDR_SIZE = (PHY_ADDR_SIZE < VIRT_ADDR_SIZE) ? VIRT_ADDR_SIZE : PHY_ADDR_SIZE;

parameter PHISIC_MEM_LIMIT = (64'h01 << PHY_ADDR_SIZE) - 64'h01; 

parameter ICACHE_IDX_BITS_SIZE = 12;
parameter ICACHE_VPN_BITS_SIZE = PHY_VIRT_MAX_ADDR_SIZE - ICACHE_IDX_BITS_SIZE;

parameter ICACHELINE_SIZE = 512;
parameter DATA_SIZE = 64;
parameter logic [6:0] DCACHE_MAXELEM = DCACHE_BUS_WIDTH/8;
// TODO (Arnau): I don't think this is the best way of implementing this, but
//               making it fully parametrizable would introduce a *lot* of changes...
//               But at least it's better than having it hardcoded.
`ifdef CONF_SARGANTANA_DCACHE_BUS_WIDTH
parameter DCACHE_BUS_WIDTH = `CONF_SARGANTANA_DCACHE_BUS_WIDTH;
`elsif CONF_HPDCACHE_REQ_WORDS
parameter DCACHE_BUS_WIDTH = `CONF_HPDCACHE_REQ_WORDS * 64;
`else
parameter DCACHE_BUS_WIDTH = 10'd512;
`endif
parameter DCACHE_MAXELEM_LOG = $clog2(DCACHE_MAXELEM);
parameter REGFILE_WIDTH = 5;
parameter CSR_ADDR_SIZE = 12;
parameter CSR_CMD_SIZE = 4;
parameter HPM_NUM_EVENTS = 40;
parameter HPM_NUM_COUNTERS = 29;

// RISCV
//parameter OPCODE_WIDTH = 6;
//parameter REG_WIDTH = 5;

typedef reg   [63:0]  reg64_t;
typedef logic [127:0] bus128_t;
typedef logic [63:0]  bus64_t;
typedef logic [31:0]  bus32_t;
typedef logic [DCACHE_BUS_WIDTH-1:0] bus_dcache_data_t;


typedef logic [REGFILE_WIDTH-1:0] reg_t;
typedef reg   [riscv_pkg::XLEN-1:0] regPC_t;
typedef logic [riscv_pkg::XLEN-1:0] addrPC_t;
typedef logic [PHY_VIRT_MAX_ADDR_SIZE-1:0] addr_t;
typedef logic [PHY_ADDR_SIZE-1:0] phy_addr_t;
typedef reg   [PHY_VIRT_MAX_ADDR_SIZE-1:0] reg_addr_t;
typedef logic [CSR_ADDR_SIZE-1:0] csr_addr_t;
typedef reg   [CSR_ADDR_SIZE-1:0] reg_csr_addr_t;
//typedef logic [CSR_CMD_SIZE-1:0] csr_cmd_t;
//typedef reg   [CSR_CMD_SIZE-1:0] reg_csr_cmd_t;

typedef logic [riscv_pkg::INST_SIZE-1:0] inst_t;
typedef logic [ICACHELINE_SIZE-1:0] icache_line_t;
typedef reg   [ICACHELINE_SIZE-1:0] icache_line_reg_t;
typedef logic [ICACHE_IDX_BITS_SIZE-1:0] icache_idx_t;
typedef logic [ICACHE_VPN_BITS_SIZE-1:0] icache_vpn_t;

// Branch predictor
// Least significative bit from address used to index
parameter LEAST_SIGNIFICATIVE_INDEX_BIT_BP = 2;

// Most significative bit from address used to index
parameter MOST_SIGNIFICATIVE_INDEX_BIT_BP = 8;

localparam NrMaxRules = 16;

typedef struct packed {  
    int                                   NIOSections;
    logic [NrMaxRules-1:0][PHY_ADDR_SIZE-1:0] InitIOBase;
    logic [NrMaxRules-1:0][PHY_ADDR_SIZE-1:0] InitIOEnd;
  
    int                                   NMappedSections;
    logic [NrMaxRules-1:0][PHY_ADDR_SIZE-1:0] InitMappedBase;
    logic [NrMaxRules-1:0][PHY_ADDR_SIZE-1:0] InitMappedEnd;

    logic [PHY_ADDR_SIZE-1:0] InitBROMBase;
    logic [PHY_ADDR_SIZE-1:0] InitBROMEnd;

    logic [PHY_ADDR_SIZE-1:0] DebugProgramBufferBase;
    logic [PHY_ADDR_SIZE-1:0] DebugProgramBufferEnd;
} drac_cfg_t;

function automatic logic range_check(addr_t start_region, addr_t end_region, bus64_t address);
    // if len is a power of two, and base is properly aligned, this check could be simplified
    return (address >= {{{64-PHY_VIRT_MAX_ADDR_SIZE}{1'b0}}, start_region}) && (address < {{{64-PHY_VIRT_MAX_ADDR_SIZE}{1'b0}}, end_region});
endfunction : range_check

function automatic logic is_inside_IO_sections (drac_cfg_t Cfg, bus64_t address);
    // if we don't specify any region we assume everything is accessible
    logic[NrMaxRules-1:0] pass;
    pass = '0;
    for (int unsigned k = 0; k < Cfg.NIOSections; k++) begin
        pass[k] = range_check(Cfg.InitIOBase[k], Cfg.InitIOEnd[k], address);
    end
    return |pass;
endfunction : is_inside_IO_sections

function automatic logic is_inside_mapped_sections (drac_cfg_t Cfg, bus64_t address);
    // if we don't specify any region we assume everything is accessible
    logic[NrMaxRules-1:0] pass;
    pass = '0;
    for (int unsigned k = 0; k < Cfg.NMappedSections; k++) begin
        pass[k] = range_check(Cfg.InitMappedBase[k], Cfg.InitMappedEnd[k], address);
    end
    return |pass;
endfunction : is_inside_mapped_sections

typedef enum logic [1:0] {
    NEXT_PC_SEL_BP_OR_PC_4  = 2'b00,
    NEXT_PC_SEL_KEEP_PC     = 2'b01,
    NEXT_PC_SEL_JUMP        = 2'b10,
    NEXT_PC_SEL_DEBUG       = 2'b11
} next_pc_sel_t;    // Enum PC Selection


typedef enum logic [1:0] {
    NOT_MEM  = 2'b00,
    LOAD     = 2'b01,
    STORE    = 2'b10,
    AMO      = 2'b11
} mem_type_t; 

typedef enum logic [2:0] {
    SEL_JUMP_EXECUTION = 3'b000,
    SEL_JUMP_CSR       = 3'b001,
    SEL_JUMP_DECODE    = 3'b010,
    SEL_JUMP_DEBUG     = 3'b011,
    SEL_JUMP_CSR_RW    = 3'b100
} jump_addr_fetch_t;

typedef enum logic [1:0]{
    TLBMiss    = 2'b00,
    NoReq      = 2'b01,
    ReqValid   = 2'b10,
    Replay     = 2'b11
} icache_state_t;   // Enum Icache Interface Machine

typedef enum logic {
    NOT_TAKEN,
    TAKEN
} branch_decision_t;   // Enum Branch Prediction resolution

typedef struct packed {
    logic is_branch;                    // Was predicted to be branch
    branch_decision_t decision;    // Taken or not taken
    addrPC_t pred_addr;                 // Predicted Address
} branch_pred_t;            // Struct for Branch Prediction

typedef struct packed {
    riscv_pkg::exception_cause_t cause; // Cause of exception vector 64 bits
    bus64_t origin; // Addr or PC generating exception
    logic valid;    // There is an eception
} exception_t;      // Struct contains exceptions

typedef struct packed {
    addrPC_t        pc_execution;              // Program counter at Execution Stage
    addrPC_t        branch_addr_result_exe;    // Final address generated by branch in Execution Stage (for RAS push as well)
    addrPC_t        branch_addr_target_exe;    // Target address generated by branch in Execution Stage (for RAS push as well)
    logic           branch_taken_result_exe;   // Taken or not taken branch result in Execution Stage
    logic           is_branch_exe;             // The instruction in the Execution Stage is a branch
} exe_if_branch_pred_t;

// Response coming from ICache
typedef struct packed {
    logic   valid;               // Response valid
    inst_t  data;                // Word of 32 bits from Icache
    logic   instr_page_fault;    // Page Fault from TLB
} resp_icache_cpu_t;

// Request send to ICache
typedef struct packed {
    logic   valid;               // Request valid
    addr_t  vaddr;               // Virtual Addr requested
    logic   invalidate_icache;   // Petition to invalidate cache content
    logic   invalidate_buffer;   // Petition to invalidate buffer, which also serves as repeat the req
    logic   inval_fetch;         //
} req_cpu_icache_t;

typedef enum logic [2:0] {
    SEL_SRC1_REGFILE,           // Source one from register file
    SEL_SRC2_REGFILE,           // Source two from register file
    SEL_IMM,                    // Immediate from decode
    SEL_PC,                     // Select PC
    SEL_PC_4,                   // Select PC + 4
    SEL_BYPASS                  // Select bypass from previous stage
} alu_sel_t;        // ALU Source Selection

typedef enum logic [2:0]{
    UNIT_ALU,                   // Select ALU
    UNIT_DIV,                   // Select DIVISION
    UNIT_MUL,                   // Select MULTIPLICATION
    UNIT_BRANCH,                // Select Branch computation
    UNIT_MEM,                   // Select Memory unit
    UNIT_CONTROL,               // Select CONTROL
    UNIT_SYSTEM                 // Select CSR
} functional_unit_t;   // Selection of funtional unit in exe stage 

typedef enum logic [1:0]{
    SEL_FROM_MEM,               // Select source from Memory
    SEL_FROM_ALU,               // Select source from ALU
    SEL_FROM_BRANCH,            // Select source from Branch computation
    SEL_FROM_CONTROL            // Select source from control
} reg_sel_t;          // Selection of the result from functional unit 

typedef enum logic [6:0] { 
    // basic ALU op
   ADD, SUB, ADDW, SUBW,
   // logic operations
   XOR, OR, AND,
   // shifts
   SRA, SRL, SLL, SRLW, SLLW, SRAW,
   // comparisons
   BLT, BLTU, BGE, BGEU, BEQ, BNE,
   // jumps
   JALR, JAL,
   // set lower than operations
   SLT, SLTU,
   // CSR functions
   MRET, SRET, URET, ECALL, EBREAK, WFI, FENCE, FENCE_I, SFENCE_VMA,
   // Old ISA CSR functions
   ERET, MRTS, MRTH, HRTS,
   //CSR_WRITE, CSR_READ, CSR_SET, CSR_CLEAR,
   CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI,
   // LSU functions
   LD, SD, LW, LWU, SW, LH, LHU, SH, LB, SB, LBU,
   // Atomic Memory Operations
   AMO_LRW, AMO_LRD, AMO_SCW, AMO_SCD,
   AMO_SWAPW, AMO_ADDW, AMO_ANDW, AMO_ORW, AMO_XORW, AMO_MAXW, AMO_MAXWU, AMO_MINW, AMO_MINWU,
   AMO_SWAPD, AMO_ADDD, AMO_ANDD, AMO_ORD, AMO_XORD, AMO_MAXD, AMO_MAXDU, AMO_MIND, AMO_MINDU,
   // Multiplications
   MUL, MULH, MULHU, MULHSU, MULW,
   // Divisions
   DIV, DIVU, DIVW, DIVUW, REM, REMU, REMW, REMUW,
   // Vectorial Floating-Point Instructions that don't directly map onto the scalar ones
   VFMIN, VFMAX, VFSGNJ, VFSGNJN, VFSGNJX, VFEQ, VFNE, VFLT, VFGE, VFLE, VFGT, VFCPKAB_S, VFCPKCD_S, VFCPKAB_D, VFCPKCD_D
} instr_type_t;

typedef enum logic[CSR_CMD_SIZE-1:0] {
    CSR_CMD_NOPE      = 4'b0000,
    CSR_CMD_RW        = 4'b0001,
    CSR_CMD_SET       = 4'b0010,
    CSR_CMD_CLEAR     = 4'b0011,
    CSR_CMD_SYS       = 4'b0100,
    CSR_CMD_READ      = 4'b0101,
    CSR_CMD_VSETVL    = 4'b0110,
    CSR_CMD_VSETVLMAX = 4'b0111,
    CSR_CMD_WRITE     = 4'b1000,
    CSR_CMD_VLEFF     = 4'b1001
} csr_cmd_t;                // Comands to lowrisc CSR


// Response coming from Dcache
typedef struct packed {
    logic       valid;      // Response valid
    logic       ready;      // Ready to accept requests
    bus_dcache_data_t data;      // Data from load
    logic[6:0]     rd;      // Tag of the mem access
    logic io_address_space; // Request in Input/Output Address Space
    logic     ordered;    
} resp_dcache_cpu_t;

// Request send to DCache
typedef struct packed {
    logic                valid;        // New memory request
    bus64_t           data_rs1;        // Data operand 1
    bus64_t           data_rs2;        // Data operand 2
    instr_type_t    instr_type;        // Type of instruction
    logic [3:0]       mem_size;        // Granularity of mem. access
    logic [6:0]             rd;        // Destination register. Used for identify a pending Miss
    logic      is_amo_or_store;        // Type of instruction is amo or store
    logic               is_amo;        // Type of instruction is amo
    logic             is_store;        // Type of instruction is amo
} req_cpu_dcache_t;

// Fetch 1 Stage
typedef struct packed {
    addrPC_t                 pc_inst;   // Actual PC
    logic                    valid;     // Valid instruction
    branch_pred_t            bpred;     // Branch prediction
    exception_t              ex;        // Exceptions
    `ifdef SIM_KONATA_DUMP
    bus64_t id;
    `endif
} if_1_if_2_stage_t;       // FETCH 1 STAGE TO DECODE STAGE

// Fetch 2 Stage
typedef struct packed {
    addrPC_t                 pc_inst;   // Actual PC
    riscv_pkg::instruction_t inst;      // Bits of the instruction
    logic                    valid;     // Valid instruction
    branch_pred_t            bpred;     // Branch prediction
    exception_t              ex;        // Exceptions
    `ifdef SIM_KONATA_DUMP
    bus64_t id;
    `endif
} if_id_stage_t;       // FETCH STAGE TO DECODE STAGE

// This is created by decode
typedef struct packed {
    logic valid;                        // Valid instruction
    addrPC_t pc;                        // PC of the instruction
    branch_pred_t bpred;                // Branch Prediciton
    exception_t ex;                     // Exceptions
    reg_t rs1;                          // Register Source 1
    reg_t rs2;                          // Register Source 2
    reg_t rd;                           // Destination register
    
    logic use_imm;                      // Use Immediate later
    logic use_pc;                       // Use PC later
    logic op_32;                        // Operation of 32 bits
    functional_unit_t unit;             // Functional unit

    // Control bits
    logic change_pc_ena;                // Change PC 
    logic regfile_we;                   // Write to register file
    instr_type_t instr_type;            // Type of instruction
    bus64_t result;                     // Result or Immediate
    logic signed_op;                    // Signed Operation
    logic [3:0] mem_size;               // Memory operation size (Byte, Word, etc)
    logic stall_csr_fence;              // CSR or fence
    mem_type_t mem_type;                // Mem instruction type
    `ifdef SIM_COMMIT_LOG
    riscv_pkg::instruction_t inst;
    bus64_t id;
    `elsif SIM_KONATA_DUMP
    bus64_t id;
    `endif
} instr_entry_t;

typedef struct packed {
    instr_entry_t instr;                // Instruction
    bus64_t data_rs1;                   // Data operand 1
    bus64_t data_rs2;                   // Data operand 2
    // any interrupt
    logic       csr_interrupt;
    // save until the instruction then 
    // give the interrupt cause as xcpt cause
    bus64_t     csr_interrupt_cause;
} rr_exe_instr_t;       //  Read Regfile to Execution stage

typedef struct packed {
    logic valid;                        // Valid instruction
    addrPC_t pc;                        // PC of the instruction
    reg_t rs1;                          // Register Source 1
    instr_type_t instr_type;            // Type of instruction
    addrPC_t result_pc;                 // PC result
    reg_t rd;                           // Destination Register
    bus64_t result;                     // Result or immediate  
    logic branch_taken;                 // Branch taken
    branch_pred_t bpred;                // Branch Prediciton
    exception_t ex;                     // Exceptions
    logic regfile_we;                   // Write to register file
    logic change_pc_ena;                // Change PC
    logic stall_csr_fence;              // CSR or fence
    reg_csr_addr_t csr_addr;            // CSR Address
    mem_type_t mem_type;
    `ifdef SIM_COMMIT_LOG
    riscv_pkg::instruction_t inst;
    bus64_t id;
    addr_t addr;
    functional_unit_t unit;
    `elsif SIM_KONATA_DUMP
    bus64_t id;
    functional_unit_t unit;
    `endif
} exe_wb_instr_t;       //  Execution Stage to Write Back

// For bypass
typedef struct packed {
    logic valid;                        // Valid instruction
    reg_t rd;                           // Destination register
    bus64_t data;                       // Result data
} wb_exe_instr_t;      // WB Stage to Execution

// Control Unit signals
typedef struct packed {
    logic valid_fetch;      // Fetch is valid
} if_cu_t;      // Fetch to Control Unit

typedef struct packed {
    logic valid;        // valid
    logic valid_jal;        // JAL is valid
    logic stall_csr_fence;  // CSR or fence
} id_cu_t;      // Decode to Control Unit

typedef struct packed {
    logic valid;        // valid
    logic stall_csr_fence;  // CSR or fence
    logic data_dependency;  // Data dependency
} rr_cu_t;      // Read Register to Control Unit 

typedef struct packed {
    next_pc_sel_t next_pc;      // Select next PC
} cu_if_t;      // Control Unit to Fetch

typedef struct packed {
    logic write_enable;         // Enable write on register file
    logic write_enable_dbg;     // Enable write on register file dbg usage
} cu_rr_t;      // Control unit to Register File

// Control Unit signals
typedef struct packed {
    logic valid;
    logic change_pc_ena;
    logic is_branch;
    logic stall;                // Execution unit stalled
    logic stall_csr_fence;      // CSR or fence
} exe_cu_t;

// Control Unit signals
typedef struct packed {
    addrPC_t pc;                // PC of the instruction
    logic valid;                // Valid Intruction
    logic change_pc_ena;        // Enable PC write
    logic csr_enable_wb;        // CSR that needs to write to register file
    logic write_enable;         // Write Enable to Register File
    logic stall_csr_fence;      // CSR or fence
    logic xcpt;                 // Exception
    logic ecall_taken;          // Ecall 
    logic fence;                // Is fence
    logic fence_i;              // Is fence i
} wb_cu_t;      // Write Back to Control Unit


// Control Unit signals
typedef struct packed {
    logic enable_commit;        // Enable Commit
} cu_wb_t;      // Control Unit to Write Back

// Pipeline control
typedef struct packed {
    logic stall_if_1;       // Stop Fetch 1
    logic stall_if_2;       // Stop Fetch 2
    logic stall_id;         // Stop Decode
    logic stall_rr;         // Stop Read Register
    logic stall_exe;        // Stop Exe
    logic stall_wb;         // Stop Write Back

    // whether insert in fetch from dec or commit
    jump_addr_fetch_t sel_addr_if;
} pipeline_ctrl_t;  // Control signals of the pipeline

// Pipeline control
typedef struct packed {
    logic flush_if;         // Flush Fetch
    logic flush_id;         // Flush instruction in Decode
    logic flush_rr;         // Flush instruction in Read Register
    logic flush_exe;        // Flush instruction in Execution Stage
    logic flush_wb;         // Flush instruction in Write Back
} pipeline_flush_t;

// Pipeline control
typedef struct packed {
    logic       valid; // whether is a jal or not
    addrPC_t    jump_addr;
} jal_id_if_t;

//PMU flags
typedef struct packed {
    logic stall_if   ;         // Stop Fetch
    logic stall_id   ;         // Stop Decode
    logic stall_rr   ;         // Stop Read Register
    logic stall_exe  ;         // Stop Exe
    logic stall_wb   ;         // Stop Write Back
    logic branch_miss;         // Stop Write Back
    logic is_branch  ;         // Stop Write Back
    logic branch_taken;         // Stop Write Back
    logic stall_data_dependency; // Stop Write Back
    logic stall_structural; // Stop Write Back
} to_PMU_t;  // Control signals to PMU counters

// CSR output
typedef struct packed {
    // csr addr or 12 imm bits from system instr
    csr_addr_t  csr_rw_addr;
    // internal cmd of csr
    csr_cmd_t   csr_rw_cmd;
    // if xcpt, pass misaligned addr
    // data to write in CSR 
    bus64_t     csr_rw_data;
    // exception from wb
    logic       csr_exception;
    // every time commit send this
    logic       csr_retire;
    // exception cause
    bus64_t     csr_xcpt_cause;
    // exception origin
    bus64_t     csr_xcpt_origin;
    // xcpt pc 
    bus64_t     csr_pc; 
} req_cpu_csr_t;

// CSR input
typedef struct packed {
    bus64_t     csr_rw_rdata;
    // if sending a csr command while 
    // CSR is not responding
    // EX: send a CSR petition to PCR
    // but thery are busy, at same cycle 
    // replay wil be to 1
    logic       csr_replay;
    // petition to CSR takes more than one cycle
    // when down value ready
    // while up doing req
    // or WFI
    logic       csr_stall;
    // CSR exception
    // read CSR without enough privilege
    // or eret/ecall
    // CSR handles all the usual logic
    // don't care about cause and do the typical
    // flush and charge evec? 
    logic       csr_exception;
    bus64_t     csr_exception_cause;
    // old uret, sret, mret
    // return  from system to user
    logic       csr_eret;
    // pc to go if xcpt or eret
    bus64_t     csr_evec;
    // any interrupt
    logic       csr_interrupt;
    // save until the instruction then 
    // give the interrupt cause as xcpt cause
    bus64_t     csr_interrupt_cause;
    // tval
    bus64_t     csr_tval;
    // Step_mode_enbled
    logic       debug_step;
    // Ebreak in CSR module
    logic       debug_ebreak;
} resp_csr_cpu_t;

typedef struct packed {
    // Triggers a halt on the pipeline 
    logic           halt_req;
    // Triggers a restart on the pipeline
    logic           resume_req;
    // The core starts to fetch from the debug program buffer
    logic           progbuf_req;
    // Indicates that the core should halt after exiting form a reset
    logic           halt_on_reset;
} debug_contr_in_t;


parameter PHISICAL_REGFILE_WIDTH = 6;
typedef logic [PHISICAL_REGFILE_WIDTH-1:0] phreg_t;


typedef struct packed {
    // Rename read request 
    logic           rnm_read_en;
    // Virtual register address that needs to be renamed 
    reg_t           rnm_read_reg;
    // Register file read request 
    logic           rf_en;
    // Physical register address that needs to be read/write
    phreg_t         rf_preg;
    // Write enable of the register file 
    logic           rf_we;
    // Data to be written 
    bus64_t         rf_wdata;
} debug_reg_in_t;

typedef struct packed {
    // ACKs the halt of the pipeline 
    logic           halt_ack;
    // The pipelines is halted
    logic           halted;
    // ACKs the restart of the pipeline
    logic           resume_ack;
    // The pipeline is running
    logic           running;
    // ACKs the probram buffer execution request
    logic           progbuf_ack;
    // The pipeline is halted and is not executing from the probram buffer (parked state)
    logic           parked;
    // Indicates if the debugging is unavailable
    logic           unavail;
    // The execution of the program buffer stopped because of an exception
    logic           progbuf_xcpt;
    // ACKs the reset of the pipeline 
    logic           havereset;
} debug_contr_out_t;

typedef struct packed {
    // Response of a rename request 
    phreg_t         rnm_read_resp;
    // Response form a read request
    bus64_t         rf_rdata;
} debug_reg_out_t;

typedef enum logic[2:0] {
    DEBUG_STATE_RESET,
    DEBUG_STATE_HAVERESET,
    DEBUG_STATE_RUNNING,
    DEBUG_STATE_RESUME, 
    DEBUG_STATE_HALTING, 
    DEBUG_STATE_HALTED,
    DEBUG_STATE_PROGBUFF
} debug_contr_state_t;

localparam drac_cfg_t DracDefaultConfig = '{
    NIOSections: 1, // number of IO space sections
    InitIOBase:  {40'h40000000}, // IO base 0 address after reset
    InitIOEnd:  {40'h80000000}, // IO end 0 address after reset

    NMappedSections: 3, // number of Memory space sections
    InitMappedBase: {40'h0040000000, 40'h0000000100, 40'h0000010000}, // Memory base address after reset
    InitMappedEnd: {40'h3fffffffff, 40'h000000ffff, 40'h0000010020}, // Memory end 0 address after reset

    InitBROMBase: 40'h0000000100,
    InitBROMEnd: 40'h000000ffff,

    DebugProgramBufferBase: 40'h0000010000,
    DebugProgramBufferEnd:  40'h0000010020
};

////////////////////////////////      
//
//  Cache-TLB communication
//
///////////////////////////////   

parameter VPN_SIZE = 27;
parameter PPN_SIZE = 44;
parameter ASID_SIZE = 7;

// Cache-TLB request
typedef struct packed {
    logic valid;   
    logic [ASID_SIZE-1:0] asid;
    logic [VPN_SIZE:0] vpn;
    logic passthrough;
    logic instruction;
    logic store;
} cache_tlb_req_t;

typedef struct packed {
    cache_tlb_req_t req;
    logic [1:0] priv_lvl;   
    logic vm_enable; 
} cache_tlb_comm_t;

typedef struct packed {
    logic load;
    logic store;
    logic fetch;
} tlb_ex_t;

// TLB-Cache response
typedef struct packed { 
    logic miss;
    logic [PPN_SIZE-1:0] ppn; 
    tlb_ex_t xcpt;
    logic [7:0] hit_idx;
} tlb_cache_resp_t;

typedef struct packed {
    logic tlb_ready;  
    tlb_cache_resp_t resp;
} tlb_cache_comm_t;

typedef struct packed {
    logic [63:0] satp;
    logic flush;
    logic [63:0] mstatus;
} csr_ptw_comm_t;

typedef struct packed {
    logic exe_load;
    logic exe_store;
    logic icache_req;
    logic icache_kill;
    logic icache_miss_l2_hit;
    logic icache_miss_kill;
    logic icache_busy;
    logic icache_miss_time;
    logic itlb_access;
    logic itlb_miss;
    logic dtlb_access;
    logic dtlb_miss;
    logic ptw_buffer_hit;
    logic ptw_buffer_miss;
    logic itlb_stall;
    logic dcache_stall;
    logic dcache_stall_refill;
    logic dcache_rtab_rollback;
    logic dcache_req_onhold;
    logic dcache_prefetch_req;
    logic dcache_read_req;
    logic dcache_write_req;
    logic dcache_cmo_req;
    logic dcache_uncached_req;
    logic dcache_miss_read_req;
    logic dcache_miss_write_req;
} pmu_interface_t;

typedef struct packed {
    logic commit_valid0;
    logic commit_xcpt;
    logic [4:0] commit_xcpt_cause;
    logic stall_if_1;       
    logic stall_if_2;      
    logic stall_id;         
    logic stall_iq;             
    logic stall_rr;       
    logic stall_exe;       
    logic stall_commit;   
    logic flush_if;
    logic flush_rr;
} visa_signals_t;

`ifdef SIM_COMMIT_LOG
typedef struct packed {
    longint unsigned pc;
    longint unsigned inst;
    longint unsigned dst;
    longint unsigned reg_wr_valid;
    longint unsigned data;
    longint unsigned csr_wr_valid;
    longint unsigned csr_dst;
    longint unsigned csr_data;
    longint unsigned xcpt;
    longint unsigned xcpt_cause;
    longint unsigned csr_priv_lvl;
    longint unsigned csr_rw_data;
    longint unsigned csr_xcpt;
    longint unsigned csr_xcpt_cause;
    longint unsigned csr_tval;
    longint unsigned mem_type;
    longint unsigned mem_addr;
} commit_data_t;
`endif

endpackage

