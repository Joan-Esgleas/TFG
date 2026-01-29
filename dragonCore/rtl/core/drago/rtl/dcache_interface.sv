//`default_nettype none
//`include "drac_pkg.sv"
import drac_pkg::*;

/* -----------------------------------------------
 * Project Name   : DRAC
 * File           : mem_unit.v
 * Organization   : Barcelona Supercomputing Center
 * Author(s)      : Rubén Langarita
 * Email(s)       : ruben.langarita@bsc.es
 * -----------------------------------------------
 * Revision History
 *  Revision   | Author     | Description
 *  0.1        | Ruben. L   |
 *  0.2        | Victor. SP | Improve Doc. and pass tb
 * -----------------------------------------------
 */
 
// Interface with Data Cache. Stores a Memory request until it finishes

module dcache_interface (
    input  wire         clk_i,               // Clock
    input  wire         rstn_i,              // Negative Reset Signal

    input req_cpu_dcache_t req_cpu_dcache_i, // Interface with cpu

    // DCACHE Answer
    input  logic        dmem_resp_replay_i,  // Miss ready
    input  bus64_t      dmem_resp_data_i,    // Readed data from Cache
    input  logic        dmem_req_ready_i,    // Dcache ready to accept request
    input  logic        dmem_resp_valid_i,   // Response is valid
    input  logic        dmem_resp_nack_i,    // Cache request not accepted
    input  logic        dmem_xcpt_ma_st_i,   // Missaligned store
    input  logic        dmem_xcpt_ma_ld_i,   // Missaligned load
    input  logic        dmem_xcpt_pf_st_i,   // DTLB miss on store
    input  logic        dmem_xcpt_pf_ld_i,   // DTLB miss on load

    // Request TO DCACHE

    output logic        dmem_req_valid_o,    // Sending valid request
    output logic [4:0]  dmem_req_cmd_o,      // Type of memory access
    output addr_t       dmem_req_addr_o,     // Address of memory access
    output logic [3:0]  dmem_op_type_o,      // Granularity of memory access
    output bus64_t      dmem_req_data_o,     // Data to store
    output logic [7:0]  dmem_req_tag_o,      // Tag for the MSHR
    output logic        dmem_req_invalidate_lr_o, // Reset load-reserved/store-conditional
    output logic        dmem_req_kill_o,     // Kill actual memory access

    // DCACHE Answer to WB
    output resp_dcache_cpu_t resp_dcache_cpu_o, // Dcache to CPU

    // PMU
    output logic dmem_is_store_o,
    output logic dmem_is_load_o
);

// Declarations of internal variables


endmodule
//`default_nettype wire

