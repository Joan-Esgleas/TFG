/* -----------------------------------------------
* File           : dp_ram_asic.sv
* Organization   : Barcelona Supercomputing Center
* Author(s)      : Junaid Ahmed; Xabier Abancens
* Email(s)       : {author}@bsc.es
* References     : Openpiton 
* https://github.com/PrincetonUniversity/openpiton 
* -----------------------------------------------
* Revision History
*  Revision   | Author          |  Description
*  0.1        | Junaid Ahmed;   | 
*             | Xabier Abancens | 
* -----------------------------------------------
*/

// Dual port RAM module that instantiates physical memories
// It supports the memories that have been generated for ongoing projects. 
// If a new memory is generated, please add its macro 

module dp_ram_asic #(
    parameter ADDR_WIDTH=1, 
    parameter DATA_WIDTH=1
) (
   `ifdef INTEL_PHYSICAL_MEM_CTRL
   input wire [27:0]               INTEL_MEM_CTRL,
   `endif
   input wire [ADDR_WIDTH-1  : 0]  AA,AB,
   input wire [DATA_WIDTH-1  : 0]  DB,
   input wire [DATA_WIDTH-1  : 0]  BWB,  // Bit enable, 1: write bit enable
   input wire CLKA,CEA,  // 1: read
   input wire CLKB,CEB,  // 1: write
   output wire [DATA_WIDTH-1  : 0]  QA
);   


    // Memory macros for dual port, 2P, ARM7FF technology: RF & SRAM
    // RF_2P_XX Two-Port High-Density Register File
    // SRAM_SP_XX Two-Port Ultra-High-Density SRAM
    // No Power_Pins
    // CENA, CENB: Read & Write Enables (active low)
    // WENB[]: Write Enable (active low, WENB[0]=LSB)
    `define ARM7FF_2P_INTERFACE (\
            `ifdef INTEL_PHYSICAL_MEM_CTRL \
            .INTEL_MEM_CTRL(INTEL_MEM_CTRL), \
            `endif \
            .CENA(~CEA), \
            .AA(AA), \
            .CENB(~CEB), \
            .AB(AB), \
            .DB(DB), \
            .WENB(~BWB), \
            .CLKA(CLKA), \
            .CLKB(CLKB), \
            .QA(QA), \
            .STOV(1'b0), \
            .EMAA(3'b000), \
            .EMASA(1'b0), \
            .EMAB(3'b000), \
            .RET(1'b0), \
            .QNAPA(1'b0), \
            .QNAPB(1'b0));

   
    localparam DEPTH = 2 ** ADDR_WIDTH;
    generate

    if ((DEPTH == 64) && (DATA_WIDTH == 64))  begin : ram64x64
        RF_2P_64x64_M1B1S1 the_RAM `ARM7FF_2P_INTERFACE

    end else if ((DEPTH == 64) && (DATA_WIDTH == 55))  begin : ram64x55
        RF_2P_64x55_M2B2S1 the_RAM `ARM7FF_2P_INTERFACE

    end else if ((DEPTH == 128) && (DATA_WIDTH == 256))  begin : ram128x256
        RF_2P_128x256_M1B2S2 the_RAM `ARM7FF_2P_INTERFACE

    end else if ((DEPTH == 1024) && (DATA_WIDTH == 66))  begin : ram1024x66  
        RF_2P_1024x66_M4B4S2 the_RAM `ARM7FF_2P_INTERFACE
    
    end else begin : ram_undef   //should not reach here at all 
        ASIC_2P_RAM_UNDEF  #(.DEPTH(DEPTH), .DATA_WIDTH(DATA_WIDTH))  UNDEF_RAM   `ARM7FF_2P_INTERFACE
    end
    
    endgenerate


endmodule
