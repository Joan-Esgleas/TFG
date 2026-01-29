/* -----------------------------------------------
* File           : sp_ram_asic.sv
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

// Single port RAM module that instantiates physical memories
// It supports the memories that have been generated for ongoing projects. 
// If a new memory is generated, please add its macro 

module sp_ram_asic #(
    parameter ADDR_WIDTH=1, 
    parameter DATA_WIDTH=1
) (
   `ifdef INTEL_PHYSICAL_MEM_CTRL
   input wire  [27:0]               INTEL_MEM_CTRL,
   `endif
   input wire  [ADDR_WIDTH-1  : 0]  A,
   input wire  [DATA_WIDTH-1  : 0]  DI,
   input wire  [DATA_WIDTH-1  : 0]  BW,
   input wire  CLK,CE, RDWEN,  // RDWEN: 1=WR and 0=RD
   output wire [DATA_WIDTH-1  : 0]  DO
);   
    
    // Memory macros for Single Port ARM7FF technology: RF & SRAM
    // RF_SP_XX Single-Port High-Density Register File
    // SRAM_SP_XX Ultra-High-Density SRAM
    // No Power_Pins
    // CEN: Chip Enable (active low)
    // GWEN: Write Enable (active low)
    // WEN[]: Write Enable (active low, WEN[0]=LSB)
    // RET: Retention mode enable, active-HIGH
    // QNAP: Quick Nap mode enable, active-HIGH
    `define ARM7FF_SP_INTERFACE ( \
            `ifdef INTEL_PHYSICAL_MEM_CTRL \
            .INTEL_MEM_CTRL(INTEL_MEM_CTRL), \
            `endif \
            .A(A), \
            .D(DI), \
            .CLK(CLK), \
            .CEN(~CE), \
            .GWEN(~RDWEN), \
            .WEN(~BW), \
            .Q(DO), \
            .EMA(3'b000), \
            .EMAW(2'b00), \
            .EMAS(1'b0), \
            .STOV(1'b0), \
            .RET(1'b0), \
            .QNAP(1'b0));

    localparam DEPTH = 2 ** ADDR_WIDTH;
    generate
   
    if          ((DEPTH == 32 ) && ( DATA_WIDTH == 106))   begin : ram32x106
        RF_SP_32x106_M2B1S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 32 ) && ( DATA_WIDTH == 88))    begin : ram32x88
        RF_SP_32x88_M2B1S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 64 ) && ( DATA_WIDTH == 104))   begin : ram64x104
        RF_SP_64x104_M2B1S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 64 ) && ( DATA_WIDTH == 112))   begin : ram64x112
        RF_SP_64x112_M2B1S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 128 ) && ( DATA_WIDTH == 29))   begin : ram128x29
        RF_SP_128x29_M4B1S1 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 128 ) && ( DATA_WIDTH == 38))   begin : ram128x38
        RF_SP_128x38_M2B1S1 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 128 ) && ( DATA_WIDTH == 64))   begin : ram128x64
        RF_SP_128x64_M2B1S1 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 128 ) && ( DATA_WIDTH == 108))  begin : ram128x108
        RF_SP_128x108_M2B1S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 128 ) && ( DATA_WIDTH == 128))  begin : ram128x128
        RF_SP_128x128_M2B1S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 512 ) && ( DATA_WIDTH == 38))   begin : ram512x38
        RF_SP_512x38_M2B1S1 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 512 ) && ( DATA_WIDTH == 128))  begin : ram512x128
        RF_SP_512x128_M2B1S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 512 ) && ( DATA_WIDTH == 132))  begin : ram512x132
        SRAM_SP_HDE_512x132_M2B4S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 1024 ) && ( DATA_WIDTH == 66))  begin : ram1024x66
        RF_SP_1024x66_M2B2S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 1024 ) && ( DATA_WIDTH == 96))  begin : ram1024x96
        RF_SP_1024x96_M2B2S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 2048 ) && ( DATA_WIDTH == 128)) begin : ram2048x128
        SRAM_SP_HDE_2048x128_M2B4S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 2048 ) && ( DATA_WIDTH == 32))  begin : ram2048x32
        SRAM_SP_HDE_2048x32_M2B4S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 2048 ) && ( DATA_WIDTH == 92))  begin : ram2048x92
        SRAM_SP_HDE_2048x92_M2B4S2  the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 2048 ) && ( DATA_WIDTH == 132)) begin : ram2048x132
        RF_SP_2048x132_M4B2S2  the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 4096 ) && ( DATA_WIDTH == 64))  begin : ram4096x64
        RF_SP_4096x64_M8B2S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 8192 ) && ( DATA_WIDTH == 32))  begin : ram8192x32
        RF_SP_8192x32_M16B2S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 8192 ) && ( DATA_WIDTH == 64))  begin : ram8192x64
        SRAM_SP_HDE_8192x64_M4B8S2 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 8192 ) && ( DATA_WIDTH == 128)) begin : ram8192x128
        SRAM_SP_HDE_8192x128_M4B8S2 the_RAM `ARM7FF_SP_INTERFACE
        
    //multi-asic rams:
    end else if ((DEPTH == 64 ) && ( DATA_WIDTH == 512))   begin : ram64x512  // 4 memories of 64x128
        ASIC_RAM_W512_D64 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 256 ) && ( DATA_WIDTH == 264))   begin : ram256x264  // 2 memories of 256x132
        ASIC_RAM_W264_D256 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 512 ) && ( DATA_WIDTH == 200))   begin: ram512x200
        ASIC_RAM_W200_D512 the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 16384 ) && ( DATA_WIDTH == 144)) begin: ram16384x144
        ASIC_RAM_W144_D16384  the_RAM `ARM7FF_SP_INTERFACE
    end else if ((DEPTH == 32768 ) && ( DATA_WIDTH == 144)) begin: ram32768x144
        ASIC_RAM_W144_D32768  the_RAM `ARM7FF_SP_INTERFACE
    end else begin : ram_undef
        ASIC_SP_RAM_UNDEF  #(.DEPTH(DEPTH), .DATA_WIDTH(DATA_WIDTH))  UNDEF_RAM   `ARM7FF_SP_INTERFACE
    end
    
    endgenerate
endmodule


module ASIC_RAM_W200_D512   (
    `ifdef INTEL_PHYSICAL_MEM_CTRL
    input  wire [27:0] INTEL_MEM_CTRL,
    `endif
    input  wire [8 : 0] A,
    input  wire [199: 0] D,
    input  wire CLK,
    input  wire CEN,
    input  wire GWEN,
    input  wire [199: 0] WEN,
    output wire [199 : 0] Q,
    input  wire [2:0] EMA,
    input  wire [1:0] EMAW,
    input  wire EMAS,
    input  wire STOV,
    input  wire RET,
    input  wire QNAP
);    
    genvar i;
    generate 
    for (i=0;i<2;i=i+1)begin : SRAM_
        SRAM_SP_HDE_512x100_M2B4S2 sram_inst (
                        `ifdef INTEL_PHYSICAL_MEM_CTRL
                        .INTEL_MEM_CTRL(INTEL_MEM_CTRL),
                        `endif
                        .A     (A), // 13-bit
                        .D     (D[(i+1)*100-1 : i*100]), // 100-bit
                        .CLK   (CLK),
                        .CEN   (CEN), // chip-enable active-low
                        .GWEN  (GWEN),       // Globa; write-enable active-low
                        .WEN   (WEN[(i+1)*100-1 : i*100]), // write-enable per bit active-low (WEN[0]=LSB)
                        .Q     (Q[(i+1)*100-1 : i*100]), // 36-bit
                        .EMA   (EMA),
                        .EMAW  (EMAW),
                        .EMAS  (EMAS),                       
                        .STOV  (STOV),
                        .RET   (RET),
                        .QNAP  (QNAP)
          );    
    end
    endgenerate
endmodule


module ASIC_RAM_W144_D32768 (
    `ifdef INTEL_PHYSICAL_MEM_CTRL
    input  wire [27:0] INTEL_MEM_CTRL,
    `endif
    input  wire [14 : 0] A,
    input  wire [143: 0] D,
    input  wire CLK,
    input  wire CEN,
    input  wire GWEN,
    input  wire [143: 0] WEN,
    output wire [143 : 0] Q,
    input  wire [2:0] EMA,
    input  wire [1:0] EMAW,
    input  wire EMAS,
    input  wire STOV,
    input  wire RET,
    input  wire QNAP
);    
    genvar i;
    generate 
    for (i=0;i<4;i=i+1)begin : SRAM_
        SRAM_SP_HDE_32768x36_M16B8S2 sram_inst (
                        `ifdef INTEL_PHYSICAL_MEM_CTRL
                        .INTEL_MEM_CTRL(INTEL_MEM_CTRL),
                        `endif
                        .A     (A), // 14-bit
                        .D     (D[(i+1)*36-1 : i*36]), // 36-bit
                        .CLK   (CLK),
                        .CEN   (CEN), // chip-enable active-low
                        .GWEN  (GWEN ),       // Globa; write-enable active-low
                        .WEN   (WEN[(i+1)*36-1 : i*36]), // write-enable per bit active-low (WEN[0]=LSB)
                        .Q     (Q[(i+1)*36-1 : i*36]), // 36-bit
                        .EMA   (EMA),
                        .EMAW  (EMAW),
                        .EMAS  (EMAS),
                        .STOV  (STOV),
                        .RET   (RET),
                        .QNAP  (QNAP)
          );    
    end
    endgenerate
endmodule

module ASIC_RAM_W264_D256 (
    `ifdef INTEL_PHYSICAL_MEM_CTRL
    input  wire [27:0] INTEL_MEM_CTRL,
    `endif
    input  wire [8-1 : 0] A,
    input  wire [264-1 : 0] D,
    input  wire CLK,
    input  wire CEN,
    input  wire GWEN,
    input  wire [264-1 : 0] WEN,
    output wire [264-1 : 0] Q,
    input  wire [2:0] EMA,
    input  wire [1:0] EMAW,
    input  wire EMAS,
    input  wire STOV,
    input  wire RET,
    input  wire QNAP
);
    genvar i;
    generate
    for (i=0;i<2;i=i+1)begin : SRAM_
        RF_SP_256x132_M2B1S2 sram_inst (
                        `ifdef INTEL_PHYSICAL_MEM_CTRL
                        .INTEL_MEM_CTRL(INTEL_MEM_CTRL),
                        `endif
                        .A     (A), // 8-bit
                        .D     (D[(i+1)*132-1 : i*132]), // 132-bit
                        .CLK   (CLK),
                        .CEN   (CEN), // chip-enable active-low
                        .GWEN  (GWEN ),       // Globa; write-enable active-low
                        .WEN   (WEN[(i+1)*132-1 : i*132]), // write-enable per bit active-low (WEN[0]=LSB)
                        .Q     (Q[(i+1)*132-1 : i*132]), // 132-bit
                        .EMA   (EMA),
                        .EMAW  (EMAW),
                        .EMAS  (EMAS),
                        .STOV  (STOV),
                        .RET   (RET),
                        .QNAP  (QNAP)
          );
    end
    endgenerate
endmodule

module ASIC_RAM_W512_D64 (
    `ifdef INTEL_PHYSICAL_MEM_CTRL
    input  wire [27:0]   INTEL_MEM_CTRL,
    `endif
    input  wire [6-1 : 0] A,
    input  wire [512-1 : 0] D,
    input  wire CLK,
    input  wire CEN,
    input  wire GWEN,
    input  wire [512-1 : 0] WEN,
    output wire [512-1 : 0] Q,
    input  wire [2:0] EMA,
    input  wire [1:0] EMAW,
    input  wire EMAS,
    input  wire STOV,
    input  wire RET,
    input  wire QNAP
);    
    genvar i;
    generate 
    for (i=0;i<4;i=i+1)begin : SRAM_
        RF_SP_64x128_M2B1S2 sram_inst (
                        `ifdef INTEL_PHYSICAL_MEM_CTRL
                        .INTEL_MEM_CTRL(INTEL_MEM_CTRL),
                        `endif
                        .A     (A), // 6-bit
                        .D     (D[(i+1)*128-1 : i*128]), // 128-bit
                        .CLK   (CLK),
                        .CEN   (CEN), // chip-enable active-low
                        .GWEN  (GWEN ),       // Globa; write-enable active-low
                        .WEN   (WEN[(i+1)*128-1 : i*128]), // write-enable per bit active-low (WEN[0]=LSB)
                        .Q     (Q[(i+1)*128-1 : i*128]), // 128-bit
                        .EMA   (EMA),
                        .EMAW  (EMAW),
                        .EMAS  (EMAS),
                        .STOV  (STOV),
                        .RET   (RET),
                        .QNAP  (QNAP)
          );    
    end
    endgenerate
endmodule

//use 4 narrow width srams in parallel
module ASIC_RAM_W144_D16384 (
    `ifdef INTEL_PHYSICAL_MEM_CTRL
    input  wire [27:0] INTEL_MEM_CTRL,
    `endif
    input  wire [13 : 0] A,
    input  wire [143: 0] D,
    input  wire CLK,
    input  wire CEN,
    input  wire GWEN,
    input  wire [143: 0] WEN,
    output wire [143 : 0] Q,
    input  wire [2:0] EMA,
    input  wire [1:0] EMAW,
    input  wire EMAS,
    input  wire STOV,
    input  wire RET,
    input  wire QNAP
);    
    genvar i;
    generate 
    for (i=0;i<4;i=i+1)begin : SRAM_
        SRAM_SP_HDE_16384x36_M16B4S2 sram_inst (
                        `ifdef INTEL_PHYSICAL_MEM_CTRL
                        .INTEL_MEM_CTRL(INTEL_MEM_CTRL),
                        `endif
                        .A     (A), // 13-bit
                        .D     (D[(i+1)*36-1 : i*36]), // 36-bit
                        .CLK   (CLK),
                        .CEN   (CEN), // chip-enable active-low
                        .GWEN  (GWEN ),       // Globa; write-enable active-low
                        .WEN   (WEN[(i+1)*36-1 : i*36]), // write-enable per bit active-low (WEN[0]=LSB)
                        .Q     (Q[(i+1)*36-1 : i*36]), // 36-bit
                        .EMA   (EMA),
                        .EMAW  (EMAW),
                        .EMAS  (EMAS),
                        .STOV  (STOV),
                        .RET   (RET),
                        .QNAP  (QNAP)
          );
    end
    endgenerate
endmodule


//use two half depth rams
module ASIC_RAM_W144_D16384_2 (
    `ifdef INTEL_PHYSICAL_MEM_CTRL
    input  wire [27:0] INTEL_MEM_CTRL,
    `endif
    input  wire [13 : 0] A,
    input  wire [143: 0] D,
    input  wire CLK,
    input  wire CEN,
    input  wire GWEN,
    input  wire [143: 0] WEN,
    output wire [143 : 0] Q,
    input  wire [2:0] EMA,
    input  wire [1:0] EMAW,
    input  wire EMAS,
    input  wire STOV,
    input  wire RET,
    input  wire QNAP
);    

    wire [143: 0] Q_tmp [0 : 1];
    wire [1: 0] CEN_tmp;
    assign CEN_tmp[0] = CEN | A  [13];
    assign CEN_tmp[1] = CEN | ~A [13];

    assign Q = Q_tmp[A[13]];

    genvar i;
    generate 
    for (i=0;i<2;i=i+1)begin : SRAM_
        SRAM_SP_HDE_8192x128_M4B8S2 sram_inst (
                        `ifdef INTEL_PHYSICAL_MEM_CTRL
                        .INTEL_MEM_CTRL(INTEL_MEM_CTRL),
                        `endif
                        .A     (A[12:0]), 
                        .D     (D),
                        .CLK   (CLK),
                        .CEN   (CEN_tmp[i]), // chip-enable active-low
                        .GWEN  (GWEN ),       // Globa; write-enable active-low
                        .WEN   (WEN), // write-enable per bit active-low (WEN[0]=LSB)
                        .Q     (Q_tmp[i]),  
                        .EMA   (EMA),
                        .EMAW  (EMAW),
                        .EMAS  (EMAS),
                        .STOV  (STOV),
                        .RET   (RET),
                        .QNAP  (QNAP)
          );
    end
    endgenerate


endmodule
