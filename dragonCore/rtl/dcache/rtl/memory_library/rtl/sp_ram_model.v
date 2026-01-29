/* -----------------------------------------------
* File           : sp_ram_model.sv
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

// Single port RAM model for simulation and FPGA. 
// The memory is mapped to BRAMs for FPGA.

module sp_ram_model #(
    parameter ADDR_WIDTH=1, 
    parameter DATA_WIDTH=1
) (
   input wire  [ADDR_WIDTH-1  : 0]  A,
   input wire  [DATA_WIDTH-1  : 0]  DI,
   input wire  [DATA_WIDTH-1  : 0]  BW,
   input wire  CLK,CE, RDWEN,
   output wire [DATA_WIDTH-1  : 0]  DO
);   
    
    localparam DEPTH = 2 ** ADDR_WIDTH;
    
    //RDWEN = 1 means write, 0 means read
    
    // SRAM
    reg [DATA_WIDTH-1:0] ram [DEPTH-1:0];
    reg [DATA_WIDTH-1  : 0] DATA_OUT;
    
    always@(posedge CLK) begin
        if(CE) begin
            if(RDWEN)
                ram[A] <= (DI & BW) | (ram[A] & ~BW);
            else
                DATA_OUT <= ram[A];
        end
    end
    
    assign DO = DATA_OUT; 

endmodule
