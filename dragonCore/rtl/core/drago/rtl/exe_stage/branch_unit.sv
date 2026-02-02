module branch_unit
    import drac_pkg::*;
(
    input instr_type_t                  instr_type_i,

    input addrPC_t                      pc_i,
    input bus64_t                       data_rs1_i,
    input bus64_t                       data_rs2_i,
    input bus64_t                       imm_i,

    output branch_decision_t       taken_o,
    output addrPC_t                     result_o,
    output addrPC_t                     link_pc_o
);

logic equal;
logic less;
logic less_u;

addrPC_t  target;

// Calculate all posible conditions
assign equal = data_rs1_i == data_rs2_i; // inputs are equal
assign less = $signed(data_rs1_i) < $signed(data_rs2_i); // rs1 < rs2 (signed)
assign less_u = data_rs1_i < data_rs2_i; // rs1 < rs2 (unsigned)

// Calculate target
always_comb begin
    case (instr_type_i)
        JAL: begin
            target = pc_i + imm_i; 
        end
        JALR: begin
            target = (data_rs1_i + imm_i) & 64'hfffffffffffffffe;
        end
        BEQ: begin
            target = pc_i + imm_i; 
        end
        BNE: begin
            target = pc_i + imm_i; 
        end
        BLT: begin
            target = pc_i + imm_i; 
        end
        BGE: begin
            target = pc_i + imm_i; 
        end
        BLTU: begin
            target = pc_i + imm_i; 
        end
        BGEU: begin
            target = pc_i + imm_i; 
        end
        default: begin
            target = 64'h0;
        end
    endcase
end

// Calculate taken
always_comb begin
    case (instr_type_i)
        JAL: begin
            taken_o = NOT_TAKEN; // Redirection done in the decode stage 
        end
        JALR: begin
            taken_o = TAKEN; // Redirection done in the decode stage 
        end
         BEQ: begin
            taken_o = (equal)?TAKEN:NOT_TAKEN; 
        end
        BNE: begin
            taken_o = (equal)?NOT_TAKEN:TAKEN; 
        end
        BLT: begin
            taken_o = (less)?TAKEN:NOT_TAKEN; 
        end
        BGE: begin
            taken_o = (less)?NOT_TAKEN:TAKEN; 
        end
        BLTU: begin
            taken_o = (less_u)?TAKEN:NOT_TAKEN; 
        end
        BGEU: begin
            taken_o = (less_u)?NOT_TAKEN:TAKEN; 
        end
        default: begin
            taken_o = NOT_TAKEN;
        end
    endcase
end

assign result_o = (taken_o == TAKEN)? target : pc_i+4; // Calculate the new PC
assign link_pc_o = pc_i+4; // Calculate the link PC stored in rd

endmodule
