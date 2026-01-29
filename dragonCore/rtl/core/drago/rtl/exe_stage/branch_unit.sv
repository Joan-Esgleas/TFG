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
assign equal = ---; // inputs are equal
assign less = --- < ---; // rs1 < rs2 (signed)
assign less_u = --- < ---; // rs1 < rs2 (unsigned)

// Calculate target
always_comb begin
    case (instr_type_i)
        JAL: begin
            target = ---; 
        end
        JALR: begin
            target = ---;
        end

        --- // other branches

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
            taken_o = ---;
        end
        
        --- // other branches

        default: begin
            taken_o = NOT_TAKEN;
        end
    endcase
end

assign result_o = (taken_o == TAKEN)? --- : ---; // Calculate the new PC
assign link_pc_o = ---; // Calculate the link PC stored in rd

endmodule
