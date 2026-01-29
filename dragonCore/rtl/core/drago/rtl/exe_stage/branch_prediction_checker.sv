module branch_prediction_checker (
    input logic stall_o,
    input instr_type_t instr_type,              // execution stage instruction type
    input branch_decision_t taken_branch,  // real branch decision (taken or not taken)
    input addrPC_t result_branch,               // real branch address
    input branch_pred_t bpred,                  // branch predictor prediction (.is_branch, .decision, .pred_addr)
    output logic correct_branch_pred_o          // branch prediction correctness
);
    // Function to check if the instruction is a conditional branch
    function logic is_inst_branch(instr_type_t instr_type_int);
        return  instr_type_int == BLT  |
                instr_type_int == BLTU |
                instr_type_int == BGE  |
                instr_type_int == BGEU |
                instr_type_int == BEQ  |
                instr_type_int == BNE;
    endfunction

    `ifdef BP_ENABLE
        always_comb begin
            // If the instruction is a JAL or if there is a stall
            if (instr_type == JAL || stall_o) begin
                correct_branch_pred_o = 1'b1;
            // If it is not a branch instruction
            end else if (!is_inst_branch(instr_type) && instr_type != JALR) begin
                // Check that the prediction instr is not a branch or the prediction is NOT_TAKEN            
                correct_branch_pred_o = ...;
            // If it is a branch instruction
            end else begin
                // the prediction is correct if:
                // 1. The branch is taken and prediction is taken and addresses are the same
                // 2. The branch is not taken and prediction is not taken
                // 3. The branch is not taken and the prediction is taken but the branch address are the same
                //    (since the branch address is correct, the next pc will be correct) 
                correct_branch_pred_o = ...;
            end
        end
    `else
        always_comb begin
            // If the instruction is a JAL or if there is a stall
            if (instr_type == JAL || stall_o) begin
                correct_branch_pred_o = 1'b1;
            end else begin
                // miss prediction if the instruction is a branch           
                correct_branch_pred_o = (!is_inst_branch(instr_type) && instr_type != JALR);
            end 
        end 
    `endif 

endmodule