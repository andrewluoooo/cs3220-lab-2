`include "define.vh"

module AGEX_STAGE(
  input wire clk,
  input wire reset,
  input wire [`from_MEM_to_AGEX_WIDTH-1:0] from_MEM_to_AGEX,
  input wire [`from_WB_to_AGEX_WIDTH-1:0] from_WB_to_AGEX,
  input wire [`DE_latch_WIDTH-1:0] from_DE_latch,
  output wire [`AGEX_latch_WIDTH-1:0] AGEX_latch_out,
  output wire [`from_AGEX_to_FE_WIDTH-1:0] from_AGEX_to_FE,
  output wire [`from_AGEX_to_DE_WIDTH-1:0] from_AGEX_to_DE
);

  `UNUSED_VAR (from_MEM_to_AGEX)
  `UNUSED_VAR (from_WB_to_AGEX)

  reg [`AGEX_latch_WIDTH-1:0] AGEX_latch;
  // wire to send the AGEX latch contents to other pipeline stages
  assign AGEX_latch_out = AGEX_latch;

  wire[`AGEX_latch_WIDTH-1:0] AGEX_latch_contents;

  wire valid_AGEX;
  wire [`INSTBITS-1:0]inst_AGEX;
  wire [`DBITS-1:0]PC_AGEX;
  wire [`DBITS-1:0] inst_count_AGEX;
  wire [`DBITS-1:0] pcplus_AGEX;
  wire [`IOPBITS-1:0] op_I_AGEX;
  reg br_cond_AGEX;

  wire is_br_AGEX;
  wire is_jmp_AGEX;
  wire rd_mem_AGEX;
  wire wr_mem_AGEX;
  wire wr_reg_AGEX;
  wire [`REGNOBITS-1:0] wregno_AGEX;

  wire [`DBITS-1:0] regval1_AGEX;
  wire [`DBITS-1:0] regval2_AGEX;
  wire [`DBITS-1:0] sxt_imm_AGEX;

  // Branch predictor signals propagated from FE stage
  wire [`DBITS-1:0] predicted_next_pc_AGEX;
  wire [7:0] pht_index_AGEX;

  reg [`DBITS-1:0] aluout_AGEX;
  reg [`DBITS-1:0] memaddr_AGEX;

  // Branch prediction validation signals
  reg [`DBITS-1:0] computed_target_AGEX;
  wire actual_taken_AGEX;
  wire [`DBITS-1:0] actual_next_pc_AGEX;
  wire br_mispred_AGEX;
  wire is_br_or_jmp_AGEX;

  // Calculate branch condition
  always @ (*) begin
    case (op_I_AGEX)
    `BEQ_I : br_cond_AGEX = (regval1_AGEX == regval2_AGEX);
    `BNE_I : br_cond_AGEX = (regval1_AGEX != regval2_AGEX);
    `BLT_I : br_cond_AGEX = ($signed(regval1_AGEX) < $signed(regval2_AGEX));
    `BGE_I : br_cond_AGEX = ($signed(regval1_AGEX) >= $signed(regval2_AGEX));
    `BLTU_I: br_cond_AGEX = (regval1_AGEX < regval2_AGEX);
    `BGEU_I: br_cond_AGEX = (regval1_AGEX >= regval2_AGEX);
    default: br_cond_AGEX = 1'b0;
    endcase
  end

  // Compute ALU operations
  always @ (*) begin
    case (op_I_AGEX)
    `ADD_I:   aluout_AGEX = regval1_AGEX + regval2_AGEX;
    `SUB_I:   aluout_AGEX = regval1_AGEX - regval2_AGEX;
    `AND_I:   aluout_AGEX = regval1_AGEX & regval2_AGEX;
    `OR_I:    aluout_AGEX = regval1_AGEX | regval2_AGEX;
    `XOR_I:   aluout_AGEX = regval1_AGEX ^ regval2_AGEX;
    `SLT_I:   aluout_AGEX = ($signed(regval1_AGEX) < $signed(regval2_AGEX)) ? 1 : 0;
    `SLTU_I:  aluout_AGEX = (regval1_AGEX < regval2_AGEX) ? 1 : 0;
    `SRA_I:   aluout_AGEX = $signed(regval1_AGEX) >>> $signed(regval2_AGEX[4:0]);
    `SRL_I:   aluout_AGEX = regval1_AGEX >> regval2_AGEX[4:0];
    `SLL_I:   aluout_AGEX = regval1_AGEX << regval2_AGEX[4:0];
    `MUL_I:   aluout_AGEX = $signed(regval1_AGEX) * $signed(regval2_AGEX);
    `ADDI_I:  aluout_AGEX = regval1_AGEX + sxt_imm_AGEX;
    `ANDI_I:  aluout_AGEX = regval1_AGEX & sxt_imm_AGEX;
    `ORI_I:   aluout_AGEX = regval1_AGEX | sxt_imm_AGEX;
    `XORI_I:  aluout_AGEX = regval1_AGEX ^ sxt_imm_AGEX;
    `SLTI_I:  aluout_AGEX = ($signed(regval1_AGEX) < $signed(sxt_imm_AGEX)) ? 1 : 0;
    `SLTIU_I: aluout_AGEX = (regval1_AGEX < sxt_imm_AGEX) ? 1 : 0;
    `SRAI_I:  aluout_AGEX = $signed(regval1_AGEX) >>> $signed(sxt_imm_AGEX[4:0]);
    `SRLI_I:  aluout_AGEX = regval1_AGEX >> sxt_imm_AGEX[4:0];
    `SLLI_I:  aluout_AGEX = regval1_AGEX << sxt_imm_AGEX[4:0];
    `LUI_I:   aluout_AGEX = sxt_imm_AGEX;
    `AUIPC_I: aluout_AGEX = PC_AGEX + sxt_imm_AGEX;
    `JAL_I,
    `JALR_I:  aluout_AGEX = pcplus_AGEX;
    `LW_I:    memaddr_AGEX = regval1_AGEX + sxt_imm_AGEX;
    `SW_I: begin
      memaddr_AGEX = regval1_AGEX + sxt_imm_AGEX;
      aluout_AGEX = regval2_AGEX;
    end
    default: begin
      aluout_AGEX  = '0;
      memaddr_AGEX = '0;
    end
    endcase
  end

  // Computed branch/jump target (always as-if-taken, for BTB update)
  always @(*) begin
    if (op_I_AGEX == `JAL_I)
      computed_target_AGEX = PC_AGEX + sxt_imm_AGEX;
    else if (op_I_AGEX == `JR_I)
      computed_target_AGEX = regval1_AGEX;
    else if (op_I_AGEX == `JALR_I)
      computed_target_AGEX = (regval1_AGEX + sxt_imm_AGEX) & 32'hfffffffe;
    else if (is_br_AGEX)
      computed_target_AGEX = PC_AGEX + sxt_imm_AGEX;
    else
      computed_target_AGEX = pcplus_AGEX;
  end

  // Branch prediction validation
  assign is_br_or_jmp_AGEX = is_br_AGEX || is_jmp_AGEX;
  assign actual_taken_AGEX = is_jmp_AGEX || (is_br_AGEX && br_cond_AGEX);
  assign actual_next_pc_AGEX = actual_taken_AGEX ? computed_target_AGEX : pcplus_AGEX;
  assign br_mispred_AGEX = is_br_or_jmp_AGEX && (actual_next_pc_AGEX != predicted_next_pc_AGEX);

  // Unpack DE latch (order must match de_stage.v packing)
    assign  {
                                  valid_AGEX,
                                  inst_AGEX,
                                  PC_AGEX,
                                  pcplus_AGEX,
                                  op_I_AGEX,
                                  inst_count_AGEX,
                                  regval1_AGEX,
                                  regval2_AGEX,
                                  sxt_imm_AGEX,
                                  predicted_next_pc_AGEX,
                                  pht_index_AGEX,
                                  is_br_AGEX,
                                  is_jmp_AGEX,
                                  rd_mem_AGEX,
                                  wr_mem_AGEX,
                                  wr_reg_AGEX,
                                  wregno_AGEX
                                  } = from_DE_latch;


  assign AGEX_latch_contents = {
                                valid_AGEX,
                                inst_AGEX,
                                PC_AGEX,
                                op_I_AGEX,
                                inst_count_AGEX,
                                memaddr_AGEX,
                                aluout_AGEX,
                                rd_mem_AGEX,
                                wr_mem_AGEX,
                                wr_reg_AGEX,
                                wregno_AGEX
                                 };

  always @ (posedge clk ) begin
    if(reset) begin
      AGEX_latch <= {`AGEX_latch_WIDTH{1'b0}};
        end
    else
        begin
            AGEX_latch <= AGEX_latch_contents ;
        end
  end


  // forward signals to FE stage (BP update signals)
  assign from_AGEX_to_FE = {
    br_mispred_AGEX,
    is_br_or_jmp_AGEX,
    actual_taken_AGEX,
    PC_AGEX,
    computed_target_AGEX,
    pht_index_AGEX
  };

  // forward signals to DE stage
  assign from_AGEX_to_DE = {
    br_mispred_AGEX
  };

endmodule
