 `include "define.vh"


module FE_STAGE(
  input wire clk,
  input wire reset,
  input wire [`from_DE_to_FE_WIDTH-1:0] from_DE_to_FE,
  input wire [`from_AGEX_to_FE_WIDTH-1:0] from_AGEX_to_FE,
  input wire [`from_MEM_to_FE_WIDTH-1:0] from_MEM_to_FE,
  input wire [`from_WB_to_FE_WIDTH-1:0] from_WB_to_FE,
  output wire [`FE_latch_WIDTH-1:0] FE_latch_out
);

  `UNUSED_VAR (from_MEM_to_FE)
  `UNUSED_VAR (from_WB_to_FE)

  // I-MEM
  (* ram_init_file = `IDMEMINITFILE *)
  reg [`DBITS-1:0] imem [`IMEMWORDS-1:0];

  initial begin
      $readmemh(`IDMEMINITFILE , imem);
  end

  /* pipeline latch */
  reg [`FE_latch_WIDTH-1:0] FE_latch;  // FE latch
  wire valid_FE;

  `UNUSED_VAR(valid_FE)
  reg [`DBITS-1:0] PC_FE_latch; // PC latch in the FE stage

  reg [`DBITS-1:0] inst_count_FE; /* for debugging purpose */

  wire [`DBITS-1:0] inst_count_AGEX; /* for debugging purpose */

  wire [`INSTBITS-1:0] inst_FE;  // instruction value in the FE stage
  wire [`DBITS-1:0] pcplus_FE;  // pc plus value in the FE stage
  wire stall_pipe_FE; // signal to indicate when a front-end needs to be stall

  wire [`FE_latch_WIDTH-1:0] FE_latch_contents;  // the signals that will be FE latch contents

  // reading instruction from imem
  assign inst_FE = imem[PC_FE_latch[`IMEMADDRBITS-1:`IMEMWORDBITS]];

  // wire to send the FE latch contents to the DE stage
  assign FE_latch_out = FE_latch;

  // This is the value of "incremented PC", computed in the FE stage
  assign pcplus_FE = PC_FE_latch + `INSTSIZE;

  // =============================================
  // Branch Predictor: PHT, BTB, BHR
  // =============================================

  // Branch History Register (8-bit)
  reg [7:0] BHR;

  // Pattern History Table: 256 entries of 2-bit saturating counters
  reg [1:0] PHT [0:255];

  // Branch Target Buffer: 16 entries
  reg [15:0] BTB_valid;
  reg [25:0] BTB_tag [0:15];
  reg [31:0] BTB_target [0:15];

  // Prediction logic (combinational)
  wire [7:0] pht_index_FE;
  wire [3:0] btb_index_FE;
  wire btb_hit_FE;
  wire [1:0] pht_counter_FE;
  wire pht_predict_taken_FE;
  wire [`DBITS-1:0] predicted_next_pc_FE;

  assign pht_index_FE = PC_FE_latch[9:2] ^ BHR;
  assign btb_index_FE = PC_FE_latch[5:2];
  assign btb_hit_FE = BTB_valid[btb_index_FE] && (BTB_tag[btb_index_FE] == PC_FE_latch[31:6]);
  assign pht_counter_FE = PHT[pht_index_FE];
  assign pht_predict_taken_FE = (pht_counter_FE >= 2'd2);
  assign predicted_next_pc_FE = (btb_hit_FE && pht_predict_taken_FE) ? BTB_target[btb_index_FE] : pcplus_FE;

  // =============================================
  // FE latch contents (must match DE stage unpacking)
  // =============================================
  assign FE_latch_contents = {
                                valid_FE,
                                inst_FE,
                                PC_FE_latch,
                                pcplus_FE,
                                predicted_next_pc_FE,
                                pht_index_FE,
                                inst_count_FE
                                };

  // =============================================
  // Signals from other stages
  // =============================================
  wire br_mispred_AGEX;
  wire is_br_or_jmp_AGEX;
  wire actual_taken_AGEX;
  wire [`DBITS-1:0] PC_AGEX;
  wire [`DBITS-1:0] computed_target_AGEX;
  wire [7:0] pht_idx_update_AGEX;

  assign {
    stall_pipe_FE
  } = from_DE_to_FE[0];

  assign {
    br_mispred_AGEX,
    is_br_or_jmp_AGEX,
    actual_taken_AGEX,
    PC_AGEX,
    computed_target_AGEX,
    pht_idx_update_AGEX
  } = from_AGEX_to_FE;

  // Compute correct redirect PC on misprediction
  wire [`DBITS-1:0] correct_redirect_pc;
  assign correct_redirect_pc = actual_taken_AGEX ? computed_target_AGEX : (PC_AGEX + `INSTSIZE);

  // =============================================
  // PC update logic
  // =============================================
  always @ (posedge clk) begin
   if (reset) begin
      PC_FE_latch <= `STARTPC;
      inst_count_FE <= 1;
      end
    else if (br_mispred_AGEX)
      PC_FE_latch <= correct_redirect_pc;
    else if (stall_pipe_FE)
      PC_FE_latch <= PC_FE_latch;
    else begin
      PC_FE_latch <= predicted_next_pc_FE;
      inst_count_FE <= inst_count_FE + 1;
      end
  end

  // =============================================
  // FE latch update
  // =============================================
  always @ (posedge clk) begin
    if (reset) begin
      FE_latch <= '0;
    end else begin
      if (br_mispred_AGEX)
        FE_latch <= '0;
      else if (stall_pipe_FE)
        FE_latch <= FE_latch;
      else
        FE_latch <= FE_latch_contents;
    end
  end

  // =============================================
  // BTB, PHT, BHR update (from AGEX stage)
  // =============================================
  integer i;
  always @ (posedge clk) begin
    if (reset) begin
      BHR <= 8'b0;
      BTB_valid <= 16'b0;
      for (i = 0; i < 256; i = i + 1)
        PHT[i] <= 2'b01;  // weakly not taken
      for (i = 0; i < 16; i = i + 1) begin
        BTB_tag[i] <= 26'b0;
        BTB_target[i] <= 32'b0;
      end
    end else begin
      if (is_br_or_jmp_AGEX) begin
        // Update BTB
        BTB_valid[PC_AGEX[5:2]] <= 1'b1;
        BTB_tag[PC_AGEX[5:2]] <= PC_AGEX[31:6];
        BTB_target[PC_AGEX[5:2]] <= computed_target_AGEX;

        // Update PHT (saturating counter)
        if (actual_taken_AGEX && PHT[pht_idx_update_AGEX] < 2'd3)
          PHT[pht_idx_update_AGEX] <= PHT[pht_idx_update_AGEX] + 2'd1;
        else if (!actual_taken_AGEX && PHT[pht_idx_update_AGEX] > 2'd0)
          PHT[pht_idx_update_AGEX] <= PHT[pht_idx_update_AGEX] - 2'd1;

        // Update BHR (shift left, insert taken bit)
        BHR <= {BHR[6:0], actual_taken_AGEX};
      end
    end
  end

endmodule
