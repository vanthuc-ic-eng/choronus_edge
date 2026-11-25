// ============================================================================
// Project: CHRONOS — Beyond-Q1 Zero-DSP Logarithmic Selective State-Space SoC
// File   : rtl/engine/chronos_activity_gate.sv
// Desc   : ★ BEYOND-Q1 UPGRADE — Level-1 Coarse Input-Domain Silent-Frame
//          Early-Exit Detector (0 DSP) ★
//          Computes L1 temporal variation across input vector u[t] vs u[t-1]:
//            L1_diff = sum_{f=0..D_in-1} |u_f[t] - u_f[t-1]|
//          When L1_diff < tau_step (flat ECG baseline / audio silence),
//          triggers 1-cycle FULL-TIMESTEP EARLY EXIT — bypassing Engine A,
//          Engine B, and ALL 4 TCDM SRAM banks for maximum power savings!
// ============================================================================

`timescale 1ns / 1ps

(* use_dsp = "no" *)
module chronos_activity_gate #(
    parameter int unsigned MAX_D_IN = 32,
    parameter int unsigned F_ADDR_W = $clog2(MAX_D_IN)
)(
    input  logic               clk_i,
    input  logic               rst_ni,
    input  logic               seq_rst_i,       // Clear prev u[t-1] on new sequence

    input  logic               step_skip_en_i,
    input  logic [7:0]         tau_step_i,      // Threshold for L1_diff (scaled by 2)
    input  logic               is_first_step_i, // Never early-exit on t=0!

    input  logic               eval_clr_i,      // Start evaluating new timestep t
    input  logic               eval_valid_i,    // Feed one feature u_f[t]
    input  logic [F_ADDR_W-1:0] feat_idx_i,
    input  logic signed [7:0]  u_feat_i,

    output logic [15:0]        l1_diff_o,
    output logic               early_exit_o     // 1 = Silent timestep! Bypass entire step t!
);

    logic signed [7:0] prev_u [0:MAX_D_IN-1];
    logic [15:0]       l1_acc_q;

    // Combinational |u_f[t] - u_f[t-1]|
    logic signed [8:0] diff_s9;
    logic [8:0]        abs_diff;

    always_comb begin
        diff_s9  = {u_feat_i[7], u_feat_i} - {prev_u[feat_idx_i][7], prev_u[feat_idx_i]};
        abs_diff = diff_s9[8] ? (~diff_s9 + 9'd1) : diff_s9;
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            l1_acc_q <= 16'd0;
            for (int i = 0; i < MAX_D_IN; i++) prev_u[i] <= 8'sd0;
        end else if (seq_rst_i) begin
            l1_acc_q <= 16'd0;
            for (int i = 0; i < MAX_D_IN; i++) prev_u[i] <= 8'sd0;
        end else begin
            if (eval_clr_i) begin
                l1_acc_q <= 16'd0;
            end else if (eval_valid_i) begin
                l1_acc_q          <= l1_acc_q + {7'd0, abs_diff};
                prev_u[feat_idx_i] <= u_feat_i;
            end
        end
    end

    assign l1_diff_o    = l1_acc_q;
    assign early_exit_o = step_skip_en_i && (!is_first_step_i) && (l1_acc_q < {7'd0, tau_step_i, 1'b0});

endmodule : chronos_activity_gate
