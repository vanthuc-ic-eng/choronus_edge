// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/arith/chronos_pe_log8_mac.sv
// Desc   : Retimed 2-Stage Zero-DSP Output-Stationary Log8 MAC PE.
//          Keeps Engine A lean and >120 MHz clean (single mode: LOG8_MAC).
//          - Stage 1: Log8 zero-gate unpack + 9-bit log addition + Antilog
//          - Stage 2: 32-bit signed accumulation w/ bias load & zero-skip
// ============================================================================

`timescale 1ns / 1ps

(* use_dsp = "no" *)
module chronos_pe_log8_mac (
    input  logic               clk_i,
    input  logic               rst_ni,
    input  logic               valid_i,       // MAC enable
    input  logic               clr_i,         // Clear accumulator / load bias

    // Pre-encoded Activation (broadcast from Shared LOD)
    input  logic               act_sign_i,
    input  logic               act_zero_i,
    input  logic        [8:0]  act_log_i,     // Unsigned Q3.6

    // Packed Log8 Weight directly from 32-bit Bank 2 BRAM
    // Format: {sign[7], k[6:4], m[3:0]}, 8'h7F = Exact Zero
    input  logic        [7:0]  weight_log8_i,

    // Initial 32-bit Bias
    input  logic signed [31:0] bias_i,

    // Accumulated 32-bit Output
    output logic signed [31:0] acc_o
);

    // ------------------------------------------------------------------------
    // Stage 1: Zero-Gate Weight Unpack + Log Addition + Pipeline Register
    // ------------------------------------------------------------------------
    logic               w_sign;
    logic               w_zero;
    logic        [8:0]  w_log_q3_6;
    logic               p_sign_d;
    logic               p_zero_d;
    logic signed [10:0] p_log_sum_d;

    always_comb begin
        w_zero     = (weight_log8_i == chronos_pkg::LOG8_ZERO_CODE);
        w_sign     = weight_log8_i[7];
        w_log_q3_6 = {weight_log8_i[6:4], weight_log8_i[3:0], weight_log8_i[3:2]};

        p_sign_d    = act_sign_i ^ w_sign;
        p_zero_d    = act_zero_i | w_zero;
        p_log_sum_d = $signed({2'b00, act_log_i}) + $signed({2'b00, w_log_q3_6});
    end

    // Retimed Pipeline Registers between Log-Sum (Stage 1) and Antilog+Acc (Stage 2)
    // Balances combinational delay cleanly (~3.2 ns/stage on Zynq-7010 -1)
    logic               valid_q;
    logic               p_sign_q;
    logic               p_zero_q;
    logic signed [10:0] p_log_sum_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_q     <= 1'b0;
            p_sign_q    <= 1'b0;
            p_zero_q    <= 1'b1;
            p_log_sum_q <= 11'sd0;
        end else begin
            valid_q     <= valid_i;
            p_sign_q    <= p_sign_d;
            p_zero_q    <= p_zero_d;
            p_log_sum_q <= p_log_sum_d;
        end
    end

    // ------------------------------------------------------------------------
    // Stage 2: Unbiased Curvature-Compensated Antilog + 32-bit Accumulator
    // ------------------------------------------------------------------------
    logic signed [15:0] prod_q;

    chronos_log_antilog_s16 u_antilog (
        .sign_i    ( p_sign_q    ),
        .zero_i    ( p_zero_q    ),
        .log_val_i ( p_log_sum_q ),
        .lin_o     ( prod_q      )
    );

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            acc_o <= 32'sd0;
        end else if (clr_i) begin
            acc_o <= bias_i;
        end else if (valid_q && !p_zero_q) begin
            acc_o <= acc_o + {{16{prod_q[15]}}, prod_q};
        end
    end

endmodule : chronos_pe_log8_mac
