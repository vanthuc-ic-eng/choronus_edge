// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/engine/chronos_proj_engine_1x4.sv
// Desc   : ENGINE A — 1x4 SIMD Zero-DSP Log8 Projection & Conv1D Core.
//          Single lean mode (LOG8_MAC) for >=120 MHz clean timing:
//            - 1 Shared chronos_lod_u8 (cuts LOD logic by 75%)
//            - 4 Retimed chronos_pe_log8_mac lanes
//            - Glitch-free ICG clock gating when idle
//            - Built-in arithmetic right-shift + rounding INT8 requantizer
// ============================================================================

`timescale 1ns / 1ps

(* use_dsp = "no" *)
module chronos_proj_engine_1x4 (
    input  logic               clk_i,
    input  logic               rst_ni,
    input  logic               cg_en_i,       // Clock-gate enable for Engine A
    input  logic               test_en_i,

    input  logic               valid_i,
    input  logic               clr_i,
    input  logic        [4:0]  requant_sh_i,  // Right-shift amount for INT8 output

    // Shared Activation Input (INT8)
    input  logic signed [7:0]  act_i,

    // 4 Packed Log8 Weights (from Bank 2)
    input  logic        [7:0]  w0_log8_i,
    input  logic        [7:0]  w1_log8_i,
    input  logic        [7:0]  w2_log8_i,
    input  logic        [7:0]  w3_log8_i,

    // 4 Initial 32-bit Biases (from Bank 3)
    input  logic signed [31:0] bias0_i,
    input  logic signed [31:0] bias1_i,
    input  logic signed [31:0] bias2_i,
    input  logic signed [31:0] bias3_i,

    // Raw 32-bit Accumulator Outputs
    output logic signed [31:0] acc0_o,
    output logic signed [31:0] acc1_o,
    output logic signed [31:0] acc2_o,
    output logic signed [31:0] acc3_o,

    // Requantized INT8 Outputs (rounded & clamped [-128, +127])
    output logic signed [7:0]  q0_o,
    output logic signed [7:0]  q1_o,
    output logic signed [7:0]  q2_o,
    output logic signed [7:0]  q3_o
);

    // 1. Glitch-Free ICG Cell
    logic clk_gated;

    chronos_icg u_icg_eng_a (
        .clk_i     ( clk_i     ),
        .en_i      ( cg_en_i   ),
        .test_en_i ( test_en_i ),
        .clk_o     ( clk_gated )
    );

    // 2. Shared Activation LOD Encoder (encoded ONCE, broadcast to all 4 PEs)
    logic       act_sign;
    logic       act_zero;
    logic [8:0] act_log;

    chronos_lod_u8 u_shared_lod (
        .data_i ( act_i    ),
        .sign_o ( act_sign ),
        .zero_o ( act_zero ),
        .log_o  ( act_log  )
    );

    // 3. 1x4 SIMD Log8 MAC Array
    chronos_pe_log8_mac u_pe0 (
        .clk_i         ( clk_gated ),
        .rst_ni        ( rst_ni    ),
        .valid_i       ( valid_i   ),
        .clr_i         ( clr_i     ),
        .act_sign_i    ( act_sign  ),
        .act_zero_i    ( act_zero  ),
        .act_log_i     ( act_log   ),
        .weight_log8_i ( w0_log8_i ),
        .bias_i        ( bias0_i   ),
        .acc_o         ( acc0_o    )
    );

    chronos_pe_log8_mac u_pe1 (
        .clk_i         ( clk_gated ),
        .rst_ni        ( rst_ni    ),
        .valid_i       ( valid_i   ),
        .clr_i         ( clr_i     ),
        .act_sign_i    ( act_sign  ),
        .act_zero_i    ( act_zero  ),
        .act_log_i     ( act_log   ),
        .weight_log8_i ( w1_log8_i ),
        .bias_i        ( bias1_i   ),
        .acc_o         ( acc1_o    )
    );

    chronos_pe_log8_mac u_pe2 (
        .clk_i         ( clk_gated ),
        .rst_ni        ( rst_ni    ),
        .valid_i       ( valid_i   ),
        .clr_i         ( clr_i     ),
        .act_sign_i    ( act_sign  ),
        .act_zero_i    ( act_zero  ),
        .act_log_i     ( act_log   ),
        .weight_log8_i ( w2_log8_i ),
        .bias_i        ( bias2_i   ),
        .acc_o         ( acc2_o    )
    );

    chronos_pe_log8_mac u_pe3 (
        .clk_i         ( clk_gated ),
        .rst_ni        ( rst_ni    ),
        .valid_i       ( valid_i   ),
        .clr_i         ( clr_i     ),
        .act_sign_i    ( act_sign  ),
        .act_zero_i    ( act_zero  ),
        .act_log_i     ( act_log   ),
        .weight_log8_i ( w3_log8_i ),
        .bias_i        ( bias3_i   ),
        .acc_o         ( acc3_o    )
    );

    // 4. Combinational Round-and-Clamp Requantizer (INT32 -> INT8)
    function automatic logic signed [7:0] requant_s8 (
        input logic signed [31:0] val,
        input logic        [4:0]  sh
    );
        logic signed [31:0] shifted;
        logic signed [31:0] rnd;
        rnd     = (sh == 5'd0) ? 32'sd0 : (32'sd1 <<< (sh - 5'd1));
        shifted = (val + rnd) >>> sh;
        if (shifted > 32'sd127)
            requant_s8 = 8'sd127;
        else if (shifted < -32'sd128)
            requant_s8 = -8'sd128;
        else
            requant_s8 = shifted[7:0];
    endfunction

    assign q0_o = requant_s8(acc0_o, requant_sh_i);
    assign q1_o = requant_s8(acc1_o, requant_sh_i);
    assign q2_o = requant_s8(acc2_o, requant_sh_i);
    assign q3_o = requant_s8(acc3_o, requant_sh_i);

endmodule : chronos_proj_engine_1x4
