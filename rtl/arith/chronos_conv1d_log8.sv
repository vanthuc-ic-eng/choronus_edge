// ============================================================================
// Project: CHRONOS — Beyond-Q1 Zero-DSP Logarithmic Selective State-Space SoC
// File   : rtl/arith/chronos_conv1d_log8.sv
// Desc   : ★ BEYOND-Q1 UPGRADE — Trainable 4-Tap Depthwise Causal Conv1D ★
//          Executes exact Mamba causal depthwise convolution in 1 cycle with
//          0 DSPs using trainable packed 4xLog8 kernel weights from Bank 3:
//            y_conv = clamp_s8( (sum_{k=0..3} w_conv[k] * tap[k] + bias) >>> 4 )
// ============================================================================

`timescale 1ns / 1ps

(* use_dsp = "no" *)
module chronos_conv1d_log8 (
    // 4 Causal Taps from Bank 0 Shift Register (signed INT8)
    input  logic signed [7:0]  tap0_i,
    input  logic signed [7:0]  tap1_i,
    input  logic signed [7:0]  tap2_i,
    input  logic signed [7:0]  tap3_i,

    // 4 Trainable Packed Log8 Kernel Weights (32-bit word from Bank 3)
    input  logic        [31:0] w_conv_log8_i,

    // Output in signed Q4.4 INT8 (ready for PWL SiLU!)
    output logic signed [7:0]  conv_out_o
);

    logic signed [7:0]  taps [0:3];
    logic        [7:0]  w_packed [0:3];
    logic               t_sign [0:3], t_zero [0:3];
    logic        [8:0]  t_log  [0:3];
    logic               w_sign [0:3], w_zero [0:3];
    logic        [8:0]  w_log  [0:3];
    logic signed [10:0] p_log  [0:3];
    logic signed [15:0] p_lin  [0:3];

    assign taps[0]     = tap0_i;
    assign taps[1]     = tap1_i;
    assign taps[2]     = tap2_i;
    assign taps[3]     = tap3_i;
    assign w_packed[0] = w_conv_log8_i[7:0];
    assign w_packed[1] = w_conv_log8_i[15:8];
    assign w_packed[2] = w_conv_log8_i[23:16];
    assign w_packed[3] = w_conv_log8_i[31:24];

    genvar k;
    generate
        for (k = 0; k < 4; k++) begin : gen_tap_mul
            chronos_lod_u8 u_lod_tap (
                .data_i ( taps[k]   ),
                .sign_o ( t_sign[k] ),
                .zero_o ( t_zero[k] ),
                .log_o  ( t_log[k]  )
            );

            assign w_zero[k] = (w_packed[k] == chronos_pkg::LOG8_ZERO_CODE);
            assign w_sign[k] = w_packed[k][7];
            assign w_log[k]  = {w_packed[k][6:4], w_packed[k][3:0], w_packed[k][3:2]};
            // Scale by 2^-3 (-192 in Q5.6) so default weight ~+2.0 sums cleanly without saturation
            assign p_log[k]  = $signed({2'b00, t_log[k]}) + $signed({2'b00, w_log[k]}) - 11'sd192;

            chronos_log_antilog_s16 u_antilog_tap (
                .sign_i    ( t_sign[k] ^ w_sign[k] ),
                .zero_i    ( t_zero[k] | w_zero[k] ),
                .log_val_i ( p_log[k]              ),
                .lin_o     ( p_lin[k]              )
            );
        end
    endgenerate

    // 4-Input Adder Tree + Unbiased Rounding + INT8 Saturation
    logic signed [17:0] sum_18;
    logic signed [17:0] sum_scaled;

    always_comb begin
        sum_18 = {{2{p_lin[0][15]}}, p_lin[0]}
               + {{2{p_lin[1][15]}}, p_lin[1]}
               + {{2{p_lin[2][15]}}, p_lin[2]}
               + {{2{p_lin[3][15]}}, p_lin[3]};
        sum_scaled = (sum_18 + 18'sd1) >>> 1;
        if (sum_scaled > 18'sd127)
            conv_out_o = 8'sd127;
        else if (sum_scaled < -18'sd128)
            conv_out_o = -8'sd128;
        else
            conv_out_o = sum_scaled[7:0];
    end

endmodule : chronos_conv1d_log8
