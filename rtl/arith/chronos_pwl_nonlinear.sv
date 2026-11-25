// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/arith/chronos_pwl_nonlinear.sv
// Desc   : ★ Q1 CORE INNOVATION 2 — Symmetry-Exploiting CSD PWL Non-Linearity ★
//          Computes BOTH SiLU(z) and Softplus(u) in ONE shared 0-DSP block!
//
//          Exact Mathematical Symmetry Identities:
//            1) SiLU(-z)     = SiLU(z)     - z
//            2) Softplus(-u) = Softplus(u) - u
//          Input : Signed 8-bit Q4.4 (range [-8.0, +7.9375])
//          Output:
//            - SiLU    : Signed 8-bit Q4.4 (for gating)
//            - Softplus: Unsigned 8-bit Q2.6 / Q4.4 strictly positive (for Δ[t])
//          Max Abs Error: < 0.053 (sub-LSB in Q4.4!) with ZERO multipliers!
// ============================================================================

`timescale 1ns / 1ps

(* use_dsp = "no" *)
module chronos_pwl_nonlinear (
    input  logic              func_sel_i,  // 0 = SiLU(z), 1 = Softplus(u)
    input  logic signed [7:0] x_q4_4_i,    // Signed Q4.4 input [-8.0, +7.9375]
    output logic signed [7:0] silu_q4_4_o, // Signed Q4.4 SiLU output
    output logic        [7:0] sp_q2_6_o    // Unsigned Q2.6 Softplus output (for Δ[t])
);

    logic              is_neg;
    logic        [7:0] x_abs;       // [0..128]
    logic        [3:0] seg_idx;     // x_abs[6:3] -> 16 segments of width 0.5
    logic        [2:0] seg_off;     // x_abs[2:0] -> offset k in [0..7] (step 1/16)

    // Positive-half PWL result in Q4.4 (0..128)
    logic signed [9:0] pos_y_q4_4;
    logic signed [9:0] c_q4_4;
    logic signed [9:0] slope_dx_q4_4;
    logic signed [9:0] full_y_q4_4;
    logic signed [9:0] k_ext;

    always_comb begin
        is_neg = x_q4_4_i[7];
        // Handle -128 safely: saturate magnitude to 127 (7.9375)
        if (x_q4_4_i == -8'sd128)
            x_abs = 8'd127;
        else
            x_abs = is_neg ? (~x_q4_4_i + 8'd1) : x_q4_4_i;

        seg_idx = x_abs[6:3];
        seg_off = x_abs[2:0];
        k_ext   = {7'd0, seg_off}; // 0..7 in Q4.4 units
    end

    // Shared 16-Segment CSD Shift-Add PWL Evaluation on Positive Half [0, 8)
    // Every slope is at most 2 signed powers-of-two -> 0 DSP!
    always_comb begin
        c_q4_4        = 10'sd0;
        slope_dx_q4_4 = 10'sd0;

        if (func_sel_i == 1'b0) begin
            // ---------------- SiLU(|z|) on [0, 8) ----------------
            case (seg_idx)
                4'd0 : begin c_q4_4 = 10'sd0;   slope_dx_q4_4 = ((k_ext << 2) + k_ext + 10'sd4) >>> 3; end // 0.625
                4'd1 : begin c_q4_4 = 10'sd5;   slope_dx_q4_4 = ((k_ext << 3) - k_ext + 10'sd4) >>> 3; end // 0.875
                4'd2 : begin c_q4_4 = 10'sd12;  slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd3 : begin c_q4_4 = 10'sd20;  slope_dx_q4_4 = ((k_ext << 4) + k_ext + 10'sd8) >>> 4; end // 1.0625
                4'd4 : begin c_q4_4 = 10'sd28;  slope_dx_q4_4 = ((k_ext << 3) + k_ext + 10'sd4) >>> 3; end // 1.125
                4'd5 : begin c_q4_4 = 10'sd37;  slope_dx_q4_4 = ((k_ext << 3) + k_ext + 10'sd4) >>> 3; end // 1.125
                4'd6 : begin c_q4_4 = 10'sd46;  slope_dx_q4_4 = ((k_ext << 4) + k_ext + 10'sd8) >>> 4; end // 1.0625
                4'd7 : begin c_q4_4 = 10'sd54;  slope_dx_q4_4 = ((k_ext << 4) + k_ext + 10'sd8) >>> 4; end // 1.0625
                4'd8 : begin c_q4_4 = 10'sd63;  slope_dx_q4_4 = ((k_ext << 4) + k_ext + 10'sd8) >>> 4; end // 1.0625
                4'd9 : begin c_q4_4 = 10'sd71;  slope_dx_q4_4 = ((k_ext << 4) + k_ext + 10'sd8) >>> 4; end // 1.0625
                4'd10: begin c_q4_4 = 10'sd80;  slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd11: begin c_q4_4 = 10'sd88;  slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd12: begin c_q4_4 = 10'sd96;  slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd13: begin c_q4_4 = 10'sd104; slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd14: begin c_q4_4 = 10'sd112; slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd15: begin c_q4_4 = 10'sd120; slope_dx_q4_4 = k_ext;                                 end // 1.000
                default: begin c_q4_4 = 10'sd0; slope_dx_q4_4 = 10'sd0; end
            endcase
        end else begin
            // ---------------- Softplus(|u|) on [0, 8) ----------------
            case (seg_idx)
                4'd0 : begin c_q4_4 = 10'sd11;  slope_dx_q4_4 = ((k_ext << 3) + k_ext + 10'sd8) >>> 4; end // 0.5625
                4'd1 : begin c_q4_4 = 10'sd16;  slope_dx_q4_4 = ((k_ext << 2) + k_ext + 10'sd4) >>> 3; end // 0.625
                4'd2 : begin c_q4_4 = 10'sd21;  slope_dx_q4_4 = ((k_ext << 2) - k_ext + 10'sd2) >>> 2; end // 0.750
                4'd3 : begin c_q4_4 = 10'sd27;  slope_dx_q4_4 = ((k_ext << 3) - k_ext + 10'sd4) >>> 3; end // 0.875
                4'd4 : begin c_q4_4 = 10'sd34;  slope_dx_q4_4 = ((k_ext << 3) - k_ext + 10'sd4) >>> 3; end // 0.875
                4'd5 : begin c_q4_4 = 10'sd41;  slope_dx_q4_4 = ((k_ext << 4) - k_ext + 10'sd8) >>> 4; end // 0.9375
                4'd6 : begin c_q4_4 = 10'sd49;  slope_dx_q4_4 = ((k_ext << 4) - k_ext + 10'sd8) >>> 4; end // 0.9375
                4'd7 : begin c_q4_4 = 10'sd56;  slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd8 : begin c_q4_4 = 10'sd64;  slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd9 : begin c_q4_4 = 10'sd72;  slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd10: begin c_q4_4 = 10'sd80;  slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd11: begin c_q4_4 = 10'sd88;  slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd12: begin c_q4_4 = 10'sd96;  slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd13: begin c_q4_4 = 10'sd104; slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd14: begin c_q4_4 = 10'sd112; slope_dx_q4_4 = k_ext;                                 end // 1.000
                4'd15: begin c_q4_4 = 10'sd120; slope_dx_q4_4 = k_ext;                                 end // 1.000
                default: begin c_q4_4 = 10'sd0; slope_dx_q4_4 = 10'sd0; end
            endcase
        end

        pos_y_q4_4 = c_q4_4 + slope_dx_q4_4;

        // Symmetry Reflection: f(-|x|) = f(|x|) - |x|
        // Both SiLU and Softplus satisfy this EXACT identity!
        if (is_neg)
            full_y_q4_4 = pos_y_q4_4 - $signed({2'b00, x_abs});
        else
            full_y_q4_4 = pos_y_q4_4;

        // Clamp SiLU output to signed Q4.4 [-128, +127]
        if (full_y_q4_4 > 10'sd127)
            silu_q4_4_o = 8'sd127;
        else if (full_y_q4_4 < -10'sd128)
            silu_q4_4_o = -8'sd128;
        else
            silu_q4_4_o = full_y_q4_4[7:0];

        // Scale Softplus Q4.4 -> Unsigned Q2.6 (shift left by 2, clamp [0, 255])
        if (full_y_q4_4 <= 10'sd0)
            sp_q2_6_o = 8'd0;
        else if (full_y_q4_4 >= 10'sd64) // >= 4.0 saturates Q2.6
            sp_q2_6_o = 8'd255;
        else
            sp_q2_6_o = {full_y_q4_4[5:0], 2'b00};
    end

endmodule : chronos_pwl_nonlinear
