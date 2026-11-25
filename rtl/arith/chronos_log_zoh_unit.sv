// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/arith/chronos_log_zoh_unit.sv
// Desc   : ★ Q1 CORE INNOVATION 1 — Log-Constant-Fused Zero-DSP Log-ZOH Unit ★
//          Computes decay_log2 = |log2(A_bar)| = Δ * |A| * log2(e) without DSPs,
//          without exp() LUTs, and WITHOUT a post-Antilog CSD adder tree!
//
//          Key Mathematical & Critical-Path Breakthrough (v1.1.0):
//            Instead of computing p = Antilog(log2(Δ) + log2(|A|)) and then
//            passing p through a slow 5-term 26-bit CSD shift-add tree to
//            multiply by log2(e) = 1.442695, we fuse log2(log2(e)) directly
//            into the log-domain exponent sum BEFORE the Antilog decoder:
//              log2(decay_log2) = log2(Δ) + log2(|A|) + log2(log2(e))
//            In Q5.6 fixed-point (scale = 64):
//              round(log2(log2(e)) * 64) = round(0.528766 * 64) = +34
//            Combined with the Q-format alignment offset (-256), the net
//            compile-time constant offset is -256 + 34 = -222 (or -256 if
//            +34 is pre-fused offline into A_log in Bank 3).
//            This eliminates the entire 5-term 26-bit CSD adder tree from
//            Stage 1's critical path!
// ============================================================================

`timescale 1ns / 1ps

(* use_dsp = "no" *)
module chronos_log_zoh_unit #(
    parameter bit A_LOG_PREFUSED = 1'b0 // 0: fuse +34 in RTL (-222); 1: +34 pre-added in Bank 3 A_log (-256)
)(
    input  logic        [7:0] delta_q2_6_i,   // Unsigned Q2.6 (0..3.984)
    input  logic        [8:0] a_log_q3_6_i,   // Pre-encoded Q3.6 log2(|A|*16)
    input  logic              a_zero_i,       // 1 if A == 0
    input  logic              lyap_guard_i,   // 1 = enforce Lyapunov spectral stability
    output logic        [9:0] decay_log2_o    // Unsigned Q4.6 |log2(exp(-Δ*|A|))|
);

    localparam logic signed [10:0] LOG2_LOG2E_Q5_6 = 11'sd34; // round(log2(log2(e)) * 64)
    localparam logic signed [10:0] ZOH_EXP_OFFSET  = A_LOG_PREFUSED ? -11'sd256
                                                                    : (-11'sd256 + LOG2_LOG2E_Q5_6); // -222

    // Step 1: Encode positive unsigned Δ_d[t] into Q3.6 log2(delta_raw)
    logic       delta_zero;
    logic [2:0] d_exp;
    logic [5:0] d_raw_mant;
    logic [5:0] d_lut_frac;
    logic [8:0] delta_log_q3_6;

    assign delta_zero = (delta_q2_6_i == 8'd0);

    always_comb begin
        d_exp      = 3'd0;
        d_raw_mant = 6'd0;
        if      (delta_q2_6_i[7]) begin d_exp = 3'd7; d_raw_mant = delta_q2_6_i[6:1];        end
        else if (delta_q2_6_i[6]) begin d_exp = 3'd6; d_raw_mant = delta_q2_6_i[5:0];        end
        else if (delta_q2_6_i[5]) begin d_exp = 3'd5; d_raw_mant = {delta_q2_6_i[4:0], 1'b0}; end
        else if (delta_q2_6_i[4]) begin d_exp = 3'd4; d_raw_mant = {delta_q2_6_i[3:0], 2'b0}; end
        else if (delta_q2_6_i[3]) begin d_exp = 3'd3; d_raw_mant = {delta_q2_6_i[2:0], 3'b0}; end
        else if (delta_q2_6_i[2]) begin d_exp = 3'd2; d_raw_mant = {delta_q2_6_i[1:0], 4'b0}; end
        else if (delta_q2_6_i[1]) begin d_exp = 3'd1; d_raw_mant = {delta_q2_6_i[0],   5'b0}; end
        else                      begin d_exp = 3'd0; d_raw_mant = 6'd0;                     end
    end

    // Curvature LUT6 (identical exact log2 table)
    always_comb begin
        case (d_raw_mant)
            6'd0 : d_lut_frac = 6'd0;   6'd1 : d_lut_frac = 6'd1;
            6'd2 : d_lut_frac = 6'd3;   6'd3 : d_lut_frac = 6'd4;
            6'd4 : d_lut_frac = 6'd6;   6'd5 : d_lut_frac = 6'd7;
            6'd6 : d_lut_frac = 6'd8;   6'd7 : d_lut_frac = 6'd10;
            6'd8 : d_lut_frac = 6'd11;  6'd9 : d_lut_frac = 6'd12;
            6'd10: d_lut_frac = 6'd13;  6'd11: d_lut_frac = 6'd15;
            6'd12: d_lut_frac = 6'd16;  6'd13: d_lut_frac = 6'd17;
            6'd14: d_lut_frac = 6'd18;  6'd15: d_lut_frac = 6'd19;
            6'd16: d_lut_frac = 6'd21;  6'd17: d_lut_frac = 6'd22;
            6'd18: d_lut_frac = 6'd23;  6'd19: d_lut_frac = 6'd24;
            6'd20: d_lut_frac = 6'd25;  6'd21: d_lut_frac = 6'd26;
            6'd22: d_lut_frac = 6'd27;  6'd23: d_lut_frac = 6'd28;
            6'd24: d_lut_frac = 6'd29;  6'd25: d_lut_frac = 6'd30;
            6'd26: d_lut_frac = 6'd31;  6'd27: d_lut_frac = 6'd32;
            6'd28: d_lut_frac = 6'd34;  6'd29: d_lut_frac = 6'd35;
            6'd30: d_lut_frac = 6'd35;  6'd31: d_lut_frac = 6'd36;
            6'd32: d_lut_frac = 6'd37;  6'd33: d_lut_frac = 6'd38;
            6'd34: d_lut_frac = 6'd39;  6'd35: d_lut_frac = 6'd40;
            6'd36: d_lut_frac = 6'd41;  6'd37: d_lut_frac = 6'd42;
            6'd38: d_lut_frac = 6'd43;  6'd39: d_lut_frac = 6'd44;
            6'd40: d_lut_frac = 6'd45;  6'd41: d_lut_frac = 6'd46;
            6'd42: d_lut_frac = 6'd47;  6'd43: d_lut_frac = 6'd47;
            6'd44: d_lut_frac = 6'd48;  6'd45: d_lut_frac = 6'd49;
            6'd46: d_lut_frac = 6'd50;  6'd47: d_lut_frac = 6'd51;
            6'd48: d_lut_frac = 6'd52;  6'd49: d_lut_frac = 6'd52;
            6'd50: d_lut_frac = 6'd53;  6'd51: d_lut_frac = 6'd54;
            6'd52: d_lut_frac = 6'd55;  6'd53: d_lut_frac = 6'd56;
            6'd54: d_lut_frac = 6'd56;  6'd55: d_lut_frac = 6'd57;
            6'd56: d_lut_frac = 6'd58;  6'd57: d_lut_frac = 6'd59;
            6'd58: d_lut_frac = 6'd60;  6'd59: d_lut_frac = 6'd60;
            6'd60: d_lut_frac = 6'd61;  6'd61: d_lut_frac = 6'd62;
            6'd62: d_lut_frac = 6'd63;  6'd63: d_lut_frac = 6'd63;
            default: d_lut_frac = 6'd0;
        endcase
    end

    assign delta_log_q3_6 = delta_zero ? 9'd0 : {d_exp, d_lut_frac};

    // Step 2: Direct Log-Fused Product Δ * |A| * log2(e) in Q4.6!
    //         Single 11-bit addition + 1 Antilog16 look-up (0 CSD adder tree!)
    logic signed [10:0] log_fused_q5_6;
    logic signed [15:0] decay_lin_q4_6;

    assign log_fused_q5_6 = $signed({2'b00, delta_log_q3_6})
                          + $signed({2'b00, a_log_q3_6_i})
                          + ZOH_EXP_OFFSET;

    chronos_log_antilog_s16 u_da_antilog (
        .sign_i    ( 1'b0                  ),
        .zero_i    ( delta_zero | a_zero_i ),
        .log_val_i ( log_fused_q5_6        ),
        .lin_o     ( decay_lin_q4_6        )
    );

    wire _unused_zoh = &{1'b0, decay_lin_q4_6[15]};

    // Step 3: Lyapunov Spectral Stability Guard (|A_bar| <= 2^(-1/64) < 1)
    always_comb begin
        if (delta_zero || a_zero_i) begin
            decay_log2_o = lyap_guard_i ? chronos_pkg::MIN_LYAP_DECAY : 10'd0;
        end else if (decay_lin_q4_6[14:0] > 15'd1023) begin
            decay_log2_o = 10'd1023; // Max decay (>= 16 bits shift -> exact zero)
        end else if (lyap_guard_i && (decay_lin_q4_6[9:0] < chronos_pkg::MIN_LYAP_DECAY)) begin
            decay_log2_o = chronos_pkg::MIN_LYAP_DECAY;
        end else begin
            decay_log2_o = decay_lin_q4_6[9:0];
        end
    end

endmodule : chronos_log_zoh_unit
