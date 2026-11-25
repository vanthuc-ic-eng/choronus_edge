// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/arith/chronos_log_zoh_unit.sv
// Desc   : ★ Q1 CORE INNOVATION 1 — Zero-DSP CSD-Fused Log-ZOH Discretizer ★
//          Computes log2(A_bar) = -|Delta * A| * log2(e) entirely without DSPs
//          or hardware exp() units!
//
//          Inputs:
//            - delta_q2_6_i   : Unsigned 8-bit Q2.6 time-step Δ_d[t] (value = raw/64)
//            - a_log_q3_6_i   : Unsigned 9-bit Q3.6 log2(|A_{d,n}| * 16) (pre-encoded)
//                               where |A_{d,n}| is Q4.4 -> raw_A = |A|*16,
//                               so log2(raw_delta) + log2(raw_A) = log2(Δ*|A|*1024)
//            - lyap_guard_i   : Enforce spectral stability |A_bar| <= 2^(-1/64) < 1
//          Output:
//            - decay_log2_o   : Unsigned 10-bit Q4.6 representing |log2(A_bar)| >= 0
//                               Ready to be SUBTRACTED directly from log2(|h[t-1]|)!
//
//          CSD Constant Approximation:
//            log2(e) = 1.44269504...
//            CSD     = 1 + 2^-1 - 2^-4 + 2^-8 + 2^-10 = 1.44238281 (err < 0.022%!)
// ============================================================================

`timescale 1ns / 1ps

(* use_dsp = "no" *)
module chronos_log_zoh_unit (
    input  logic        [7:0] delta_q2_6_i,   // Unsigned Q2.6 (0..3.984)
    input  logic        [8:0] a_log_q3_6_i,   // Pre-encoded Q3.6 log2(|A|*16)
    input  logic              a_zero_i,       // 1 if A == 0
    input  logic              lyap_guard_i,   // 1 = enforce Lyapunov spectral stability
    output logic        [9:0] decay_log2_o    // Unsigned Q4.6 |log2(exp(-Δ*|A|))|
);

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

    // Step 2: Log-domain product Δ * |A| in Q4.6 (offset = -4 because raw_delta*raw_A has scale 2^10,
    // and we want linear result in Q4.6 which has scale 2^6 -> shift right by 4 in exponent!)
    logic signed [10:0] log_prod_q5_6;
    logic signed [15:0] da_lin_q4_6;

    assign log_prod_q5_6 = $signed({2'b00, delta_log_q3_6})
                         + $signed({2'b00, a_log_q3_6_i})
                         - 11'sd256; // -4.0 in Q5.6 (4 * 64 = 256)

    chronos_log_antilog_s16 u_da_antilog (
        .sign_i    ( 1'b0                  ),
        .zero_i    ( delta_zero | a_zero_i ),
        .log_val_i ( log_prod_q5_6         ),
        .lin_o     ( da_lin_q4_6           )
    );

    // Step 3: Exact 5-Term CSD Shift-Add Multiplication by log2(e) = 1.4423828
    //   log2(e) ≈ 1 + 2^-1 - 2^-4 + 2^-8 + 2^-10  (0 DSP, err < 0.022%!)
    //   Compute in extended precision (10 fractional guard bits) then round!
    logic [25:0] x_ext;
    logic [25:0] csd_scaled;
    logic [25:0] csd_rounded_wide;
    logic [15:0] decay_rounded;

    wire _unused_zoh = &{1'b0, da_lin_q4_6[15], csd_rounded_wide[9:0]};

    always_comb begin
        x_ext = {1'b0, da_lin_q4_6[14:0], 10'd0};
        // 1*x + (x>>1) - (x>>4) + (x>>8) + (x>>10)
        csd_scaled = x_ext
                   + (x_ext >> 1)
                   - (x_ext >> 4)
                   + (x_ext >> 8)
                   + (x_ext >> 10);

        // Round-to-nearest back to Q4.6
        csd_rounded_wide = csd_scaled + 26'd512;
        decay_rounded    = csd_rounded_wide[25:10];

        if (delta_zero || a_zero_i) begin
            decay_log2_o = lyap_guard_i ? chronos_pkg::MIN_LYAP_DECAY : 10'd0;
        end else if (decay_rounded > 16'd1023) begin
            decay_log2_o = 10'd1023; // Max decay (>= 16 bits shift -> exact zero)
        end else if (lyap_guard_i && (decay_rounded[9:0] < chronos_pkg::MIN_LYAP_DECAY)) begin
            // Hardware Lyapunov Spectral Stability Clamp: guarantees |A_bar| < 1.0
            decay_log2_o = chronos_pkg::MIN_LYAP_DECAY;
        end else begin
            decay_log2_o = decay_rounded[9:0];
        end
    end

endmodule : chronos_log_zoh_unit
