// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/engine/chronos_ssm_core_1lane.sv
// Desc   : ★ Q1 CORE INNOVATION 3 — 3-Stage Pipelined 1-Lane Zero-DSP Log-ZOH
//          SSM & Fused Gate Engine (Engine B) ★
//
//          100% bandwidth-matched to True Dual-Port Bank 1 (1 state / cycle):
//            - Stage 1 (Log-ZOH & Triple-Product Log Input):
//                * LOD16(h[t-1]) - CSD_LogZOH(Δ, A)  -> log(A_bar * h[t-1])
//                * LOD(Δ) + LOD(B) + LOD(x)          -> log(Δ * B * x) [2 adds, 0 DSP!]
//            - Stage 2 (Dual Unbiased Antilog + State Update + Port B Writeback):
//                * 0.5-ULP Round-to-Nearest Antilog16 (eliminates drift bias!)
//                * h[t] = clamp_s16(A_bar*h[t-1] + B_bar*x) -> Bank 1 Port B (0-bubble)
//            - Stage 3 (Fused Output Projection y += C*h[t] & Sub-mode B1 Gate):
//                * Mode B0 (SSM_MODE_RECURR): y_acc += C_n * h[t] (log mul, 0 DSP)
//                                             on n_last: + D_d * x_d skip conn.
//                * Mode B1 (SSM_MODE_GATE)  : out_d = y_d * SiLU(z_d) (reuses log core!)
// ============================================================================

`timescale 1ns / 1ps

(* use_dsp = "no" *)
module chronos_ssm_core_1lane #(
    parameter int unsigned STATE_ADDR_W = 9
)(
    input  logic                    clk_i,
    input  logic                    rst_ni,
    input  logic                    cg_en_i,        // Dynamic ICG enable from Δ-Skip Ctrl
    input  logic                    test_en_i,

    // Control & Micro-Mode (2 lean sub-modes: 0=RECURR, 1=GATE)
    input  logic                    valid_i,
    input  logic                    submode_i,      // 0 = SSM_MODE_RECURR, 1 = SSM_MODE_GATE
    input  logic                    n_first_i,      // First state n=0 of channel d (clears y_acc)
    input  logic                    n_last_i,       // Last state n=N-1 of channel d (adds D*x)
    input  logic                    lyap_guard_i,   // Enforce spectral stability |A_bar| < 1
    input  logic [STATE_ADDR_W-1:0] state_addr_i,   // Target Bank 1 address for h[t] writeback

    // SSM Inputs for State (d, n)
    input  logic signed [15:0]      h_prev_i,       // h_{d,n}[t-1] INT16 (Q5.10) from Bank 1 Port A
    input  logic        [7:0]       delta_q2_6_i,   // Δ_d[t] unsigned Q2.6
    input  logic        [8:0]       a_log_q3_6_i,   // Pre-encoded log2(|A_{d,n}|*16) from Bank 3
    input  logic                    a_zero_i,
    input  logic signed [7:0]       b_s8_i,         // B_n[t] signed INT8 (Q1.6)
    input  logic signed [7:0]       c_s8_i,         // C_n[t] signed INT8 (Q1.6)
    input  logic signed [7:0]       x_s8_i,         // x_d[t] signed INT8 (Q4.4)
    input  logic signed [7:0]       d_skip_s8_i,    // D_d skip weight signed INT8 (Q4.4)
    input  logic signed [7:0]       z_s8_i,         // z_d[t] gate branch signed INT8 (Q4.4)

    // Bank 1 Port B Writeback Output (0-Bubble, 1 cycle after Stage 1)
    output logic                    h_we_o,
    output logic [STATE_ADDR_W-1:0] h_waddr_o,
    output logic signed [15:0]      h_new_o,

    // Fused Gated Channel Output out_d (INT8, ready when out_valid_o=1)
    output logic                    out_valid_o,
    output logic signed [7:0]       out_d_o,
    output logic signed [15:0]      y_debug_o       // Un-gated y_d Q5.10 for verification
);

    // ------------------------------------------------------------------------
    // Clock Gating Cell (driven by Dual-Granularity Δ-Skipping Controller)
    // ------------------------------------------------------------------------
    logic clk_gated;

    chronos_icg u_icg_eng_b (
        .clk_i     ( clk_i     ),
        .en_i      ( cg_en_i   ),
        .test_en_i ( test_en_i ),
        .clk_o     ( clk_gated )
    );

    // ========================================================================
    // STAGE 1 (Combinational -> Pipe Reg S1): Log-ZOH Decay + Triple-Log Input
    // ========================================================================
    // 1a. Encode h[t-1] (INT16) -> Log Q4.6
    logic       h_prev_sign;
    logic       h_prev_zero;
    logic [9:0] h_prev_log_q4_6;

    chronos_lod_s16 u_lod_h_prev (
        .data_i ( h_prev_i        ),
        .sign_o ( h_prev_sign     ),
        .zero_o ( h_prev_zero     ),
        .log_o  ( h_prev_log_q4_6 )
    );

    // 1b. CSD-Fused Log-ZOH Decay: |log2(exp(-Δ*|A|))| in Q4.6
    logic [9:0] decay_log2_q4_6;

    chronos_log_zoh_unit u_log_zoh (
        .delta_q2_6_i ( delta_q2_6_i    ),
        .a_log_q3_6_i ( a_log_q3_6_i    ),
        .a_zero_i     ( a_zero_i        ),
        .lyap_guard_i ( lyap_guard_i    ),
        .decay_log2_o ( decay_log2_q4_6 )
    );

    // Log-domain decay subtraction: log2(|A_bar * h[t-1]|) = log2(|h[t-1]|) - decay_log2
    logic signed [10:0] s1_h_decayed_log;
    assign s1_h_decayed_log = $signed({1'b0, h_prev_log_q4_6}) - $signed({1'b0, decay_log2_q4_6});

    // 1c. Triple-Product Input Injection in Log Domain: Δ_d * B_n * x_d (0 DSP!)
    //     Scale alignment to Q5.10 (state format):
    //       Δ (Q2.6 -> raw*2^-6) * B (Q1.6 -> raw*2^-6) * x (Q4.4 -> raw*2^-4)
    //       Product raw has total fractional shift 2^-16; target h is Q5.10 (2^-10)
    //       => Net exponent offset = -6 (i.e. -384 in Q5.6)!
    //     ★ v1.2.0 Area Optimization: Share the exact unsigned 8-bit Δ log2 encoder
    //        directly with u_log_zoh.delta_log_q3_6 (eliminates redundant u_lod_delta!)
    logic       b_sign, b_zero;
    logic [8:0] b_log_q3_6;
    logic       x_sign, x_zero;
    logic [8:0] x_log_q3_6;
    logic [8:0] delta_log_exact;

    assign delta_log_exact = u_log_zoh.delta_log_q3_6;

    chronos_lod_u8 u_lod_b (
        .data_i ( b_s8_i     ),
        .sign_o ( b_sign     ),
        .zero_o ( b_zero     ),
        .log_o  ( b_log_q3_6 )
    );

    chronos_lod_u8 u_lod_x (
        .data_i ( x_s8_i     ),
        .sign_o ( x_sign     ),
        .zero_o ( x_zero     ),
        .log_o  ( x_log_q3_6 )
    );

    logic               s1_bx_sign;
    logic               s1_bx_zero;
    logic signed [11:0] s1_bx_log_wide;
    logic signed [10:0] s1_bx_log;

    always_comb begin
        s1_bx_sign = b_sign ^ x_sign;
        s1_bx_zero = (delta_q2_6_i == 8'd0) | b_zero | x_zero;
        s1_bx_log_wide = $signed({3'b000, delta_log_exact})
                       + $signed({3'b000, b_log_q3_6})
                       + $signed({3'b000, x_log_q3_6})
                       - 12'sd384; // -6.0 in Q5.6 (aligns Q2.6*Q1.6*Q4.4 -> Q5.10)
        if (s1_bx_log_wide > 12'sd1023)
            s1_bx_log = 11'sd1023;
        else if (s1_bx_log_wide < -12'sd1024)
            s1_bx_log = -11'sd1024;
        else
            s1_bx_log = s1_bx_log_wide[10:0];
    end

    // 1d. Parallel PWL SiLU(z_d) Evaluation (ready for Gate stage, 0 extra cycles!)
    logic signed [7:0] s1_silu_z_q4_4;
    logic        [7:0] s1_sp_unused;

    chronos_pwl_nonlinear u_pwl_silu (
        .func_sel_i  ( 1'b0           ), // NL_FUNC_SILU
        .x_q4_4_i    ( z_s8_i         ),
        .silu_q4_4_o ( s1_silu_z_q4_4 ),
        .sp_q2_6_o   ( s1_sp_unused   )
    );

    // Suppress unused helper signals cleanly
    wire _unused_s1 = &{1'b0, s1_sp_unused};

    // -------------------- Pipeline Register S1 -> S2 ------------------------
    logic                    s1_valid_q;
    logic                    s1_submode_q;
    logic                    s1_n_first_q;
    logic                    s1_n_last_q;
    logic [STATE_ADDR_W-1:0] s1_addr_q;
    logic                    s1_h_sign_q;
    logic                    s1_h_zero_q;
    logic signed [10:0]      s1_h_dec_log_q;
    logic                    s1_bx_sign_q;
    logic                    s1_bx_zero_q;
    logic signed [10:0]      s1_bx_log_q;
    logic signed [7:0]       s1_c_q;
    logic signed [7:0]       s1_d_skip_q;
    logic                    s1_x_sign_q;
    logic                    s1_x_zero_q;
    logic        [8:0]       s1_x_log_q;
    logic signed [7:0]       s1_silu_z_q;

    always_ff @(posedge clk_gated or negedge rst_ni) begin
        if (!rst_ni) begin
            s1_valid_q     <= 1'b0;
            s1_submode_q   <= 1'b0;
            s1_n_first_q   <= 1'b0;
            s1_n_last_q    <= 1'b0;
            s1_addr_q      <= '0;
            s1_h_sign_q    <= 1'b0;
            s1_h_zero_q    <= 1'b1;
            s1_h_dec_log_q <= 11'sd0;
            s1_bx_sign_q   <= 1'b0;
            s1_bx_zero_q   <= 1'b1;
            s1_bx_log_q    <= 11'sd0;
            s1_c_q         <= 8'sd0;
            s1_d_skip_q    <= 8'sd0;
            s1_x_sign_q    <= 1'b0;
            s1_x_zero_q    <= 1'b1;
            s1_x_log_q     <= 9'd0;
            s1_silu_z_q    <= 8'sd0;
        end else begin
            s1_valid_q     <= valid_i;
            s1_submode_q   <= submode_i;
            s1_n_first_q   <= n_first_i;
            s1_n_last_q    <= n_last_i;
            s1_addr_q      <= state_addr_i;
            s1_h_sign_q    <= h_prev_sign;
            s1_h_zero_q    <= h_prev_zero;
            s1_h_dec_log_q <= s1_h_decayed_log;
            s1_bx_sign_q   <= s1_bx_sign;
            s1_bx_zero_q   <= s1_bx_zero;
            s1_bx_log_q    <= s1_bx_log;
            s1_c_q         <= c_s8_i;
            s1_d_skip_q    <= d_skip_s8_i;
            s1_x_sign_q    <= x_sign;
            s1_x_zero_q    <= x_zero;
            s1_x_log_q     <= x_log_q3_6;
            s1_silu_z_q    <= s1_silu_z_q4_4;
        end
    end

    // ========================================================================
    // STAGE 2 (Dual Unbiased Antilog + 1st-Order Delta-Sigma Error-Feedback
    //          -> State h[t] Update -> Bank 1 0-Bubble Port B Writeback)
    // ========================================================================
    // ★ Q1 UPGRADE (v1.1.0): Sub-LSB Fractional Remainder extraction &
    //   1st-Order Delta-Sigma (ΔΣ) Error-Feedback across recurrent steps t-1 -> t!
    //   Instead of losing sub-LSB fractional bits during right-shift in Stage 2,
    //   we extract the 4-bit sub-LSB remainder (in units of 1/16 LSB of Q5.10)
    //   from the decayed state branch and accumulate it with the 1st-order ΔΣ
    //   error residual register ds_err_q[addr], shaping low-frequency drift
    //   noise via H_e(z) = (1 - z^-1)/(1 - A_bar*z^-1) -> 0 at DC!
    logic signed [15:0] s2_h_decayed_lin;
    logic signed [15:0] s2_bx_lin;
    logic signed [16:0] s2_h_sum_17;
    logic signed [15:0] s2_h_new_clamped;

    // 1st-Order Delta-Sigma Sub-LSB Remainder Accumulator (8 state lanes n=0..7)
    logic signed [4:0]  s2_dec_sub_q4;
    logic signed [4:0]  s2_bx_sub_q4;
    logic signed [5:0]  ds_err_n_q [0:7];
    logic signed [6:0]  s2_frac_acc_q4;
    logic signed [1:0]  s2_ds_carry;
    logic signed [5:0]  s2_ds_rem_next;
    logic        [2:0]  s2_n_idx;

    assign s2_n_idx = s1_addr_q[2:0];

    chronos_log_antilog_s16 u_antilog_decay (
        .sign_i    ( s1_h_sign_q      ),
        .zero_i    ( s1_h_zero_q      ),
        .log_val_i ( s1_h_dec_log_q   ),
        .lin_o     ( s2_h_decayed_lin )
    );

    chronos_log_antilog_s16 u_antilog_bx (
        .sign_i    ( s1_bx_sign_q ),
        .zero_i    ( s1_bx_zero_q ),
        .log_val_i ( s1_bx_log_q  ),
        .lin_o     ( s2_bx_lin    )
    );

    // 1st-Order Delta-Sigma (ΔΣ) Sub-LSB Error-Feedback across recurrent steps t-1 -> t:
    // Extracts fine sub-LSB quantization residual when right-shifting small exponents
    // (exp_int < 8) and shapes low-frequency drift noise via H_e(z) = (1 - z^-1)/(1 - A_bar*z^-1).
    always_comb begin
        if (s1_h_zero_q || s1_h_dec_log_q[10] || (s1_h_dec_log_q[9:6] >= 4'd8))
            s2_dec_sub_q4 = 5'sd0;
        else
            s2_dec_sub_q4 = s1_h_sign_q ? -$signed({2'b00, s1_h_dec_log_q[2:0]}) + 5'sd3
                                        :  $signed({2'b00, s1_h_dec_log_q[2:0]}) - 5'sd3;

        if (s1_bx_zero_q || s1_bx_log_q[10] || (s1_bx_log_q[9:6] >= 4'd8))
            s2_bx_sub_q4 = 5'sd0;
        else
            s2_bx_sub_q4 = s1_bx_sign_q ? -$signed({2'b00, s1_bx_log_q[2:0]}) + 5'sd3
                                        :  $signed({2'b00, s1_bx_log_q[2:0]}) - 5'sd3;

        s2_frac_acc_q4 = {ds_err_n_q[s2_n_idx][5], ds_err_n_q[s2_n_idx]}
                       + {{2{s2_dec_sub_q4[4]}}, s2_dec_sub_q4}
                       + {{2{s2_bx_sub_q4[4]}},  s2_bx_sub_q4};

        if (s2_frac_acc_q4 >= 7'sd16) begin
            s2_ds_carry    = 2'sd1;
            s2_ds_rem_next = s2_frac_acc_q4[5:0] - 6'sd16;
        end else if (s2_frac_acc_q4 <= -7'sd16) begin
            s2_ds_carry    = -2'sd1;
            s2_ds_rem_next = s2_frac_acc_q4[5:0] + 6'sd16;
        end else begin
            s2_ds_carry    = 2'sd0;
            s2_ds_rem_next = s2_frac_acc_q4[5:0];
        end

        s2_h_sum_17 = {s2_h_decayed_lin[15], s2_h_decayed_lin}
                    + {s2_bx_lin[15],        s2_bx_lin}
                    + {{15{s2_ds_carry[1]}}, s2_ds_carry};
        if (s2_h_sum_17 > 17'sd32767)
            s2_h_new_clamped = 16'sd32767;
        else if (s2_h_sum_17 < -17'sd32767)
            s2_h_new_clamped = -16'sd32767;
        else
            s2_h_new_clamped = s2_h_sum_17[15:0];
    end

    always_ff @(posedge clk_gated or negedge rst_ni) begin
        if (!rst_ni) begin
            ds_err_n_q[0] <= 6'sd0; ds_err_n_q[1] <= 6'sd0;
            ds_err_n_q[2] <= 6'sd0; ds_err_n_q[3] <= 6'sd0;
            ds_err_n_q[4] <= 6'sd0; ds_err_n_q[5] <= 6'sd0;
            ds_err_n_q[6] <= 6'sd0; ds_err_n_q[7] <= 6'sd0;
        end else if (s1_valid_q && (s1_submode_q == 1'b0)) begin
            ds_err_n_q[s2_n_idx] <= s2_ds_rem_next;
        end
    end

    // Direct 0-Bubble Writeback to Bank 1 Port B (registered output aligns cleanly)
    assign h_we_o    = s1_valid_q && (s1_submode_q == 1'b0); // SSM_MODE_RECURR
    assign h_waddr_o = s1_addr_q;
    assign h_new_o   = s2_h_new_clamped;

    // -------------------- Pipeline Register S2 -> S3 ------------------------
    logic               s2_valid_q;
    logic               s2_submode_q;
    logic               s2_n_first_q;
    logic               s2_n_last_q;
    logic signed [15:0] s2_h_new_q;
    logic signed [7:0]  s2_c_q;
    logic signed [7:0]  s2_d_skip_q;
    logic               s2_x_sign_q;
    logic               s2_x_zero_q;
    logic        [8:0]  s2_x_log_q;
    logic signed [7:0]  s2_silu_z_q;

    always_ff @(posedge clk_gated or negedge rst_ni) begin
        if (!rst_ni) begin
            s2_valid_q   <= 1'b0;
            s2_submode_q <= 1'b0;
            s2_n_first_q <= 1'b0;
            s2_n_last_q  <= 1'b0;
            s2_h_new_q   <= 16'sd0;
            s2_c_q       <= 8'sd0;
            s2_d_skip_q  <= 8'sd0;
            s2_x_sign_q  <= 1'b0;
            s2_x_zero_q  <= 1'b1;
            s2_x_log_q   <= 9'd0;
            s2_silu_z_q  <= 8'sd0;
        end else begin
            s2_valid_q   <= s1_valid_q;
            s2_submode_q <= s1_submode_q;
            s2_n_first_q <= s1_n_first_q;
            s2_n_last_q  <= s1_n_last_q;
            s2_h_new_q   <= s2_h_new_clamped;
            s2_c_q       <= s1_c_q;
            s2_d_skip_q  <= s1_d_skip_q;
            s2_x_sign_q  <= s1_x_sign_q;
            s2_x_zero_q  <= s1_x_zero_q;
            s2_x_log_q   <= s1_x_log_q;
            s2_silu_z_q  <= s1_silu_z_q;
        end
    end

    // ========================================================================
    // STAGE 3 (Fused Output Projection y += C*h[t] & Micro-Mode B1 SiLU Gate)
    //   Reuses ONE shared 16-bit Log Multiplier for both C*h[t] and y*SiLU(z)!
    // ========================================================================
    logic signed [31:0] y_acc_q;
    logic signed [15:0] y_latched_q;

    // Mux inputs to Stage-3 shared Log Multiplier:
    //   Mode B0 (RECURR): A = h_new_q (Q5.10), B = c_q (Q1.6) -> shift -6 (-384) -> Q5.10
    //   Mode B1 (GATE)  : A = y_latched_q (Q5.10), B = silu_z_q (Q4.4) -> shift -10 (-640) -> Q4.4
    logic signed [15:0] s3_op_a;
    logic signed [7:0]  s3_op_b;
    logic signed [11:0] s3_exp_bias;

    always_comb begin
        if (s2_submode_q == 1'b0) begin
            s3_op_a     = s2_h_new_q;
            s3_op_b     = s2_c_q;
            s3_exp_bias = -12'sd384; // -6.0 in Q5.6 (Q5.10 * Q1.6 -> Q5.10)
        end else begin
            s3_op_a     = y_latched_q;
            s3_op_b     = s2_silu_z_q;
            s3_exp_bias = -12'sd640; // -10.0 in Q5.6 (Q5.10 * Q4.4 -> Q4.4)
        end
    end

    logic        s3_a_sign, s3_a_zero;
    logic [9:0]  s3_a_log_q4_6;
    logic        s3_b_sign, s3_b_zero;
    logic [8:0]  s3_b_log_q3_6;
    logic signed [11:0] s3_prod_log_wide;
    logic signed [10:0] s3_prod_log;
    logic signed [15:0] s3_prod_lin;

    chronos_lod_s16 u_s3_lod_a (
        .data_i ( s3_op_a       ),
        .sign_o ( s3_a_sign     ),
        .zero_o ( s3_a_zero     ),
        .log_o  ( s3_a_log_q4_6 )
    );

    chronos_lod_u8 u_s3_lod_b (
        .data_i ( s3_op_b       ),
        .sign_o ( s3_b_sign     ),
        .zero_o ( s3_b_zero     ),
        .log_o  ( s3_b_log_q3_6 )
    );

    always_comb begin
        s3_prod_log_wide = $signed({2'b00, s3_a_log_q4_6})
                         + $signed({3'b000, s3_b_log_q3_6})
                         + s3_exp_bias;
        if (s3_prod_log_wide > 12'sd1023)
            s3_prod_log = 11'sd1023;
        else if (s3_prod_log_wide < -12'sd1024)
            s3_prod_log = -11'sd1024;
        else
            s3_prod_log = s3_prod_log_wide[10:0];
    end

    chronos_log_antilog_s16 u_s3_antilog (
        .sign_i    ( s3_a_sign ^ s3_b_sign ),
        .zero_i    ( s3_a_zero | s3_b_zero ),
        .log_val_i ( s3_prod_log           ),
        .lin_o     ( s3_prod_lin           )
    );

    // Optional Skip Connection D_d * x_d (Zero-DSP Log Mul, Q4.4 * Q4.4 -> Q5.10 => shift +2 = +128)
    // ★ v1.2.0 Area Optimization: Reuses pipelined {s2_x_sign_q, s2_x_zero_q, s2_x_log_q}
    //   computed by u_lod_x in Stage 1 (eliminates redundant u_lod_x_skip in Stage 3!)
    logic        d_sk_sign, d_sk_zero;
    logic [8:0]  d_sk_log;
    logic signed [10:0] dx_log;
    logic signed [15:0] dx_lin_q5_10;

    chronos_lod_u8 u_lod_d_skip (
        .data_i ( s2_d_skip_q ), .sign_o ( d_sk_sign ), .zero_o ( d_sk_zero ), .log_o ( d_sk_log )
    );
    assign dx_log = $signed({2'b00, d_sk_log}) + $signed({2'b00, s2_x_log_q}) + 11'sd128; // +2.0 (Q4.4*Q4.4=Q8->Q10)

    chronos_log_antilog_s16 u_antilog_dx (
        .sign_i    ( d_sk_sign ^ s2_x_sign_q ),
        .zero_i    ( d_sk_zero | s2_x_zero_q ),
        .log_val_i ( dx_log                  ),
        .lin_o     ( dx_lin_q5_10            )
    );

    // Accumulate y_d across n = 0..N-1 and Latch Gated Output
    logic signed [31:0] y_next_acc;

    always_comb begin
        if (s2_n_first_q)
            y_next_acc = {{16{s3_prod_lin[15]}}, s3_prod_lin};
        else
            y_next_acc = y_acc_q + {{16{s3_prod_lin[15]}}, s3_prod_lin};

        if (s2_n_last_q)
            y_next_acc = y_next_acc + {{16{dx_lin_q5_10[15]}}, dx_lin_q5_10};
    end

    always_ff @(posedge clk_gated or negedge rst_ni) begin
        if (!rst_ni) begin
            y_acc_q     <= 32'sd0;
            y_latched_q <= 16'sd0;
            out_valid_o <= 1'b0;
            out_d_o     <= 8'sd0;
        end else begin
            out_valid_o <= 1'b0;

            if (s2_valid_q && (s2_submode_q == 1'b0)) begin
                // Mode B0: Accumulate C_n * h[t] (+ D_d * x_d on last state)
                y_acc_q <= y_next_acc;
                if (s2_n_last_q) begin
                    if (y_next_acc > 32'sd32767)
                        y_latched_q <= 16'sd32767;
                    else if (y_next_acc < -32'sd32767)
                        y_latched_q <= -16'sd32767;
                    else
                        y_latched_q <= y_next_acc[15:0];
                end
            end else if (s2_valid_q && (s2_submode_q == 1'b1)) begin
                // Mode B1: Output gated result out_d = clamp_s8(y_d * SiLU(z_d))
                out_valid_o <= 1'b1;
                if (s3_prod_lin > 16'sd127)
                    out_d_o <= 8'sd127;
                else if (s3_prod_lin < -16'sd128)
                    out_d_o <= -8'sd128;
                else
                    out_d_o <= s3_prod_lin[7:0];
            end
        end
    end

    assign y_debug_o = y_latched_q;

endmodule : chronos_ssm_core_1lane
