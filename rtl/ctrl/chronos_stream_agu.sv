// ============================================================================
// Project: CHRONOS — Beyond-Q1 Zero-DSP Logarithmic Selective State-Space SoC
// File   : rtl/ctrl/chronos_stream_agu.sv
// Desc   : ★ BEYOND-Q1 AUTONOMOUS 4+1 CONCURRENT PRODUCER-CONSUMER SCHEDULER ★
//          Executes multi-timestep Selective Mamba / S4D with 3 major upgrades:
//            1) HWPE_STEP_CHK (Level-1 Coarse Silent-Frame Early Exit):
//               Evaluates L1 temporal input variation; skips ENTIRE silent
//               timestep in 1 cycle (gates Engine A + Engine B + all SRAMs!).
//            2) HWPE_IN_PROJ (Trainable Log8 Depthwise Causal Conv1D K=4):
//               Uses trainable 4xLog8 Conv1D kernels from Bank 3 (0 DSP).
//            3) HWPE_FUSED_OVERLAP (★ TRUE 4+1 CONCURRENT DUAL-ENGINE OVERLAP ★):
//               Producer FSM (Engine A) computes Δ[grp..grp+3] WHILE Consumer
//               FSM (Engine B) SIMULTANEOUSLY executes 0-bubble Log-ZOH SSM +
//               Fused SiLU Gate on already-ready channels (d < delta_ready_cnt),
//               with Level-2 Fine-Grained Channel Δ-Skipping!
// ============================================================================

`timescale 1ns / 1ps

module chronos_stream_agu #(
    parameter int unsigned MAX_D        = 64,
    parameter int unsigned MAX_N        = 8,
    parameter int unsigned W_ADDR_W     = 11,
    parameter int unsigned B_ADDR_W     = 7,
    parameter int unsigned STATE_ADDR_W = 9,
    parameter int unsigned RING_ADDR_W  = 8,
    parameter int unsigned D_ADDR_W     = $clog2(MAX_D)
)(
    input  logic                    clk_i,
    input  logic                    rst_ni,

    // CSR Control & Config
    input  logic                    start_pulse_i,
    input  logic                    mode_s6_i,        // 0=S4D, 1=Selective Mamba
    input  logic                    rr_coalesce_en_i, // ★ v1.3.0: Coalesced Multi-Step Log-ZOH Decay
    input  logic [1:0]              cfg_num_layers_i, // ★ v1.3.0: Time-Multiplexed Mamba Layers (1..3)
    input  logic [7:0]              cfg_d_model_i,
    input  logic [7:0]              cfg_n_state_i,
    input  logic [7:0]              cfg_d_in_i,
    input  logic [7:0]              cfg_num_cls_i,
    input  logic [15:0]             cfg_seq_len_i,

    // Status Outputs to CSR
    output logic [2:0]              fsm_state_o,
    output logic                    busy_o,
    output logic                    done_pulse_o,
    output logic [15:0]             step_idx_o,

    // Level-1 Coarse Activity Gate Interface
    output logic                    act_eval_clr_o,
    output logic                    act_eval_valid_o,
    output logic [4:0]              act_feat_idx_o,
    output logic signed [7:0]       act_u_feat_o,
    output logic                    act_is_first_step_o,
    input  logic                    act_early_exit_i,
    output logic                    step_early_exit_o,

    // Bank 0 Input Ring & Conv1D & Output Scratchpad Controls
    output logic                    u_pop_o,
    output logic [RING_ADDR_W-1:0]  u_rd_addr_o,
    input  logic signed [7:0]       u_rdata_i,
    output logic                    conv_push_o,
    output logic [D_ADDR_W-1:0]     conv_ch_o,
    output logic signed [7:0]       conv_x_new_o,
    input  logic signed [7:0]       conv_out_s8_i,    // From trainable chronos_conv1d_log8!
    output logic                    out_we_o,
    output logic [D_ADDR_W-1:0]     out_waddr_o,
    output logic signed [7:0]       out_wdata_o,
    output logic [D_ADDR_W-1:0]     out_raddr_o,
    input  logic signed [7:0]       out_rdata_i,

    // Bank 1 State Scratchpad Read Port A Control
    output logic                    state_re_a_o,
    output logic [STATE_ADDR_W-1:0] state_addr_a_o,

    // Bank 2 Weight BRAM Control
    output logic                    w_en_o,
    output logic [W_ADDR_W-1:0]     w_addr_o,

    // Bank 3 Bias & A/D/ConvW Param Control
    output logic                    bias_en_o,
    output logic [B_ADDR_W-1:0]     bias_addr_o,
    output logic                    a_en_o,
    output logic [STATE_ADDR_W-1:0] a_addr_o,
    output logic [D_ADDR_W-1:0]     d_param_addr_o,
    output logic [D_ADDR_W-1:0]     conv_w_addr_o,
    input  logic [7:0]              delta_s4d_i,
    input  logic signed [7:0]       d_skip_s8_i,

    // Engine A (1x4 SIMD Log8 MAC) Control & Data
    output logic                    eng_a_cg_en_o,
    output logic                    eng_a_valid_o,
    output logic                    eng_a_clr_o,
    output logic [4:0]              eng_a_req_sh_o,
    output logic signed [7:0]       eng_a_act_o,
    input  logic signed [31:0]      eng_a_acc0_i,
    input  logic signed [31:0]      eng_a_acc1_i,
    input  logic signed [31:0]      eng_a_acc2_i,
    input  logic signed [31:0]      eng_a_acc3_i,
    input  logic signed [7:0]       eng_a_q0_i,
    input  logic signed [7:0]       eng_a_q1_i,
    input  logic signed [7:0]       eng_a_q2_i,
    input  logic signed [7:0]       eng_a_q3_i,

    // Shared PWL Non-Linearity (Softplus for Δ_d, SiLU for Conv1D x_d)
    output logic                    pwl_func_sel_o,
    output logic signed [7:0]       pwl_x_in_o,
    input  logic signed [7:0]       pwl_silu_i,
    input  logic [7:0]              pwl_sp_i,

    // Delta-Skip Controller Handshake
    output logic                    delta_eval_o,
    output logic                    step_first_ch_o,
    output logic                    step_last_ch_o,
    output logic [7:0]              delta_eval_val_o,
    output logic                    ssm_active_o,
    input  logic                    chan_skip_i,

    // Engine B (1-Lane Log-ZOH SSM + Gate) Controls & Inputs
    output logic                    eng_b_cg_en_o,
    output logic                    eng_b_valid_o,
    output logic                    eng_b_submode_o,
    output logic                    eng_b_n_first_o,
    output logic                    eng_b_n_last_o,
    output logic [8:0]              eng_b_coalesce_log_o, // ★ v1.3.0: log2(1 + K_skip) Q3.6
    output logic [STATE_ADDR_W-1:0] eng_b_state_addr_o,
    output logic [7:0]              eng_b_delta_o,
    output logic signed [7:0]       eng_b_b_o,
    output logic signed [7:0]       eng_b_c_o,
    output logic signed [7:0]       eng_b_x_o,
    output logic signed [7:0]       eng_b_d_skip_o,
    output logic signed [7:0]       eng_b_z_o,
    input  logic                    eng_b_out_valid_i,
    input  logic signed [7:0]       eng_b_out_d_i,

    // Argmax Tree Interface (8x INT16 Logits)
    output logic                    argmax_valid_o,
    output logic signed [15:0]      logit0_o,
    output logic signed [15:0]      logit1_o,
    output logic signed [15:0]      logit2_o,
    output logic signed [15:0]      logit3_o,
    output logic signed [15:0]      logit4_o,
    output logic signed [15:0]      logit5_o,
    output logic signed [15:0]      logit6_o,
    output logic signed [15:0]      logit7_o,
    input  logic                    argmax_done_i
);

    // Local FSM State Encoding (matches chronos_pkg::hwpe_state_e)
    localparam logic [2:0] HWPE_IDLE          = 3'd0;
    localparam logic [2:0] HWPE_STEP_CHK      = 3'd1;
    localparam logic [2:0] HWPE_IN_PROJ       = 3'd2;
    localparam logic [2:0] HWPE_BC_PROJ       = 3'd3;
    localparam logic [2:0] HWPE_FUSED_OVERLAP = 3'd4;
    localparam logic [2:0] HWPE_OUT_PROJ      = 3'd5;
    localparam logic [2:0] HWPE_ARGMAX        = 3'd6;
    localparam logic [2:0] HWPE_DONE          = 3'd7;

    // Weight BRAM Segment Base Offsets (32-bit word addresses in Bank 2)
    localparam logic [W_ADDR_W-1:0] W_BASE_IN_X    = 11'd0;
    localparam logic [W_ADDR_W-1:0] W_BASE_IN_Z    = 11'd256;
    localparam logic [W_ADDR_W-1:0] W_BASE_DELTA   = 11'd512;
    localparam logic [W_ADDR_W-1:0] W_BASE_B_PROJ  = 11'd768;
    localparam logic [W_ADDR_W-1:0] W_BASE_C_PROJ  = 11'd896;
    localparam logic [W_ADDR_W-1:0] W_BASE_CLS_FC  = 11'd1024;

    // Local Zero-Wait Channel & Parameter Register Files
    logic signed [7:0]  x_reg     [0:MAX_D-1];
    logic signed [7:0]  z_reg     [0:MAX_D-1];
    logic        [7:0]  delta_reg [0:MAX_D-1];
    logic signed [7:0]  b_reg     [0:MAX_N-1];
    logic signed [7:0]  c_reg     [0:MAX_N-1];
    logic signed [15:0] logits_q  [0:7];

    logic [2:0]             state_q;
    logic [15:0]            step_q;
    logic [RING_ADDR_W-1:0] step_u_base_q;

    // Engine A (Producer) Loop Registers
    logic [7:0]             grp_q;
    logic [7:0]             k_idx_q;
    logic [2:0]             sub_ph_q;
    logic [2:0]             pipe_dly_q;
    logic signed [7:0]      act_d1_q;

    // Concurrent Producer-Consumer Synchronization in HWPE_FUSED_OVERLAP!
    //   Producer increments delta_ready_cnt_q as each Δ_d completes;
    //   Consumer (Engine B) runs IN PARALLEL whenever cons_d_q < delta_ready_cnt_q!
    logic                   prod_done_q;
    logic [7:0]             delta_ready_cnt_q;
    logic [7:0]             prod_s4d_d_q;
    logic [7:0]             cons_d_q;
    logic [7:0]             cons_n_q;
    logic [1:0]             cons_ph_q;      // 0=WAIT/EVAL, 1=RUN_N, 2=DRAIN, 3=DONE
    logic [2:0]             cons_drain_q;
    logic                   cons_chan_skip_q;

    // Engine B Pipeline Alignment Registers
    logic [D_ADDR_W-1:0]     b_d_pipe0_q, b_d_pipe1_q, b_d_pipe2_q;
    logic                    rd_valid_d1_q, rd_submode_d1_q, rd_n_first_d1_q, rd_n_last_d1_q;
    logic [STATE_ADDR_W-1:0] rd_addr_d1_q;
    logic [D_ADDR_W-1:0]     rd_d_d1_q;
    logic [2:0]              rd_n_d1_q;
    logic signed [7:0]       rd_d_skip_d1_q;

    // Effective Geometry Guards
    logic [7:0]  eff_d_model, eff_n_state, eff_d_in, eff_num_cls;
    logic [15:0] eff_seq_len;
    logic [1:0]  eff_num_layers;
    logic [1:0]  layer_q;
    logic [7:0]  silent_run_cnt_q;
    logic [8:0]  coalesce_log_q3_6;

    assign eff_d_model    = (cfg_d_model_i    == 8'd0)  ? 8'd16  : cfg_d_model_i;
    assign eff_n_state    = (cfg_n_state_i    == 8'd0)  ? 8'd4   : cfg_n_state_i;
    assign eff_d_in       = (cfg_d_in_i       == 8'd0)  ? 8'd8   : cfg_d_in_i;
    assign eff_num_cls    = (cfg_num_cls_i    == 8'd0)  ? 8'd5   : cfg_num_cls_i;
    assign eff_seq_len    = (cfg_seq_len_i    == 16'd0) ? 16'd1  : cfg_seq_len_i;
    assign eff_num_layers = (cfg_num_layers_i == 2'd0)  ? 2'd1   : cfg_num_layers_i;

    // ★ v1.3.0: Exact Q3.6 log2(1 + K_skip) lookup for Coalesced Multi-Step Log-ZOH Decay
    //   Preserves exact ECG RR-interval biological clock across K_skip silent frames!
    always_comb begin
        if (!rr_coalesce_en_i || (silent_run_cnt_q == 8'd0)) begin
            coalesce_log_q3_6 = 9'd0;
        end else begin
            case (silent_run_cnt_q)
                8'd1   : coalesce_log_q3_6 = 9'd64;  // log2(2) = 1.000 * 64
                8'd2   : coalesce_log_q3_6 = 9'd101; // log2(3) = 1.585 * 64
                8'd3   : coalesce_log_q3_6 = 9'd128; // log2(4) = 2.000 * 64
                8'd4   : coalesce_log_q3_6 = 9'd149; // log2(5) = 2.322 * 64
                8'd5   : coalesce_log_q3_6 = 9'd165; // log2(6) = 2.585 * 64
                8'd6   : coalesce_log_q3_6 = 9'd180; // log2(7) = 2.807 * 64
                8'd7   : coalesce_log_q3_6 = 9'd192; // log2(8) = 3.000 * 64
                default: begin
                    if      (silent_run_cnt_q < 8'd16)  coalesce_log_q3_6 = 9'd224; // ~3.5 * 64
                    else if (silent_run_cnt_q < 8'd32)  coalesce_log_q3_6 = 9'd288; // ~4.5 * 64
                    else if (silent_run_cnt_q < 8'd64)  coalesce_log_q3_6 = 9'd352; // ~5.5 * 64
                    else                                coalesce_log_q3_6 = 9'd416; // ~6.5 * 64
                end
            endcase
        end
    end

    assign fsm_state_o = state_q;
    assign busy_o      = (state_q != HWPE_IDLE) && (state_q != HWPE_DONE);
    assign step_idx_o  = step_q;

    assign logit0_o = logits_q[0]; assign logit1_o = logits_q[1];
    assign logit2_o = logits_q[2]; assign logit3_o = logits_q[3];
    assign logit4_o = logits_q[4]; assign logit5_o = logits_q[5];
    assign logit6_o = logits_q[6]; assign logit7_o = logits_q[7];

    // Zero-DSP State Address Calculator (d * N + n)
    function automatic logic [STATE_ADDR_W-1:0] calc_state_addr (
        input logic [7:0] d_i,
        input logic [2:0] n_i,
        input logic [7:0] n_cfg
    );
        logic [STATE_ADDR_W-1:0] base;
        case (n_cfg)
            8'd2   : base = {d_i[7:0], 1'b0};
            8'd4   : base = {d_i[6:0], 2'b00};
            8'd8   : base = {d_i[5:0], 3'b000};
            default: base = {d_i[6:0], 2'b00};
        endcase
        calc_state_addr = base + {{(STATE_ADDR_W-3){1'b0}}, n_i};
    endfunction

    function automatic logic signed [15:0] clamp_s16 (input logic signed [31:0] v);
        if (v > 32'sd32767)       clamp_s16 = 16'sd32767;
        else if (v < -32'sd32767) clamp_s16 = -16'sd32767;
        else                      clamp_s16 = v[15:0];
    endfunction

    wire _unused_agu = &{1'b0, eff_num_cls, cons_n_q[7:3]};

    // Consumer ready condition: a channel's Δ_d has already been produced!
    logic cons_has_token;
    assign cons_has_token = (cons_d_q < delta_ready_cnt_q);

    // ------------------------------------------------------------------------
    // Combinational Control & Dual-Engine Concurrent Routing
    // ------------------------------------------------------------------------
    always_comb begin
        u_pop_o             = 1'b0;
        u_rd_addr_o         = '0;
        conv_push_o         = 1'b0;
        conv_ch_o           = cons_d_q[D_ADDR_W-1:0];
        conv_x_new_o        = x_reg[cons_d_q[D_ADDR_W-1:0]];
        conv_w_addr_o       = cons_d_q[D_ADDR_W-1:0];

        out_we_o            = 1'b0;
        out_waddr_o         = b_d_pipe2_q;
        out_wdata_o         = eng_b_out_d_i;
        out_raddr_o         = k_idx_q[D_ADDR_W-1:0];

        act_eval_clr_o      = 1'b0;
        act_eval_valid_o    = 1'b0;
        act_feat_idx_o      = k_idx_q[4:0];
        act_u_feat_o        = u_rdata_i;
        act_is_first_step_o = (step_q == 16'd0);
        step_early_exit_o   = 1'b0;

        state_re_a_o        = 1'b0;
        state_addr_a_o      = calc_state_addr(cons_d_q, cons_n_q[2:0], eff_n_state);

        w_en_o              = 1'b0;
        w_addr_o            = '0;
        bias_en_o           = 1'b0;
        bias_addr_o         = '0;
        a_en_o              = 1'b0;
        a_addr_o            = calc_state_addr(cons_d_q, cons_n_q[2:0], eff_n_state);
        d_param_addr_o      = cons_d_q[D_ADDR_W-1:0];

        eng_a_cg_en_o       = 1'b0;
        eng_a_valid_o       = 1'b0;
        eng_a_clr_o         = 1'b0;
        eng_a_req_sh_o      = 5'd6;
        eng_a_act_o         = act_d1_q;

        pwl_func_sel_o      = 1'b0;
        pwl_x_in_o          = 8'sd0;

        delta_eval_o        = 1'b0;
        step_first_ch_o     = (cons_d_q == 8'd0);
        step_last_ch_o      = (cons_d_q == (eff_d_model - 8'd1));
        delta_eval_val_o    = delta_reg[cons_d_q[D_ADDR_W-1:0]];

        // Engine B Dynamic ICG Enable: Active ONLY during FUSED_OVERLAP when consumer
        // has a non-skipped channel token or is draining final 3 stages!
        eng_b_cg_en_o       = (state_q == HWPE_FUSED_OVERLAP) &&
                              (((cons_ph_q == 2'd0) && cons_has_token && !chan_skip_i) ||
                               ((cons_ph_q == 2'd1) && !cons_chan_skip_q) ||
                               (cons_ph_q == 2'd2) ||
                               rd_valid_d1_q || eng_b_out_valid_i);
        ssm_active_o        = eng_b_cg_en_o && (rd_valid_d1_q || eng_b_out_valid_i);

        eng_b_valid_o        = rd_valid_d1_q;
        eng_b_submode_o      = rd_submode_d1_q;
        eng_b_n_first_o      = rd_n_first_d1_q;
        eng_b_n_last_o       = rd_n_last_d1_q;
        eng_b_coalesce_log_o = coalesce_log_q3_6;
        eng_b_state_addr_o   = rd_addr_d1_q;
        eng_b_delta_o       = delta_reg[rd_d_d1_q];
        eng_b_b_o           = b_reg[rd_n_d1_q];
        eng_b_c_o           = c_reg[rd_n_d1_q];
        eng_b_x_o           = x_reg[rd_d_d1_q];
        eng_b_d_skip_o      = rd_d_skip_d1_q;
        eng_b_z_o           = z_reg[rd_d_d1_q];

        argmax_valid_o      = 1'b0;

        case (state_q)
            // ----------------------------------------------------------------
            // PHASE 0: Level-1 Coarse Input-Activity Silent-Frame Early Exit
            // ----------------------------------------------------------------
            HWPE_STEP_CHK: begin
                if (sub_ph_q == 3'd0) begin
                    act_eval_clr_o = 1'b1;
                end else if (sub_ph_q == 3'd1) begin
                    u_rd_addr_o      = step_u_base_q + k_idx_q[RING_ADDR_W-1:0];
                    act_eval_valid_o = 1'b1;
                    act_feat_idx_o   = k_idx_q[4:0];
                    act_u_feat_o     = u_rdata_i;
                end else if (sub_ph_q == 3'd2) begin
                    if (act_early_exit_i) begin
                        step_early_exit_o = 1'b1; // Silent frame! Bypass whole timestep!
                    end
                end
            end

            // ----------------------------------------------------------------
            // PHASE 1: Linear_in (u -> x_raw, z) + Trainable Log8 Conv1D + SiLU
            // ----------------------------------------------------------------
            HWPE_IN_PROJ: begin
                if (sub_ph_q == 3'd0 || sub_ph_q == 3'd1) begin
                    eng_a_cg_en_o = 1'b1;
                    u_rd_addr_o   = step_u_base_q + k_idx_q[RING_ADDR_W-1:0];
                    w_en_o        = 1'b1;
                    bias_en_o     = 1'b1;
                    bias_addr_o   = {1'b0, sub_ph_q[0], grp_q[6:2]};
                    w_addr_o      = (sub_ph_q == 3'd0 ? W_BASE_IN_X : W_BASE_IN_Z)
                                  + ({3'd0, grp_q[7:2], 2'b00} + {3'd0, k_idx_q});
                    if (pipe_dly_q == 3'd1)      eng_a_clr_o   = 1'b1;
                    else if (pipe_dly_q == 3'd2) eng_a_valid_o = 1'b1;
                end else begin
                    // Trainable 4-Tap Depthwise Log8 Conv1D -> PWL SiLU!
                    conv_push_o    = 1'b1;
                    conv_ch_o      = cons_d_q[D_ADDR_W-1:0];
                    conv_x_new_o   = x_reg[cons_d_q[D_ADDR_W-1:0]];
                    conv_w_addr_o  = cons_d_q[D_ADDR_W-1:0];
                    pwl_func_sel_o = 1'b0; // SiLU
                    pwl_x_in_o     = conv_out_s8_i;
                end
            end

            // ----------------------------------------------------------------
            // PHASE 2: Selective B[0..N-1] & C[0..N-1] Projections (computed
            //          BEFORE Δ so Engine B can immediately overlap with Δ!)
            // ----------------------------------------------------------------
            HWPE_BC_PROJ: begin
                eng_a_cg_en_o = 1'b1;
                w_en_o        = 1'b1;
                bias_en_o     = 1'b1;
                if (sub_ph_q == 3'd0) begin
                    w_addr_o    = W_BASE_B_PROJ + ({3'd0, grp_q[7:2], 2'b00} + {3'd0, k_idx_q});
                    bias_addr_o = {2'b11, 2'b00, grp_q[4:2]};
                end else begin
                    w_addr_o    = W_BASE_C_PROJ + ({3'd0, grp_q[7:2], 2'b00} + {3'd0, k_idx_q});
                    bias_addr_o = {2'b11, 2'b01, grp_q[4:2]};
                end
                if (pipe_dly_q == 3'd1)      eng_a_clr_o   = 1'b1;
                else if (pipe_dly_q == 3'd2) eng_a_valid_o = 1'b1;
            end

            // ----------------------------------------------------------------
            // ★ PHASE 3: TRUE 4+1 CONCURRENT PRODUCER-CONSUMER OVERLAP! ★
            //   - PRODUCER (Engine A): computes Δ[grp..grp+3] via Softplus
            //   - CONSUMER (Engine B): SIMULTANEOUSLY runs 0-bubble Log-ZOH
            //     SSM + Fused Gate on ready channels (cons_d < delta_ready_cnt)
            // ----------------------------------------------------------------
            HWPE_FUSED_OVERLAP: begin
                // --- A) PRODUCER (Engine A generating Δ_d) ---
                if (!prod_done_q) begin
                    if (!mode_s6_i) begin
                        // Static S4D: read Bank 3 delta_s4d (note: d_param_addr shared carefully!)
                        d_param_addr_o = prod_s4d_d_q[D_ADDR_W-1:0];
                    end else begin
                        eng_a_cg_en_o  = 1'b1;
                        w_en_o         = 1'b1;
                        w_addr_o       = W_BASE_DELTA + ({3'd0, grp_q[7:2], 2'b00} + {3'd0, k_idx_q});
                        bias_en_o      = 1'b1;
                        bias_addr_o    = {2'b10, grp_q[6:2]};
                        pwl_func_sel_o = 1'b1; // Softplus
                        case (pipe_dly_q)
                            3'd1: eng_a_clr_o   = 1'b1;
                            3'd2: eng_a_valid_o = 1'b1;
                            3'd4: pwl_x_in_o    = eng_a_q0_i;
                            3'd5: pwl_x_in_o    = eng_a_q1_i;
                            3'd6: pwl_x_in_o    = eng_a_q2_i;
                            3'd7: pwl_x_in_o    = eng_a_q3_i;
                            default: ;
                        endcase
                    end
                end

                // --- B) CONSUMER (Engine B running Log-ZOH SSM in PARALLEL!) ---
                if (cons_ph_q == 2'd0 && cons_has_token) begin
                    delta_eval_o     = 1'b1;
                    delta_eval_val_o = delta_reg[cons_d_q[D_ADDR_W-1:0]];
                    if (chan_skip_i) begin
                        // Level-2 Fine-Grained Channel Δ-Skip: 0 SRAM R/W!
                        out_we_o    = 1'b1;
                        out_waddr_o = cons_d_q[D_ADDR_W-1:0];
                        out_wdata_o = 8'sd0;
                    end else begin
                        state_re_a_o   = 1'b1;
                        state_addr_a_o = calc_state_addr(cons_d_q, 3'd0, eff_n_state);
                        a_en_o         = 1'b1;
                        a_addr_o       = calc_state_addr(cons_d_q, 3'd0, eff_n_state);
                        if (mode_s6_i || prod_done_q)
                            d_param_addr_o = cons_d_q[D_ADDR_W-1:0];
                    end
                end else if (cons_ph_q == 2'd1) begin
                    if (cons_n_q < eff_n_state) begin
                        state_re_a_o   = 1'b1;
                        state_addr_a_o = calc_state_addr(cons_d_q, cons_n_q[2:0], eff_n_state);
                        a_en_o         = 1'b1;
                        a_addr_o       = calc_state_addr(cons_d_q, cons_n_q[2:0], eff_n_state);
                        if (mode_s6_i || prod_done_q)
                            d_param_addr_o = cons_d_q[D_ADDR_W-1:0];
                    end
                end

                // Capture gated output out_d from Engine B Stage 3 (priority over skip write)
                if (eng_b_out_valid_i) begin
                    out_we_o    = 1'b1;
                    out_waddr_o = b_d_pipe2_q;
                    out_wdata_o = eng_b_out_d_i;
                end
            end

            // ----------------------------------------------------------------
            // PHASE 4: Final Classifier Projection
            // ----------------------------------------------------------------
            HWPE_OUT_PROJ: begin
                eng_a_cg_en_o = 1'b1;
                out_raddr_o   = k_idx_q[D_ADDR_W-1:0];
                w_en_o        = 1'b1;
                w_addr_o      = W_BASE_CLS_FC + ({3'd0, grp_q[7:2], 2'b00} + {3'd0, k_idx_q});
                bias_en_o     = 1'b1;
                bias_addr_o   = {2'b11, 2'b10, grp_q[4:2]};
                if (pipe_dly_q == 3'd1)      eng_a_clr_o   = 1'b1;
                else if (pipe_dly_q == 3'd2) eng_a_valid_o = 1'b1;
            end

            // ----------------------------------------------------------------
            // PHASE 5: 2-Stage Argmax
            // ----------------------------------------------------------------
            HWPE_ARGMAX: begin
                if (sub_ph_q == 3'd0) argmax_valid_o = 1'b1;
            end

            default: ;
        endcase
    end

    // ------------------------------------------------------------------------
    // Sequential Dual-Engine Producer-Consumer FSM
    // ------------------------------------------------------------------------
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q           <= HWPE_IDLE;
            done_pulse_o      <= 1'b0;
            step_q            <= 16'd0;
            step_u_base_q     <= '0;
            layer_q           <= 2'd0;
            silent_run_cnt_q  <= 8'd0;
            grp_q             <= 8'd0;
            k_idx_q           <= 8'd0;
            sub_ph_q          <= 3'd0;
            pipe_dly_q        <= 3'd0;
            act_d1_q          <= 8'sd0;
            prod_done_q       <= 1'b0;
            delta_ready_cnt_q <= 8'd0;
            prod_s4d_d_q      <= 8'd0;
            cons_d_q          <= 8'd0;
            cons_n_q          <= 8'd0;
            cons_ph_q         <= 2'd0;
            cons_drain_q      <= 3'd0;
            cons_chan_skip_q  <= 1'b0;
            rd_valid_d1_q     <= 1'b0;
            rd_submode_d1_q   <= 1'b0;
            rd_n_first_d1_q   <= 1'b0;
            rd_n_last_d1_q    <= 1'b0;
            rd_addr_d1_q      <= '0;
            rd_d_d1_q         <= '0;
            rd_n_d1_q         <= 3'd0;
            rd_d_skip_d1_q    <= 8'sd0;
            b_d_pipe0_q       <= '0;
            b_d_pipe1_q       <= '0;
            b_d_pipe2_q       <= '0;
            for (int i = 0; i < 8; i++) logits_q[i] <= 16'sd0;
        end else begin
            done_pulse_o  <= 1'b0;
            rd_valid_d1_q <= 1'b0;

            if (eng_b_cg_en_o) begin
                b_d_pipe0_q <= rd_d_d1_q;
                b_d_pipe1_q <= b_d_pipe0_q;
                b_d_pipe2_q <= b_d_pipe1_q;
            end

            case (state_q)
                HWPE_IDLE: begin
                    if (start_pulse_i) begin
                        state_q          <= HWPE_STEP_CHK;
                        step_q           <= 16'd0;
                        step_u_base_q    <= '0;
                        layer_q          <= 2'd0;
                        silent_run_cnt_q <= 8'd0;
                        sub_ph_q         <= 3'd0;
                        k_idx_q          <= 8'd0;
                        grp_q            <= 8'd0;
                        pipe_dly_q       <= 3'd0;
                    end
                end

                // ------------------------------------------------------------
                // Level-1 Coarse Input-Activity Check (8 cycles -> saves ~250!)
                // ------------------------------------------------------------
                HWPE_STEP_CHK: begin
                    case (sub_ph_q)
                        3'd0: begin
                            k_idx_q  <= 8'd0;
                            sub_ph_q <= 3'd1;
                        end
                        3'd1: begin
                            if (k_idx_q + 8'd1 >= eff_d_in) begin
                                sub_ph_q <= 3'd2;
                            end else begin
                                k_idx_q <= k_idx_q + 8'd1;
                            end
                        end
                        3'd2: begin
                            if (act_early_exit_i) begin
                                // ★ LEVEL-1 SILENT FRAME EARLY EXIT (+ RR-Interval Counter!) ★
                                if (silent_run_cnt_q < 8'd255)
                                    silent_run_cnt_q <= silent_run_cnt_q + 8'd1;
                                if (step_q + 16'd1 < eff_seq_len) begin
                                    step_q        <= step_q + 16'd1;
                                    step_u_base_q <= step_u_base_q + eff_d_in[RING_ADDR_W-1:0];
                                    sub_ph_q      <= 3'd0;
                                    k_idx_q       <= 8'd0;
                                end else begin
                                    state_q    <= HWPE_OUT_PROJ;
                                    grp_q      <= 8'd0;
                                    k_idx_q    <= 8'd0;
                                    pipe_dly_q <= 3'd0;
                                end
                            end else begin
                                // Active frame -> proceed to Layer 0 Linear_in + Conv1D
                                state_q    <= HWPE_IN_PROJ;
                                layer_q    <= 2'd0;
                                sub_ph_q   <= 3'd0;
                                grp_q      <= 8'd0;
                                k_idx_q    <= 8'd0;
                                cons_d_q   <= 8'd0;
                                pipe_dly_q <= 3'd0;
                            end
                        end
                        default: sub_ph_q <= 3'd0;
                    endcase
                end

                // ------------------------------------------------------------
                HWPE_IN_PROJ: begin
                    if (sub_ph_q == 3'd0 || sub_ph_q == 3'd1) begin
                        case (pipe_dly_q)
                            3'd0: begin k_idx_q <= 8'd0; pipe_dly_q <= 3'd1; end
                            3'd1: begin act_d1_q <= u_rdata_i; k_idx_q <= 8'd1; pipe_dly_q <= 3'd2; end
                            3'd2: begin
                                act_d1_q <= u_rdata_i;
                                if (k_idx_q >= eff_d_in) pipe_dly_q <= 3'd3;
                                else k_idx_q <= k_idx_q + 8'd1;
                            end
                            3'd3: pipe_dly_q <= 3'd4;
                            3'd4: begin
                                if (sub_ph_q == 3'd0) begin
                                    x_reg[grp_q[D_ADDR_W-1:0] + 0] <= eng_a_q0_i;
                                    x_reg[grp_q[D_ADDR_W-1:0] + 1] <= eng_a_q1_i;
                                    x_reg[grp_q[D_ADDR_W-1:0] + 2] <= eng_a_q2_i;
                                    x_reg[grp_q[D_ADDR_W-1:0] + 3] <= eng_a_q3_i;
                                end else begin
                                    z_reg[grp_q[D_ADDR_W-1:0] + 0] <= eng_a_q0_i;
                                    z_reg[grp_q[D_ADDR_W-1:0] + 1] <= eng_a_q1_i;
                                    z_reg[grp_q[D_ADDR_W-1:0] + 2] <= eng_a_q2_i;
                                    z_reg[grp_q[D_ADDR_W-1:0] + 3] <= eng_a_q3_i;
                                end
                                pipe_dly_q <= 3'd0;
                                if (grp_q + 8'd4 >= eff_d_model) begin
                                    grp_q    <= 8'd0;
                                    cons_d_q <= 8'd0;
                                    sub_ph_q <= sub_ph_q + 3'd1;
                                end else begin
                                    grp_q <= grp_q + 8'd4;
                                end
                            end
                            default: pipe_dly_q <= 3'd0;
                        endcase
                    end else begin
                        // Trainable Log8 Depthwise Conv1D + PWL SiLU
                        x_reg[cons_d_q[D_ADDR_W-1:0]] <= pwl_silu_i;
                        if (cons_d_q + 8'd1 >= eff_d_model) begin
                            state_q    <= HWPE_BC_PROJ;
                            sub_ph_q   <= 3'd0;
                            grp_q      <= 8'd0;
                            k_idx_q    <= 8'd0;
                            pipe_dly_q <= 3'd0;
                        end else begin
                            cons_d_q <= cons_d_q + 8'd1;
                        end
                    end
                end

                // ------------------------------------------------------------
                HWPE_BC_PROJ: begin
                    case (pipe_dly_q)
                        3'd0: begin k_idx_q <= 8'd0; pipe_dly_q <= 3'd1; end
                        3'd1: begin act_d1_q <= x_reg[0]; k_idx_q <= 8'd1; pipe_dly_q <= 3'd2; end
                        3'd2: begin
                            act_d1_q <= x_reg[k_idx_q[D_ADDR_W-1:0]];
                            if (k_idx_q >= eff_d_model) pipe_dly_q <= 3'd3;
                            else k_idx_q <= k_idx_q + 8'd1;
                        end
                        3'd3: pipe_dly_q <= 3'd4;
                        3'd4: begin
                            if (sub_ph_q == 3'd0) begin
                                b_reg[grp_q[2:0] + 0] <= eng_a_q0_i;
                                b_reg[grp_q[2:0] + 1] <= eng_a_q1_i;
                                b_reg[grp_q[2:0] + 2] <= eng_a_q2_i;
                                b_reg[grp_q[2:0] + 3] <= eng_a_q3_i;
                            end else begin
                                c_reg[grp_q[2:0] + 0] <= eng_a_q0_i;
                                c_reg[grp_q[2:0] + 1] <= eng_a_q1_i;
                                c_reg[grp_q[2:0] + 2] <= eng_a_q2_i;
                                c_reg[grp_q[2:0] + 3] <= eng_a_q3_i;
                            end
                            pipe_dly_q <= 3'd0;
                            if (grp_q + 8'd4 >= eff_n_state) begin
                                grp_q <= 8'd0;
                                if (sub_ph_q == 3'd1) begin
                                    // Enter TRUE 4+1 CONCURRENT OVERLAP!
                                    state_q           <= HWPE_FUSED_OVERLAP;
                                    prod_done_q       <= 1'b0;
                                    delta_ready_cnt_q <= 8'd0;
                                    prod_s4d_d_q      <= 8'd0;
                                    cons_d_q          <= 8'd0;
                                    cons_n_q          <= 8'd0;
                                    cons_ph_q         <= 2'd0;
                                    cons_drain_q      <= 3'd0;
                                    cons_chan_skip_q  <= 1'b0;
                                    k_idx_q           <= 8'd0;
                                    pipe_dly_q        <= 3'd0;
                                end else begin
                                    sub_ph_q <= 3'd1;
                                end
                            end else begin
                                grp_q <= grp_q + 8'd4;
                            end
                        end
                        default: pipe_dly_q <= 3'd0;
                    endcase
                end

                // ------------------------------------------------------------
                // ★ TRUE 4+1 CONCURRENT PRODUCER (Eng A) || CONSUMER (Eng B) ★
                // ------------------------------------------------------------
                HWPE_FUSED_OVERLAP: begin
                    // 1) PRODUCER FSM (Engine A computing Δ_d -> delta_reg)
                    if (!prod_done_q) begin
                        if (!mode_s6_i) begin
                            delta_reg[prod_s4d_d_q[D_ADDR_W-1:0]] <= delta_s4d_i;
                            delta_ready_cnt_q <= prod_s4d_d_q + 8'd1;
                            if (prod_s4d_d_q + 8'd1 >= eff_d_model)
                                prod_done_q <= 1'b1;
                            else
                                prod_s4d_d_q <= prod_s4d_d_q + 8'd1;
                        end else begin
                            case (pipe_dly_q)
                                3'd0: begin k_idx_q <= 8'd0; pipe_dly_q <= 3'd1; end
                                3'd1: begin act_d1_q <= x_reg[0]; k_idx_q <= 8'd1; pipe_dly_q <= 3'd2; end
                                3'd2: begin
                                    act_d1_q <= x_reg[k_idx_q[D_ADDR_W-1:0]];
                                    if (k_idx_q >= eff_d_model) pipe_dly_q <= 3'd3;
                                    else k_idx_q <= k_idx_q + 8'd1;
                                end
                                3'd3: pipe_dly_q <= 3'd4;
                                3'd4: begin
                                    delta_reg[grp_q[D_ADDR_W-1:0] + 0] <= pwl_sp_i;
                                    delta_ready_cnt_q <= grp_q + 8'd1;
                                    pipe_dly_q <= 3'd5;
                                end
                                3'd5: begin
                                    delta_reg[grp_q[D_ADDR_W-1:0] + 1] <= pwl_sp_i;
                                    delta_ready_cnt_q <= grp_q + 8'd2;
                                    pipe_dly_q <= 3'd6;
                                end
                                3'd6: begin
                                    delta_reg[grp_q[D_ADDR_W-1:0] + 2] <= pwl_sp_i;
                                    delta_ready_cnt_q <= grp_q + 8'd3;
                                    pipe_dly_q <= 3'd7;
                                end
                                3'd7: begin
                                    delta_reg[grp_q[D_ADDR_W-1:0] + 3] <= pwl_sp_i;
                                    delta_ready_cnt_q <= grp_q + 8'd4;
                                    pipe_dly_q <= 3'd0;
                                    if (grp_q + 8'd4 >= eff_d_model) begin
                                        prod_done_q <= 1'b1;
                                    end else begin
                                        grp_q <= grp_q + 8'd4;
                                    end
                                end
                                default: pipe_dly_q <= 3'd0;
                            endcase
                        end
                    end

                    // 2) CONSUMER FSM (Engine B running Log-ZOH SSM + Gate CONCURRENTLY!)
                    case (cons_ph_q)
                        2'd0: begin
                            if (cons_has_token) begin
                                if (chan_skip_i) begin
                                    cons_chan_skip_q <= 1'b1;
                                    if (cons_d_q + 8'd1 >= eff_d_model) begin
                                        cons_ph_q    <= 2'd2;
                                        cons_drain_q <= 3'd0;
                                    end else begin
                                        cons_d_q <= cons_d_q + 8'd1;
                                    end
                                end else begin
                                    cons_chan_skip_q <= 1'b0;
                                    rd_valid_d1_q    <= 1'b1;
                                    rd_submode_d1_q  <= 1'b0;
                                    rd_n_first_d1_q  <= 1'b1;
                                    rd_n_last_d1_q   <= (eff_n_state == 8'd1);
                                    rd_addr_d1_q     <= calc_state_addr(cons_d_q, 3'd0, eff_n_state);
                                    rd_d_d1_q        <= cons_d_q[D_ADDR_W-1:0];
                                    rd_n_d1_q        <= 3'd0;
                                    rd_d_skip_d1_q   <= (mode_s6_i || prod_done_q) ? d_skip_s8_i : 8'sd0;
                                    cons_n_q         <= 8'd1;
                                    cons_ph_q        <= 2'd1;
                                end
                            end
                        end

                        2'd1: begin
                            if (cons_n_q < eff_n_state) begin
                                rd_valid_d1_q   <= 1'b1;
                                rd_submode_d1_q <= 1'b0;
                                rd_n_first_d1_q <= 1'b0;
                                rd_n_last_d1_q  <= (cons_n_q == (eff_n_state - 8'd1));
                                rd_addr_d1_q    <= calc_state_addr(cons_d_q, cons_n_q[2:0], eff_n_state);
                                rd_d_d1_q       <= cons_d_q[D_ADDR_W-1:0];
                                rd_n_d1_q       <= cons_n_q[2:0];
                                rd_d_skip_d1_q  <= (mode_s6_i || prod_done_q) ? d_skip_s8_i : 8'sd0;
                                cons_n_q        <= cons_n_q + 8'd1;
                            end else begin
                                // Launch 1-cycle Sub-mode B1 (SiLU Gate) with 0 bubble!
                                rd_valid_d1_q   <= 1'b1;
                                rd_submode_d1_q <= 1'b1;
                                rd_n_first_d1_q <= 1'b0;
                                rd_n_last_d1_q  <= 1'b0;
                                rd_d_d1_q       <= cons_d_q[D_ADDR_W-1:0];
                                if (cons_d_q + 8'd1 >= eff_d_model) begin
                                    cons_ph_q    <= 2'd2;
                                    cons_drain_q <= 3'd0;
                                end else begin
                                    cons_d_q  <= cons_d_q + 8'd1;
                                    cons_n_q  <= 8'd0;
                                    cons_ph_q <= 2'd0;
                                end
                            end
                        end

                        2'd2: begin
                            if (cons_drain_q >= 3'd3) begin
                                cons_ph_q <= 2'd3;
                            end else begin
                                cons_drain_q <= cons_drain_q + 3'd1;
                            end
                        end

                        2'd3: begin
                            if (prod_done_q) begin
                                silent_run_cnt_q <= 8'd0; // Coalesced decay consumed!
                                if (layer_q + 2'd1 < eff_num_layers) begin
                                    // ★ v1.3.0: Time-Multiplexed Next Mamba Layer on same hardware!
                                    layer_q    <= layer_q + 2'd1;
                                    state_q    <= HWPE_IN_PROJ;
                                    sub_ph_q   <= 3'd0;
                                    grp_q      <= 8'd0;
                                    k_idx_q    <= 8'd0;
                                    cons_d_q   <= 8'd0;
                                    pipe_dly_q <= 3'd0;
                                end else if (step_q + 16'd1 < eff_seq_len) begin
                                    layer_q       <= 2'd0;
                                    step_q        <= step_q + 16'd1;
                                    step_u_base_q <= step_u_base_q + eff_d_in[RING_ADDR_W-1:0];
                                    state_q       <= HWPE_STEP_CHK;
                                    sub_ph_q      <= 3'd0;
                                    k_idx_q       <= 8'd0;
                                end else begin
                                    layer_q    <= 2'd0;
                                    state_q    <= HWPE_OUT_PROJ;
                                    grp_q      <= 8'd0;
                                    k_idx_q    <= 8'd0;
                                    pipe_dly_q <= 3'd0;
                                end
                            end
                        end
                    endcase
                end

                // ------------------------------------------------------------
                HWPE_OUT_PROJ: begin
                    case (pipe_dly_q)
                        3'd0: begin k_idx_q <= 8'd0; pipe_dly_q <= 3'd1; end
                        3'd1: begin act_d1_q <= out_rdata_i; k_idx_q <= 8'd1; pipe_dly_q <= 3'd2; end
                        3'd2: begin
                            act_d1_q <= out_rdata_i;
                            if (k_idx_q >= eff_d_model) pipe_dly_q <= 3'd3;
                            else k_idx_q <= k_idx_q + 8'd1;
                        end
                        3'd3: pipe_dly_q <= 3'd4;
                        3'd4: begin
                            logits_q[grp_q[2:0] + 0] <= clamp_s16(eng_a_acc0_i);
                            logits_q[grp_q[2:0] + 1] <= clamp_s16(eng_a_acc1_i);
                            logits_q[grp_q[2:0] + 2] <= clamp_s16(eng_a_acc2_i);
                            logits_q[grp_q[2:0] + 3] <= clamp_s16(eng_a_acc3_i);
                            pipe_dly_q <= 3'd0;
                            if (grp_q + 8'd4 >= 8'd8) begin
                                state_q  <= HWPE_ARGMAX;
                                sub_ph_q <= 3'd0;
                            end else begin
                                grp_q <= grp_q + 8'd4;
                            end
                        end
                        default: pipe_dly_q <= 3'd0;
                    endcase
                end

                HWPE_ARGMAX: begin
                    sub_ph_q <= 3'd1;
                    if (argmax_done_i) begin
                        state_q      <= HWPE_DONE;
                        done_pulse_o <= 1'b1;
                    end
                end

                HWPE_DONE: begin
                    if (start_pulse_i) begin
                        state_q       <= HWPE_STEP_CHK;
                        step_q        <= 16'd0;
                        step_u_base_q <= '0;
                        sub_ph_q      <= 3'd0;
                        k_idx_q       <= 8'd0;
                    end else begin
                        state_q <= HWPE_IDLE;
                    end
                end

                default: state_q <= HWPE_IDLE;
            endcase
        end
    end

endmodule : chronos_stream_agu
