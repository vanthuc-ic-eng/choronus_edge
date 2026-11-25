// ============================================================================
// Project: CHRONOS — Beyond-Q1 Zero-DSP Logarithmic Selective State-Space SoC
// File   : rtl/top/chronos_ssm_hwpe_top.sv
// Desc   : Top-Level Autonomous 4+1 Concurrent Zero-DSP Log-SSM HWPE.
//          Upgraded with:
//            - Level-1 Coarse Input-Domain Silent-Frame Early-Exit Gate
//            - Trainable 4-Tap Log8 Depthwise Causal Conv1D (0 DSP)
//            - True 4+1 Concurrent Engine A || Engine B Overlap Scheduler
//            - PMU Activity & Telemetry Hooks (Energy Proxy, Sleep, Overlap)
// ============================================================================

`timescale 1ns / 1ps

(* use_dsp = "no" *)
module chronos_ssm_hwpe_top #(
    parameter int unsigned MAX_D        = 64,
    parameter int unsigned MAX_N        = 8,
    parameter int unsigned RING_DEPTH   = 256,
    parameter int unsigned STATE_DEPTH  = 512,
    parameter int unsigned WEIGHT_DEPTH = 2048,
    parameter int unsigned BIAS_DEPTH   = 128,
    parameter int unsigned W_ADDR_W     = $clog2(WEIGHT_DEPTH),
    parameter int unsigned B_ADDR_W     = $clog2(BIAS_DEPTH),
    parameter int unsigned S_ADDR_W     = $clog2(STATE_DEPTH),
    parameter int unsigned D_ADDR_W     = $clog2(MAX_D)
)(
    input  logic                    clk_i,
    input  logic                    rst_ni,
    input  logic                    test_en_i,

    // 1. 32-bit Memory-Mapped CSR Interface
    input  logic                    csr_req_i,
    input  logic                    csr_we_i,
    input  logic [7:0]              csr_addr_i,
    input  logic [31:0]             csr_wdata_i,
    output logic [31:0]             csr_rdata_o,
    output logic                    irq_o,

    // 2. Real-Time Streaming Sample Input Port (DMA / UART Rx / CPU)
    input  logic                    stream_we_i,
    input  logic signed [7:0]       stream_data_i,
    output logic                    ring_full_o,

    // 3. Host / DMA Parameter Load Ports (Weights, Bias, A, D, Trainable Conv1D)
    input  logic                    w_load_we_i,
    input  logic [W_ADDR_W-1:0]     w_load_addr_i,
    input  logic [31:0]             w_load_data_i,

    input  logic                    bias_load_we_i,
    input  logic [B_ADDR_W-1:0]     bias_load_addr_i,
    input  logic [127:0]            bias_load_data_i,

    input  logic                    a_load_we_i,
    input  logic [S_ADDR_W-1:0]     a_load_addr_i,
    input  logic [9:0]              a_load_data_i,

    input  logic                    d_load_we_i,
    input  logic [D_ADDR_W-1:0]     d_load_addr_i,
    input  logic [15:0]             d_load_data_i,

    input  logic                    conv_w_load_we_i,
    input  logic [D_ADDR_W-1:0]     conv_w_load_addr_i,
    input  logic [31:0]             conv_w_load_data_i,

    // 4. PMU Telemetry & Activity Hooks
    input  logic [31:0]             pmu_energy_proxy_i,
    input  logic [31:0]             pmu_cpu_sleep_cyc_i,
    input  logic [31:0]             pmu_overlap_cyc_i,
    output logic                    pmu_cnt_clr_o,
    output logic                    pmu_eng_a_active_o,
    output logic                    pmu_eng_b_active_o,
    output logic                    pmu_sram_rw_active_o,

    // 5. Direct Status / Verification Taps
    output logic                    busy_o,
    output logic                    done_o,
    output logic [3:0]              pred_class_o
);

    localparam int unsigned R_ADDR_W = $clog2(RING_DEPTH);

    logic        start_pulse, stream_en, state_rst_pulse;
    logic        step_skip_en, chan_skip_en, mode_s6, lyap_guard_en, rr_coalesce_en;
    logic [1:0]  cfg_num_layers;
    logic [8:0]  eng_b_coalesce_log;
    logic [7:0]  cfg_d_model, cfg_n_state, cfg_d_in, cfg_num_cls;
    logic [15:0] cfg_seq_len, step_idx;
    logic [7:0]  cfg_tau_chan, cfg_tau_step;
    logic        irq_en, anomaly_irq_pulse, done_pulse;
    logic [7:0]  anomaly_mask;
    logic [2:0]  fsm_state;
    logic [3:0]  pred_class;
    logic signed [15:0] max_logit;

    logic [31:0] total_cyc, active_cyc, chan_skip_cnt, step_skip_cnt, sram_saved_cnt;

    // Activity Gate wires
    logic              act_eval_clr, act_eval_valid, act_is_first_step, act_early_exit, step_early_exit;
    logic [4:0]        act_feat_idx;
    logic signed [7:0] act_u_feat;
    logic [15:0]       l1_diff_unused;

    // Bank 0 + Conv1D wires
    logic                u_pop, conv_push, out_we;
    logic [R_ADDR_W-1:0] u_rd_addr;
    logic [R_ADDR_W:0]   ring_count_unused;
    logic signed [7:0]   u_rdata, conv_x_new, tap0, tap1, tap2, tap3, conv_out_s8;
    logic [D_ADDR_W-1:0] conv_ch, conv_w_addr, out_waddr, out_raddr;
    logic signed [7:0]   out_wdata, out_rdata;
    logic [31:0]         conv_w_log8;

    // Bank 1 wires
    logic                state_re_a, h_we_b;
    logic [S_ADDR_W-1:0] state_addr_a, h_waddr_b;
    logic signed [15:0]  h_prev_a, h_new_b;

    // Bank 2 & Bank 3 wires
    logic                agu_w_en, agu_bias_en, agu_a_en;
    logic [W_ADDR_W-1:0] agu_w_addr;
    logic [B_ADDR_W-1:0] agu_bias_addr;
    logic [S_ADDR_W-1:0] agu_a_addr;
    logic [D_ADDR_W-1:0] agu_d_addr;
    logic [7:0]          w0_log8, w1_log8, w2_log8, w3_log8;
    logic signed [31:0]  bias0, bias1, bias2, bias3;
    logic                a_zero;
    logic [8:0]          a_log_q3_6;
    logic [7:0]          delta_s4d;
    logic signed [7:0]   d_skip_s8;

    // Engine A, PWL, Delta-Skip, Engine B, Argmax wires
    logic               eng_a_cg_en, eng_a_valid, eng_a_clr;
    logic [4:0]         eng_a_req_sh;
    logic signed [7:0]  eng_a_act;
    logic signed [31:0] eng_a_acc0, eng_a_acc1, eng_a_acc2, eng_a_acc3;
    logic signed [7:0]  eng_a_q0, eng_a_q1, eng_a_q2, eng_a_q3;

    logic              pwl_func_sel;
    logic signed [7:0] pwl_x_in, pwl_silu;
    logic [7:0]        pwl_sp;

    logic       delta_eval, step_first_ch, step_last_ch, ssm_active;
    logic [7:0] delta_eval_val;
    logic       chan_skip, step_skip_unused;

    logic                eng_b_cg_en, eng_b_valid, eng_b_submode;
    logic                eng_b_n_first, eng_b_n_last;
    logic [S_ADDR_W-1:0] eng_b_state_addr;
    logic [7:0]          eng_b_delta;
    logic signed [7:0]   eng_b_b, eng_b_c, eng_b_x, eng_b_d_skip, eng_b_z;
    logic                eng_b_out_valid;
    logic signed [7:0]   eng_b_out_d;
    logic signed [15:0]  eng_b_y_debug_unused;

    logic               argmax_valid, argmax_done;
    logic signed [15:0] l0, l1, l2, l3, l4, l5, l6, l7;

    wire _unused_top = &{1'b0, stream_en, ring_count_unused, step_skip_unused,
                         eng_b_y_debug_unused, l1_diff_unused};

    assign done_o               = done_pulse;
    assign pred_class_o         = pred_class;
    assign pmu_cnt_clr_o        = start_pulse;
    assign pmu_eng_a_active_o   = eng_a_cg_en;
    assign pmu_eng_b_active_o   = eng_b_cg_en;
    assign pmu_sram_rw_active_o = state_re_a | h_we_b;

    // 1. CSR Control & Beyond-Q1 Telemetry Interface
    chronos_csr_if u_csr (
        .clk_i               ( clk_i               ),
        .rst_ni              ( rst_ni              ),
        .csr_req_i           ( csr_req_i           ),
        .csr_we_i            ( csr_we_i            ),
        .csr_addr_i          ( csr_addr_i          ),
        .csr_wdata_i         ( csr_wdata_i         ),
        .csr_rdata_o         ( csr_rdata_o         ),
        .start_pulse_o       ( start_pulse         ),
        .stream_en_o         ( stream_en           ),
        .state_rst_pulse_o   ( state_rst_pulse     ),
        .step_skip_en_o      ( step_skip_en        ),
        .chan_skip_en_o      ( chan_skip_en        ),
        .mode_s6_o           ( mode_s6             ),
        .lyap_guard_en_o     ( lyap_guard_en       ),
        .rr_coalesce_en_o    ( rr_coalesce_en      ),
        .cfg_num_layers_o    ( cfg_num_layers      ),
        .cfg_d_model_o       ( cfg_d_model         ),
        .cfg_n_state_o       ( cfg_n_state         ),
        .cfg_d_in_o          ( cfg_d_in            ),
        .cfg_num_cls_o       ( cfg_num_cls         ),
        .cfg_seq_len_o       ( cfg_seq_len         ),
        .cfg_tau_chan_o      ( cfg_tau_chan        ),
        .cfg_tau_step_o      ( cfg_tau_step        ),
        .irq_en_o            ( irq_en              ),
        .anomaly_mask_o      ( anomaly_mask        ),
        .irq_o               ( irq_o               ),
        .fsm_state_i         ( fsm_state           ),
        .busy_i              ( busy_o              ),
        .done_pulse_i        ( done_pulse          ),
        .ring_full_i         ( ring_full_o         ),
        .step_idx_i          ( step_idx            ),
        .pred_class_i        ( pred_class          ),
        .max_logit_i         ( max_logit           ),
        .anomaly_irq_pulse_i ( anomaly_irq_pulse   ),
        .total_cyc_i         ( total_cyc           ),
        .active_cyc_i        ( active_cyc          ),
        .chan_skip_cnt_i     ( chan_skip_cnt       ),
        .step_skip_cnt_i     ( step_skip_cnt       ),
        .sram_saved_cnt_i    ( sram_saved_cnt      ),
        .energy_proxy_i      ( pmu_energy_proxy_i  ),
        .cpu_sleep_cyc_i     ( pmu_cpu_sleep_cyc_i ),
        .overlap_cyc_i       ( pmu_overlap_cyc_i   )
    );

    // 2. Level-1 Coarse Input-Domain Silent-Frame Early-Exit Detector
    chronos_activity_gate u_activity_gate (
        .clk_i           ( clk_i             ),
        .rst_ni          ( rst_ni            ),
        .seq_rst_i       ( state_rst_pulse   ),
        .step_skip_en_i  ( step_skip_en      ),
        .tau_step_i      ( cfg_tau_step      ),
        .is_first_step_i ( act_is_first_step ),
        .eval_clr_i      ( act_eval_clr      ),
        .eval_valid_i    ( act_eval_valid    ),
        .feat_idx_i      ( act_feat_idx      ),
        .u_feat_i        ( act_u_feat        ),
        .l1_diff_o       ( l1_diff_unused    ),
        .early_exit_o    ( act_early_exit    )
    );

    // 3. Hybrid 4-Bank TCDM + Trainable 4-Tap Log8 Causal Conv1D
    chronos_tcdm_input_ring #(
        .MAX_D ( MAX_D ), .RING_DEPTH ( RING_DEPTH )
    ) u_bank0_input_ring (
        .clk_i         ( clk_i             ),
        .rst_ni        ( rst_ni            ),
        .seq_rst_i     ( state_rst_pulse   ),
        .stream_we_i   ( stream_we_i       ),
        .stream_data_i ( stream_data_i     ),
        .ring_full_o   ( ring_full_o       ),
        .ring_count_o  ( ring_count_unused ),
        .u_pop_i       ( u_pop             ),
        .u_rd_addr_i   ( u_rd_addr         ),
        .u_rdata_o     ( u_rdata           ),
        .conv_push_i   ( conv_push         ),
        .conv_ch_i     ( conv_ch           ),
        .conv_x_new_i  ( conv_x_new        ),
        .tap0_o        ( tap0              ),
        .tap1_o        ( tap1              ),
        .tap2_o        ( tap2              ),
        .tap3_o        ( tap3              ),
        .out_we_i      ( out_we            ),
        .out_waddr_i   ( out_waddr         ),
        .out_wdata_i   ( out_wdata         ),
        .out_raddr_i   ( out_raddr         ),
        .out_rdata_o   ( out_rdata         )
    );

    chronos_conv1d_log8 u_trainable_conv1d (
        .tap0_i        ( tap0        ),
        .tap1_i        ( tap1        ),
        .tap2_i        ( tap2        ),
        .tap3_i        ( tap3        ),
        .w_conv_log8_i ( conv_w_log8 ),
        .conv_out_o    ( conv_out_s8 )
    );

    chronos_tcdm_state_dp #(.DEPTH ( STATE_DEPTH )) u_bank1_state_dp (
        .clk_i       ( clk_i           ),
        .rst_ni      ( rst_ni          ),
        .state_rst_i ( state_rst_pulse ),
        .re_a_i      ( state_re_a      ),
        .addr_a_i    ( state_addr_a    ),
        .rdata_a_o   ( h_prev_a        ),
        .we_b_i      ( h_we_b          ),
        .addr_b_i    ( h_waddr_b       ),
        .wdata_b_i   ( h_new_b         )
    );

    chronos_tcdm_weight_log8 #(.DEPTH ( WEIGHT_DEPTH )) u_bank2_weight_log8 (
        .clk_i     ( clk_i                                   ),
        .en_i      ( w_load_we_i | agu_w_en                  ),
        .we_i      ( w_load_we_i                             ),
        .addr_i    ( w_load_we_i ? w_load_addr_i : agu_w_addr ),
        .wdata_i   ( w_load_data_i                           ),
        .w0_log8_o ( w0_log8                                 ),
        .w1_log8_o ( w1_log8                                 ),
        .w2_log8_o ( w2_log8                                 ),
        .w3_log8_o ( w3_log8                                 )
    );

    chronos_tcdm_bias_param #(
        .BIAS_DEPTH ( BIAS_DEPTH ), .SSM_DEPTH ( STATE_DEPTH ), .MAX_D ( MAX_D )
    ) u_bank3_bias_param (
        .clk_i          ( clk_i                                               ),
        .bias_en_i      ( bias_load_we_i | agu_bias_en                        ),
        .bias_we_i      ( bias_load_we_i                                      ),
        .bias_addr_i    ( bias_load_we_i ? bias_load_addr_i : agu_bias_addr   ),
        .bias_wdata_i   ( bias_load_data_i                                    ),
        .bias0_o        ( bias0                                               ),
        .bias1_o        ( bias1                                               ),
        .bias2_o        ( bias2                                               ),
        .bias3_o        ( bias3                                               ),
        .a_en_i         ( a_load_we_i | agu_a_en                              ),
        .a_we_i         ( a_load_we_i                                         ),
        .a_addr_i       ( a_load_we_i ? a_load_addr_i : agu_a_addr            ),
        .a_wdata_i      ( a_load_data_i                                       ),
        .a_zero_o       ( a_zero                                              ),
        .a_log_q3_6_o   ( a_log_q3_6                                          ),
        .d_we_i         ( d_load_we_i                                         ),
        .d_addr_i       ( d_load_we_i ? d_load_addr_i : agu_d_addr            ),
        .d_wdata_i      ( d_load_data_i                                       ),
        .delta_s4d_o    ( delta_s4d                                           ),
        .d_skip_s8_o    ( d_skip_s8                                           ),
        .conv_w_we_i    ( conv_w_load_we_i                                    ),
        .conv_w_addr_i  ( conv_w_load_we_i ? conv_w_load_addr_i : conv_w_addr ),
        .conv_w_wdata_i ( conv_w_load_data_i                                  ),
        .conv_w_log8_o  ( conv_w_log8                                         )
    );

    // 4. Engine A + Shared PWL + Engine B + Dual-Granularity Δ-Skip + AGU + Argmax
    chronos_proj_engine_1x4 u_engine_a (
        .clk_i        ( clk_i        ), .rst_ni       ( rst_ni       ),
        .cg_en_i      ( eng_a_cg_en  ), .test_en_i    ( test_en_i    ),
        .valid_i      ( eng_a_valid  ), .clr_i        ( eng_a_clr    ),
        .requant_sh_i ( eng_a_req_sh ), .act_i        ( eng_a_act    ),
        .w0_log8_i    ( w0_log8      ), .w1_log8_i    ( w1_log8      ),
        .w2_log8_i    ( w2_log8      ), .w3_log8_i    ( w3_log8      ),
        .bias0_i      ( bias0        ), .bias1_i      ( bias1        ),
        .bias2_i      ( bias2        ), .bias3_i      ( bias3        ),
        .acc0_o       ( eng_a_acc0   ), .acc1_o       ( eng_a_acc1   ),
        .acc2_o       ( eng_a_acc2   ), .acc3_o       ( eng_a_acc3   ),
        .q0_o         ( eng_a_q0     ), .q1_o         ( eng_a_q1     ),
        .q2_o         ( eng_a_q2     ), .q3_o         ( eng_a_q3     )
    );

    chronos_pwl_nonlinear u_shared_pwl (
        .func_sel_i  ( pwl_func_sel ),
        .x_q4_4_i    ( pwl_x_in     ),
        .silu_q4_4_o ( pwl_silu     ),
        .sp_q2_6_o   ( pwl_sp       )
    );

    chronos_ssm_core_1lane #(.STATE_ADDR_W ( S_ADDR_W )) u_engine_b (
        .clk_i        ( clk_i                ), .rst_ni       ( rst_ni               ),
        .cg_en_i      ( eng_b_cg_en          ), .test_en_i    ( test_en_i            ),
        .valid_i      ( eng_b_valid          ), .submode_i    ( eng_b_submode        ),
        .n_first_i    ( eng_b_n_first        ), .n_last_i     ( eng_b_n_last         ),
        .lyap_guard_i ( lyap_guard_en        ), .coalesce_log_q3_6_i ( eng_b_coalesce_log ),
        .state_addr_i ( eng_b_state_addr     ),
        .h_prev_i     ( h_prev_a             ), .delta_q2_6_i ( eng_b_delta          ),
        .a_log_q3_6_i ( a_log_q3_6           ), .a_zero_i     ( a_zero               ),
        .b_s8_i       ( eng_b_b              ), .c_s8_i       ( eng_b_c              ),
        .x_s8_i       ( eng_b_x              ), .d_skip_s8_i  ( eng_b_d_skip         ),
        .z_s8_i       ( eng_b_z              ), .h_we_o       ( h_we_b               ),
        .h_waddr_o    ( h_waddr_b            ), .h_new_o      ( h_new_b              ),
        .out_valid_o  ( eng_b_out_valid      ), .out_d_o      ( eng_b_out_d          ),
        .y_debug_o    ( eng_b_y_debug_unused )
    );

    chronos_delta_skip_ctrl u_delta_skip (
        .clk_i             ( clk_i            ), .rst_ni           ( rst_ni           ),
        .cnt_clr_i         ( start_pulse      ), .step_skip_en_i   ( step_skip_en     ),
        .chan_skip_en_i    ( chan_skip_en     ), .tau_chan_i       ( cfg_tau_chan     ),
        .tau_step_i        ( cfg_tau_step     ), .n_state_i        ( cfg_n_state      ),
        .step_early_exit_i ( step_early_exit  ), .delta_eval_i     ( delta_eval       ),
        .step_first_ch_i   ( step_first_ch    ), .step_last_ch_i   ( step_last_ch     ),
        .delta_d_i         ( delta_eval_val   ), .d_model_i        ( cfg_d_model      ),
        .ssm_busy_i        ( busy_o           ), .ssm_active_i     ( ssm_active       ),
        .chan_skip_o       ( chan_skip        ), .step_skip_o      ( step_skip_unused ),
        .total_cyc_o       ( total_cyc        ), .active_cyc_o     ( active_cyc       ),
        .chan_skip_cnt_o   ( chan_skip_cnt    ), .step_skip_cnt_o  ( step_skip_cnt    ),
        .sram_saved_cnt_o  ( sram_saved_cnt   )
    );

    chronos_stream_agu #(
        .MAX_D ( MAX_D ), .MAX_N ( MAX_N ), .W_ADDR_W ( W_ADDR_W ),
        .B_ADDR_W ( B_ADDR_W ), .STATE_ADDR_W ( S_ADDR_W ), .RING_ADDR_W ( R_ADDR_W )
    ) u_agu (
        .clk_i               ( clk_i             ), .rst_ni             ( rst_ni            ),
        .start_pulse_i       ( start_pulse       ), .mode_s6_i          ( mode_s6           ),
        .rr_coalesce_en_i    ( rr_coalesce_en    ), .cfg_num_layers_i   ( cfg_num_layers    ),
        .cfg_d_model_i       ( cfg_d_model       ), .cfg_n_state_i      ( cfg_n_state       ),
        .cfg_d_in_i          ( cfg_d_in          ), .cfg_num_cls_i      ( cfg_num_cls       ),
        .cfg_seq_len_i       ( cfg_seq_len       ), .fsm_state_o        ( fsm_state         ),
        .busy_o              ( busy_o            ), .done_pulse_o       ( done_pulse        ),
        .step_idx_o          ( step_idx          ), .act_eval_clr_o     ( act_eval_clr      ),
        .act_eval_valid_o    ( act_eval_valid    ), .act_feat_idx_o     ( act_feat_idx      ),
        .act_u_feat_o        ( act_u_feat        ), .act_is_first_step_o( act_is_first_step ),
        .act_early_exit_i    ( act_early_exit    ), .step_early_exit_o  ( step_early_exit   ),
        .u_pop_o             ( u_pop             ), .u_rd_addr_o        ( u_rd_addr         ),
        .u_rdata_i           ( u_rdata           ), .conv_push_o        ( conv_push         ),
        .conv_ch_o           ( conv_ch           ), .conv_x_new_o       ( conv_x_new        ),
        .conv_out_s8_i       ( conv_out_s8       ), .out_we_o           ( out_we            ),
        .out_waddr_o         ( out_waddr         ), .out_wdata_o        ( out_wdata         ),
        .out_raddr_o         ( out_raddr         ), .out_rdata_i        ( out_rdata         ),
        .state_re_a_o        ( state_re_a        ), .state_addr_a_o     ( state_addr_a      ),
        .w_en_o              ( agu_w_en          ), .w_addr_o           ( agu_w_addr        ),
        .bias_en_o           ( agu_bias_en       ), .bias_addr_o        ( agu_bias_addr     ),
        .a_en_o              ( agu_a_en          ), .a_addr_o           ( agu_a_addr        ),
        .d_param_addr_o      ( agu_d_addr        ), .conv_w_addr_o      ( conv_w_addr       ),
        .delta_s4d_i         ( delta_s4d         ), .d_skip_s8_i        ( d_skip_s8         ),
        .eng_a_cg_en_o       ( eng_a_cg_en       ), .eng_a_valid_o      ( eng_a_valid       ),
        .eng_a_clr_o         ( eng_a_clr         ), .eng_a_req_sh_o     ( eng_a_req_sh      ),
        .eng_a_act_o         ( eng_a_act         ), .eng_a_acc0_i       ( eng_a_acc0        ),
        .eng_a_acc1_i        ( eng_a_acc1        ), .eng_a_acc2_i       ( eng_a_acc2        ),
        .eng_a_acc3_i        ( eng_a_acc3        ), .eng_a_q0_i         ( eng_a_q0          ),
        .eng_a_q1_i          ( eng_a_q1          ), .eng_a_q2_i         ( eng_a_q2          ),
        .eng_a_q3_i          ( eng_a_q3          ), .pwl_func_sel_o     ( pwl_func_sel      ),
        .pwl_x_in_o          ( pwl_x_in          ), .pwl_silu_i         ( pwl_silu          ),
        .pwl_sp_i            ( pwl_sp            ), .delta_eval_o       ( delta_eval        ),
        .step_first_ch_o     ( step_first_ch     ), .step_last_ch_o     ( step_last_ch      ),
        .delta_eval_val_o    ( delta_eval_val    ), .ssm_active_o       ( ssm_active        ),
        .chan_skip_i         ( chan_skip         ), .eng_b_cg_en_o      ( eng_b_cg_en       ),
        .eng_b_valid_o       ( eng_b_valid       ), .eng_b_submode_o    ( eng_b_submode     ),
        .eng_b_n_first_o     ( eng_b_n_first     ), .eng_b_n_last_o     ( eng_b_n_last      ),
        .eng_b_coalesce_log_o( eng_b_coalesce_log), .eng_b_state_addr_o ( eng_b_state_addr  ),
        .eng_b_delta_o       ( eng_b_delta       ),
        .eng_b_b_o           ( eng_b_b           ), .eng_b_c_o          ( eng_b_c           ),
        .eng_b_x_o           ( eng_b_x           ), .eng_b_d_skip_o     ( eng_b_d_skip      ),
        .eng_b_z_o           ( eng_b_z           ), .eng_b_out_valid_i  ( eng_b_out_valid   ),
        .eng_b_out_d_i       ( eng_b_out_d       ), .argmax_valid_o     ( argmax_valid      ),
        .logit0_o            ( l0                ), .logit1_o           ( l1                ),
        .logit2_o            ( l2                ), .logit3_o           ( l3                ),
        .logit4_o            ( l4                ), .logit5_o           ( l5                ),
        .logit6_o            ( l6                ), .logit7_o           ( l7                ),
        .argmax_done_i       ( argmax_done       )
    );

    chronos_argmax_tree u_argmax (
        .clk_i               ( clk_i             ), .rst_ni             ( rst_ni            ),
        .valid_i             ( argmax_valid      ), .num_cls_i          ( cfg_num_cls[3:0]  ),
        .logit0_i            ( l0                ), .logit1_i           ( l1                ),
        .logit2_i            ( l2                ), .logit3_i           ( l3                ),
        .logit4_i            ( l4                ), .logit5_i           ( l5                ),
        .logit6_i            ( l6                ), .logit7_i           ( l7                ),
        .irq_en_i            ( irq_en            ), .anomaly_mask_i     ( anomaly_mask      ),
        .valid_o             ( argmax_done       ), .pred_class_o       ( pred_class        ),
        .max_logit_o         ( max_logit         ), .anomaly_irq_pulse_o( anomaly_irq_pulse )
    );

endmodule : chronos_ssm_hwpe_top
