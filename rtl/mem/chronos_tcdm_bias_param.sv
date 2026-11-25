// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/mem/chronos_tcdm_bias_param.sv
// Desc   : Bank 3 TCDM — Wide 128-bit Bias Memory + Diagonal SSM Parameter
//          Memory (A_log[d,n], D_skip[d], static S4D Δ/B/C tables).
// ============================================================================

`timescale 1ns / 1ps

module chronos_tcdm_bias_param #(
    parameter int unsigned BIAS_DEPTH = 128,
    parameter int unsigned SSM_DEPTH  = 512,
    parameter int unsigned MAX_D      = 64,
    parameter int unsigned B_ADDR_W   = $clog2(BIAS_DEPTH),
    parameter int unsigned S_ADDR_W   = $clog2(SSM_DEPTH),
    parameter int unsigned D_ADDR_W   = $clog2(MAX_D)
)(
    input  logic               clk_i,

    // 128-bit Bias Port (4x INT32/clk for Engine A)
    input  logic               bias_en_i,
    input  logic               bias_we_i,
    input  logic [B_ADDR_W-1:0] bias_addr_i,
    input  logic [127:0]       bias_wdata_i,
    output logic signed [31:0] bias0_o,
    output logic signed [31:0] bias1_o,
    output logic signed [31:0] bias2_o,
    output logic signed [31:0] bias3_o,

    // Diagonal A_log Parameter Port (indexed by {d, n})
    // Packed 10-bit: {a_zero[9], a_log_q3_6[8:0]}
    input  logic               a_en_i,
    input  logic               a_we_i,
    input  logic [S_ADDR_W-1:0] a_addr_i,
    input  logic [9:0]         a_wdata_i,
    output logic               a_zero_o,
    output logic [8:0]         a_log_q3_6_o,

    // Skip Connection D_d (INT8) & Static S4D Delta (Q2.6) per channel d
    input  logic               d_we_i,
    input  logic [D_ADDR_W-1:0] d_addr_i,
    input  logic [15:0]        d_wdata_i,     // {delta_s4d[15:8], d_skip_s8[7:0]}
    output logic [7:0]         delta_s4d_o,
    output logic signed [7:0]  d_skip_s8_o,

    // Trainable Depthwise Causal Conv1D K=4 Packed Log8 Kernel Weights per channel d
    input  logic               conv_w_we_i,
    input  logic [D_ADDR_W-1:0] conv_w_addr_i,
    input  logic [31:0]        conv_w_wdata_i,
    output logic [31:0]        conv_w_log8_o
);

    logic [127:0] bias_word;

    chronos_sp_sram #(
        .DATA_W ( 128        ),
        .DEPTH  ( BIAS_DEPTH )
    ) u_bias_sram (
        .clk_i   ( clk_i        ),
        .en_i    ( bias_en_i    ),
        .we_i    ( bias_we_i    ),
        .addr_i  ( bias_addr_i  ),
        .wdata_i ( bias_wdata_i ),
        .rdata_o ( bias_word    )
    );

    assign bias0_o = $signed(bias_word[31:0]);
    assign bias1_o = $signed(bias_word[63:32]);
    assign bias2_o = $signed(bias_word[95:64]);
    assign bias3_o = $signed(bias_word[127:96]);

    // Diagonal A_log table (10-bit per state element)
    logic [9:0] a_rdata;

    chronos_sp_sram #(
        .DATA_W ( 10        ),
        .DEPTH  ( SSM_DEPTH )
    ) u_a_param_sram (
        .clk_i   ( clk_i     ),
        .en_i    ( a_en_i    ),
        .we_i    ( a_we_i    ),
        .addr_i  ( a_addr_i  ),
        .wdata_i ( a_wdata_i ),
        .rdata_o ( a_rdata   )
    );

    assign a_zero_o     = a_rdata[9];
    assign a_log_q3_6_o = a_rdata[8:0];

    // Per-channel D_skip & S4D Static Delta table
    logic [15:0] d_param_mem [0:MAX_D-1];

    initial begin
        for (int i = 0; i < MAX_D; i++) begin
            d_param_mem[i] = 16'h10_00; // Default delta_s4d=0.25 (16/64), D_skip=0
        end
    end

    always_ff @(posedge clk_i) begin
        if (d_we_i) begin
            d_param_mem[d_addr_i] <= d_wdata_i;
        end
    end

    assign delta_s4d_o = d_param_mem[d_addr_i][15:8];
    assign d_skip_s8_o = $signed(d_param_mem[d_addr_i][7:0]);

    // Trainable 4-Tap Causal Conv1D Packed Log8 Kernel Table (4 bytes/ch, 256 B total)
    // Default smooth causal kernel: {tap3=+1.0(00), tap2=+1.0(00), tap1=+2.0(10), tap0=+4.0(20)}
    logic [31:0] conv_w_mem [0:MAX_D-1];

    initial begin
        for (int i = 0; i < MAX_D; i++) begin
            conv_w_mem[i] = 32'h00_00_10_20;
        end
    end

    always_ff @(posedge clk_i) begin
        if (conv_w_we_i) begin
            conv_w_mem[conv_w_addr_i] <= conv_w_wdata_i;
        end
    end

    assign conv_w_log8_o = conv_w_mem[conv_w_addr_i];

endmodule : chronos_tcdm_bias_param
