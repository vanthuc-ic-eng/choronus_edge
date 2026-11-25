// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/mem/chronos_tcdm_weight_log8.sv
// Desc   : Bank 2 TCDM — Packed 32-bit Log8 Weight Memory.
//          Delivers 4 packed Log8 weights per cycle (32-bit bus) directly to
//          Engine A's 1x4 SIMD MAC array (saving 41.7% BRAM vs 48-bit bus).
// ============================================================================

`timescale 1ns / 1ps

module chronos_tcdm_weight_log8 #(
    parameter int unsigned DEPTH    = 2048,        // 2048 x 32b = 8 KB (expandable to 32 KB)
    parameter int unsigned ADDR_W   = $clog2(DEPTH),
    parameter              INIT_HEX = ""
)(
    input  logic              clk_i,
    input  logic              en_i,
    input  logic              we_i,
    input  logic [ADDR_W-1:0] addr_i,
    input  logic [31:0]       wdata_i,
    output logic [7:0]        w0_log8_o,
    output logic [7:0]        w1_log8_o,
    output logic [7:0]        w2_log8_o,
    output logic [7:0]        w3_log8_o
);

    logic [31:0] rdata_word;

    chronos_sp_sram #(
        .DATA_W   ( 32       ),
        .DEPTH    ( DEPTH    ),
        .INIT_HEX ( INIT_HEX )
    ) u_weight_bram (
        .clk_i   ( clk_i      ),
        .en_i    ( en_i       ),
        .we_i    ( we_i       ),
        .addr_i  ( addr_i     ),
        .wdata_i ( wdata_i    ),
        .rdata_o ( rdata_word )
    );

    assign w0_log8_o = rdata_word[7:0];
    assign w1_log8_o = rdata_word[15:8];
    assign w2_log8_o = rdata_word[23:16];
    assign w3_log8_o = rdata_word[31:24];

endmodule : chronos_tcdm_weight_log8
