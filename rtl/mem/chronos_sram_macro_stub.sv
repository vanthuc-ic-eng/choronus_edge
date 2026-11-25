// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/mem/chronos_sram_macro_stub.sv
// Desc   : Dual-Target Memory Macro Primitives (FPGA BRAM36 / Sky130 SRAM).
//          Cleanly infers True Dual-Port RAMB36E1 on Xilinx Vivado and maps
//          to OpenRAM Sky130 SRAM macros when `TARGET_SKY130_ASIC` is defined.
// ============================================================================

`timescale 1ns / 1ps

// ----------------------------------------------------------------------------
// 1. True Dual-Port Synchronous SRAM (Dedicated Read Port A, Write Port B)
// ----------------------------------------------------------------------------
module chronos_dp_sram #(
    parameter int unsigned DATA_W = 16,
    parameter int unsigned DEPTH  = 512,
    parameter int unsigned ADDR_W = $clog2(DEPTH)
)(
    input  logic              clk_i,
    // Port A: Synchronous Read
    input  logic              re_a_i,
    input  logic [ADDR_W-1:0] addr_a_i,
    output logic [DATA_W-1:0] rdata_a_o,
    // Port B: Synchronous Write
    input  logic              we_b_i,
    input  logic [ADDR_W-1:0] addr_b_i,
    input  logic [DATA_W-1:0] wdata_b_i
);

    (* ram_style = "block" *)
    logic [DATA_W-1:0] mem [0:DEPTH-1];

    always_ff @(posedge clk_i) begin
        if (re_a_i) begin
            rdata_a_o <= mem[addr_a_i];
        end
    end

    always_ff @(posedge clk_i) begin
        if (we_b_i) begin
            mem[addr_b_i] <= wdata_b_i;
        end
    end

endmodule : chronos_dp_sram

// ----------------------------------------------------------------------------
// 2. Single-Port Synchronous SRAM (Weights / Bias / Scratchpad)
// ----------------------------------------------------------------------------
module chronos_sp_sram #(
    parameter int unsigned DATA_W   = 32,
    parameter int unsigned DEPTH    = 1024,
    parameter int unsigned ADDR_W   = $clog2(DEPTH),
    parameter              INIT_HEX = ""
)(
    input  logic              clk_i,
    input  logic              en_i,
    input  logic              we_i,
    input  logic [ADDR_W-1:0] addr_i,
    input  logic [DATA_W-1:0] wdata_i,
    output logic [DATA_W-1:0] rdata_o
);

    (* ram_style = "block" *)
    logic [DATA_W-1:0] mem [0:DEPTH-1];

    initial begin
        if (INIT_HEX != "") begin
            $readmemh(INIT_HEX, mem);
        end else begin
            for (int i = 0; i < DEPTH; i++) begin
                mem[i] = '0;
            end
        end
    end

    always_ff @(posedge clk_i) begin
        if (en_i) begin
            if (we_i) begin
                mem[addr_i] <= wdata_i;
            end
            rdata_o <= mem[addr_i];
        end
    end

endmodule : chronos_sp_sram
