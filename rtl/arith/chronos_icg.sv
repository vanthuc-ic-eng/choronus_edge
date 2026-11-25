// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/arith/chronos_icg.sv
// Desc   : Glitch-free Latch-Based Integrated Clock Gating (ICG) Cell.
//          Supports dual-target synthesis:
//            - ASIC (Sky130 sky130_fd_sc_hd__dlclkp_1 / Nangate CLKGATE)
//            - FPGA (Safe behavioral latch or BUFGCE / Slice CE)
// ============================================================================

`timescale 1ns / 1ps

module chronos_icg (
    input  logic clk_i,
    input  logic en_i,
    input  logic test_en_i,
    output logic clk_o
);

`ifdef TARGET_SKY130_ASIC
    // Direct foundry macro mapping for SkyWater 130nm HD library
    sky130_fd_sc_hd__dlclkp_1 u_sky130_icg (
        .CLK  ( clk_i            ),
        .GATE ( en_i | test_en_i ),
        .GCLK ( clk_o            )
    );
`elsif SYNTHESIS_FPGA_NO_GATED_CLK
    // FPGA-safe mode: pass clock un-gated (logic uses CE ports directly)
    wire _unused_ok = &{1'b0, en_i, test_en_i};
    assign clk_o = clk_i;
`else
    // Standard glitch-free negative-level latch ICG for simulation & ASIC RTL
    logic en_latched;

    /* verilator lint_off LATCH */
    always_latch begin
        if (!clk_i) begin
            en_latched = en_i | test_en_i;
        end
    end
    /* verilator lint_on LATCH */

    assign clk_o = clk_i & en_latched;
`endif

endmodule : chronos_icg
