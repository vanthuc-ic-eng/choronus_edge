// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/mem/chronos_tcdm_state_dp.sv
// Desc   : ★ Q1 CORE INNOVATION 5 — Zero-Bubble In-Place State Scratchpad ★
//          Bank 1 TCDM holding hidden state matrix h[t] in signed INT16 (Q5.10).
//          Features:
//            1) True Dual-Port BRAM/SRAM: Port A reads h[t-1] while Port B
//               writes h[t] in the EXACT same clock cycle (0-bubble RMW).
//            2) 1-Cycle Virtual State Reset (`valid_mask`): Resets all 512
//               state words to 0 in ONE clock cycle (zero SRAM flush energy!).
//            3) Same-Address Write-to-Read Forwarding Bypass: Prevents RAW
//               pipeline hazards if N=1 back-to-back access occurs.
// ============================================================================

`timescale 1ns / 1ps

module chronos_tcdm_state_dp #(
    parameter int unsigned DEPTH  = 512,            // 512 x 16b = 1 KB (fits D=64, N=8)
    parameter int unsigned ADDR_W = $clog2(DEPTH)
)(
    input  logic               clk_i,
    input  logic               rst_ni,
    input  logic               state_rst_i,         // 1-cycle instant clear of all h[t] = 0

    // Port A: Synchronous Read h[t-1]
    input  logic               re_a_i,
    input  logic [ADDR_W-1:0]  addr_a_i,
    output logic signed [15:0] rdata_a_o,

    // Port B: Synchronous Write h[t]
    input  logic               we_b_i,
    input  logic [ADDR_W-1:0]  addr_b_i,
    input  logic signed [15:0] wdata_b_i
);

    logic [15:0] raw_rdata_a;

    // True Dual-Port SRAM Macro
    chronos_dp_sram #(
        .DATA_W ( 16    ),
        .DEPTH  ( DEPTH )
    ) u_state_dp_sram (
        .clk_i     ( clk_i       ),
        .re_a_i    ( re_a_i      ),
        .addr_a_i  ( addr_a_i    ),
        .rdata_a_o ( raw_rdata_a ),
        .we_b_i    ( we_b_i      ),
        .addr_b_i  ( addr_b_i    ),
        .wdata_b_i ( wdata_b_i   )
    );

    // 1-Cycle Virtual Clear Bitmask + RAW Same-Address Forwarding
    logic [DEPTH-1:0]  valid_mask_q;
    logic              rd_valid_q;
    logic              fwd_hit_q;
    logic signed [15:0] fwd_data_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_mask_q <= '0;
            rd_valid_q   <= 1'b0;
            fwd_hit_q    <= 1'b0;
            fwd_data_q   <= 16'sd0;
        end else if (state_rst_i) begin
            valid_mask_q <= '0; // Instant 1-cycle zero clear!
            rd_valid_q   <= 1'b0;
            fwd_hit_q    <= 1'b0;
            fwd_data_q   <= 16'sd0;
        end else begin
            if (we_b_i) begin
                valid_mask_q[addr_b_i] <= 1'b1;
            end
            if (re_a_i) begin
                rd_valid_q <= valid_mask_q[addr_a_i];
                fwd_hit_q  <= we_b_i && (addr_a_i == addr_b_i);
                fwd_data_q <= wdata_b_i;
            end
        end
    end

    always_comb begin
        if (fwd_hit_q)
            rdata_a_o = fwd_data_q;
        else if (!rd_valid_q)
            rdata_a_o = 16'sd0;
        else
            rdata_a_o = $signed(raw_rdata_a);
    end

endmodule : chronos_tcdm_state_dp
