// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/mem/chronos_tcdm_input_ring.sv
// Desc   : Bank 0 TCDM — Streaming Input Ring Buffer + Causal Conv1D Tap
//          Shift Register (K=4) + Channel Scratchpad (x, z, out).
//          Provides 1-cycle simultaneous access to all K=4 causal taps
//          {x[t], x[t-1], x[t-2], x[t-3]} for any channel d in [0..MAX_D-1]!
// ============================================================================

`timescale 1ns / 1ps

module chronos_tcdm_input_ring #(
    parameter int unsigned MAX_D      = 64,
    parameter int unsigned RING_DEPTH = 256,
    parameter int unsigned D_ADDR_W   = $clog2(MAX_D),
    parameter int unsigned R_ADDR_W   = $clog2(RING_DEPTH)
)(
    input  logic               clk_i,
    input  logic               rst_ni,
    input  logic               seq_rst_i,     // Clear causal Conv1D tap history

    // Streaming Sample Write Interface (Host / DMA / UART Rx)
    input  logic               stream_we_i,
    input  logic signed [7:0]  stream_data_i,
    output logic               ring_full_o,
    output logic [R_ADDR_W:0]  ring_count_o,

    // Read Port for Engine A (Input vector u[t])
    input  logic               u_pop_i,       // Advance ring read pointer
    input  logic [R_ADDR_W-1:0] u_rd_addr_i,
    output logic signed [7:0]  u_rdata_o,

    // Causal Conv1D K=4 Tap Shift Register Interface (per channel d)
    input  logic               conv_push_i,   // Shift in new x_raw[t] for channel conv_ch_i
    input  logic [D_ADDR_W-1:0] conv_ch_i,
    input  logic signed [7:0]  conv_x_new_i,
    output logic signed [7:0]  tap0_o,        // x[t]   (newest)
    output logic signed [7:0]  tap1_o,        // x[t-1]
    output logic signed [7:0]  tap2_o,        // x[t-2]
    output logic signed [7:0]  tap3_o,        // x[t-3] (oldest)

    // Channel Scratchpad (stores gated out_d for Linear_out without DRAM traffic)
    input  logic               out_we_i,
    input  logic [D_ADDR_W-1:0] out_waddr_i,
    input  logic signed [7:0]  out_wdata_i,
    input  logic [D_ADDR_W-1:0] out_raddr_i,
    output logic signed [7:0]  out_rdata_o
);

    // ------------------------------------------------------------------------
    // 1. Streaming Circular Ring Buffer
    // ------------------------------------------------------------------------
    logic signed [7:0] ring_mem [0:RING_DEPTH-1];
    logic [R_ADDR_W-1:0] wr_ptr;
    logic [R_ADDR_W-1:0] rd_ptr;
    logic [R_ADDR_W:0]   count_q;

    assign ring_full_o  = (count_q == RING_DEPTH[R_ADDR_W:0]);
    assign ring_count_o = count_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            wr_ptr  <= '0;
            rd_ptr  <= '0;
            count_q <= '0;
        end else if (seq_rst_i) begin
            wr_ptr  <= '0;
            rd_ptr  <= '0;
            count_q <= '0;
        end else begin
            if (stream_we_i && !ring_full_o) begin
                ring_mem[wr_ptr] <= stream_data_i;
                wr_ptr <= wr_ptr + 1'b1;
            end
            if (u_pop_i && (count_q != '0)) begin
                rd_ptr <= rd_ptr + 1'b1;
            end
            case ({stream_we_i && !ring_full_o, u_pop_i && (count_q != '0)})
                2'b10:   count_q <= count_q + 1'b1;
                2'b01:   count_q <= count_q - 1'b1;
                default: count_q <= count_q;
            endcase
        end
    end

    assign u_rdata_o = ring_mem[u_rd_addr_i];

    // ------------------------------------------------------------------------
    // 2. Causal Conv1D Tap History (3 previous samples per channel -> 192 B)
    //    1-cycle virtual clear bitmask avoids flushing array on seq_rst_i!
    // ------------------------------------------------------------------------
    logic signed [7:0] hist_t1 [0:MAX_D-1];
    logic signed [7:0] hist_t2 [0:MAX_D-1];
    logic signed [7:0] hist_t3 [0:MAX_D-1];
    logic [MAX_D-1:0]  hist_valid_mask;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            hist_valid_mask <= '0;
        end else if (seq_rst_i) begin
            hist_valid_mask <= '0; // Instant 1-cycle zero history reset!
        end else if (conv_push_i) begin
            if (hist_valid_mask[conv_ch_i]) begin
                hist_t3[conv_ch_i] <= hist_t2[conv_ch_i];
                hist_t2[conv_ch_i] <= hist_t1[conv_ch_i];
            end else begin
                hist_t3[conv_ch_i] <= 8'sd0;
                hist_t2[conv_ch_i] <= 8'sd0;
            end
            hist_t1[conv_ch_i]         <= conv_x_new_i;
            hist_valid_mask[conv_ch_i] <= 1'b1;
        end
    end

    assign tap0_o = conv_x_new_i;
    assign tap1_o = hist_valid_mask[conv_ch_i] ? hist_t1[conv_ch_i] : 8'sd0;
    assign tap2_o = hist_valid_mask[conv_ch_i] ? hist_t2[conv_ch_i] : 8'sd0;
    assign tap3_o = hist_valid_mask[conv_ch_i] ? hist_t3[conv_ch_i] : 8'sd0;

    // ------------------------------------------------------------------------
    // 3. Channel Output Scratchpad (out_d from Engine B -> Linear_out Engine A)
    // ------------------------------------------------------------------------
    logic signed [7:0] out_buf [0:MAX_D-1];

    always_ff @(posedge clk_i) begin
        if (out_we_i) begin
            out_buf[out_waddr_i] <= out_wdata_i;
        end
    end

    assign out_rdata_o = out_buf[out_raddr_i];

endmodule : chronos_tcdm_input_ring
