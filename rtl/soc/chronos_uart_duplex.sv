// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Edge AI SoC
// File   : rtl/soc/chronos_uart_duplex.sv
// Desc   : Full-Duplex 8-N-1 UART Controller with Hardware Auto-Stream DMA!
//          Register Map (Base 0x5000_0000):
//            - 0x00: TX_DATA (W) / RX_DATA (R, clears rx_valid)
//            - 0x04: STATUS  (R): [0]=tx_ready, [1]=rx_valid, [2]=auto_stream
//            - 0x08: CTRL    (R/W): [0]=auto_stream_en (hardware-pushes RX
//                    bytes directly into CHRONOS Bank 0 Ring Buffer with
//                    ZERO CPU overhead!)
// ============================================================================

`timescale 1ns / 1ps

module chronos_uart_duplex #(
    parameter int unsigned CLK_FREQ_HZ = 100_000_000,
    parameter int unsigned BAUD_RATE   = 115_200
)(
    input  logic        clk_i,
    input  logic        rst_ni,

    // Memory-Mapped Bus Slave Port
    input  logic        req_i,
    input  logic        we_i,
    input  logic [3:0]  addr_i,
    input  logic [31:0] wdata_i,
    output logic [31:0] rdata_o,
    output logic        valid_o,

    // Hardware Auto-Stream Direct Feed to CHRONOS Bank 0 Ring Buffer
    output logic        auto_stream_we_o,
    output logic signed [7:0] auto_stream_data_o,

    // Physical Serial Pins
    input  logic        uart_rxd_i,
    output logic        uart_txd_o
);

    localparam int unsigned CLKS_PER_BIT  = (CLK_FREQ_HZ / BAUD_RATE);
    localparam int unsigned HALF_BIT_CLKS = CLKS_PER_BIT / 2;

    localparam logic [1:0] S_IDLE  = 2'd0;
    localparam logic [1:0] S_START = 2'd1;
    localparam logic [1:0] S_DATA  = 2'd2;
    localparam logic [1:0] S_STOP  = 2'd3;

    // TX State
    logic [1:0]  tx_state_q;
    logic [15:0] tx_cnt_q;
    logic [2:0]  tx_bit_q;
    logic [7:0]  tx_shift_q;
    logic        tx_busy;

    // RX State (2-FF metastability synchronizer on uart_rxd_i)
    logic        rx_sync0_q, rx_sync1_q;
    logic [1:0]  rx_state_q;
    logic [15:0] rx_cnt_q;
    logic [2:0]  rx_bit_q;
    logic [7:0]  rx_shift_q;
    logic [7:0]  rx_data_q;
    logic        rx_valid_q;
    logic        rx_done_pulse;
    logic        auto_stream_en_q;

    wire _unused_uart = &{1'b0, addr_i[1:0], wdata_i[31:8]};

    assign tx_busy            = (tx_state_q != S_IDLE);
    assign auto_stream_we_o   = rx_done_pulse && auto_stream_en_q;
    assign auto_stream_data_o = $signed(rx_shift_q);

    // 2-FF RX Synchronizer
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rx_sync0_q <= 1'b1;
            rx_sync1_q <= 1'b1;
        end else begin
            rx_sync0_q <= uart_rxd_i;
            rx_sync1_q <= rx_sync0_q;
        end
    end

    // Bus Interface & Control Registers
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_o          <= 1'b0;
            rdata_o          <= 32'd0;
            auto_stream_en_q <= 1'b0;
        end else begin
            valid_o <= req_i;
            if (req_i && we_i && (addr_i[3:2] == 2'b10)) begin
                auto_stream_en_q <= wdata_i[0];
            end
            if (req_i && !we_i) begin
                case (addr_i[3:2])
                    2'b00:   rdata_o <= {24'd0, rx_data_q};
                    2'b01:   rdata_o <= {29'd0, auto_stream_en_q, rx_valid_q, !tx_busy};
                    2'b10:   rdata_o <= {31'd0, auto_stream_en_q};
                    default: rdata_o <= 32'd0;
                endcase
            end else begin
                rdata_o <= 32'd0;
            end
        end
    end

    // UART Transmitter FSM
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            tx_state_q <= S_IDLE;
            tx_cnt_q   <= 16'd0;
            tx_bit_q   <= 3'd0;
            tx_shift_q <= 8'd0;
            uart_txd_o <= 1'b1;
        end else begin
            case (tx_state_q)
                S_IDLE: begin
                    uart_txd_o <= 1'b1;
                    tx_cnt_q   <= 16'd0;
                    tx_bit_q   <= 3'd0;
                    if (req_i && we_i && (addr_i[3:2] == 2'b00)) begin
                        tx_shift_q <= wdata_i[7:0];
                        tx_state_q <= S_START;
                    end
                end
                S_START: begin
                    uart_txd_o <= 1'b0;
                    if (tx_cnt_q < 16'(CLKS_PER_BIT - 1)) tx_cnt_q <= tx_cnt_q + 16'd1;
                    else begin tx_cnt_q <= 16'd0; tx_state_q <= S_DATA; end
                end
                S_DATA: begin
                    uart_txd_o <= tx_shift_q[tx_bit_q];
                    if (tx_cnt_q < 16'(CLKS_PER_BIT - 1)) tx_cnt_q <= tx_cnt_q + 16'd1;
                    else begin
                        tx_cnt_q <= 16'd0;
                        if (tx_bit_q < 3'd7) tx_bit_q <= tx_bit_q + 3'd1;
                        else begin tx_bit_q <= 3'd0; tx_state_q <= S_STOP; end
                    end
                end
                S_STOP: begin
                    uart_txd_o <= 1'b1;
                    if (tx_cnt_q < 16'(CLKS_PER_BIT - 1)) tx_cnt_q <= tx_cnt_q + 16'd1;
                    else begin tx_cnt_q <= 16'd0; tx_state_q <= S_IDLE; end
                end
                default: tx_state_q <= S_IDLE;
            endcase
        end
    end

    // UART Receiver FSM
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rx_state_q    <= S_IDLE;
            rx_cnt_q      <= 16'd0;
            rx_bit_q      <= 3'd0;
            rx_shift_q    <= 8'd0;
            rx_data_q     <= 8'd0;
            rx_valid_q    <= 1'b0;
            rx_done_pulse <= 1'b0;
        end else begin
            rx_done_pulse <= 1'b0;
            if (req_i && !we_i && (addr_i[3:2] == 2'b00)) begin
                rx_valid_q <= 1'b0; // Reading RX_DATA clears valid
            end

            case (rx_state_q)
                S_IDLE: begin
                    rx_cnt_q <= 16'd0;
                    rx_bit_q <= 3'd0;
                    if (!rx_sync1_q) begin // Falling edge = Start bit
                        rx_state_q <= S_START;
                    end
                end
                S_START: begin
                    if (rx_cnt_q < 16'(HALF_BIT_CLKS - 1)) begin
                        rx_cnt_q <= rx_cnt_q + 16'd1;
                    end else begin
                        rx_cnt_q   <= 16'd0;
                        rx_state_q <= (!rx_sync1_q) ? S_DATA : S_IDLE;
                    end
                end
                S_DATA: begin
                    if (rx_cnt_q < 16'(CLKS_PER_BIT - 1)) begin
                        rx_cnt_q <= rx_cnt_q + 16'd1;
                    end else begin
                        rx_cnt_q             <= 16'd0;
                        rx_shift_q[rx_bit_q] <= rx_sync1_q;
                        if (rx_bit_q < 3'd7) rx_bit_q <= rx_bit_q + 3'd1;
                        else begin rx_bit_q <= 3'd0; rx_state_q <= S_STOP; end
                    end
                end
                S_STOP: begin
                    if (rx_cnt_q < 16'(CLKS_PER_BIT - 1)) begin
                        rx_cnt_q <= rx_cnt_q + 16'd1;
                    end else begin
                        rx_cnt_q      <= 16'd0;
                        rx_state_q    <= S_IDLE;
                        rx_data_q     <= rx_shift_q;
                        rx_valid_q    <= 1'b1;
                        rx_done_pulse <= 1'b1;
                    end
                end
                default: rx_state_q <= S_IDLE;
            endcase
        end
    end

endmodule : chronos_uart_duplex
